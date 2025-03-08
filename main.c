#include <linux/vfio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include "desc_queue.h"
#include "mmio_api.h"

#define NUM_DEVICES                 3
#define NUM_BARS                    3
#define BAR4                        (NUM_BARS-1)
#define SIGNATURE_OFFSET            0x2000000 /* BAR4 index 8 is at this offset */
#define OCTBOOT_NET_MAXQ            1

#define OCTNET_TX_DESCQ_OFFSET      0x2000400
#define OCTNET_RX_DESCQ_OFFSET      0x2010000
#define TX_DESCQ_OFFSET(mdev)       ((uint8_t*)mdev->bar_map[BAR4].bar_addr + OCTNET_TX_DESCQ_OFFSET)
#define RX_DESCQ_OFFSET(mdev)       ((uint8_t*)mdev->bar_map[BAR4].bar_addr + OCTNET_RX_DESCQ_OFFSET)
#define OCTBOOT_NET_DESCQ_CLEAN 0
#define OCTBOOT_NET_DESCQ_READY 1

#define DPI_MAX_PTR_1500_MTU 15
#define DPI_MAX_PTR_9600_MTU 4
#define RECV_BUF_SIZE_1500_MTU 2048
#define RECV_BUF_SIZE_9600_MTU 12288
//#define MTU_1500_SETTINGS
#define MTU_9600_SETTINGS
#if defined(MTU_1500_SETTINGS)
#define OCTBOOT_NET_MAX_MTU 1500
#define DPIX_MAX_PTR DPI_MAX_PTR_1500_MTU
#define OCTBOOT_NET_RX_BUF_SIZE RECV_BUF_SIZE_1500_MTU
#elif defined(MTU_9600_SETTINGS)
#define OCTBOOT_NET_MAX_MTU 9600
#define DPIX_MAX_PTR DPI_MAX_PTR_9600_MTU
#define OCTBOOT_NET_RX_BUF_SIZE RECV_BUF_SIZE_9600_MTU
#endif

#define OCTBOOT_NET_NUM_ELEMENTS 256

struct user_queue_lock {
    pthread_mutex_t mutex;
    atomic_flag spinlock;
};

static inline void queue_lock_init(struct user_queue_lock *lock) {
    pthread_mutex_init(&lock->mutex, NULL);
    atomic_flag_clear(&lock->spinlock);
}

static inline void queue_spin_lock(atomic_flag *lock) {
    while (atomic_flag_test_and_set_explicit(lock, memory_order_acquire))
        ; // Spin
}

static inline void queue_spin_unlock(atomic_flag *lock) {
    atomic_flag_clear_explicit(lock, memory_order_release);
}

static inline void queue_mutex_lock(pthread_mutex_t *lock) {
    pthread_mutex_lock(lock);
}

static inline void queue_mutex_unlock(pthread_mutex_t *lock) {
    pthread_mutex_unlock(lock);
}

struct octboot_net_sw_desc_ptr {
    union {
        uint64_t val;
        struct {
			uint64_t rsvd:47;
			uint64_t used:1; /* allocated or not */
			uint64_t ptr_len:16; /* length of this buf */
        } s_mgmt_net;
    } hdr;
    uint64_t ptr; /* pktbuf address */
}__attribute__((packed));

struct octboot_net_sw_descq {
    struct user_queue_lock lock;
    uint32_t mask;
    uint32_t status;
    uint32_t pending;

    uint32_t local_prod_idx;
    uint32_t local_cons_idx;
    void* vaddr_pktbuf;
    void* iova_pktbuf;

    void* vaddr_prod_idx;
    void* iova_prod_idx;
    void* vaddr_cons_idx;
    void* iova_cons_idx;

    struct octboot_net_sw_desc_ptr sw_desc_arr[OCTBOOT_NET_NUM_ELEMENTS];
    uint8_t* hw_descq;
    uint32_t* hw_prod_idx;
    uint32_t* hw_cons_idx;
};

typedef struct {
    void* bar_addr;
    int bar_size;
} bar_map_t;

typedef struct {
    char vfio_path[32];
    char pci_addr[32];
	int container_fd;
    int group_fd;
    int device_fd;
    bar_map_t bar_map[NUM_BARS];
    struct octboot_net_sw_descq rxq[OCTBOOT_NET_MAXQ];
	struct octboot_net_sw_descq txq[OCTBOOT_NET_MAXQ];
} octboot_net_device_t;

static inline uint32_t octboot_net_circq_add(uint32_t index, uint32_t add,
    uint32_t mask)
{
return (index + add) & mask;
}

static inline uint32_t octboot_net_circq_inc(uint32_t index, uint32_t mask)
{
return octboot_net_circq_add(index, 1, mask);
}

static inline uint32_t octboot_net_circq_depth(uint32_t pi, uint32_t ci,
  uint32_t mask)
{
return (pi - ci) & mask;
}

static inline uint32_t octboot_net_circq_space(uint32_t pi, uint32_t ci,
  uint32_t mask)
{
return mask - octboot_net_circq_depth(pi, ci, mask);
}

int mdev_clean_rx_ring(octboot_net_device_t* mdev) {
    struct octboot_net_sw_descq* rq = &mdev->rxq[0];
    if (rq->status == OCTBOOT_NET_DESCQ_CLEAN)
        return 0;

    rq->mask = 0;
    rq->pending = 0;
    rq->local_prod_idx = 0;
    rq->local_cons_idx = 0;
    int sw_descq_tot_size = OCTBOOT_NET_NUM_ELEMENTS * sizeof(struct octboot_net_sw_desc_ptr);
    memset(rq->sw_desc_arr, 0, sw_descq_tot_size);

    int descq_tot_size = sizeof(struct octboot_net_hw_descq) +
		(OCTBOOT_NET_NUM_ELEMENTS * sizeof(struct octboot_net_hw_desc_ptr));
	wmb();
	mmio_memset(rq->hw_descq, 0, descq_tot_size);

    rq->hw_descq = NULL;
    rq->hw_prod_idx = NULL;
    rq->hw_cons_idx = NULL;
    rq->status = OCTBOOT_NET_DESCQ_CLEAN;
    return 0;
}

int vfio_dma_unmap_pktbuf(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        fprintf(stderr, "invalid parameter of mdev\n");
        return -1;
    }

    struct vfio_iommu_type1_dma_unmap dma_unmap1 = {
        .argsz = sizeof(dma_unmap1),
        .size = OCTBOOT_NET_RX_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS,
        .iova = (uint64_t)mdev->rxq[0].iova_pktbuf
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap1);
    munmap(mdev->rxq[0].vaddr_pktbuf, OCTBOOT_NET_RX_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS);
    mdev->rxq[0].vaddr_pktbuf = NULL;
    mdev->rxq[0].iova_pktbuf = NULL;

    struct vfio_iommu_type1_dma_unmap dma_unmap2 = {
        .argsz = sizeof(dma_unmap2),
        .size = OCTBOOT_NET_RX_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS,
        .iova = (uint64_t)mdev->txq[0].iova_pktbuf
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap2);
    munmap(mdev->txq[0].vaddr_pktbuf, OCTBOOT_NET_RX_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS);
    mdev->txq[0].vaddr_pktbuf = NULL;
    mdev->txq[0].iova_pktbuf = NULL;
    return 0;
}

int vfio_dma_unmap_circq(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        fprintf(stderr, "invalid parameter of mdev\n");
        return -1;
    }

    struct vfio_iommu_type1_dma_unmap dma_unmap1 = {
        .argsz = sizeof(dma_unmap1),
        .size = OCTBOOT_NET_MAXQ,
        .iova = (uint64_t)mdev->rxq[0].iova_prod_idx
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap1);
    munmap(mdev->rxq[0].vaddr_prod_idx, OCTBOOT_NET_MAXQ);
    mdev->rxq[0].vaddr_prod_idx = NULL;
    mdev->rxq[0].iova_prod_idx = NULL;

    struct vfio_iommu_type1_dma_unmap dma_unmap2 = {
        .argsz = sizeof(dma_unmap2),
        .size = OCTBOOT_NET_MAXQ,
        .iova = (uint64_t)mdev->rxq[0].iova_cons_idx
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap2);
    munmap(mdev->rxq[0].vaddr_cons_idx, OCTBOOT_NET_MAXQ);
    mdev->rxq[0].vaddr_cons_idx = NULL;
    mdev->rxq[0].iova_cons_idx = NULL;

    struct vfio_iommu_type1_dma_unmap dma_unmap3 = {
        .argsz = sizeof(dma_unmap3),
        .size = OCTBOOT_NET_MAXQ,
        .iova = (uint64_t)mdev->txq[0].iova_prod_idx
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap3);
    munmap(mdev->txq[0].vaddr_prod_idx, OCTBOOT_NET_MAXQ);
    mdev->txq[0].vaddr_prod_idx = NULL;
    mdev->txq[0].iova_prod_idx = NULL;

    struct vfio_iommu_type1_dma_unmap dma_unmap4 = {
        .argsz = sizeof(dma_unmap4),
        .size = OCTBOOT_NET_MAXQ,
        .iova = (uint64_t)mdev->txq[0].iova_cons_idx
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap4);
    munmap(mdev->txq[0].vaddr_cons_idx, OCTBOOT_NET_MAXQ);
    mdev->txq[0].vaddr_cons_idx = NULL;
    mdev->txq[0].iova_cons_idx = NULL;
    return 0;
}

int vfio_bar_unmap(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        fprintf(stderr, "invalid parameter of mdev\n");
        return -1;
    }

    for (int i = 0; i < NUM_BARS; i++) {
        if (mdev->bar_map[i].bar_addr != NULL && mdev->bar_map[i].bar_addr != MAP_FAILED) {
            munmap(mdev->bar_map[i].bar_addr, mdev->bar_map[i].bar_size);
            mdev->bar_map[i].bar_addr = NULL;
            mdev->bar_map[i].bar_size = 0;
        }
    }

    return 0;
}

int vfio_uninit(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        fprintf(stderr, "invalid parameter of mdev\n");
        return -1;
    }

    vfio_dma_unmap_pktbuf(mdev);
    vfio_dma_unmap_circq(mdev);
    vfio_bar_unmap(mdev);

    if (mdev->device_fd > 0) {
        close(mdev->device_fd);
        mdev->device_fd = -1;
    }

    if (mdev->group_fd > 0) {
        close(mdev->group_fd);
        mdev->group_fd = -1;
    }

    if (mdev->container_fd > 0) {
        close(mdev->container_fd);
        mdev->container_fd = -1;
    }

    return 0;
}

int vfio_bar_map(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        fprintf(stderr, "invalid parameter of mdev\n");
        return -1;
    }

    for (int i = 0; i < NUM_BARS; i++) {
        struct vfio_region_info reg = {
            .argsz = sizeof(reg),
            .index = VFIO_PCI_BAR0_REGION_INDEX + (i * 2) // BAR0, BAR2, BAR4
        };
        if (ioctl(mdev->device_fd, VFIO_DEVICE_GET_REGION_INFO, &reg)) {
            fprintf(stderr, "failed to get region info\n");
            vfio_uninit(mdev);
            return -1;
        }

        mdev->bar_map[i].bar_size = reg.size;
        mdev->bar_map[i].bar_addr = mmap(NULL, reg.size, 
            PROT_READ | PROT_WRITE,
            MAP_SHARED,  // Note: should be MAP_SHARED not MAP_PRIVATE
            mdev->device_fd,
            reg.offset);  // Use reg.offset for BAR mapping
        if (mdev->bar_map[i].bar_addr == MAP_FAILED) {
            fprintf(stderr, "failed to map bar %d\n", i);
            vfio_uninit(mdev);
            return -1;
        }
    }

    return 0;
}

int vfio_dma_map_circq(octboot_net_device_t* mdev) {
    mdev->rxq[0].vaddr_prod_idx = mmap(NULL, OCTBOOT_NET_MAXQ, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map1 = {
        .argsz = sizeof(dma_map1),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->rxq[0].vaddr_prod_idx,
        .size = OCTBOOT_NET_MAXQ,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map1)) {
        fprintf(stderr, "failed to map dma\n");
        return -1;
    }
    mdev->rxq[0].iova_prod_idx = (void*)dma_map1.iova;

    mdev->rxq[0].vaddr_cons_idx = mmap(NULL, OCTBOOT_NET_MAXQ, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map2 = {
        .argsz = sizeof(dma_map2),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->rxq[0].vaddr_cons_idx,
        .size = OCTBOOT_NET_MAXQ,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map2)) {
        fprintf(stderr, "failed to map dma\n");
        return -1;
    }
    mdev->rxq[0].iova_cons_idx = (void*)dma_map2.iova;

    mdev->txq[0].vaddr_prod_idx = mmap(NULL, OCTBOOT_NET_MAXQ, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map3 = {
        .argsz = sizeof(dma_map3),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->txq[0].vaddr_prod_idx,
        .size = OCTBOOT_NET_MAXQ,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map3)) {
        fprintf(stderr, "failed to map dma\n");
        return -1;
    }
    mdev->txq[0].iova_prod_idx = (void*)dma_map3.iova;

    mdev->txq[0].vaddr_cons_idx = mmap(NULL, OCTBOOT_NET_MAXQ, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map4 = {
        .argsz = sizeof(dma_map4),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->txq[0].vaddr_cons_idx,
        .size = OCTBOOT_NET_MAXQ,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map4)) {
        fprintf(stderr, "failed to map dma\n");
        return -1;
    }
    mdev->txq[0].iova_cons_idx = (void*)dma_map4.iova;
    return 0;
}

int vfio_dma_map_pktbuf(octboot_net_device_t* mdev) {

    mdev->rxq[0].vaddr_pktbuf = mmap(NULL, OCTBOOT_NET_RX_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map1 = {
        .argsz = sizeof(dma_map1),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->rxq[0].vaddr_pktbuf,
        .size = OCTBOOT_NET_RX_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map1)) {
        fprintf(stderr, "failed to map dma pktbuf\n");
        return -1;
    }
    mdev->rxq[0].iova_pktbuf = (void*)dma_map1.iova;

    mdev->txq[0].vaddr_pktbuf = mmap(NULL, OCTBOOT_NET_RX_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map2 = {
        .argsz = sizeof(dma_map2),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->txq[0].vaddr_pktbuf,
        .size = OCTBOOT_NET_RX_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map2)) {
        fprintf(stderr, "failed to map dma pktbuf\n");
        return -1;
    }
    mdev->txq[0].iova_pktbuf = (void*)dma_map2.iova;
    return 0;
}

int mdev_setup_rx_ring(octboot_net_device_t* mdev) {
    struct octboot_net_sw_descq* rq = &mdev->rxq[0];
    rq->mask = OCTBOOT_NET_NUM_ELEMENTS - 1;
    rq->pending = 0;
    rq->local_prod_idx = 0;
    rq->local_cons_idx = 0;
    rq->hw_descq = (uint8_t*)RX_DESCQ_OFFSET(mdev);
    rq->hw_prod_idx = (uint32_t *)(rq->hw_descq +
        offsetof(struct octboot_net_hw_descq, prod_idx));
    *(uint32_t*)rq->vaddr_cons_idx = 0;

    int descq_tot_size = sizeof(struct octboot_net_hw_descq) + (OCTBOOT_NET_NUM_ELEMENTS *
		sizeof(struct octboot_net_hw_desc_ptr));
    struct octboot_net_hw_descq *descq = calloc(1, descq_tot_size);
    if (!descq) {
        fprintf(stderr, "Failed to allocate descq\n");
        return -1;
    }

    descq->num_entries = OCTBOOT_NET_NUM_ELEMENTS;
    descq->buf_size = OCTBOOT_NET_RX_BUF_SIZE;
    descq->shadow_cons_idx_addr = (uint64_t)mdev->rxq[0].iova_cons_idx;

    for (int i = 0; i < OCTBOOT_NET_NUM_ELEMENTS; i++) {
        struct octboot_net_sw_desc_ptr* swptr = &rq->sw_desc_arr[i];
        memset(swptr, 0, sizeof(struct octboot_net_sw_desc_ptr));
        swptr->ptr = (uint64_t)((uint8_t*)mdev->rxq[0].vaddr_pktbuf + (i * OCTBOOT_NET_RX_BUF_SIZE));

        struct octboot_net_hw_desc_ptr* ptr = &descq->desc_arr[i];
        memset(ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
        ptr->hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
        ptr->ptr = (uint64_t)((uint8_t*)mdev->rxq[0].iova_pktbuf + (i * OCTBOOT_NET_RX_BUF_SIZE));
        rq->local_prod_idx = octboot_net_circq_inc(rq->local_prod_idx,
            rq->mask);
        descq->prod_idx = octboot_net_circq_inc(descq->prod_idx, rq->mask);
    }
    rq->status = OCTBOOT_NET_DESCQ_READY;
    queue_lock_init(&rq->lock);

    wmb();
    mmio_memwrite(rq->hw_descq, descq, descq_tot_size);
    free(descq);
    return 0;
}

int vfio_init(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        fprintf(stderr, "invalid parameter of mdev\n");
        return -1;
    }

    mdev->container_fd = open("/dev/vfio/vfio", O_RDWR);
    if (mdev->container_fd < 0) {
        fprintf(stderr, "failed to open vfio container\n");
        vfio_uninit(mdev);
        return -1;
    }

    mdev->group_fd = open(mdev->vfio_path, O_RDWR);
    if (mdev->group_fd < 0) {
        fprintf(stderr, "failed to open vfio group\n");
        vfio_uninit(mdev);
        return -1;
    }

    if (ioctl(mdev->group_fd, VFIO_GROUP_SET_CONTAINER, &mdev->container_fd)) {
        fprintf(stderr, "failed to bind vfio group to vfio container\n");
        vfio_uninit(mdev);
        return -1;
    }

    if (ioctl(mdev->container_fd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU)) {
        fprintf(stderr, "failed to set iommu type\n");
        vfio_uninit(mdev);
        return -1;
    }

    mdev->device_fd = ioctl(mdev->group_fd, VFIO_GROUP_GET_DEVICE_FD, "0000:b3:00.0");
    if (mdev->device_fd < 0) {
        fprintf(stderr, "failed to get device fd\n");
        vfio_uninit(mdev);
        return -1;
    }


    if (vfio_bar_map(mdev) < 0) {
        fprintf(stderr, "failed to map bars\n");
        vfio_uninit(mdev);
        return -1;
    }

    if (vfio_dma_map_circq(mdev) < 0) {
        fprintf(stderr, "failed to map dma\n");
        vfio_uninit(mdev);
        return -1;
    }

    if (vfio_dma_map_pktbuf(mdev) < 0) {
        fprintf(stderr, "failed to map dma pktbuf\n");
        vfio_uninit(mdev);
        return -1;
    }

    return 0;
}

int main(int argc, char *argv[]) {
    int ret = 0;
    octboot_net_device_t octbootdev[NUM_DEVICES];
    for (int i = 0; i < NUM_DEVICES; i++) {
        octbootdev[i].container_fd = -1;
        octbootdev[i].group_fd = -1;
        octbootdev[i].device_fd = -1;
        for (int j = 0; j < NUM_BARS; j++) {
            octbootdev[i].bar_map[j].bar_addr = NULL;
            octbootdev[i].bar_map[j].bar_size = 0;
        }
        octbootdev[i].rxq[0].vaddr_prod_idx = NULL;
        octbootdev[i].rxq[0].iova_prod_idx = NULL;
        octbootdev[i].rxq[0].vaddr_cons_idx = NULL;
        octbootdev[i].rxq[0].iova_cons_idx = NULL;
        octbootdev[i].txq[0].vaddr_prod_idx = NULL;
        octbootdev[i].txq[0].iova_prod_idx = NULL;
        octbootdev[i].txq[0].vaddr_cons_idx = NULL;
        octbootdev[i].txq[0].iova_cons_idx = NULL;
        octbootdev[i].rxq[0].vaddr_pktbuf = NULL;
        octbootdev[i].rxq[0].iova_pktbuf = NULL;
        octbootdev[i].txq[0].vaddr_pktbuf = NULL;
        octbootdev[i].txq[0].iova_pktbuf = NULL;
    }

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <vfio_group_id> <pci_addr>\n", argv[0]);
        fprintf(stderr, "Example: %s 5 0000:b3:00.0\n", argv[0]);
        return -1;
    }

    snprintf(octbootdev[0].vfio_path, sizeof(octbootdev[0].vfio_path), "/dev/vfio/%s", argv[1]);
    snprintf(octbootdev[0].pci_addr, sizeof(octbootdev[0].pci_addr), "%s", argv[2]);

    if (vfio_init(&octbootdev[0]) < 0) {
        fprintf(stderr, "failed to init vfio\n");
        return -1;
    }

    return ret;
}
