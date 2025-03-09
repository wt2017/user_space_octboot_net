#include <linux/vfio.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h> 
#include "desc_queue.h"
#include "mmio_api.h"

#define NUM_DEVICES                 3
#define NUM_BARS                    3
#define BAR4                        (NUM_BARS-1)
#define OCTBOOT_NET_MAXQ            1
#define OCTBOOT_NET_DESCQ_CLEAN 0
#define OCTBOOT_NET_DESCQ_READY 1

#define NPU_HANDSHAKE_SIGNATURE 0xABCDABCD
#define SIGNATURE_OFFSET 0x2000000 /* BAR4 index 8 is at this offset */
#define HOST_VERSION_OFFSET 0x2000008
#define HOST_STATUS_REG_OFFSET 0x2000080

#define OCTNET_HOST_DOWN                 0
#define OCTNET_HOST_READY                1
#define OCTNET_HOST_RUNNING              2
#define OCTNET_HOST_GOING_DOWN           3
#define OCTNET_HOST_FATAL                4

#define HOST_RESET_STATUS_REG_OFFSET 0x2000088
#define OCTNET_HOST_RESET_STATUS_BIT     0

#define HOST_MBOX_ACK_OFFSET 0x2000090
#define HOST_MBOX_OFFSET 0x2000098    /* Eight words at this offset */
#define TARGET_VERSION_OFFSET 0x2000060
#define TARGET_STATUS_REG_OFFSET 0x2000100


#define HOST_STATUS_REG(mdev)      ((uint8_t*)mdev->bar_map[BAR4].bar_addr + HOST_STATUS_REG_OFFSET)
#define HOST_RESET_STATUS_REG(mdev) ((uint8_t*)mdev->bar_map[BAR4].bar_addr + HOST_RESET_STATUS_REG_OFFSET)
#define HOST_VERSION_REG(mdev)      ((uint8_t*)mdev->bar_map[BAR4].bar_addr + HOST_VERSION_OFFSET)
#define HOST_MBOX_ACK_REG(mdev)    ((uint8_t*)mdev->bar_map[BAR4].bar_addr + HOST_MBOX_ACK_OFFSET)
#define HOST_MBOX_MSG_REG(mdev, i)    \
	((uint8_t*)mdev->bar_map[BAR4].bar_addr + HOST_MBOX_OFFSET + (i * 8))


#define OCTNET_TARGET_DOWN               0
#define OCTNET_TARGET_READY              1
#define OCTNET_TARGET_RUNNING            2
#define OCTNET_TARGET_GOING_DOWN         3
#define OCTNET_TARGET_FATAL              4


#define TARGET_MBOX_OFFSET 0x2000118
#define TARGET_MBOX_ACK_OFFSET 0x2000110
#define OCTNET_RX_DESC_OFFSET 0x20000B8
#define OCTNET_TX_DESC_OFFSET 0x20000c0

#define OCTNET_TX_DESCQ_OFFSET      0x2000400
#define OCTNET_RX_DESCQ_OFFSET      0x2010000
#define TX_DESCQ_OFFSET(mdev)       ((uint8_t*)mdev->bar_map[BAR4].bar_addr + OCTNET_TX_DESCQ_OFFSET)
#define RX_DESCQ_OFFSET(mdev)       ((uint8_t*)mdev->bar_map[BAR4].bar_addr + OCTNET_RX_DESCQ_OFFSET)

#define TARGET_STATUS_REG(mdev)        ((uint8_t*)mdev->bar_map[BAR4].bar_addr + TARGET_STATUS_REG_OFFSET)
#define TARGET_VERSION_REG(mdev)        ((uint8_t*)mdev->bar_map[BAR4].bar_addr + TARGET_VERSION_OFFSET)
#define TARGET_MBOX_MSG_REG(mdev, i)  \
	((uint8_t*)mdev->bar_map[BAR4].bar_addr + TARGET_MBOX_OFFSET + (i * 8))
#define TARGET_MBOX_ACK_REG(mdev)    \
	((uint8_t*)mdev->bar_map[BAR4].bar_addr + TARGET_MBOX_ACK_OFFSET)

#define OCTBOOT_NET_MBOX_SIZE_WORDS 8
#define OCTBOOT_NET_MBOX_HOST_STATUS_CHANGE 1
#define OCTBOOT_NET_MBOX_TARGET_STATUS_CHANGE 2
#define OCTBOOT_NET_MBOX_OPCODE_INVALID 0xFF

#define OCTBOOT_NET_MBOX_TIMEOUT_MS 100
#define OCTBOOT_NET_MBOX_WAIT_MS 10
#define OCTBOOT_NET_MBOX_DBELL_ID 0

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

#define THREAD_SLEEP_US 1000
#define VETH_INTERFACE_NAME "veth0"

// kernel -> user space
#define READ_ONCE(x) __atomic_load_n(&(x), __ATOMIC_RELAXED)
#define WRITE_ONCE(x, val) __atomic_store_n(&(x), (val), __ATOMIC_RELAXED)
#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

struct octboot_net_mbox_hdr {
	uint64_t opcode  :8;
	uint64_t id      :8;
	uint64_t req_ack :1;
	uint64_t sizew   :3; /* size in words excluding hdr */
	uint64_t rsvd    :44;
} _packed;

union octboot_net_mbox_msg {
	uint64_t words[OCTBOOT_NET_MBOX_SIZE_WORDS];
	struct {
		 struct octboot_net_mbox_hdr hdr;
		 uint64_t data[7];
	} s;
} __packed;

struct user_queue_lock {
    pthread_mutex_t mutex;
    atomic_flag spinlock;
};

struct octboot_net_sw_descq {
    struct user_queue_lock lock;
    uint32_t mask;
    uint32_t status;
    uint64_t pkts;
	uint64_t bytes;
	uint64_t errors;

    uint32_t local_prod_idx;
    uint32_t local_cons_idx;
    void* vaddr_pktbuf;
    void* iova_pktbuf;

    void* vaddr_prod_idx;
    void* iova_prod_idx;
    void* vaddr_cons_idx;
    void* iova_cons_idx;

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

    uint32_t send_mbox_id;
	uint32_t recv_mbox_id;
} octboot_net_device_t;

typedef struct {
    octboot_net_device_t* mdev;
    int veth_fd;
} thread_params_t;

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

static inline uint32_t octboot_net_circq_add(uint32_t index, uint32_t add,
    uint32_t mask) {
    return (index + add) & mask;
}

static inline uint32_t octboot_net_circq_inc(uint32_t index, uint32_t mask) {
    return octboot_net_circq_add(index, 1, mask);
}

static inline uint32_t octboot_net_circq_depth(uint32_t pi, uint32_t ci,
  uint32_t mask) {
    return (pi - ci) & mask;
}

static inline uint32_t octboot_net_circq_space(uint32_t pi, uint32_t ci,
  uint32_t mask) {
    return mask - octboot_net_circq_depth(pi, ci, mask);
}

int mdev_clean_tx_ring(octboot_net_device_t* mdev) {
    struct octboot_net_sw_descq* tq = &mdev->txq[0];
    tq->mask = 0;
    tq->local_prod_idx = 0;
    tq->local_cons_idx = 0;
    tq->pkts = 0;
    tq->bytes = 0;
    tq->errors = 0;

    int descq_tot_size = sizeof(struct octboot_net_hw_descq) +
        (OCTBOOT_NET_NUM_ELEMENTS * sizeof(struct octboot_net_hw_desc_ptr));
    wmb();
    mmio_memset(tq->hw_descq, 0, descq_tot_size);

    tq->hw_descq = NULL;
    tq->hw_prod_idx = NULL;
    tq->hw_cons_idx = NULL;
    tq->status = OCTBOOT_NET_DESCQ_CLEAN;
    return 0;
}

int mdev_clean_rx_ring(octboot_net_device_t* mdev) {
    struct octboot_net_sw_descq* rq = &mdev->rxq[0];
    rq->mask = 0;
    rq->local_prod_idx = 0;
    rq->local_cons_idx = 0;
    rq->pkts = 0;
    rq->bytes = 0;
    rq->errors = 0;

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
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
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
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
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
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
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
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
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
    rq->local_prod_idx = 0;
    rq->local_cons_idx = 0;
    rq->pkts = 0;
    rq->bytes = 0;
    rq->errors = 0;
    rq->hw_descq = (uint8_t*)RX_DESCQ_OFFSET(mdev);
    rq->hw_prod_idx = (uint32_t *)(rq->hw_descq +
        offsetof(struct octboot_net_hw_descq, prod_idx));
    rq->hw_cons_idx = (uint32_t *)(rq->hw_descq +
        offsetof(struct octboot_net_hw_descq, cons_idx));
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

int mdev_setup_tx_ring(octboot_net_device_t* mdev) {
    struct octboot_net_sw_descq* tq = &mdev->txq[0];
    tq->mask = OCTBOOT_NET_NUM_ELEMENTS - 1;
    tq->local_prod_idx = 0;
    tq->local_cons_idx = 0;
    tq->pkts = 0;
    tq->bytes = 0;
    tq->errors = 0;
    tq->hw_descq = (uint8_t*)TX_DESCQ_OFFSET(mdev);
    tq->hw_prod_idx = (uint32_t *)(tq->hw_descq +
        offsetof(struct octboot_net_hw_descq, prod_idx));
    tq->hw_cons_idx = (uint32_t *)(tq->hw_descq +
        offsetof(struct octboot_net_hw_descq, cons_idx));
    *(uint32_t*)tq->vaddr_cons_idx = 0;

    int descq_tot_size = sizeof(struct octboot_net_hw_descq) + (OCTBOOT_NET_NUM_ELEMENTS *
        sizeof(struct octboot_net_hw_desc_ptr));
    struct octboot_net_hw_descq *descq = calloc(1, descq_tot_size);
    if (!descq) {
        fprintf(stderr, "Failed to allocate descq\n");
        return -1;
    }

    descq->num_entries = OCTBOOT_NET_NUM_ELEMENTS;
    descq->buf_size = OCTBOOT_NET_RX_BUF_SIZE;
    descq->shadow_cons_idx_addr = (uint64_t)mdev->txq[0].iova_cons_idx;

    for (int i = 0; i < OCTBOOT_NET_NUM_ELEMENTS; i++) {
        struct octboot_net_hw_desc_ptr* ptr = &descq->desc_arr[i];
        memset(ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
        ptr->hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
        ptr->ptr = (uint64_t)((uint8_t*)mdev->txq[0].iova_pktbuf + (i * OCTBOOT_NET_RX_BUF_SIZE));
        tq->local_prod_idx = octboot_net_circq_inc(tq->local_prod_idx,
            tq->mask);
        descq->prod_idx = octboot_net_circq_inc(descq->prod_idx, tq->mask);
    }
    tq->status = OCTBOOT_NET_DESCQ_READY;
    queue_lock_init(&tq->lock);

    wmb();
    mmio_memwrite(tq->hw_descq, descq, descq_tot_size);
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

static int mbox_check_msg_rcvd(octboot_net_device_t* mdev,
    union octboot_net_mbox_msg *msg) {
    int i = 0;
    msg->words[0] = readq(TARGET_MBOX_MSG_REG(mdev, 0));
	if (mdev->recv_mbox_id != msg->s.hdr.id) {
		mdev->recv_mbox_id = msg->s.hdr.id;
		for (i = 1; i <= msg->s.hdr.sizew; i++)
			msg->words[i] = readq(TARGET_MBOX_MSG_REG(mdev, i));
		return i;
	}

    return 0;
}

static void octeon_handle_mbox(octboot_net_device_t* mdev) {
    union octboot_net_mbox_msg msg;
    int word_num = mbox_check_msg_rcvd(mdev, &msg);
    if (word_num == 0) {
        return;
    }
}

static void octeon_handle_rxq(octboot_net_device_t* mdev, int sock_fd) {
    struct octboot_net_sw_descq* rq = &mdev->rxq[0];
    if (rq->status != OCTBOOT_NET_DESCQ_READY) {
        return;
    }

	uint32_t hw_cons_idx = READ_ONCE(*(uint32_t*)rq->vaddr_cons_idx);
    uint32_t cons_idx = READ_ONCE(rq->local_cons_idx);
    if (unlikely(cons_idx == 0xFFFFFFFF) || unlikely(hw_cons_idx == 0xFFFFFFFF)) {
        fprintf(stderr, "hw_cons_idx=0x%x cons_idx=0x%x\n", hw_cons_idx, cons_idx);
        return;
	}

    int count = octboot_net_circq_depth(hw_cons_idx,  cons_idx, rq->mask);
    if (count == 0) {
        return;
    }

    int start = cons_idx;
    uint8_t *hw_desc_ptr;
    struct octboot_net_hw_desc_ptr ptr;
    ssize_t bytes_sent;
    for (int i = 0; i < count; i++) {
        hw_desc_ptr = rq->hw_descq + OCTBOOT_NET_DESC_ARR_ENTRY_OFFSET(start);
        mmio_memread(&ptr, hw_desc_ptr, sizeof(struct octboot_net_hw_desc_ptr));
        if (unlikely(ptr.hdr.s_mgmt_net.total_len < ETH_ZLEN ||
            ptr.hdr.s_mgmt_net.total_len > OCTBOOT_NET_RX_BUF_SIZE ||
		    ptr.hdr.s_mgmt_net.is_frag ||
		    ptr.hdr.s_mgmt_net.ptr_len != ptr.hdr.s_mgmt_net.ptr_len)) {
			/* dont handle frags now */
			rq->errors++;
            fprintf(stderr, "rq->error increases to %ld due to frags\n", rq->errors);
        } else {
            rq->pkts += 1;
			rq->bytes += ptr.hdr.s_mgmt_net.total_len;
            uint8_t* pktbuf = (uint8_t*)rq->iova_pktbuf + (start * OCTBOOT_NET_RX_BUF_SIZE);
            if (ptr.ptr != (uint64_t)pktbuf) {
                fprintf(stderr, "ptr.ptr != pktbuf in rxq\n");
                return;
            }
            bytes_sent = send(sock_fd, (uint8_t*)rq->vaddr_pktbuf + (start * OCTBOOT_NET_RX_BUF_SIZE),
                ptr.hdr.s_mgmt_net.total_len, 0);
            if (bytes_sent != ptr.hdr.s_mgmt_net.total_len) {
                fprintf(stderr, "Partial send: %zd of %u bytes\n", bytes_sent, ptr.hdr.s_mgmt_net.total_len);
                return;
            }
        }
        start = octboot_net_circq_inc(start, rq->mask);
    }

    wmb();
	cons_idx = octboot_net_circq_add(cons_idx, count, rq->mask);
    WRITE_ONCE(rq->local_cons_idx, cons_idx);
    wmb();

    // receive all the packets until no increasing of cons_idx
    hw_cons_idx = READ_ONCE(*(uint32_t*)rq->vaddr_cons_idx);
    cons_idx = READ_ONCE(rq->local_cons_idx);
    if (unlikely(cons_idx == 0xFFFFFFFF) || unlikely(hw_cons_idx == 0xFFFFFFFF)) {
        fprintf(stderr, "hw_cons_idx=0x%x cons_idx=0x%x\n", hw_cons_idx, cons_idx);
        return;
	}
    count = octboot_net_circq_depth(hw_cons_idx,  cons_idx, rq->mask);
    if (count > 0) {
        octeon_handle_rxq(mdev, sock_fd);
    }
}

void* octeon_thread_func(void* arg) {
    thread_params_t* params = (thread_params_t*)arg;
    while (1) {
        usleep(THREAD_SLEEP_US);

        octeon_handle_mbox(params->mdev); // not completed yet
        octeon_handle_rxq(params->mdev, params->veth_fd);
    }
    return NULL;
}

static void veth_handle_rawsock(int sock_fd, octboot_net_device_t* mdev) {
    struct octboot_net_sw_descq* tq = &mdev->txq[0];
    if (tq->status != OCTBOOT_NET_DESCQ_READY) {
        return;
    }

    while (1) {

        uint32_t prod_idx = READ_ONCE(tq->local_prod_idx);
        uint32_t cons_idx = READ_ONCE(*(uint32_t*)tq->vaddr_cons_idx);
        
        if (unlikely(prod_idx == 0xFFFFFFFF) || unlikely(cons_idx == 0xFFFFFFFF)) {
            fprintf(stderr, "Invalid indices prod_idx=0x%x cons_idx=0x%x\n", 
                    prod_idx, cons_idx);
            break;
        }

        if (octboot_net_circq_space(prod_idx, cons_idx, tq->mask) == 0) {
            // no send buffer left
            break;
        }

        uint8_t* pkt_buffer = (uint8_t*)tq->vaddr_pktbuf + (prod_idx * OCTBOOT_NET_RX_BUF_SIZE);
        ssize_t recv_len = recv(sock_fd, pkt_buffer, OCTBOOT_NET_RX_BUF_SIZE, MSG_DONTWAIT);
        if (recv_len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "Failed to receive packet: %s\n", strerror(errno));
            }
            break;
        }

        if (recv_len < ETH_ZLEN || recv_len > OCTBOOT_NET_RX_BUF_SIZE) {
            tq->errors++;
            break;
        }

        struct octboot_net_hw_desc_ptr ptr;
        memset(&ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
        ptr.hdr.s_mgmt_net.total_len = recv_len;
        ptr.hdr.s_mgmt_net.ptr_len = recv_len;
        ptr.hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
        ptr.ptr = (uint64_t)((uint8_t*)tq->iova_pktbuf + (prod_idx * OCTBOOT_NET_RX_BUF_SIZE));
        uint8_t* hw_desc_ptr = tq->hw_descq + OCTBOOT_NET_DESC_ARR_ENTRY_OFFSET(prod_idx);
        mmio_memwrite(hw_desc_ptr, &ptr, sizeof(struct octboot_net_hw_desc_ptr));

        wmb();
        prod_idx = octboot_net_circq_inc(prod_idx, tq->mask);
        WRITE_ONCE(tq->local_prod_idx, prod_idx);
        tq->pkts  += 1;
        tq->bytes += recv_len;
    }

	wmb();
    writel(tq->local_prod_idx, tq->hw_prod_idx);
}

void* veth_thread_func(void* arg) {
    thread_params_t* params = (thread_params_t*)arg;
    while (1) {
        usleep(THREAD_SLEEP_US);

        veth_handle_rawsock(params->veth_fd, params->mdev);
    }
    return NULL;
}

int veth_setup_raw_socket(const char* if_name) {
    int sock_fd;
    struct ifreq ifr;
    struct sockaddr_ll sll;
    
    // Create RAW socket
    sock_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sock_fd < 0) {
        fprintf(stderr, "Failed to create raw socket: %s\n", strerror(errno));
        return -1;
    }

    // Get interface index
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1);
    if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "Failed to get interface index: %s\n", strerror(errno));
        close(sock_fd);
        return -1;
    }

    // Bind to interface
    memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex = ifr.ifr_ifindex;
    if (bind(sock_fd, (struct sockaddr*)&sll, sizeof(sll)) < 0) {
        fprintf(stderr, "Failed to bind socket: %s\n", strerror(errno));
        close(sock_fd);
        return -1;
    }

    // Set promiscuous mode
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1);
    if (ioctl(sock_fd, SIOCGIFFLAGS, &ifr) < 0) {
        fprintf(stderr, "Failed to get interface flags: %s\n", strerror(errno));
        close(sock_fd);
        return -1;
    }
    ifr.ifr_flags |= IFF_PROMISC;
    if (ioctl(sock_fd, SIOCSIFFLAGS, &ifr) < 0) {
        fprintf(stderr, "Failed to set promiscuous mode: %s\n", strerror(errno));
        close(sock_fd);
        return -1;
    }

    return sock_fd;
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

    if (mdev_setup_rx_ring(&octbootdev[0]) < 0) {
        fprintf(stderr, "failed to setup rx ring\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        vfio_uninit(&octbootdev[0]);
        return -1;
    }

    if (mdev_setup_tx_ring(&octbootdev[0]) < 0) {
        fprintf(stderr, "failed to setup tx ring\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_clean_tx_ring(&octbootdev[0]);
        vfio_uninit(&octbootdev[0]);
        return -1;
    }

    int veth_fd = veth_setup_raw_socket(VETH_INTERFACE_NAME);
    if (veth_fd < 0) {
        fprintf(stderr, "failed to setup veth\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_clean_tx_ring(&octbootdev[0]);
        vfio_uninit(&octbootdev[0]);
        return -1;
    }

    pthread_t octeon_thread, veth_thread;
    thread_params_t params;
    params.mdev = &octbootdev[0];
    params.veth_fd = -1;
    if (pthread_create(&octeon_thread, NULL, octeon_thread_func, &params) != 0) {
        fprintf(stderr, "failed to create octeon thread\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_clean_tx_ring(&octbootdev[0]);
        vfio_uninit(&octbootdev[0]);
        return -1;
    }
    if (pthread_create(&veth_thread, NULL, veth_thread_func, &params) != 0) {
        fprintf(stderr, "failed to create veth thread\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_clean_tx_ring(&octbootdev[0]);
        vfio_uninit(&octbootdev[0]);
        return -1;
    }

    pthread_join(octeon_thread, NULL);
    pthread_join(veth_thread, NULL);
    return ret;
}
