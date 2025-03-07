#include <linux/vfio.h>
#include <sys/ioctl.h>
#include <sys/mman.h> 
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#define NUM_DEVICES                 3
#define NUM_BARS                    3
#define BAR4                        (NUM_BARS-1)
#define SIGNATURE_OFFSET            0x2000000 /* BAR4 index 8 is at this offset */
#define OCTBOOT_NET_MAXQ            1

#define OCTNET_TX_DESCQ_OFFSET      0x2000400
#define OCTNET_RX_DESCQ_OFFSET      0x2010000
#define TX_DESCQ_OFFSET(mdev)       (mdev->bar_map[BAR4] + OCTNET_TX_DESCQ_OFFSET)
#define RX_DESCQ_OFFSET(mdev)       (mdev->bar_map[BAR4] + OCTNET_RX_DESCQ_OFFSET)


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

struct octboot_net_sw_descq {
    int q_num;
    int local_prod_idx;
    int local_cons_idx;
    int element_count;
    int mask;
    int pending;
    void* cons_idx_shadow;
    void* dma_list;
    void* skb_list;
    void* hw_descq;
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
    int dma_size;
    void* mmap_addr;
    void* iova_addr;

    struct octboot_net_sw_descq rxq[OCTBOOT_NET_MAXQ];
	struct octboot_net_sw_descq txq[OCTBOOT_NET_MAXQ];
} octboot_net_device_t;

int vfio_dma_unmap(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        fprintf(stderr, "invalid parameter of mdev\n");
        return -1;
    }

    struct vfio_iommu_type1_dma_unmap dma_unmap = {
        .argsz = sizeof(dma_unmap),
        .size = mdev->dma_size,
        .iova = (uint64_t)mdev->iova_addr
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap);
    munmap(mdev->mmap_addr, mdev->dma_size);
    mdev->mmap_addr = NULL;
    mdev->iova_addr = NULL;
    mdev->dma_size = 0;
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

int vfio_uninit(octboot_net_device_t* mdev)
{
    if (mdev == NULL) {
        fprintf(stderr, "invalid parameter of mdev\n");
        return -1;
    }

    vfio_dma_unmap(mdev);
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

int vfio_bar_map(octboot_net_device_t* mdev)
{
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

int vfio_dma_map(octboot_net_device_t* mdev) {
    mdev->dma_size = OCTBOOT_NET_RX_BUF_SIZE;
    mdev->mmap_addr = mmap(NULL, mdev->dma_size, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map = {
        .argsz = sizeof(dma_map),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->mmap_addr,
        .size = mdev->dma_size,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map)) {
        fprintf(stderr, "failed to map dma\n");
        return -1;
    }
    mdev->iova_addr = (void*)dma_map.iova;
    return 0;
}

int vfio_init(octboot_net_device_t* mdev)
{
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

    if (vfio_dma_map(mdev) < 0) {
        fprintf(stderr, "failed to map dma\n");
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
        octbootdev[i].mmap_addr = NULL;
        octbootdev[i].iova_addr = NULL;
        octbootdev[i].dma_size = 0;
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
