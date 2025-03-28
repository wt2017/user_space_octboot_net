#define _LARGEFILE64_SOURCE
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
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h> 
#include "desc_queue.h"
#include "mmio_api.h"

//#define VFIO_ENABLED 1

#define NUM_DEVICES                 3
#define NUM_BARS                    3  // BAR0, BAR2, BAR4
#define BAR0                        0
#define BAR2                        1
#define BAR4                        2

#define OCTBOOT_NET_MAXQ            1
#define OCTBOOT_NET_DESCQ_CLEAN 0
#define OCTBOOT_NET_DESCQ_READY 1

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
#define OCTBOOT_NET_RING_BUF_SIZE RECV_BUF_SIZE_1500_MTU
#elif defined(MTU_9600_SETTINGS)
#define OCTBOOT_NET_MAX_MTU 9600
#define DPIX_MAX_PTR DPI_MAX_PTR_9600_MTU
#define OCTBOOT_NET_RING_BUF_SIZE RECV_BUF_SIZE_9600_MTU
#endif

#define OCTBOOT_NET_NUM_ELEMENTS 256

#define INIT_SLEEP_US 1000000
#define THREAD_SLEEP_US 1000
#define OCTBOOT_NET_SERVICE_TASK_US_FLR 6000000 // 6s
#define VETH_INTERFACE_NAME "veth0"

// kernel -> user space
#define READ_ONCE(x) __atomic_load_n(&(x), __ATOMIC_RELAXED)
#define WRITE_ONCE(x, val) __atomic_store_n(&(x), (val), __ATOMIC_RELAXED)
#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

/***********************************     pci_regs.h   ***************************************
#define PCI_VENDOR_ID		0x00
#define PCI_DEVICE_ID		0x02
#define PCI_COMMAND		0x04
#define  PCI_COMMAND_IO		0x1
#define  PCI_COMMAND_MEMORY	0x2
#define  PCI_COMMAND_MASTER	0x4

#define PCI_CAPABILITY_LIST	0x34
#define PCI_EXP_DEVCTL		0x08
********************************************************************************************/
#define PCI_VENDOR_ID		0x00
#define PCI_DEVICE_ID   0x02	/* 16 bits */
#define PCI_COMMAND     0x04	/* 16 bits */
#define PCI_COMMAND_MEMORY 0x2	/* Enable response in Memory space */
#define PCI_COMMAND_MASTER 0x4	/* Enable bus mastering */
#define PCI_REVISION_ID		0x08

#define PCI_CAPABILITY_LIST	                    0x34
#define  PCI_CAP_ID_EXP		                    0x10
#define PCI_EXP_DEVCTL		                    0x08
#define  PCI_EXP_DEVCTL_BCR_FLR                 0x8000
#define PCI_CONF_REG(mdev)                      ((uint8_t*)mdev->conf_map.conf_addr)
#define PCI_CONF_CMD_REG(mdev)                  ((uint8_t*)mdev->conf_map.conf_addr + PCI_COMMAND)

#define VFIO_PCI_CONFIG_REGION_SIZE 256

// hugepage
#define HUGEPAGE_SIZE (1 * 1024 * 1024 * 1024) // 1 GiB
#define HUGEPAGE_PKTBUF_OFFSET (512 * 1024 * 1024) // 100 MiB

// dpdk logic
#define RTE_PCI_BASE_ADDRESS_0	0x10
#define RTE_PCI_BASE_ADDRESS_SPACE_IO	0x01
#define VFIO_CAP_OFFSET(x) (x->cap_offset)
#define RTE_PTR_ADD(ptr, x) ((void*)((uintptr_t)(ptr) + (x)))
#define RTE_PCI_CAPABILITY_LIST	0x34
#define RTE_PCI_CAP_ID_MSIX		0x11
#define RTE_PCI_CAP_SIZEOF		4
#define RTE_PCI_CAP_NEXT		1
#define RTE_PCI_CFG_SPACE_SIZE		256
#define RTE_PCI_CFG_SPACE_EXP_SIZE	4096
#define RTE_PCI_STD_HEADER_SIZEOF	64
#define RTE_PCI_MSIX_FLAGS		2	/* Message Control */
#define RTE_PCI_MSIX_FLAGS_QSIZE	0x07ff	/* Table size */
#define RTE_PCI_MSIX_FLAGS_MASKALL	0x4000	/* Mask all vectors for this function */
#define RTE_PCI_MSIX_FLAGS_ENABLE	0x8000	/* MSI-X enable */
#define RTE_PCI_MSIX_TABLE		4	/* Table offset */
#define RTE_PCI_MSIX_TABLE_BIR		0x00000007 /* BAR index */
#define RTE_PCI_MSIX_TABLE_OFFSET	0xfffffff8 /* Offset into specified BAR */
#define RTE_PCI_COMMAND		0x04
#define RTE_PCI_COMMAND_MEMORY		0x2	/* Enable response in Memory space */
#define RTE_PCI_COMMAND_MASTER		0x4	/* Bus Master Enable */
#define RTE_PCI_COMMAND_INTX_DISABLE	0x400	/* INTx Emulation Disable */

enum rte_map_flags {
	/** Changes to the mapped memory are visible to other processes. */
	RTE_MAP_SHARED = 1 << 0,
	/** Mapping is not backed by a regular file. */
	RTE_MAP_ANONYMOUS = 1 << 1,
	/** Copy-on-write mapping, changes are invisible to other processes. */
	RTE_MAP_PRIVATE = 1 << 2,
	/**
	 * Force mapping to the requested address. This flag should be used
	 * with caution, because to fulfill the request implementation
	 * may remove all other mappings in the requested region. However,
	 * it is not required to do so, thus mapping with this flag may fail.
	 */
	RTE_MAP_FORCE_ADDRESS = 1 << 3
};
enum rte_mem_prot {
	RTE_PROT_READ = 1 << 0,   /**< Read access. */
	RTE_PROT_WRITE = 1 << 1,  /**< Write access. */
	RTE_PROT_EXECUTE = 1 << 2 /**< Code execution. */
};

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

struct octboot_net_sw_descq {
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

    uint8_t* addr_hw_descq;
    uint32_t* addr_hw_prod_idx;
    uint32_t* addr_hw_cons_idx;
};

typedef struct {
    int fd;
    uint64_t phy_bar_addr;
    void* bar_addr;
    uint64_t bar_size;
    uint64_t offset;
    bool iobar;
    __u32 bar_flags;
} bar_map_t;

typedef struct {
    int fd;
    void* conf_addr;
    uint64_t conf_size;
    uint64_t conf_offset;
} conf_map_t;

struct uboot_pcinet_barmap {
	uint64_t signature;
	uint64_t host_version;
	uint64_t host_status_reg;
	uint64_t host_mailbox_ack;
	uint64_t host_mailbox[8];
	uint64_t target_version;
	uint64_t target_status_reg;
	uint64_t target_mailbox_ack;
	uint64_t target_mailbox[8];
	uint64_t rx_descriptor_offset;
	uint64_t tx_descriptor_offset;
};

struct pci_msix_table {
	int bar_index;
	uint32_t offset;
	uint32_t size;
};

typedef struct {
    char vfio_path[32];
    char pci_addr[32];
	int container_fd;
    int group_fd;
    int device_fd;
    bool bar_mapped;
    struct uboot_pcinet_barmap npu_memmap_info;
    bool signature_found;
    int hugepage_fd;
    void* hugepage_addr;
    uint32_t send_mbox_id;
	uint32_t recv_mbox_id;
    conf_map_t conf_map;
    struct pci_msix_table msix_table;
    bar_map_t bar_map[NUM_BARS];
    struct octboot_net_sw_descq rxq[OCTBOOT_NET_MAXQ];
	struct octboot_net_sw_descq txq[OCTBOOT_NET_MAXQ];
} octboot_net_device_t;

typedef struct {
    octboot_net_device_t* mdev;
    int veth_fd;
} thread_params_t;

typedef enum {
    STATUS_SUSPENDED,
    STATUS_RUNNING
} pthread_status_t;

struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    pthread_status_t octeon_status;
    pthread_status_t veth_status;
} controller = {
    PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_COND_INITIALIZER,
    STATUS_RUNNING,
    STATUS_SUSPENDED
};

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
    mmio_memset(tq->addr_hw_descq, 0, descq_tot_size);

    tq->addr_hw_descq = NULL;
    tq->addr_hw_prod_idx = NULL;
    tq->addr_hw_cons_idx = NULL;
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
	mmio_memset(rq->addr_hw_descq, 0, descq_tot_size);

    rq->addr_hw_descq = NULL;
    rq->addr_hw_prod_idx = NULL;
    rq->addr_hw_cons_idx = NULL;
    rq->status = OCTBOOT_NET_DESCQ_CLEAN;
    return 0;
}

int mdev_dma_unmap_pktbuf(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

#ifdef VFIO_ENABLED
    struct vfio_iommu_type1_dma_unmap dma_unmap1 = {
        .argsz = sizeof(dma_unmap1),
        .size = OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS,
        .iova = (uint64_t)mdev->rxq[0].iova_pktbuf
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap1);
    munmap(mdev->rxq[0].vaddr_pktbuf, OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS);
    mdev->rxq[0].vaddr_pktbuf = NULL;
    mdev->rxq[0].iova_pktbuf = NULL;

    struct vfio_iommu_type1_dma_unmap dma_unmap2 = {
        .argsz = sizeof(dma_unmap2),
        .size = OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS,
        .iova = (uint64_t)mdev->txq[0].iova_pktbuf
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap2);
    munmap(mdev->txq[0].vaddr_pktbuf, OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS);
    mdev->txq[0].vaddr_pktbuf = NULL;
    mdev->txq[0].iova_pktbuf = NULL;
#else
    mdev->rxq[0].vaddr_pktbuf = NULL;
    mdev->rxq[0].iova_pktbuf = NULL;
    mdev->txq[0].vaddr_pktbuf = NULL;
    mdev->txq[0].iova_pktbuf = NULL;
#endif
    return 0;
}

int mdev_dma_unmap_circq(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

#ifdef VFIO_ENABLED
    struct vfio_iommu_type1_dma_unmap dma_unmap1 = {
        .argsz = sizeof(dma_unmap1),
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
        .iova = (uint64_t)mdev->rxq[0].iova_prod_idx
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap1);
    munmap(mdev->rxq[0].vaddr_prod_idx, sizeof(uint32_t) * OCTBOOT_NET_MAXQ);
    mdev->rxq[0].vaddr_prod_idx = NULL;
    mdev->rxq[0].iova_prod_idx = NULL;

    struct vfio_iommu_type1_dma_unmap dma_unmap2 = {
        .argsz = sizeof(dma_unmap2),
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
        .iova = (uint64_t)mdev->rxq[0].iova_cons_idx
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap2);
    munmap(mdev->rxq[0].vaddr_cons_idx, sizeof(uint32_t) * OCTBOOT_NET_MAXQ);
    mdev->rxq[0].vaddr_cons_idx = NULL;
    mdev->rxq[0].iova_cons_idx = NULL;

    struct vfio_iommu_type1_dma_unmap dma_unmap3 = {
        .argsz = sizeof(dma_unmap3),
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
        .iova = (uint64_t)mdev->txq[0].iova_prod_idx
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap3);
    munmap(mdev->txq[0].vaddr_prod_idx, sizeof(uint32_t) * OCTBOOT_NET_MAXQ);
    mdev->txq[0].vaddr_prod_idx = NULL;
    mdev->txq[0].iova_prod_idx = NULL;

    struct vfio_iommu_type1_dma_unmap dma_unmap4 = {
        .argsz = sizeof(dma_unmap4),
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
        .iova = (uint64_t)mdev->txq[0].iova_cons_idx
    };
    ioctl(mdev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap4);
    munmap(mdev->txq[0].vaddr_cons_idx, sizeof(uint32_t) * OCTBOOT_NET_MAXQ);
    mdev->txq[0].vaddr_cons_idx = NULL;
    mdev->txq[0].iova_cons_idx = NULL;
#else
    mdev->rxq[0].vaddr_prod_idx = NULL;
    mdev->rxq[0].iova_prod_idx = NULL;
    mdev->rxq[0].vaddr_cons_idx = NULL;
    mdev->rxq[0].iova_cons_idx = NULL;
    mdev->txq[0].vaddr_prod_idx = NULL;
    mdev->txq[0].iova_prod_idx = NULL;
    mdev->txq[0].vaddr_cons_idx = NULL;
    mdev->txq[0].iova_cons_idx = NULL;
#endif
    return 0;
}

int mdev_bar_unmap(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

    for (int i = 0; i < NUM_BARS; i++) {
        if (mdev->bar_map[i].bar_addr != NULL && mdev->bar_map[i].bar_addr != MAP_FAILED) {
            munmap(mdev->bar_map[i].bar_addr, mdev->bar_map[i].bar_size);
            mdev->bar_map[i].bar_addr = NULL;
            mdev->bar_map[i].bar_size = 0;
        }

        if (mdev->bar_map[i].fd > 0) {
            close(mdev->bar_map[i].fd);
            mdev->bar_map[i].fd = -1;
        }
    }

    return 0;
}

int mdev_conf_unmap(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

    if (mdev->conf_map.conf_addr != NULL && mdev->conf_map.conf_addr != MAP_FAILED) {
        munmap(mdev->conf_map.conf_addr, mdev->conf_map.conf_size);
        mdev->conf_map.conf_addr = NULL;
        mdev->conf_map.conf_size = 0;
    }

    if (mdev->conf_map.fd > 0) {
        close(mdev->conf_map.fd);
        mdev->conf_map.fd = -1;
    }

    return 0;
}

int mdev_dma_uninit(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

    mdev_dma_unmap_pktbuf(mdev);
    mdev_dma_unmap_circq(mdev);
    return 0;
}

int mdev_mm_uninit(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

    mdev_bar_unmap(mdev);
    mdev_conf_unmap(mdev);

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

int mdev_get_phy_addr(octboot_net_device_t* mdev) {    
    char path[256];
    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/config", mdev->pci_addr);
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        printf("failed to open device config file:%s\n", path);
        return -1;
    }

    for (int i=0; i<NUM_BARS; i++) {
        uint32_t bar_low, bar_high;
        uint64_t bar_address;
        int offset = 0x10 + i * 4;
        if (pread(fd, &bar_low, sizeof(bar_low), offset) != sizeof(bar_low)) {
            printf("failed to read BAR low");
            close(fd);
            return -1;
        }

        if ((bar_low & 0x7) == 0x4) {
            // 64-bit BAR
            if (pread(fd, &bar_high, sizeof(bar_high), offset+4) != sizeof(bar_high)) {
                printf("failed to read BAR high");
                close(fd);
                return -1;
            }
            bar_address = ((uint64_t)bar_high << 32) | (bar_low & ~0xFULL);
        } else {
            // 32-bit BAR
            bar_address = bar_low & ~0xFULL;
        }
        mdev->bar_map[i].phy_bar_addr = bar_address;
        printf("BAR%d phy addr=0x%lx\n", i, mdev->bar_map[i].phy_bar_addr);
    }

    close(fd);
    return 0;
}

int mdev_conf_map(octboot_net_device_t* mdev) {
#ifdef VFIO_ENABLED
    struct vfio_region_info reg = {
        .argsz = sizeof(reg),
        .index = VFIO_PCI_CONFIG_REGION_INDEX
    };
    if (ioctl(mdev->device_fd, VFIO_DEVICE_GET_REGION_INFO, &reg)) {
        printf("failed to get conf region info\n");
        mdev_mm_uninit(mdev);
        return -1;
    }
    printf("conf region info, offset=0x%llx, size=%llu\n", reg.offset, reg.size);

    mdev->conf_map.conf_size = reg.size;
    mdev->conf_map.conf_offset = reg.offset;
#else
    // pread/pwrite conf region directly, no need to mmap it
    printf("nothing to do for physical conf region map mdev=%p\n", (void*)mdev);
#endif
    return 0;
}

bool pci_vfio_is_ioport_bar(octboot_net_device_t* mdev,
	int bar_index)
{
	uint32_t ioport_bar;
	int ret = pread(mdev->device_fd, &ioport_bar, sizeof(ioport_bar),
			  mdev->conf_map.conf_offset + RTE_PCI_BASE_ADDRESS_0 + bar_index * 4);
	if (ret != sizeof(ioport_bar)) {
		printf("Cannot read command (%x) from config space!",
			RTE_PCI_BASE_ADDRESS_0 + bar_index*4);
		return false;
	}

    if ((ioport_bar & RTE_PCI_BASE_ADDRESS_SPACE_IO) != 0) {
        printf("BAR%d is IO port\n", bar_index);
        return true;
    }

	return false;
}

int
pci_vfio_read_config(octboot_net_device_t* mdev,
		    void *buf, size_t len, off_t offs)
{
	if ((uint64_t)len + offs > mdev->conf_map.conf_size)
		return -1;

	return pread(mdev->device_fd, buf, len, mdev->conf_map.conf_offset + offs);
}

off_t
pci_find_next_capability(octboot_net_device_t* mdev, uint8_t cap,
	off_t offset)
{
	uint8_t pos;
	int ttl;

	if (offset == 0)
		offset = RTE_PCI_CAPABILITY_LIST;
	else
		offset += RTE_PCI_CAP_NEXT;
	ttl = (RTE_PCI_CFG_SPACE_SIZE - RTE_PCI_STD_HEADER_SIZEOF) / RTE_PCI_CAP_SIZEOF;

	if (pci_vfio_read_config(mdev, &pos, sizeof(pos), offset) < 0)
		return -1;

	while (pos && ttl--) {
		uint16_t ent;
		uint8_t id;

		offset = pos;
		if (pci_vfio_read_config(mdev, &ent, sizeof(ent), offset) < 0)
			return -1;

		id = ent & 0xff;
		if (id == 0xff)
			break;

		if (id == cap)
			return offset;

		pos = (ent >> 8);
	}

	return 0;
}

off_t
pci_find_capability(octboot_net_device_t* mdev, uint8_t cap)
{
	return pci_find_next_capability(mdev, cap, 0);
}

static struct vfio_info_cap_header *
pci_vfio_info_cap(struct vfio_region_info* info, int cap)
{
	struct vfio_info_cap_header *h;
	size_t offset;

	if ((info->flags & VFIO_REGION_INFO_FLAG_CAPS) == 0) {
		printf("VFIO info does not advertise capabilities\n");
		return NULL;
	}

	offset = VFIO_CAP_OFFSET(info);
	while (offset != 0) {
		h = RTE_PTR_ADD(info, offset);
		if (h->id == cap)
			return h;
		offset = h->next;
	}
	return NULL;
}

static int
pci_vfio_get_msix_bar(octboot_net_device_t* mdev,
	struct pci_msix_table* msix_table)
{
	off_t cap_offset = pci_find_capability(mdev, RTE_PCI_CAP_ID_MSIX);
	if (cap_offset < 0)
		return -1;

	if (cap_offset != 0) {
		uint16_t flags;
		uint32_t reg;

		if (pci_vfio_read_config(mdev, &reg, sizeof(reg), cap_offset +
				RTE_PCI_MSIX_TABLE) < 0) {
			printf("Cannot read MSIX table from PCI config space!");
			return -1;
		}

		if (pci_vfio_read_config(mdev, &flags, sizeof(flags), cap_offset +
				RTE_PCI_MSIX_FLAGS) < 0) {
			printf("Cannot read MSIX flags from PCI config space!");
			return -1;
		}

		msix_table->bar_index = reg & RTE_PCI_MSIX_TABLE_BIR;
		msix_table->offset = reg & RTE_PCI_MSIX_TABLE_OFFSET;
		msix_table->size = 16 * (1 + (flags & RTE_PCI_MSIX_FLAGS_QSIZE));
        printf("msix_table: bar_index=%d, offset=0x%x, size=%d\n",
            msix_table->bar_index, msix_table->offset, msix_table->size);
	}

	return 0;
}

static void* remote_map(const char *filename, int fd,
			uint64_t physical_address,
			uint64_t length)
{
	printf("0x%llx, 0x%zx", (unsigned long long)physical_address, length);
	void *result;
	uint64_t pagesize = sysconf(_SC_PAGESIZE);
	int file_handle = filename ? open(filename, O_RDWR) : fd;
	if (file_handle < 0)
	{
		printf("open failed\n");
		return NULL;
	}

	/* Align the size and address to the page boundary. */
	uint64_t offset = physical_address & (pagesize - 1);
	uint64_t alength = (length + offset + pagesize - 1) & ~(pagesize - 1);

	result = mmap64(NULL, alength, PROT_READ|PROT_WRITE, MAP_SHARED, file_handle,
			physical_address - offset);
	if (result == MAP_FAILED)
	{
        printf("mmap64 failed: %s\n", strerror(errno));
		/* only close file IF we opened it */
		if (filename)
			close(file_handle);
		return NULL;
	}

	/* only close file IF we opened it */
	if (filename)
		close(file_handle);
	printf("remote_map:%p", result + offset);
	return result + offset;
}

int mdev_bar_map(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }
#ifdef VFIO_ENABLED
    for (int i = 0; i < NUM_BARS; i++) {

        struct vfio_region_info reg = {
            .argsz = sizeof(reg),
            .index = VFIO_PCI_BAR0_REGION_INDEX + (i * 2) // BAR0, BAR2, BAR4
        };
        if (ioctl(mdev->device_fd, VFIO_DEVICE_GET_REGION_INFO, &reg)) {
            printf("failed to get bar region info\n");
            mdev_mm_uninit(mdev);
            return -1;
        }

        bool iobar = pci_vfio_is_ioport_bar(mdev, i*2);
        printf("bar%d region info, offset=0x%llx, size=%llu, iobar=%d, flags=0x%x\n", 
            i*2, reg.offset, reg.size, iobar, reg.flags);

        mdev->bar_map[i].bar_size = reg.size;
        mdev->bar_map[i].offset = reg.offset;
        mdev->bar_map[i].iobar = iobar;
        mdev->bar_map[i].bar_flags = reg.flags;

        bool mappable = (reg.flags & VFIO_REGION_INFO_FLAG_MMAP) != 0;
        if (!mappable) {
            printf("bar%d is not mappable\n", i*2);
            continue;
        }

        void* mapped_addr = remote_map(NULL, mdev->device_fd, reg.offset, reg.size);
        mdev->bar_map[i].bar_addr = (void*)((uint64_t)mapped_addr & 0xfffffffffffffff0ull);
        if (mdev->bar_map[i].bar_addr == NULL) {
            printf("failed to map bar %d\n", i*2);
            mdev_mm_uninit(mdev);
            return -1;
        }
        mdev->bar_mapped = true;
        printf("bar%d bar_addr=%p\n", i*2, mdev->bar_map[i].bar_addr);
    }
#else
    char path[256];
    for (int i=0; i<NUM_BARS; i++) {
        snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource%d", mdev->pci_addr, i*2);
        mdev->bar_map[i].fd = open(path, O_RDWR | O_SYNC);
        if (mdev->bar_map[i].fd < 0) {
            printf("failed to open device config file:%s\n", path);
            return -1;
        }

        switch (i) {
        case BAR0:
            mdev->bar_map[i].bar_size = 0x800000;    // 8MB for BAR0
            break;
        case BAR2:
            mdev->bar_map[i].bar_size = 0x10000000;  // 256MB for BAR2
            break;
        case BAR4:
            mdev->bar_map[i].bar_size = 0x4000000;   // 64MB for BAR4
            break;
        default:
            printf("invalid bar index, should never be here\n");
            return -1;
        }

        off_t target = 0;
        off_t target_base = target & ~(sysconf(_SC_PAGE_SIZE)-1);
        mdev->bar_map[i].bar_size = target + mdev->bar_map[i].bar_size - target_base;
        printf("mmap(%d, %ld, 0x%x, 0x%x, %d, 0x%x)\n", 0, mdev->bar_map[i].bar_size, PROT_READ | PROT_WRITE, MAP_SHARED, mdev->bar_map[i].fd, (int) target);
        mdev->bar_map[i].bar_addr = mmap(0, mdev->bar_map[i].bar_size, PROT_READ | PROT_WRITE, MAP_SHARED, mdev->bar_map[i].fd, target);
        printf("bar%d addr=%p\n", i*2, mdev->bar_map[i].bar_addr);
    }
#endif
    return 0;
}

uint64_t virt_to_phys(uint64_t vaddr) {
    int fd = open("/proc/self/pagemap", O_RDONLY);
    uint64_t paddr = 0;
    uint64_t index = (vaddr / sysconf(_SC_PAGE_SIZE)) * sizeof(uint64_t);
    if (pread(fd, &paddr, sizeof(paddr), index) == sizeof(paddr)) {
        // Check if the page is present
        if (!(paddr & (1ULL << 63))) {
            fprintf(stderr, "Page not present\n");
            return 0;
        }
        paddr &= 0x7fffffffffffff;
        paddr = paddr * sysconf(_SC_PAGE_SIZE) + (vaddr % sysconf(_SC_PAGE_SIZE));
    }
    close(fd);
    return paddr;
}

int mdev_dma_map_circq(octboot_net_device_t* mdev) {

#ifdef VFIO_ENABLED
    mdev->rxq[0].vaddr_prod_idx = mmap(NULL, sizeof(uint32_t) * OCTBOOT_NET_MAXQ, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map1 = {
        .argsz = sizeof(dma_map1),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->rxq[0].vaddr_prod_idx,
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map1)) {
        printf("failed to map dma\n");
        return -1;
    }
    mdev->rxq[0].iova_prod_idx = (void*)dma_map1.iova;

    mdev->rxq[0].vaddr_cons_idx = mmap(NULL, sizeof(uint32_t) * OCTBOOT_NET_MAXQ, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map2 = {
        .argsz = sizeof(dma_map2),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->rxq[0].vaddr_cons_idx,
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map2)) {
        printf("failed to map dma\n");
        return -1;
    }
    mdev->rxq[0].iova_cons_idx = (void*)dma_map2.iova;

    mdev->txq[0].vaddr_prod_idx = mmap(NULL, sizeof(uint32_t) * OCTBOOT_NET_MAXQ, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map3 = {
        .argsz = sizeof(dma_map3),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->txq[0].vaddr_prod_idx,
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map3)) {
        printf("failed to map dma\n");
        return -1;
    }
    mdev->txq[0].iova_prod_idx = (void*)dma_map3.iova;

    mdev->txq[0].vaddr_cons_idx = mmap(NULL, sizeof(uint32_t) * OCTBOOT_NET_MAXQ, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map4 = {
        .argsz = sizeof(dma_map4),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->txq[0].vaddr_cons_idx,
        .size = sizeof(uint32_t) * OCTBOOT_NET_MAXQ,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map4)) {
        printf("failed to map dma\n");
        return -1;
    }
    mdev->txq[0].iova_cons_idx = (void*)dma_map4.iova;
#else
    mdev->rxq[0].vaddr_prod_idx = mdev->hugepage_addr;
    mdev->rxq[0].iova_prod_idx = (void*)virt_to_phys((uint64_t)mdev->hugepage_addr);
    mdev->rxq[0].vaddr_cons_idx = mdev->hugepage_addr + sizeof(uint64_t) * OCTBOOT_NET_MAXQ;
    mdev->rxq[0].iova_cons_idx = (void*)virt_to_phys((uint64_t)mdev->hugepage_addr + sizeof(uint64_t) * OCTBOOT_NET_MAXQ);
    mdev->txq[0].vaddr_prod_idx = mdev->hugepage_addr + sizeof(uint64_t) * OCTBOOT_NET_MAXQ * 2;
    mdev->txq[0].iova_prod_idx = (void*)virt_to_phys((uint64_t)mdev->hugepage_addr + sizeof(uint64_t) * OCTBOOT_NET_MAXQ * 2);
    mdev->txq[0].vaddr_cons_idx = mdev->hugepage_addr + sizeof(uint64_t) * OCTBOOT_NET_MAXQ * 3;
    mdev->txq[0].iova_cons_idx = (void*)virt_to_phys((uint64_t)mdev->hugepage_addr + sizeof(uint64_t) * OCTBOOT_NET_MAXQ * 3);
    printf("rxq[0].vaddr_prod_idx=%p, rxq[0].iova_prod_idx=%p\n", mdev->rxq[0].vaddr_prod_idx, mdev->rxq[0].iova_prod_idx);
    printf("rxq[0].vaddr_cons_idx=%p, rxq[0].iova_cons_idx=%p\n", mdev->rxq[0].vaddr_cons_idx, mdev->rxq[0].iova_cons_idx);
    printf("txq[0].vaddr_prod_idx=%p, txq[0].iova_prod_idx=%p\n", mdev->txq[0].vaddr_prod_idx, mdev->txq[0].iova_prod_idx);
    printf("txq[0].vaddr_cons_idx=%p, txq[0].iova_cons_idx=%p\n", mdev->txq[0].vaddr_cons_idx, mdev->txq[0].iova_cons_idx);
#endif

    return 0;
}

int mdev_dma_map_pktbuf(octboot_net_device_t* mdev) {

#ifdef VFIO_ENABLED
    mdev->rxq[0].vaddr_pktbuf = mmap(NULL, OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map1 = {
        .argsz = sizeof(dma_map1),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->rxq[0].vaddr_pktbuf,
        .size = OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map1)) {
        printf("failed to map dma pktbuf\n");
        return -1;
    }
    mdev->rxq[0].iova_pktbuf = (void*)dma_map1.iova;

    mdev->txq[0].vaddr_pktbuf = mmap(NULL, OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct vfio_iommu_type1_dma_map dma_map2 = {
        .argsz = sizeof(dma_map2),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (__u64)mdev->txq[0].vaddr_pktbuf,
        .size = OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS,
        .iova = 0
    };
    if (ioctl(mdev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map2)) {
        printf("failed to map dma pktbuf\n");
        return -1;
    }
    mdev->txq[0].iova_pktbuf = (void*)dma_map2.iova;
#else
    mdev->rxq[0].vaddr_pktbuf = mdev->hugepage_addr + HUGEPAGE_PKTBUF_OFFSET;
    mdev->rxq[0].iova_pktbuf = (void*)virt_to_phys((uint64_t)mdev->hugepage_addr + HUGEPAGE_PKTBUF_OFFSET);
    mdev->txq[0].vaddr_pktbuf = mdev->hugepage_addr + HUGEPAGE_PKTBUF_OFFSET + OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS;
    mdev->txq[0].iova_pktbuf = (void*)virt_to_phys((uint64_t)mdev->hugepage_addr + HUGEPAGE_PKTBUF_OFFSET + OCTBOOT_NET_RING_BUF_SIZE * OCTBOOT_NET_NUM_ELEMENTS);
    printf("rxq[0].vaddr_pktbuf=%p, rxq[0].iova_pktbuf=%p\n", mdev->rxq[0].vaddr_pktbuf, mdev->rxq[0].iova_pktbuf);
    printf("txq[0].vaddr_pktbuf=%p, txq[0].iova_pktbuf=%p\n", mdev->txq[0].vaddr_pktbuf, mdev->txq[0].iova_pktbuf);
#endif
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
    rq->addr_hw_descq = (uint8_t*)RX_DESCQ_OFFSET(mdev);
    rq->addr_hw_prod_idx = (uint32_t *)(rq->addr_hw_descq +
        offsetof(struct octboot_net_hw_descq, prod_idx));
    rq->addr_hw_cons_idx = (uint32_t *)(rq->addr_hw_descq +
        offsetof(struct octboot_net_hw_descq, cons_idx));
    *(uint32_t*)rq->vaddr_cons_idx = 0;

    int descq_tot_size = sizeof(struct octboot_net_hw_descq) + (OCTBOOT_NET_NUM_ELEMENTS *
		sizeof(struct octboot_net_hw_desc_ptr));
    struct octboot_net_hw_descq *descq = calloc(1, descq_tot_size);
    if (!descq) {
        printf("Failed to allocate descq\n");
        return -1;
    }

    descq->num_entries = OCTBOOT_NET_NUM_ELEMENTS;
    descq->buf_size = OCTBOOT_NET_RING_BUF_SIZE;
    descq->shadow_cons_idx_addr = (uint64_t)mdev->rxq[0].iova_cons_idx;

    for (int i = 0; i < OCTBOOT_NET_NUM_ELEMENTS; i++) {
        struct octboot_net_hw_desc_ptr* ptr = &descq->desc_arr[i];
        memset(ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
        ptr->hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
        ptr->ptr = (uint64_t)((uint8_t*)mdev->rxq[0].iova_pktbuf + (i * OCTBOOT_NET_RING_BUF_SIZE));
        rq->local_prod_idx = octboot_net_circq_inc(rq->local_prod_idx,
            rq->mask);
        descq->prod_idx = octboot_net_circq_inc(descq->prod_idx, rq->mask);
    }
    rq->status = OCTBOOT_NET_DESCQ_READY;

    wmb();
    mmio_memwrite(rq->addr_hw_descq, descq, descq_tot_size);
    free(descq);
    printf("rxq setup done successfully\n");
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
    tq->addr_hw_descq = (uint8_t*)TX_DESCQ_OFFSET(mdev);
    tq->addr_hw_prod_idx = (uint32_t *)(tq->addr_hw_descq +
        offsetof(struct octboot_net_hw_descq, prod_idx));
    tq->addr_hw_cons_idx = (uint32_t *)(tq->addr_hw_descq +
        offsetof(struct octboot_net_hw_descq, cons_idx));
    *(uint32_t*)tq->vaddr_cons_idx = 0;

    int descq_tot_size = sizeof(struct octboot_net_hw_descq) + (OCTBOOT_NET_NUM_ELEMENTS *
        sizeof(struct octboot_net_hw_desc_ptr));
    struct octboot_net_hw_descq *descq = calloc(1, descq_tot_size);
    if (!descq) {
        printf("Failed to allocate descq\n");
        return -1;
    }

    descq->num_entries = OCTBOOT_NET_NUM_ELEMENTS;
    descq->buf_size = OCTBOOT_NET_RING_BUF_SIZE;
    descq->shadow_cons_idx_addr = (uint64_t)mdev->txq[0].iova_cons_idx;

    for (int i = 0; i < OCTBOOT_NET_NUM_ELEMENTS; i++) {
        struct octboot_net_hw_desc_ptr* ptr = &descq->desc_arr[i];
        memset(ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
        ptr->hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
        ptr->ptr = (uint64_t)((uint8_t*)mdev->txq[0].iova_pktbuf + (i * OCTBOOT_NET_RING_BUF_SIZE));
        tq->local_prod_idx = octboot_net_circq_inc(tq->local_prod_idx,
            tq->mask);
        descq->prod_idx = octboot_net_circq_inc(descq->prod_idx, tq->mask);
    }
    tq->status = OCTBOOT_NET_DESCQ_READY;

    wmb();
    mmio_memwrite(tq->addr_hw_descq, descq, descq_tot_size);
    free(descq);
    printf("txq setup done successfully\n");
    return 0;
}

int mdev_hugepage_alloc(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

    mdev->hugepage_fd = open("/dev/hugepages/hugepagefile", O_CREAT | O_RDWR, 0755);
    if (mdev->hugepage_fd < 0) {
        printf("failed to open hugepagefile\n");
        return -1;
    }
    printf("hugepage_fd = %d\n", mdev->hugepage_fd);

    mdev->hugepage_addr = mmap(NULL, HUGEPAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mdev->hugepage_fd, 0);
    if (mdev->hugepage_addr == MAP_FAILED) {
        printf("failed to mmap hugepage\n");
        return -1;
    }
    printf("hugepage_addr = %p\n", mdev->hugepage_addr);

    // Touch the memory to ensure the page is present
    void* addr = mdev->hugepage_addr;
    for (size_t i = 0; i < HUGEPAGE_SIZE; i += sysconf(_SC_PAGE_SIZE)) {
        *((volatile char *)addr + i) = 0;
    }

    return 0;
}

int mdev_dma_init(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

    if (mdev_dma_map_circq(mdev) < 0) {
        printf("failed to map dma\n");
        mdev_dma_uninit(mdev);
        return -1;
    }

    if (mdev_dma_map_pktbuf(mdev) < 0) {
        printf("failed to map dma pktbuf\n");
        mdev_dma_uninit(mdev);
        return -1;
    }

    return 0;
}

static int vfio_read(int fd, void *buf, ssize_t len, off_t off)
{
	if (pread(fd, buf, len, off) != len) {
		printf("pread(off=%#lx len=%lu)", off, len);
		return -1;
	}

	return 0;
}

static int vfio_write(int fd, void *buf, ssize_t len, off_t off)
{
	if (pwrite(fd, buf, len, off) != len) {
		printf("pwrite(off=%#lx len=%lu)", off, len);
		return -1;
	}

	return 0;
}

static int
pci_enable_bus_memory(octboot_net_device_t* mdev)
{
    int fd = -1;
    off_t offset = 0;
#ifdef VFIO_ENABLED
    fd = mdev->device_fd;
    offset = mdev->conf_map.conf_offset + PCI_COMMAND;
#else
    char path[256];
    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/config", mdev->pci_addr);
    fd = open(path, O_RDWR | O_SYNC);
    if (fd < 0) {
        printf("failed to open device config file:%s\n", path);
        return -1;
    }
    offset = PCI_COMMAND;
#endif
	uint16_t cmd;
    int len;
    len = pread(fd, &cmd, sizeof(cmd), offset);
    printf("pread fd[%d] cmd[0x%x] offset[0x%lx] len[%d]\n", fd, cmd, offset, len);

    cmd |= PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
    len = pwrite(fd, &cmd, sizeof(cmd), offset);
    printf("pread fd[%d] cmd[0x%x] offset[0x%lx] len[%d]\n", fd, cmd, offset, len);

    len = pread(fd, &cmd, sizeof(cmd), offset);
    printf("pread fd[%d] cmd[0x%x] offset[0x%lx] len[%d]\n", fd, cmd, offset, len);

#ifdef VFIO_ENABLED
    // nothing to do
#else
    if (fd)
        close(fd);
#endif
	return 0;
}

int mdev_mm_init(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }
#ifdef VFIO_ENABLED
    mdev->container_fd = open("/dev/vfio/vfio", O_RDWR);
    if (mdev->container_fd < 0) {
        printf("failed to open vfio container\n");
        mdev_mm_uninit(mdev);
        return -1;
    }

    if (ioctl(mdev->container_fd, VFIO_GET_API_VERSION) != VFIO_API_VERSION) {
        printf("VFIO API version mismatch\n");
        mdev_mm_uninit(mdev);
        return -1;
    }

    mdev->group_fd = open(mdev->vfio_path, O_RDWR);
    if (mdev->group_fd < 0) {
        printf("failed to open vfio group\n");
        mdev_mm_uninit(mdev);
        return -1;
    }

    struct vfio_group_status group_status = {.argsz = sizeof(group_status)};
    if (ioctl(mdev->group_fd, VFIO_GROUP_GET_STATUS, &group_status) < 0) {
        printf("Failed to get VFIO group status\n");
        mdev_mm_uninit(mdev);
        return -1;
    }

    if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
        printf("VFIO group is not viable (all devices bound to vfio-pci)\n");
        mdev_mm_uninit(mdev);
        return -1;
    }

    if (ioctl(mdev->group_fd, VFIO_GROUP_SET_CONTAINER, &mdev->container_fd)) {
        printf("failed to bind vfio group to vfio container\n");
        mdev_mm_uninit(mdev);
        return -1;
    }

    if (ioctl(mdev->container_fd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) <= 0) {
        printf("VFIO Type1 IOMMU not supported\n"); 
        mdev_mm_uninit(mdev);
        return -1;
    }

    if (ioctl(mdev->container_fd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU)) {
        printf("failed to set iommu type\n");
        mdev_mm_uninit(mdev);
        return -1;
    }
#if 0
    struct vfio_iommu_type1_info iommu_info;
    if (ioctl(mdev->container_fd, VFIO_IOMMU_GET_INFO, &iommu_info)) {
        printf("failed to get iommu info\n");
        mdev_mm_uninit(mdev);
        return -1;
    }
#endif
    mdev->device_fd = ioctl(mdev->group_fd, VFIO_GROUP_GET_DEVICE_FD, mdev->pci_addr);
    if (mdev->device_fd < 0) {
        printf("failed to get device fd\n");
        mdev_mm_uninit(mdev);
        return -1;
    }
    printf("device_fd=0x%x\n", mdev->device_fd);
#if 0
    struct vfio_device_info device_info;
    if (ioctl(mdev->device_fd, VFIO_DEVICE_GET_INFO, &device_info)) {
        printf("failed to get device info\n");
        mdev_mm_uninit(mdev);
        return -1;
    }
#endif
#endif
    if (mdev_conf_map(mdev) < 0) {
        printf("failed to map conf\n");
        mdev_mm_uninit(mdev);
        return -1;
    }

    if (mdev_bar_map(mdev) < 0) {
        printf("failed to map bars\n");
        mdev_mm_uninit(mdev);
        return -1;
    }

    if (pci_enable_bus_memory(mdev)) {
        printf("failed to enable bus memory\n");
        return -1;
    }
    return 0;
}

void dump_packet(const uint8_t* pktbuf, size_t N) {
    if (pktbuf == NULL || N == 0) {
        printf("Invalid packet buffer or length.\n");
        return;
    }

    printf("Packet Dump Start (Length: %zu bytes):\n", N);
    printf("Human readable format:\n");
    for (size_t i = 0; i < N; i++) {
        // Print the byte in hexadecimal format
        printf("%02X ", pktbuf[i]);

        // Add a newline every 16 bytes for readability
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }

    // Add a final newline if the last line wasn't complete
    if (N % 16 != 0) {
        printf("\n");
    }

    printf("Hex as input of .pcap translation:\n");
    for (size_t i = 0; i < N; i++) {
        // Print the byte in hexadecimal format
        printf("%02X", pktbuf[i]);
    }
    printf("\n");
    printf("Packet Dump End (Length: %zu bytes):\n", N);
}

unsigned char dhcp_offer_msg[] = {
    // Ethernet header（14 bytes）
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 目标 MAC（广播）
    0xEE, 0x41, 0x7B, 0xDD, 0x88, 0x2D, // 源 MAC（服务器 MAC）
    0x08, 0x00,                         // 以太网类型（IPv4）

    // IP header（20 bytes）
    0x45, 0x00, 0x01, 0x48,             // 版本/头长、服务类型、总长度（328 字节）
    0x00, 0x00, 0x40, 0x00,             // 标识、标志/分片偏移
    0x40, 0x11, 0x00, 0x00,             // TTL=64、协议=UDP、头部校验和（需自动计算）
    0xC0, 0xA8, 0xFD, 0x2A,             // 源 IP（192.168.253.42）
    0xFF, 0xFF, 0xFF, 0xFF,             // 目标 IP（255.255.255.255，广播）

    // UDP header（8 bytes）
    0x00, 0x43, 0x00, 0x44,             // 源端口 67（bootps）、目标端口 68（bootpc）
    0x01, 0x34, 0x00, 0x00,             // UDP 长度（308 字节）、校验和（需自动计算）

    // DHCP payload
    0x02, 0x01, 0x06, 0x00, 0x7B, 0xE4, 0x53, 0x1F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0xA8, 0xFD, 0x20, 0xC0, 0xA8, 0xFD, 0x2A,
    0x00, 0x00, 0x00, 0x00, 0xEE, 0x41, 0x7B, 0xDD,
    0x88, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x82, 0x53,
    0x63, 0x35, 0x01, 0x02, 0x36, 0x04, 0xC0, 0xA8,
    0xFD, 0x2A, 0x01, 0x04, 0xFF, 0xFF, 0xFF, 0x00,
    0x33, 0x04, 0x00, 0x01, 0x51, 0x80, 0x03, 0x04,
    0xC0, 0xA8, 0xFD, 0x2A, 0x06, 0x04, 0x08, 0x08,
    0x08, 0x08, 0xFF
};

void octeon_sendmsg(octboot_net_device_t* mdev, unsigned char* msg, size_t len) {

    struct octboot_net_sw_descq* tq = &mdev->txq[0];
    if (tq->status != OCTBOOT_NET_DESCQ_READY) {
        printf("txq not ready\n");
        return;
    }

    uint32_t prod_idx = READ_ONCE(tq->local_prod_idx);
    uint32_t cons_idx = READ_ONCE(*(uint32_t*)tq->vaddr_cons_idx);
    
    if (unlikely(prod_idx == 0xFFFFFFFF) || unlikely(cons_idx == 0xFFFFFFFF)) {
        printf("Invalid indices prod_idx=0x%x cons_idx=0x%x\n", 
                prod_idx, cons_idx);
        return;
    }

    if (octboot_net_circq_space(prod_idx, cons_idx, tq->mask) == 0) {
        printf("no send buffer left\n");
        return;
    }

    if (len > OCTBOOT_NET_RING_BUF_SIZE) {
        printf("len > OCTBOOT_NET_RING_BUF_SIZE[%ld]\n", len);
        return;
    }

    uint8_t* pkt_buffer = (uint8_t*)tq->vaddr_pktbuf + (prod_idx * OCTBOOT_NET_RING_BUF_SIZE);
    mmio_memwrite(pkt_buffer, msg, len);

    struct octboot_net_hw_desc_ptr ptr;
    memset(&ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
    ptr.hdr.s_mgmt_net.total_len = len;
    ptr.hdr.s_mgmt_net.ptr_len = len;
    ptr.hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
    ptr.ptr = (uint64_t)((uint8_t*)tq->iova_pktbuf + (prod_idx * OCTBOOT_NET_RING_BUF_SIZE));
    uint8_t* hw_desc_ptr = tq->addr_hw_descq + OCTBOOT_NET_DESC_ARR_ENTRY_OFFSET(prod_idx);
    mmio_memwrite(hw_desc_ptr, &ptr, sizeof(struct octboot_net_hw_desc_ptr));

    wmb();
    prod_idx = octboot_net_circq_inc(prod_idx, tq->mask);
    WRITE_ONCE(tq->local_prod_idx, prod_idx);
    tq->pkts  += 1;
    tq->bytes += len;

	wmb();
    writel(tq->local_prod_idx, tq->addr_hw_prod_idx);

    printf("sent dhcp offer msg size=%ld prod_idx=%d cons_idx=%d\n", 
            len, prod_idx, cons_idx);
}

bool is_dhcp_discover_msg(const uint8_t* pktbuf, size_t N) {
    // Skip Ethernet + IP + UDP headers to get to DHCP payload
    size_t dhcp_offset = 42;
    if (N < dhcp_offset + 240) { // Minimum DHCP header size
        return false;
    }

    const uint8_t* dhcp = pktbuf + dhcp_offset;
    
    // Check DHCP message type (should be BOOTREQUEST)
    if (dhcp[0] != 0x01) {
        return false;
    }

    // Parse DHCP options
    const uint8_t* options = dhcp + 240;
    size_t options_len = N - (dhcp_offset + 240);
    
    for (size_t i = 0; i < options_len;) {
        uint8_t option = options[i];
        
        if (option == 255) { // End option
            break;
        }
        if (option == 0) { // Pad option
            i++;
            continue;
        }
        if (i + 1 >= options_len) {
            break;
        }
        
        uint8_t len = options[i + 1];
        if (i + 2 + len > options_len) {
            break;
        }
        
        // Check for DHCP message type option (53)
        if (option == 53 && len == 1 && options[i + 2] == 1) { // DHCPDISCOVER = 1
            return true;
        }
        
        i += len + 2;
    }

    return false;
}

static void octeon_handle_rxq(octboot_net_device_t* mdev, int sock_fd) {
    struct octboot_net_sw_descq* rq = &mdev->rxq[0];
    if (rq->status != OCTBOOT_NET_DESCQ_READY) {
        return;
    }

	uint32_t hw_cons_idx = READ_ONCE(*(uint32_t*)rq->vaddr_cons_idx);
    uint32_t cons_idx = READ_ONCE(rq->local_cons_idx);
    if (unlikely(cons_idx == 0xFFFFFFFF) || unlikely(hw_cons_idx == 0xFFFFFFFF)) {
        printf("hw_cons_idx=0x%x cons_idx=0x%x\n", hw_cons_idx, cons_idx);
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
        hw_desc_ptr = rq->addr_hw_descq + OCTBOOT_NET_DESC_ARR_ENTRY_OFFSET(start);
        mmio_memread(&ptr, hw_desc_ptr, sizeof(struct octboot_net_hw_desc_ptr));
        if (unlikely(ptr.hdr.s_mgmt_net.total_len < ETH_ZLEN ||
            ptr.hdr.s_mgmt_net.total_len > OCTBOOT_NET_RING_BUF_SIZE ||
		    ptr.hdr.s_mgmt_net.is_frag ||
		    ptr.hdr.s_mgmt_net.ptr_len != ptr.hdr.s_mgmt_net.ptr_len)) {
			/* dont handle frags now */
			rq->errors++;
            printf("rq->error increases to %ld due to frags\n", rq->errors);
        } else {
            rq->pkts += 1;
			rq->bytes += ptr.hdr.s_mgmt_net.total_len;
            uint8_t* pktbuf = (uint8_t*)rq->iova_pktbuf + (start * OCTBOOT_NET_RING_BUF_SIZE);
            if (ptr.ptr != (uint64_t)pktbuf) {
                printf("ptr.ptr != pktbuf in rxq\n");
                return;
            }
            dump_packet((uint8_t*)rq->vaddr_pktbuf + (start * OCTBOOT_NET_RING_BUF_SIZE), ptr.hdr.s_mgmt_net.total_len);
            if (is_dhcp_discover_msg(pktbuf, ptr.hdr.s_mgmt_net.total_len)) {
                printf("dhcp discover msg\n");
                octeon_sendmsg(mdev, dhcp_offer_msg, sizeof(dhcp_offer_msg));
            }
#if 0
            bytes_sent = send(sock_fd, (uint8_t*)rq->vaddr_pktbuf + (start * OCTBOOT_NET_RING_BUF_SIZE),
                ptr.hdr.s_mgmt_net.total_len, 0);
            if (bytes_sent != ptr.hdr.s_mgmt_net.total_len) {
                printf("Partial send: %zd of %u bytes\n", bytes_sent, ptr.hdr.s_mgmt_net.total_len);
                return;
            }
#endif
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
        printf("hw_cons_idx=0x%x cons_idx=0x%x\n", hw_cons_idx, cons_idx);
        return;
	}

    printf("count=%d hw_cons_idx=0x%x cons_idx=0x%x\n", count, hw_cons_idx, cons_idx);
    count = octboot_net_circq_depth(hw_cons_idx,  cons_idx, rq->mask);
    if (count > 0) {
        octeon_handle_rxq(mdev, sock_fd);
    }
}

void* octeon_thread_func(void* arg) {
    thread_params_t* params = (thread_params_t*)arg;
    while (1) {
        usleep(THREAD_SLEEP_US);

        pthread_mutex_lock(&controller.mutex);
        while (controller.octeon_status == STATUS_SUSPENDED) {
            pthread_cond_wait(&controller.cond, &controller.mutex);
        }

        // start
        octeon_handle_rxq(params->mdev, params->veth_fd);
        // end

        pthread_mutex_unlock(&controller.mutex);
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
            printf("Invalid indices prod_idx=0x%x cons_idx=0x%x\n", 
                    prod_idx, cons_idx);
            break;
        }

        if (octboot_net_circq_space(prod_idx, cons_idx, tq->mask) == 0) {
            // no send buffer left
            break;
        }

        uint8_t* pkt_buffer = (uint8_t*)tq->vaddr_pktbuf + (prod_idx * OCTBOOT_NET_RING_BUF_SIZE);
        ssize_t recv_len = recv(sock_fd, pkt_buffer, OCTBOOT_NET_RING_BUF_SIZE, MSG_DONTWAIT);
        if (recv_len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                printf("Failed to receive packet: %s\n", strerror(errno));
            }
            break;
        }

        if (recv_len < ETH_ZLEN || recv_len > OCTBOOT_NET_RING_BUF_SIZE) {
            tq->errors++;
            break;
        }

        struct octboot_net_hw_desc_ptr ptr;
        memset(&ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
        ptr.hdr.s_mgmt_net.total_len = recv_len;
        ptr.hdr.s_mgmt_net.ptr_len = recv_len;
        ptr.hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
        ptr.ptr = (uint64_t)((uint8_t*)tq->iova_pktbuf + (prod_idx * OCTBOOT_NET_RING_BUF_SIZE));
        uint8_t* hw_desc_ptr = tq->addr_hw_descq + OCTBOOT_NET_DESC_ARR_ENTRY_OFFSET(prod_idx);
        mmio_memwrite(hw_desc_ptr, &ptr, sizeof(struct octboot_net_hw_desc_ptr));

        wmb();
        prod_idx = octboot_net_circq_inc(prod_idx, tq->mask);
        WRITE_ONCE(tq->local_prod_idx, prod_idx);
        tq->pkts  += 1;
        tq->bytes += recv_len;
    }

	wmb();
    writel(tq->local_prod_idx, tq->addr_hw_prod_idx);
}

void* veth_thread_func(void* arg) {
    thread_params_t* params = (thread_params_t*)arg;
    while (1) {
        usleep(THREAD_SLEEP_US);

        pthread_mutex_lock(&controller.mutex);
        while (controller.veth_status == STATUS_SUSPENDED) {
            pthread_cond_wait(&controller.cond, &controller.mutex);
        }

        // start
        veth_handle_rawsock(params->veth_fd, params->mdev);
        // end

        pthread_mutex_unlock(&controller.mutex);
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
        printf("Failed to create raw socket: %s\n", strerror(errno));
        return -1;
    }

    // Get interface index
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1);
    if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) {
        printf("Failed to get interface index: %s\n", strerror(errno));
        close(sock_fd);
        return -1;
    }

    // Bind to interface
    memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex = ifr.ifr_ifindex;
    if (bind(sock_fd, (struct sockaddr*)&sll, sizeof(sll)) < 0) {
        printf("Failed to bind socket: %s\n", strerror(errno));
        close(sock_fd);
        return -1;
    }

    // Set promiscuous mode
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1);
    if (ioctl(sock_fd, SIOCGIFFLAGS, &ifr) < 0) {
        printf("Failed to get interface flags: %s\n", strerror(errno));
        close(sock_fd);
        return -1;
    }
    ifr.ifr_flags |= IFF_PROMISC;
    if (ioctl(sock_fd, SIOCSIFFLAGS, &ifr) < 0) {
        printf("Failed to set promiscuous mode: %s\n", strerror(errno));
        close(sock_fd);
        return -1;
    }

    return sock_fd;
}

/********************************************************************************************
 ********************************     Target Management   ***********************************
********************************************************************************************/
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
#define HOST_VERSION_OFFSET_VAL(mdev)      (mdev->bar_map[BAR4].offset + HOST_VERSION_OFFSET)
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


#define OCTBOOT_NET_VERSION "1.0"
#define OCTBOOT_NET_VERSION_MAJOR 1
#define OCTBOOT_NET_VERSION_MINOR 0


#define CNXK_SDP_WIN_WR_MASK_REG                0x20030
#define CNXK_RST_CHIP_DOMAIN_W1S                0x000087E006001810ULL
#define CNXK_SDP_WIN_WR_ADDR64                  0x20000
#define CNXK_SDP_WIN_WR_DATA64                  0x20020
#define CNXK_SDP_WIN_WR_ADDR64_REG(mdev)        ((uint8_t*)mdev->bar_map[BAR0].bar_addr + CNXK_SDP_WIN_WR_ADDR64)
#define CNXK_SDP_WIN_WR_DATA64_REG(mdev)        ((uint8_t*)mdev->bar_map[BAR0].bar_addr + CNXK_SDP_WIN_WR_DATA64)

#define FW_STATUS_DOWNING                       0ULL
#define CNXK_PCIEEP_VSECST_CTL                  0x418
#define CNXK_PEMX_PFX_CSX_PFCFGX(pem, pf, offset)      ((0x8e0000008000 | (uint64_t)pem << 36 \
                                                | pf << 18 \
                                                | ((offset >> 16) & 1) << 16 \
                                                | (offset >> 3) << 3) \
                                                + (((offset >> 2) & 1) << 2))

#define SIGNATURE_REG(mdev)                     ((uint8_t*)mdev->bar_map[BAR4].bar_addr + SIGNATURE_OFFSET)

void reset_fw_ready_state(octboot_net_device_t* mdev) {
    writeq(CNXK_PEMX_PFX_CSX_PFCFGX(0, 0, CNXK_PCIEEP_VSECST_CTL), CNXK_SDP_WIN_WR_ADDR64_REG(mdev));
    writeq(FW_STATUS_DOWNING, CNXK_SDP_WIN_WR_DATA64_REG(mdev));
}

void pci_enable_device(octboot_net_device_t* mdev) {
    uint16_t cmd = reads(PCI_CONF_CMD_REG(mdev));
    /* 
    bit0: PCI_COMMAND_IO
    bit1: PCI_COMMAND_MEMORY
    bit2: PCI_COMMAND_MASTER, for DMA operation
    */ 
    cmd |= 0x07;
    writes(cmd, PCI_CONF_CMD_REG(mdev));
}

static int find_pcie_cap(octboot_net_device_t* mdev)
{
    uint8_t pos = 0x34;
    uint8_t cap_id;
    
    while (pos) {
        cap_id = readb(PCI_CONF_REG(mdev) + pos);
        
        if (cap_id == PCI_CAP_ID_EXP)
            return pos;

        // next pos
        pos = readb(PCI_CONF_REG(mdev) + (pos + 1));
    }
    
    return 0;
}

void pci_reset_function(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return;
    }

    int pos = find_pcie_cap(mdev);
    if (pos == 0) {
        printf("pcie_cap is not found");
        return;
    }

    uint16_t devctl = reads(PCI_CONF_REG(mdev) + pos + PCI_EXP_DEVCTL);
    devctl |= PCI_EXP_DEVCTL_BCR_FLR;
    writes(devctl, PCI_CONF_REG(mdev) + pos + PCI_EXP_DEVCTL);
    usleep(OCTBOOT_NET_SERVICE_TASK_US_FLR);
}

static uint64_t get_host_status(octboot_net_device_t* mdev)
{
	return readq(HOST_STATUS_REG(mdev));
}

static uint64_t get_target_status(octboot_net_device_t* mdev)
{
	return readq(TARGET_STATUS_REG(mdev));
}

static uint64_t get_target_version(octboot_net_device_t* mdev)
{
	return readq(TARGET_VERSION_REG(mdev));
}

static uint64_t get_target_mbox_ack(octboot_net_device_t* mdev)
{
	return readq(TARGET_MBOX_ACK_REG(mdev));
}

static void set_host_mbox_ack_reg(octboot_net_device_t* mdev, uint32_t id)
{
	writeq(id, HOST_MBOX_ACK_REG(mdev));
}

static void mbox_send_msg(octboot_net_device_t* mdev,
		union octboot_net_mbox_msg *msg)
{
	mdev->send_mbox_id++;
	msg->s.hdr.id = mdev->send_mbox_id;
	uint64_t id = msg->s.hdr.id;
	for (uint8_t i = 1; i <= msg->s.hdr.sizew; i++)
		writeq(msg->words[i], HOST_MBOX_MSG_REG(mdev, i));
	writeq(msg->words[0], HOST_MBOX_MSG_REG(mdev, 0));

	if (msg->s.hdr.req_ack || msg->s.hdr.sizew) {
        int retry = 10;
		while (get_target_mbox_ack(mdev) != id && retry > 0 ) {
			usleep(OCTBOOT_NET_MBOX_WAIT_MS*1000);
            retry--;
		}
	}
}

static void change_host_status(octboot_net_device_t* mdev, uint64_t status,
			bool ack_wait)
{
	union octboot_net_mbox_msg msg;
	writeq(status, HOST_STATUS_REG(mdev));
	memset(&msg, 0, sizeof(union octboot_net_mbox_msg));
	msg.s.hdr.opcode = OCTBOOT_NET_MBOX_HOST_STATUS_CHANGE;
	if (ack_wait)
		msg.s.hdr.req_ack = 1;
	mbox_send_msg(mdev, &msg);
}

static int mbox_check_msg_rcvd(octboot_net_device_t* mdev,
    union octboot_net_mbox_msg *msg) {
    int i = 0;
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

    msg->words[0] = readq(TARGET_MBOX_MSG_REG(mdev, 0));
	if (mdev->recv_mbox_id != msg->s.hdr.id) {
		mdev->recv_mbox_id = msg->s.hdr.id;
		for (i = 1; i <= msg->s.hdr.sizew; i++)
			msg->words[i] = readq(TARGET_MBOX_MSG_REG(mdev, i));
		return i;
	}

    return 0;
}

static int handle_target_status(octboot_net_device_t* mdev)
{
	int ret = 0;
	uint64_t host_status = get_host_status(mdev);
	uint64_t target_status = get_target_status(mdev);
	printf("host status %lu, target status %lu\n", host_status, target_status);

	switch (host_status) {
	case OCTNET_HOST_READY:
		if (target_status == OCTNET_TARGET_RUNNING) {
			printf("octboot_net: target running\n");
			change_host_status(mdev, OCTNET_HOST_RUNNING, false);
		}
		break;
	case OCTNET_HOST_RUNNING:
		target_status = get_target_status(mdev);
		if (target_status != OCTNET_TARGET_RUNNING) {
			printf("octboot_net: target stopped\n");
			change_host_status(mdev, OCTNET_HOST_GOING_DOWN,
						   false);
            /* reset when txq/rxq running case is not included */
			ret |= mdev_clean_tx_ring(mdev);
            ret |= mdev_clean_rx_ring(mdev);
            ret |= mdev_setup_tx_ring(mdev);
            ret |= mdev_setup_rx_ring(mdev);
			if (ret) {
				change_host_status(mdev, OCTNET_HOST_FATAL,
						   false);
				return ret;
			}
			change_host_status(mdev, OCTNET_HOST_READY, false);
		}
		break;
	default:
		printf("unhandled state transition host_status:%lu target_status %lu\n",
		       host_status, target_status);
		break;
	}
	return ret;
}

static int octeon_target_setup(octboot_net_device_t* mdev) {
    if (mdev == NULL) {
        printf("invalid parameter of mdev\n");
        return -1;
    }

#if 0
    if (ioctl(mdev->container_fd, VFIO_DEVICE_RESET)) {
        printf("Unable to reset device! Error: %d (%s)", errno, strerror(errno));
        return -1;
    }
#endif
    uint64_t host_version = ((OCTBOOT_NET_VERSION_MAJOR << 8)|OCTBOOT_NET_VERSION_MINOR);
    uint64_t host_version0, host_version1;

#ifdef VFIO_ENABLED
    if (mdev->bar_mapped) {
        host_version0 = readq(HOST_VERSION_REG(mdev));
        printf("read host_version0[0x%lx]\n", host_version0);
        if (host_version0 != host_version) {
            writeq(host_version, HOST_VERSION_REG(mdev));
            printf("write host_version[0x%lx]\n", host_version);
            host_version1 = readq(HOST_VERSION_REG(mdev));
            printf("read host_version1[0x%lx]\n", host_version1);
            if (host_version1 != host_version) {
                printf("Failed to set host version\n");
                return -1;
            }
        }

        uint64_t target_version = get_target_version(mdev);
        if ((host_version >> 8) != (target_version >> 8)) {
            printf("octboot_net driver imcompatible with uboot, host_version:0x%lx, target_version:0x%lx\n",
                host_version, target_version);
            return -1;
        }
    } else {
        if (vfio_read(mdev->device_fd, &host_version0,
            sizeof(host_version0), HOST_VERSION_OFFSET_VAL(mdev))) {
                printf("read fd[%d] host_version0[0x%lx] offset[0x%lx] error\n", mdev->device_fd, host_version0, HOST_VERSION_OFFSET_VAL(mdev));
                return -1;
            }
        printf("read fd[%d] host_version0[0x%lx] offset[0x%lx]\n", mdev->device_fd, host_version0, HOST_VERSION_OFFSET_VAL(mdev));
        if (vfio_write(mdev->device_fd, &host_version,
                sizeof(host_version), HOST_VERSION_OFFSET_VAL(mdev))) {
                    printf("write fd[%d] host_version[0x%lx] offset[0x%lx] error\n", mdev->device_fd, host_version,HOST_VERSION_OFFSET_VAL(mdev));
                    return -1;
                }
        printf("write fd[%d] host_version[0x%lx] offset[0x%lx]\n", mdev->device_fd, host_version,HOST_VERSION_OFFSET_VAL(mdev));
        if (vfio_read(mdev->device_fd, &host_version1,
            sizeof(host_version1), HOST_VERSION_OFFSET_VAL(mdev))) {
                printf("read fd[%d] host_version1[0x%lx] offset[0x%lx] error\n", mdev->device_fd, host_version1, HOST_VERSION_OFFSET_VAL(mdev));
                return -1;
            }
        printf("read fd[%d] host_version1[0x%lx] offset[0x%lx]\n", mdev->device_fd, host_version1, HOST_VERSION_OFFSET_VAL(mdev));
    }

#else

    host_version0 = readq(HOST_VERSION_REG(mdev));
    if (host_version0 != host_version) {
        writeq(host_version, HOST_VERSION_REG(mdev));
        host_version1 = readq(HOST_VERSION_REG(mdev));
        if (host_version1 != host_version) {
            printf("Failed to set host version\n");
            return -1;
        }
    }

	uint64_t target_version = get_target_version(mdev);
    if ((host_version >> 8) != (target_version >> 8)) {
        printf("octboot_net driver imcompatible with uboot, host_version:0x%lx, target_version:0x%lx\n",
            host_version, target_version);
        return -1;
    }

    // signature found
    memcpy(&mdev->npu_memmap_info, SIGNATURE_REG(mdev),
		sizeof(struct uboot_pcinet_barmap));
    uint64_t signature = mdev->npu_memmap_info.signature;
    if (signature == NPU_HANDSHAKE_SIGNATURE) {
        if (!mdev->signature_found) {
           mdev->signature_found = true;
        }
    } else {
        if (mdev->signature_found) {
            mdev->signature_found = false;
            printf("signature is found before, but now misaligned with target\n");
        }
        return -1;
    }

#endif

    return 0;
}

static int octeon_target_setup_2(octboot_net_device_t* mdev) {
    union octboot_net_mbox_msg msg;
    int word_num = mbox_check_msg_rcvd(mdev, &msg);
    if (word_num == 0) {
        return -1;
    }

    switch (msg.s.hdr.opcode) {
    case OCTBOOT_NET_MBOX_TARGET_STATUS_CHANGE:
        handle_target_status(mdev);
        if (msg.s.hdr.req_ack)
            set_host_mbox_ack_reg(mdev, msg.s.hdr.id);
        break;
    case OCTBOOT_NET_MBOX_OPCODE_INVALID:
        return -1;
    default:
        break;
    }

    if ((OCTNET_TARGET_RUNNING != get_target_status(mdev)) 
        || OCTNET_HOST_RUNNING != get_host_status(mdev)) {
        printf("target status is not running\n");
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
        octbootdev[i].bar_mapped = false;
        for (int j = 0; j < NUM_BARS; j++) {
            octbootdev[i].bar_map[j].bar_addr = NULL;
            octbootdev[i].bar_map[j].bar_size = 0;
        }
        octbootdev[i].signature_found = false;
        memset(&octbootdev[i].npu_memmap_info, 0, sizeof(octbootdev[i].npu_memmap_info));
        octbootdev[i].send_mbox_id = 0;
        octbootdev[i].recv_mbox_id = 0;
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
        printf("Usage: %s <vfio_group_id> <pci_addr>\n", argv[0]);
        printf("Example: %s 5 0000:b3:00.0\n", argv[0]);
        return -1;
    }

    snprintf(octbootdev[0].vfio_path, sizeof(octbootdev[0].vfio_path), "/dev/vfio/%s", argv[1]);
    snprintf(octbootdev[0].pci_addr, sizeof(octbootdev[0].pci_addr), "%s", argv[2]);

    while (mdev_mm_init(&octbootdev[0]) < 0) {
        printf("failed to init memory\n");
        usleep(INIT_SLEEP_US);
    }
    printf("memory is initiated successfully\n");

    while (octeon_target_setup(&octbootdev[0]) < 0) {
        printf("failed to setup target\n");
        usleep(INIT_SLEEP_US);
    }
    printf("octeon target comes up\n");

    if (mdev_hugepage_alloc(&octbootdev[0]) < 0) {
        printf("failed to allocate hugepage\n");
        return -1;
    }


    if (mdev_dma_init(&octbootdev[0]) < 0) {
        printf("failed to init dma\n");
        return -1;
    }

    if (mdev_setup_rx_ring(&octbootdev[0]) < 0) {
        printf("failed to setup rx ring\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_mm_uninit(&octbootdev[0]);
        return -1;
    }

    if (mdev_setup_tx_ring(&octbootdev[0]) < 0) {
        printf("failed to setup tx ring\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_clean_tx_ring(&octbootdev[0]);
        mdev_mm_uninit(&octbootdev[0]);
        return -1;
    }

    change_host_status(&octbootdev[0], OCTNET_HOST_READY, false);
    while (octeon_target_setup_2(&octbootdev[0]) < 0) {
        usleep(INIT_SLEEP_US);
    }

#if 0
    int veth_fd = veth_setup_raw_socket(VETH_INTERFACE_NAME);
    if (veth_fd < 0) {
        printf("failed to setup veth\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_clean_tx_ring(&octbootdev[0]);
        mdev_mm_uninit(&octbootdev[0]);
        return -1;
    }
#endif

    pthread_t octeon_thread, veth_thread;
    thread_params_t params;
    params.mdev = &octbootdev[0];
    params.veth_fd = -1;
    if (pthread_create(&octeon_thread, NULL, octeon_thread_func, &params) != 0) {
        printf("failed to create octeon thread\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_clean_tx_ring(&octbootdev[0]);
        mdev_mm_uninit(&octbootdev[0]);
        return -1;
    }
    if (pthread_create(&veth_thread, NULL, veth_thread_func, &params) != 0) {
        printf("failed to create veth thread\n");
        mdev_clean_rx_ring(&octbootdev[0]);
        mdev_clean_tx_ring(&octbootdev[0]);
        mdev_mm_uninit(&octbootdev[0]);
        return -1;
    }

    pthread_join(octeon_thread, NULL);
    pthread_join(veth_thread, NULL);

    return ret;
}
