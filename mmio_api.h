/* Copyright (c) 2020 Marvell.
 * SPDX-License-Identifier: GPL-2.0
 */

/* Mgmt ethernet driver
 */

#ifndef __MMIO_API_H_
#define __MMIO_API_H_

#include <stdint.h>

// Memory barrier implementations
#if defined(__x86_64__) || defined(__i386__)
    // x86 memory barriers
    #define wmb() __asm__ volatile("sfence" ::: "memory")
    #define rmb() __asm__ volatile("lfence" ::: "memory")
    #define mb()  __asm__ volatile("mfence" ::: "memory")
#elif defined(__aarch64__)
    // ARM64 memory barriers
    #define wmb() __asm__ volatile("dmb st" ::: "memory")
    #define rmb() __asm__ volatile("dmb ld" ::: "memory")
    #define mb()  __asm__ volatile("dmb sy" ::: "memory")
#else
    // Generic barriers using GCC built-ins
    #define wmb() __sync_synchronize()
    #define rmb() __sync_synchronize()
    #define mb()  __sync_synchronize()
#endif

static inline void writeb(uint8_t value, volatile void *addr)
{
    *((volatile uint8_t *)addr) = value;
    wmb();  // Write memory barrier to ensure order
}

static inline uint8_t readb(const volatile void *addr)
{
    uint8_t value;
    value = *((volatile uint8_t *)addr);
    rmb();  // Read memory barrier to ensure order
    return value;
}

static inline void writes(uint16_t value, volatile void *addr)
{
    *((volatile uint16_t *)addr) = value;
    wmb();  // Write memory barrier to ensure order
}

static inline uint16_t reads(const volatile void *addr)
{
    uint16_t value;
    value = *((volatile uint16_t *)addr);
    rmb();  // Read memory barrier to ensure order
    return value;
}

static inline void writel(uint32_t value, volatile void *addr)
{
    *((volatile uint32_t *)addr) = value;
    wmb();  // Write memory barrier to ensure order
}

static inline uint32_t readl(const volatile void *addr)
{
    uint32_t value;
    value = *((volatile uint32_t *)addr);
    rmb();  // Read memory barrier to ensure order
    return value;
}

static inline uint64_t readq(const volatile void *addr)
{
    uint64_t value;
    value = *((volatile uint64_t *)addr);
    rmb();  // Read memory barrier
    return value;
}

static inline void writeq(uint64_t value, volatile void *addr)
{
    *((volatile uint64_t *)addr) = value;
    wmb();  // Write memory barrier
}

// 8-bit MMIO operations
static inline void writeb_relaxed(uint8_t value, volatile void *addr)
{
    *((volatile uint8_t *)addr) = value;
}

static inline uint8_t readb_relaxed(const volatile void *addr)
{
    return *((volatile uint8_t *)addr);
}

// 64-bit MMIO operations
static inline void writeq_relaxed(uint64_t value, volatile void *addr)
{
    *((volatile uint64_t *)addr) = value;
}

static inline uint64_t readq_relaxed(const volatile void *addr)
{
    return *((volatile uint64_t *)addr);
}

static inline void mmio_memset(void  *mmio_addr, int val, int size)
{
	uint8_t  *baddr;

	baddr = (uint8_t *)mmio_addr;
	while (size--) {
		writeb_relaxed(val, baddr);
		baddr++;
	}
	wmb();
}

static inline void mmio_memread(void *laddr, void const   *mmio_addr,
				int size)
{
	uint64_t  *qaddr;
	uint8_t  *baddr;
	int alignl, alignr;
	uint64_t *lqaddr;
	uint8_t *lbaddr;

	alignl = (uint64_t)laddr % 8;
	alignr = (uint64_t)mmio_addr % 8;
	qaddr = (uint64_t *)mmio_addr;
	lqaddr = (uint64_t *)laddr;
	if (alignl == 0 && alignr == 0) {
		while (size >= 8) {
			*lqaddr = readq_relaxed(qaddr);
			size -= 8;
			lqaddr++;
			qaddr++;
		}
	}
	baddr = (uint8_t *)qaddr;
	lbaddr = (uint8_t *)lqaddr;
	while (size--) {
		*lbaddr = readb_relaxed(baddr);
		baddr++;
		lbaddr++;
	}
	rmb();
}

static inline void mmio_memwrite(void  *mmio_addr, void const *laddr,
				 int size)
{
	uint64_t  *qaddr;
	uint8_t  *baddr;
	int alignl, alignr;
	uint64_t *lqaddr;
	uint8_t *lbaddr;

	alignl = (uint64_t)laddr % 8;
	alignr = (uint64_t)mmio_addr % 8;
	qaddr = (uint64_t *)mmio_addr;
	lqaddr = (uint64_t *)laddr;
	if (alignl == 0 && alignr == 0) {
		while (size >= 8) {
			writeq_relaxed(*lqaddr, qaddr);
			size -= 8;
			lqaddr++;
			qaddr++;
		}
	}
	baddr = (uint8_t *)qaddr;
	lbaddr = (uint8_t *)lqaddr;
	while (size--) {
		writeb_relaxed(*lbaddr, baddr);
		baddr++;
		lbaddr++;
	}
	wmb();
}

#endif /* _MMIO_API_ */
