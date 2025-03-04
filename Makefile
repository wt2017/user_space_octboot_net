# SPDX-License-Identifier: GPL-2.0
#
# Facility driver for Marvell's Octeon PCI Endpoint NIC

CC = gcc
CFLAGS = -Wall -Wextra -I.

TARGET = user_space_octboot_net

SRCS = octboot_net.c

HEADERS = desc_queue.h mmio_api.h octboot_net.h octboot_net_compat.h

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

rebuild: clean all

.PHONY: all clean rebuild
