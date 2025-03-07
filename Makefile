# Compiler settings
CC := gcc
CFLAGS := -std=gnu11 -O2 -Wall -Wextra -Wpedantic
LDFLAGS := -lpthread

# Project configuration
TARGET := user_space_octboot_net
SRC := main.c
OBJ := $(SRC:.c=.o)

# Debug build configuration
DEBUG_CFLAGS := -g -DDEBUG

# Installation paths (optional)
PREFIX := /usr/local
BINDIR := $(PREFIX)/bin

.PHONY: all debug clean install uninstall

all: $(TARGET)

debug: CFLAGS += $(DEBUG_CFLAGS)
debug: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) *.o

install: $(TARGET)
	install -d $(BINDIR)
	install -m 755 $(TARGET) $(BINDIR)

uninstall:
	rm -f $(BINDIR)/$(TARGET)

