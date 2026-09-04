# Makefile — host-gcc unit tests for the point-queue feature.
# Builds with plain gcc on Linux (no MotoPlus toolchain, no ROS).
#
#   make        # build + run; non-zero exit on any test failure
#   make build  # build only
#   make clean

CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall
BIN      = test_point_queue
SRCS     = test_point_queue.c point_queue_ring.c admission_model.c underran_latch.c
HDRS     = motoplus_stubs.h point_queue_ring.h admission_model.h

.PHONY: all build run clean
all: run

build: $(BIN)

$(BIN): $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(SRCS) -o $(BIN)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
