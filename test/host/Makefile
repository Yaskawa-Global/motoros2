# Makefile — host-gcc unit tests for the point-queue feature.
# Builds with plain gcc on Linux (no MotoPlus toolchain, no ROS).
#
#   make        # build + run; non-zero exit on any test failure
#   make build  # build only
#   make clean
#
# The lock-free SPSC ring bodies are the REAL production ones: we compile
# ../../src/PointQueue.c directly. That file does `#include "MotoROS.h"`, so it
# is compiled with `-iquote . -I-` so the test-only MotoROS.h shim in THIS dir
# resolves (not src/MotoROS.h, which would drag the full MotoPlus + micro-ROS
# header set). `-I-` also drops the including-file's own-dir search for quote
# includes, so `-iquote .` is added to keep the other host TUs finding their
# local headers (motoplus_stubs.h, etc). No ring bodies are duplicated anymore.

CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall
IQUOTE   = -iquote . -I-
SRCDIR   = ../../src
BIN      = test_point_queue
# Host-tree TUs (test driver + consumer flush-action model + admission/underran models)
HOST_SRCS = test_point_queue.c point_queue_ring.c admission_model.c underran_latch.c
# Real production ring TU, compiled against the host MotoROS.h shim
REAL_SRCS = $(SRCDIR)/PointQueue.c
HDRS     = motoplus_stubs.h point_queue_ring.h admission_model.h MotoROS.h

.PHONY: all build run clean
all: run

build: $(BIN)

$(BIN): $(HOST_SRCS) $(REAL_SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(IQUOTE) $(HOST_SRCS) $(REAL_SRCS) -o $(BIN)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
