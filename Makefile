export PATH := /opt/homebrew/opt/llvm/bin:/opt/homebrew/bin:/usr/local/bin:$(PATH)

CC ?= clang
PREFIX ?= /usr/local
FLOW_DIR ?= ../FLOW
FLOW_LIB := $(FLOW_DIR)/build/libflow_rt.a
UNIT_VERSION := $(shell tr -d '[:space:]' < VERSION 2>/dev/null || echo "1.2.0")
CFLAGS ?= -std=c17 -Wall -Wextra -Wpedantic -Wmissing-prototypes -Werror -O2 -Isrc -Itests -I$(FLOW_DIR)/src -I$(FLOW_DIR)/build
override CFLAGS += -Isrc -Itests -I$(FLOW_DIR)/src -I$(FLOW_DIR)/build -DUNIT_VERSION_STRING=\"$(UNIT_VERSION)\" -D_GNU_SOURCE -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L

LDLIBS ?=
THREAD_FLAGS ?= -pthread
UNAME_S := $(shell uname -s 2>/dev/null || echo "Unknown")
ifeq ($(UNAME_S),Darwin)
STACK_LDFLAGS ?= -Wl,-stack_size,0x1000000
else ifeq ($(UNAME_S),Linux)
LDLIBS += -lrt
STACK_LDFLAGS ?=
else
STACK_LDFLAGS ?=
endif

BUILD_DIR := build
LIBUNIT_A := $(BUILD_DIR)/libunit.a

UNIT_OBJS := $(BUILD_DIR)/obj/unit_core.o

AR ?= ar
RANLIB ?= ranlib

CLANG_FORMAT ?= clang-format
FMT_FILES := $(wildcard src/*.c src/*.h tests/*.c tests/*.h)

.DEFAULT_GOAL := all
.PHONY: all clean test fmt fmt-check

all: $(LIBUNIT_A)

fmt:
	$(CLANG_FORMAT) -i $(FMT_FILES)

fmt-check:
	$(CLANG_FORMAT) --dry-run --Werror $(FMT_FILES)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(FLOW_LIB):
	$(MAKE) -C $(FLOW_DIR) all

$(BUILD_DIR)/obj/unit_core.o: src/unit_core.c $(wildcard src/*.h) $(FLOW_LIB) | $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/obj
	$(CC) $(CFLAGS) $(THREAD_FLAGS) -c $< -o $@

$(LIBUNIT_A): $(UNIT_OBJS)
	$(AR) rcs $@ $^
	$(RANLIB) $@

TEST_DEPS := tests/unit_test_kit.h $(LIBUNIT_A) $(FLOW_LIB)

TEST_BINARIES := \
	$(BUILD_DIR)/test-utf8-minbit \
	$(BUILD_DIR)/test-isomorphism \
	$(BUILD_DIR)/test-flow-pipeline \
	$(BUILD_DIR)/bench-falsify

$(BUILD_DIR)/test-utf8-minbit: tests/test_utf8_minbit.c $(TEST_DEPS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(STACK_LDFLAGS) $(THREAD_FLAGS) $< $(LIBUNIT_A) $(FLOW_LIB) -o $@ $(LDLIBS)

$(BUILD_DIR)/test-isomorphism: tests/test_isomorphism.c $(TEST_DEPS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(STACK_LDFLAGS) $(THREAD_FLAGS) $< $(LIBUNIT_A) $(FLOW_LIB) -o $@ $(LDLIBS)

$(BUILD_DIR)/test-flow-pipeline: tests/test_flow_pipeline.c $(TEST_DEPS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(STACK_LDFLAGS) $(THREAD_FLAGS) $< $(LIBUNIT_A) $(FLOW_LIB) -o $@ $(LDLIBS)

$(BUILD_DIR)/bench-falsify: tests/bench_falsify.c $(TEST_DEPS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(STACK_LDFLAGS) $(THREAD_FLAGS) $< $(LIBUNIT_A) $(FLOW_LIB) -o $@ $(LDLIBS)

test: $(TEST_BINARIES)
	@echo "=== [0/4] Pure Wire & Minimal-Bit Spec Headers Zero-FLOW-Dependency Audit ==="
	@$(CC) -std=c17 -Wall -Wextra -Wpedantic -Werror -Isrc -fsyntax-only src/unit_utf8_bit.h src/unit_isomorphism.h
	@echo "  ✓ unit_utf8_bit.h & unit_isomorphism.h verified standalone (0 FLOW rt dependency)!"
	@echo "=== [1/4] Observable UTF-8 <-> Minimal-Bit + Control Signal Substrate ==="
	@$(BUILD_DIR)/test-utf8-minbit
	@echo "=== [2/4] Protocol-Body <~> Opcode-Pointer Structural Isomorphism ==="
	@$(BUILD_DIR)/test-isomorphism
	@echo "=== [3/4] End-to-End UTF-8 Wire -> Minimal-Bit -> FLOW v4 ASM Execution ==="
	@$(BUILD_DIR)/test-flow-pipeline --report
	@echo "=== [4/4] Falsification & Breakthrough Verification Benchmark ==="
	@$(BUILD_DIR)/bench-falsify
	@echo "================================================================================"
	@echo "   ALL UNIT V1 TEST & FALSIFICATION SUITES PASSED!"
	@echo "================================================================================"

clean:
	rm -rf $(BUILD_DIR)
