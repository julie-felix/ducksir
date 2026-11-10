# ducksim 执行核：一份 C++ 源码 → libducksim.so（主机/板端）/ libducksim.wasm（浏览器，W7）。
#
# 没有 cmake：本机只装了 g++ 13 与 make，加一层构建系统反而多一个要维护的坑。
# 出 wasm 那一步在 W7 单独写（emscripten 的 target 与 native 不同）。

CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic -ffp-contract=off -fPIC
INCLUDE  := -Iinclude
BUILD    := build
# ORT SDK 位置（含 include/onnxruntime_c_api.h 与 lib/libonnxruntime.so）
# 默认指向 docs/env.md 里那套；换机器时用 DUCKSIR_ORT=/path make ort
ORT_DIR  ?= $(HOME)/.local/share/ducksir/ort

.PHONY: all clean test

all: $(BUILD)/libducksim.so

# 可选：推理后端。核心不依赖它，只在使用方需要时才编。
ort: $(BUILD)/libducksim_ort.so

$(BUILD)/libducksim_ort.so: src/ort_backend.cpp include/ducksim.h
	@mkdir -p $(BUILD)
	@test -f $(ORT_DIR)/include/onnxruntime_c_api.h || \
		{ echo "缺 ORT SDK：$(ORT_DIR)（见 docs/env.md）"; exit 1; }
	$(CXX) $(CXXFLAGS) $(INCLUDE) -I$(ORT_DIR)/include -shared -o $@ src/ort_backend.cpp \
		-L$(ORT_DIR)/lib -lonnxruntime -Wl,-rpath,$(ORT_DIR)/lib

$(BUILD)/libducksim.so: src/core.cpp include/ducksim.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -shared -o $@ src/core.cpp

# 与 numpy oracle 对拍：C 核 vs sim/plan/oracle.py，逐点 ≤1e-6
test: all
	python3 ../sim/test/test_core_vs_oracle.py

clean:
	rm -rf $(BUILD)
