# ducksim 执行核：一份 C++ 源码 → libducksim.so（主机/板端）/ libducksim.wasm（浏览器，W7）。
#
# 没有 cmake：本机只装了 g++ 13 与 make，加一层构建系统反而多一个要维护的坑。
# 出 wasm 那一步在 W7 单独写（emscripten 的 target 与 native 不同）。

CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic -ffp-contract=off -fPIC
INCLUDE  := -Iinclude
BUILD    := build

.PHONY: all clean test

all: $(BUILD)/libducksim.so

$(BUILD)/libducksim.so: src/core.cpp include/ducksim.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -shared -o $@ src/core.cpp

# 与 numpy oracle 对拍：C 核 vs sim/plan/oracle.py，逐点 ≤1e-6
test: all
	python3 ../sim/test/test_core_vs_oracle.py

clean:
	rm -rf $(BUILD)
