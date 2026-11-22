# Host tests for the Zenith shared code.   make        (all)
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -I../firmware/common
OUT      ?= build

all: link sim sync

$(OUT):
	mkdir -p $(OUT)

link: $(OUT)
	$(CXX) $(CXXFLAGS) link_test.cpp -o $(OUT)/link_test && $(OUT)/link_test

sim: $(OUT)
	$(CXX) $(CXXFLAGS) sim_test.cpp -o $(OUT)/sim_test && $(OUT)/sim_test

# every sketch's copy of the shared headers must equal firmware/common
sync:
	python3 ../tools/sync_common.py

.PHONY: all link sim sync
