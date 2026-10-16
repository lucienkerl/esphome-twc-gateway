CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined
SRC_DIR := ../components/twc-gateway
SOURCES := test_twc.cpp $(SRC_DIR)/twc_frames.cpp $(SRC_DIR)/twc_policy.cpp $(SRC_DIR)/twc_master.cpp

.PHONY: test clean
test: test_twc
	./test_twc

test_twc: $(SOURCES) $(wildcard $(SRC_DIR)/*.h)
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES)

clean:
	rm -f test_twc
