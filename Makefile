CXX ?= g++
CXXFLAGS ?= -std=c++20 -O2 -g -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude

.PHONY: all test clean
all: build/flight-run build/flight-live build/core-tests

build:
	mkdir -p build

build/flight-run: src/core.cpp src/runner.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/core.cpp src/runner.cpp -o $@

build/flight-live: src/core.cpp src/live.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/core.cpp src/live.cpp -o $@

build/core-tests: src/core.cpp tests/core_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/core.cpp tests/core_tests.cpp -o $@

test: all
	./build/core-tests data/validation.airframe

clean:
	rm -rf build
