CXX ?= g++
CXXFLAGS ?= -std=c++20 -O2 -g -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude
CORE = src/core.cpp src/propeller.cpp

.PHONY: all test clean
all: build/flight-run build/flight-live build/core-tests build/mechanics-review build/propulsion-probe build/propeller-tests build/governor-probe build/detachment-tests build/material-tests

build:
	mkdir -p build

build/flight-run: $(CORE) src/runner.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) src/runner.cpp -o $@

build/flight-live: $(CORE) src/live.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) src/live.cpp -o $@

build/core-tests: $(CORE) tests/core_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/core_tests.cpp -o $@

build/mechanics-review: $(CORE) tests/mechanics_review.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/mechanics_review.cpp -o $@

build/propulsion-probe: $(CORE) tests/propulsion_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/propulsion_probe.cpp -o $@

build/propeller-tests: $(CORE) tests/propeller_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/propeller_tests.cpp -o $@

build/governor-probe: $(CORE) tests/governor_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/governor_probe.cpp -o $@

build/detachment-tests: $(CORE) tests/detachment_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/detachment_tests.cpp -o $@

build/material-tests: $(CORE) tests/material_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/material_tests.cpp -o $@

test: all
	./build/core-tests data/validation.airframe
	./build/mechanics-review data/validation.airframe
	./build/propeller-tests data/validation-bem.airframe
	./build/detachment-tests data/validation-bem.airframe
	./build/material-tests data/validation-material.airframe data/validation-bem.airframe

clean:
	rm -rf build
