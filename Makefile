CXX ?= g++
CXXFLAGS ?= -std=c++20 -O2 -g -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude
CORE = src/core.cpp src/propeller.cpp

.PHONY: all test clean
all: build/surface-sampling-tests build/sensor-history-probe build/aerodynamics-tests build/sensitivity-probe build/flight-run build/flight-live build/core-tests build/mechanics-review build/propulsion-probe build/propeller-tests build/governor-probe build/detachment-tests build/material-tests build/control-tests build/control-probe

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

build/control-tests: $(CORE) tests/control_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/control_tests.cpp -o $@

build/control-probe: $(CORE) tests/control_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/control_probe.cpp -o $@

build/aerodynamics-tests: $(CORE) tests/aerodynamics_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/aerodynamics_tests.cpp -o $@

build/sensitivity-probe: $(CORE) tests/sensitivity_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/sensitivity_probe.cpp -o $@

build/surface-sampling-tests: $(CORE) tests/surface_sampling_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/surface_sampling_tests.cpp -o $@

build/sensor-history-probe: $(CORE) tests/sensor_history_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/sensor_history_probe.cpp -o $@

build/finite-span-probe: $(CORE) tests/finite_span_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/finite_span_probe.cpp -o $@

build/section-controls-probe: $(CORE) tests/section_controls_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/section_controls_probe.cpp -o $@

build/control-transition-probe: $(CORE) tests/control_transition_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) tests/control_transition_probe.cpp -o $@

test: all
	./build/surface-sampling-tests data/validation-material.airframe
	./build/aerodynamics-tests data/validation-material.airframe
	./build/core-tests data/validation.airframe
	./build/mechanics-review data/validation.airframe
	./build/propeller-tests data/validation-bem.airframe
	./build/detachment-tests data/validation-bem.airframe
	./build/material-tests data/validation-material.airframe data/validation-bem.airframe
	./build/control-tests data/validation-material.airframe

clean:
	rm -rf build
