CXX ?= g++
CXXFLAGS ?= -std=c++20 -O2 -g -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude
CORE = src/core.cpp src/propeller.cpp
CORE_OBJECTS = $(patsubst src/%.cpp,build/%.o,$(CORE))

.PHONY: all test clean
all: build/surface-sampling-tests build/sensor-history-probe build/aerodynamics-tests build/sensitivity-probe build/flight-run build/flight-live build/core-tests build/mechanics-review build/propulsion-probe build/propeller-tests build/governor-probe build/detachment-tests build/material-tests build/control-tests build/control-probe

build:
	mkdir -p build

build/%.o: src/%.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

build/flight-run: $(CORE_OBJECTS) src/runner.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) src/runner.cpp -o $@

build/flight-live: $(CORE_OBJECTS) src/live.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) src/live.cpp -o $@

build/core-tests: $(CORE_OBJECTS) tests/core_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/core_tests.cpp -o $@

build/mechanics-review: $(CORE_OBJECTS) tests/mechanics_review.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/mechanics_review.cpp -o $@

build/propulsion-probe: $(CORE_OBJECTS) tests/propulsion_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/propulsion_probe.cpp -o $@

build/propeller-tests: $(CORE_OBJECTS) tests/propeller_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/propeller_tests.cpp -o $@

build/governor-probe: $(CORE_OBJECTS) tests/governor_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/governor_probe.cpp -o $@

build/detachment-tests: $(CORE_OBJECTS) tests/detachment_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/detachment_tests.cpp -o $@

build/material-tests: $(CORE_OBJECTS) tests/material_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/material_tests.cpp -o $@

build/control-tests: $(CORE_OBJECTS) tests/control_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/control_tests.cpp -o $@

build/control-probe: $(CORE_OBJECTS) tests/control_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/control_probe.cpp -o $@

build/aerodynamics-tests: $(CORE_OBJECTS) tests/aerodynamics_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/aerodynamics_tests.cpp -o $@

build/sensitivity-probe: $(CORE_OBJECTS) tests/sensitivity_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/sensitivity_probe.cpp -o $@

build/surface-sampling-tests: $(CORE_OBJECTS) tests/surface_sampling_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/surface_sampling_tests.cpp -o $@

build/sensor-history-probe: $(CORE_OBJECTS) tests/sensor_history_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/sensor_history_probe.cpp -o $@

build/finite-span-probe: $(CORE_OBJECTS) tests/finite_span_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/finite_span_probe.cpp -o $@

build/section-controls-probe: $(CORE_OBJECTS) tests/section_controls_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/section_controls_probe.cpp -o $@

build/control-transition-probe: $(CORE_OBJECTS) tests/control_transition_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/control_transition_probe.cpp -o $@

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
