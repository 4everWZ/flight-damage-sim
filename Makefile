CXX ?= g++
CXXFLAGS ?= -std=c++20 -O2 -g -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude
CORE = src/core.cpp src/propeller.cpp
CORE_OBJECTS = $(patsubst src/%.cpp,build/%.o,$(CORE))
BUILD_CONFIG = build/.compile-config
CORE_DEPS = $(CORE_OBJECTS) $(BUILD_CONFIG)

# Compare effective settings without evaluating their contents as shell code.
# Only a changed/missing record is phony: unchanged builds remain incremental,
# while configuration changes also relink existing public executable paths.
sh_quote = '$(subst ','"'"',$(1))'
BUILD_CONFIG_VALUES = $(call sh_quote,CXX=$(CXX)) $(call sh_quote,CPPFLAGS=$(CPPFLAGS)) $(call sh_quote,CXXFLAGS=$(CXXFLAGS))
BUILD_CONFIG_CHANGED := $(shell printf '%s\000' $(BUILD_CONFIG_VALUES) | cmp -s - $(BUILD_CONFIG) || printf changed)
ifneq ($(BUILD_CONFIG_CHANGED),)
.PHONY: $(BUILD_CONFIG)
endif

.PHONY: all test clean
all: build/surface-sampling-tests build/sensor-history-probe build/aerodynamics-tests build/sensitivity-probe build/flight-run build/flight-live build/core-tests build/mechanics-review build/propulsion-probe build/propeller-tests build/governor-probe build/detachment-tests build/material-tests build/control-tests build/control-probe build/lagged-control-tests

build:
	mkdir -p build

$(BUILD_CONFIG): | build
	@tmp="$@.$$$$"; \
	  printf '%s\000' $(BUILD_CONFIG_VALUES) > "$$tmp" && \
	  mv -f "$$tmp" $@

build/%.o: src/%.cpp include/flight/core.hpp include/flight/math.hpp $(BUILD_CONFIG) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

build/flight-run: $(CORE_DEPS) src/runner.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) src/runner.cpp -o $@

build/flight-live: $(CORE_DEPS) src/live.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) src/live.cpp -o $@

build/core-tests: $(CORE_DEPS) tests/core_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/core_tests.cpp -o $@

build/mechanics-review: $(CORE_DEPS) tests/mechanics_review.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/mechanics_review.cpp -o $@

build/propulsion-probe: $(CORE_DEPS) tests/propulsion_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/propulsion_probe.cpp -o $@

build/propeller-tests: $(CORE_DEPS) tests/propeller_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/propeller_tests.cpp -o $@

build/governor-probe: $(CORE_DEPS) tests/governor_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/governor_probe.cpp -o $@

build/detachment-tests: $(CORE_DEPS) tests/detachment_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/detachment_tests.cpp -o $@

build/material-tests: $(CORE_DEPS) tests/material_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/material_tests.cpp -o $@

build/control-tests: $(CORE_DEPS) tests/control_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/control_tests.cpp -o $@

build/control-probe: $(CORE_DEPS) tests/control_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/control_probe.cpp -o $@

build/aerodynamics-tests: $(CORE_DEPS) tests/aerodynamics_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/aerodynamics_tests.cpp -o $@

build/sensitivity-probe: $(CORE_DEPS) tests/sensitivity_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/sensitivity_probe.cpp -o $@

build/surface-sampling-tests: $(CORE_DEPS) tests/surface_sampling_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/surface_sampling_tests.cpp -o $@

build/sensor-history-probe: $(CORE_DEPS) tests/sensor_history_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/sensor_history_probe.cpp -o $@

build/finite-span-probe: $(CORE_DEPS) tests/finite_span_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/finite_span_probe.cpp -o $@

build/section-controls-probe: $(CORE_DEPS) tests/section_controls_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/section_controls_probe.cpp -o $@

build/control-transition-probe: $(CORE_DEPS) tests/control_transition_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/control_transition_probe.cpp -o $@

build/build-config-probe: $(CORE_DEPS) tests/build_config_probe.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/build_config_probe.cpp -o $@

build/lagged-control-tests: $(CORE_DEPS) tests/lagged_control_tests.cpp include/flight/core.hpp include/flight/math.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE_OBJECTS) tests/lagged_control_tests.cpp -o $@

test: all
	./build/surface-sampling-tests data/validation-material.airframe
	./build/aerodynamics-tests data/validation-material.airframe
	./build/core-tests data/validation.airframe
	./build/mechanics-review data/validation.airframe
	./build/propeller-tests data/validation-bem.airframe
	./build/detachment-tests data/validation-bem.airframe
	./build/material-tests data/validation-material.airframe data/validation-bem.airframe
	./build/control-tests data/validation-material.airframe
	mkdir -p results
	./build/lagged-control-tests data/validation-material.airframe results/lagged-control-summary.json

clean:
	rm -rf build
