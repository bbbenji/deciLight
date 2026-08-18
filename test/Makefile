# deciLight host tests
#
#   make          build and run the logic tests
#   make syntax   type-check every firmware module against the stubs
#   make check    both
#   make clean
#
# No ESP32 toolchain required. See README.md for what this does and does not
# cover.

CXX      ?= g++
CXXFLAGS ?= -std=gnu++11 -O1 -g -Wall -Wextra -Wshadow
INCLUDES := -I stubs -I . -I ..

BUILD := build
BIN   := $(BUILD)/run-tests

# Firmware modules that are free of hardware dependencies and so can run here.
FIRMWARE := ../settings.cpp ../signal_light.cpp ../remote_control.cpp \
            ../display.cpp ../group_sync.cpp
TESTS    := main.cpp fakes.cpp test_settings.cpp test_signal_light.cpp \
            test_remote_control.cpp test_display.cpp test_group_sync.cpp

# Every module that should at least type-check. sound_level.cpp is included
# for the syntax pass only: its filter kernel is Xtensa assembly, so it cannot
# be linked on a host. web_control.cpp is absent because stubbing WiFi and
# WebServer would be a lot of surface for no logic worth testing - the real
# arduino-cli build in CI compiles it on every push.
SYNTAX := $(FIRMWARE) ../sound_level.cpp

.PHONY: all check run syntax clean
all: run

$(BUILD):
	@mkdir -p $(BUILD)

$(BIN): $(TESTS) $(FIRMWARE) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(TESTS) $(FIRMWARE)

run: $(BIN)
	@./$(BIN)

syntax: | $(BUILD)
	@for f in $(SYNTAX); do \
	  printf '  %-24s' "$$(basename $$f)"; \
	  $(CXX) $(CXXFLAGS) -Wno-sign-compare $(INCLUDES) -fsyntax-only "$$f" \
	    2>$(BUILD)/syntax.log && echo OK || { echo FAIL; cat $(BUILD)/syntax.log; exit 1; }; \
	done
	@printf '  %-24s' "deciLight.ino"; \
	 { echo '#include <Arduino.h>'; cat ../deciLight.ino; } > $(BUILD)/ino.cpp; \
	 $(CXX) $(CXXFLAGS) $(INCLUDES) -fsyntax-only $(BUILD)/ino.cpp \
	   2>$(BUILD)/syntax.log && echo OK || { echo FAIL; cat $(BUILD)/syntax.log; exit 1; }

check: syntax run

clean:
	@rm -rf $(BUILD)
