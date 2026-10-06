# Build with: make RACK_DIR=/path/to/Rack-SDK (or export RACK_DIR first).
# See docs/setup.html for setting up the build tools and the Rack SDK.
RACK_DIR ?= ../../../Rack-SDK

# mutablelib is header-only: adding its include directory is all it takes.
FLAGS += -I../../include

SOURCES += $(wildcard src/*.cpp)
DISTRIBUTABLES += res plugin.json

include $(RACK_DIR)/plugin.mk
