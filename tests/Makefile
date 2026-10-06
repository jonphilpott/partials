# mutablelib tests. No framework: each test_*.cpp is its own program that
# returns non-zero on failure.
#
#   make check   build and run every test, then check the docs cover every header

CXX ?= c++
CXXFLAGS = -std=c++11 -O2 -Wall -Wextra -Werror -I../include

TESTS := $(patsubst %.cpp,build/%,$(wildcard test_*.cpp))

check: $(TESTS) docs-check
	@for t in $(TESTS); do ./$$t || exit 1; done

# Rebuild a test when its source or any library header changes.
build/%: %.cpp test.h wav.h $(shell find ../include -name '*.h')
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $< -o $@

# Docs must cover every header and core function, each with an example.
docs-check:
	@./docs_check.sh

clean:
	rm -rf build

.PHONY: check docs-check clean
