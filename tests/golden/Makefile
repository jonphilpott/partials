# Golden comparison: compiles Emilie Gillet's original classes next to the
# mutablelib ports and checks that they produce the same output at the
# original sample rate. Needs the eurorack repo and stmlib on disk; it is
# only for verifying ports and is never part of the library.
#
#   make EURORACK=~/git/eurorack STMLIB=/path/to/stmlib

EURORACK ?= $(HOME)/git/eurorack
STMLIB ?= $(EURORACK)/stmlib

CXX ?= c++
CXXFLAGS = -std=c++11 -O2 -DTEST -Wno-everything -Ibuild/inc -I../../include -I..

ORIGINALS = \
  $(EURORACK)/rings/dsp/resonator.cc \
  $(EURORACK)/rings/resources.cc \
  $(EURORACK)/elements/dsp/string.cc \
  $(EURORACK)/elements/dsp/tube.cc \
  $(EURORACK)/elements/resources.cc \
  $(EURORACK)/plaits/resources.cc \
  $(EURORACK)/peaks/drums/fm_drum.cc \
  $(EURORACK)/peaks/resources.cc \
  $(EURORACK)/peaks/modulations/multistage_envelope.cc \
  $(EURORACK)/tides2/ramp/ramp_extractor.cc \
  $(EURORACK)/tides2/poly_slope_generator.cc \
  $(EURORACK)/tides2/resources.cc \
  $(EURORACK)/marbles/random/quantizer.cc \
  $(EURORACK)/marbles/random/lag_processor.cc \
  $(EURORACK)/marbles/resources.cc \
  $(EURORACK)/warps/resources.cc \
  $(EURORACK)/warps/dsp/filter_bank.cc \
  $(EURORACK)/warps/dsp/vocoder.cc \
  $(EURORACK)/warps/dsp/oscillator.cc \
  $(EURORACK)/plaits/dsp/fm/algorithms.cc \
  $(STMLIB)/dsp/units.cc \
  $(STMLIB)/utils/random.cc

check: build/golden
	./build/golden

# The originals include "stmlib/...", "rings/..." etc., so point a
# directory of symlinks at the real locations.
build/inc:
	mkdir -p build/inc
	ln -sfn $(STMLIB) build/inc/stmlib
	for m in rings elements plaits clouds warps tides2 marbles stages streams peaks frames; do ln -sfn $(EURORACK)/$$m build/inc/$$m; done

build/golden: golden.cpp golden_drums.cpp golden_mod.cpp golden_osc.cpp golden_spectral.cpp golden.h ../test.h $(shell find ../../include -name '*.h') | build/inc
	$(CXX) $(CXXFLAGS) golden.cpp golden_drums.cpp golden_mod.cpp golden_osc.cpp golden_spectral.cpp $(ORIGINALS) -o $@

clean:
	rm -rf build

.PHONY: check clean
