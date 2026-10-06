// mutablelib — tests/wav.h
//
// Minimal mono 32-bit float WAV writer for tests and examples. Not part of
// the library.
//
// A WAV file is a 44-byte header describing the format, followed by the raw
// samples. We write the header fields one at a time in little-endian order
// (the byte order WAV requires and every desktop CPU uses), so there is no
// struct-packing guesswork.

#ifndef ML_TESTS_WAV_H_
#define ML_TESTS_WAV_H_

#include <cstdint>
#include <cstdio>
#include <vector>

inline bool writeWav(const char* path, const std::vector<float>& samples,
                     uint32_t sampleRate) {
  FILE* f = std::fopen(path, "wb");
  if (!f) return false;
  auto u32 = [f](uint32_t v) { std::fwrite(&v, 4, 1, f); };
  auto u16 = [f](uint16_t v) { std::fwrite(&v, 2, 1, f); };
  uint32_t dataBytes = static_cast<uint32_t>(samples.size() * 4);
  std::fwrite("RIFF", 1, 4, f);
  u32(36 + dataBytes);       // size of everything after this field
  std::fwrite("WAVEfmt ", 1, 8, f);
  u32(16);                   // size of the format chunk
  u16(3);                    // format 3 = IEEE float
  u16(1);                    // channels
  u32(sampleRate);
  u32(sampleRate * 4);       // bytes per second
  u16(4);                    // bytes per frame
  u16(32);                   // bits per sample
  std::fwrite("data", 1, 4, f);
  u32(dataBytes);
  std::fwrite(samples.data(), 4, samples.size(), f);
  std::fclose(f);
  return true;
}

#endif  // ML_TESTS_WAV_H_
