// partials — sample/wav.h
//
// Load WAV files into an AudioBuffer: Pure Data's soundfiler.
//
// A WAV file is a "RIFF" container: a 12-byte header ("RIFF", a size,
// "WAVE") followed by chunks. Each chunk is a 4-letter id, a 32-bit size
// and that many bytes, plus one padding byte if the size is odd. Two chunks
// matter here:
// - "fmt " says how the samples are stored: the format code (1 = integer
//   PCM, 3 = float, 0xFFFE = "extensible", which keeps the real code in
//   the first two bytes of a GUID further in), channel count, sample rate,
//   bytes per frame and bits per sample.
// - "data" holds the samples, interleaved: L R L R ...
// Every other chunk (LIST metadata, cue points, ...) is skipped.
//
// Supported: integer PCM at 8, 16, 24 and 32 bits, float at 32 and 64
// bits, any channel count, plain or extensible. Not supported: compressed
// formats (ADPCM, mu-law, ...), big-endian RIFX, AIFF, FLAC.
//
// Every value is read byte by byte, least significant first (WAV is little
// endian), so the code never casts a misaligned pointer and works whatever
// the CPU's own byte order.
//
// Decoding allocates the buffer, so call it from the interface thread (a
// menu item or file dialog), never from process(). Hand the finished
// buffer to the audio thread by swapping a pointer.

#ifndef PT_SAMPLE_WAV_H_
#define PT_SAMPLE_WAV_H_

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "pt/sample/audio_buffer.h"

namespace pt {

namespace wav_detail {

inline uint32_t u16(const uint8_t* p) { return p[0] | (p[1] << 8); }
inline uint32_t u32(const uint8_t* p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// One sample to -1..1. Integer formats divide by 2^(bits-1): full-scale
// negative is exactly -1, full-scale positive a hair under +1.
inline float sample(const uint8_t* p, int format, int bits) {
  if (format == 3) {
    if (bits == 32) {
      uint32_t u = u32(p);
      float f;
      std::memcpy(&f, &u, 4);  // reinterpret the bits as a float
      return f;
    }
    uint64_t u = u32(p) | (static_cast<uint64_t>(u32(p + 4)) << 32);
    double d;
    std::memcpy(&d, &u, 8);
    return static_cast<float>(d);
  }
  switch (bits) {
    case 8: return (p[0] - 128) / 128.0f;  // 8-bit WAV alone is unsigned
    case 16: return static_cast<int16_t>(u16(p)) / 32768.0f;
    // 24-bit: put the three bytes at the top of an int32 so the sign bit
    // lands in place, then divide by 2^31.
    case 24: {
      uint32_t u = (p[0] << 8) | (p[1] << 16) | (static_cast<uint32_t>(p[2]) << 24);
      return static_cast<int32_t>(u) / 2147483648.0f;
    }
    default: return static_cast<int32_t>(u32(p)) / 2147483648.0f;
  }
}

}  // namespace wav_detail

// Decode a whole WAV file held in memory into `out` (re-initialised, one-
// shot). Returns false, leaving `out` untouched, if the data is not a WAV
// file or uses a format listed above as unsupported.
inline bool decodeWav(const uint8_t* bytes, size_t size, AudioBuffer& out) {
  using namespace wav_detail;
  // 1. The RIFF header.
  if (size < 12 || std::memcmp(bytes, "RIFF", 4) != 0 || std::memcmp(bytes + 8, "WAVE", 4) != 0)
    return false;

  // 2. Walk the chunks, noting the format and where the samples are.
  int format = 0, channels = 0, bits = 0;
  uint32_t rate = 0, blockAlign = 0;
  const uint8_t* data = nullptr;
  size_t dataSize = 0;
  for (size_t pos = 12; pos + 8 <= size;) {
    const uint8_t* id = bytes + pos;
    size_t chunk = u32(bytes + pos + 4);
    const uint8_t* body = bytes + pos + 8;
    size_t available = size - pos - 8;
    if (std::memcmp(id, "fmt ", 4) == 0 && chunk >= 16 && available >= 16) {
      format = u16(body);
      channels = u16(body + 2);
      rate = u32(body + 4);
      blockAlign = u16(body + 12);
      bits = u16(body + 14);
      if (format == 0xFFFE && chunk >= 26 && available >= 26) format = u16(body + 24);
    } else if (std::memcmp(id, "data", 4) == 0) {
      data = body;
      // Truncated files (and streaming writers that never fill the size
      // in) claim more than is there: take what actually exists.
      dataSize = chunk < available ? chunk : available;
    }
    if (chunk >= available) break;  // last chunk (or a corrupt size)
    pos += 8 + chunk + (chunk & 1);  // odd-sized chunks carry a pad byte
  }

  // 3. Check it's something we can read before touching `out`.
  bool intOk = format == 1 && (bits == 8 || bits == 16 || bits == 24 || bits == 32);
  bool floatOk = format == 3 && (bits == 32 || bits == 64);
  int bytesPerSample = bits / 8;
  if (!(intOk || floatOk) || !data || channels < 1 || rate == 0 ||
      blockAlign < static_cast<uint32_t>(channels * bytesPerSample))
    return false;

  // 4. De-interleave into the buffer, one channel array each.
  size_t frames = dataSize / blockAlign;
  out.init(frames, channels, static_cast<float>(rate));
  for (int c = 0; c < channels; ++c) {
    float* dst = out.channel(c);
    const uint8_t* src = data + c * bytesPerSample;
    for (size_t f = 0; f < frames; ++f, src += blockAlign) dst[f] = sample(src, format, bits);
  }
  return true;
}

// Read a file with std::fopen, then decodeWav(). On Windows, fopen can't
// open paths with non-ASCII characters; in Rack, read the file with
// system::readFile() and call decodeWav() instead.
inline bool loadWav(const char* path, AudioBuffer& out) {
  FILE* f = std::fopen(path, "rb");
  if (!f) return false;
  std::vector<uint8_t> bytes;
  if (std::fseek(f, 0, SEEK_END) == 0) {
    long size = std::ftell(f);
    if (size > 0) {
      bytes.resize(static_cast<size_t>(size));
      std::rewind(f);
      bytes.resize(std::fread(bytes.data(), 1, bytes.size(), f));
    }
  }
  std::fclose(f);
  return decodeWav(bytes.data(), bytes.size(), out);
}

}  // namespace pt

#endif  // PT_SAMPLE_WAV_H_
