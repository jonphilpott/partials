// Tests for the sample/ headers.

#include "pt/sample/audio_buffer.h"
#include "pt/sample/wav.h"

#include <string>

#include "test.h"
#include "wav.h"

namespace {

// Build a WAV file in memory: header, a "fmt " chunk (plain or extensible),
// optionally an odd-sized chunk to skip, then the raw sample bytes.
std::vector<uint8_t> makeWav(int format, int bits, int channels, uint32_t rate,
                             const std::vector<uint8_t>& samples,
                             bool extensible = false, bool junk = false) {
  std::vector<uint8_t> w;
  auto u16 = [&w](uint32_t v) { w.push_back(v & 0xFF); w.push_back((v >> 8) & 0xFF); };
  auto u32 = [&](uint32_t v) { u16(v & 0xFFFF); u16(v >> 16); };
  auto id = [&w](const char* s) { w.insert(w.end(), s, s + 4); };
  int block = channels * bits / 8;
  id("RIFF"); u32(0); id("WAVE");  // RIFF size isn't checked
  id("fmt "); u32(extensible ? 40 : 16);
  u16(extensible ? 0xFFFE : format); u16(channels); u32(rate);
  u32(rate * block); u16(block); u16(bits);
  if (extensible) {
    u16(22); u16(bits); u32(0);           // cbSize, valid bits, channel mask
    u16(format); for (int i = 0; i < 14; ++i) w.push_back(0);  // GUID
  }
  if (junk) { id("LIST"); u32(3); w.push_back(1); w.push_back(2); w.push_back(3); w.push_back(0); }
  id("data"); u32(static_cast<uint32_t>(samples.size()));
  w.insert(w.end(), samples.begin(), samples.end());
  return w;
}

bool decode(const std::vector<uint8_t>& w, pt::AudioBuffer& b) {
  return pt::decodeWav(w.data(), w.size(), b);
}

}  // namespace

int main() {
  // --- AudioBuffer -------------------------------------------------------
  pt::AudioBuffer b;
  b.init(8, 1, 44100.0f);
  CHECK(b.frames() == 8 && b.channels() == 1 && b.sampleRate() == 44100.0f);
  for (size_t i = 0; i < 8; ++i) b.write(0, i, static_cast<float>(i));
  b.write(0, 99, 5.0f);  // out of range: ignored
  CHECK_NEAR(b.read(0, 3.0), 3.0f, 1e-6f);
  CHECK_NEAR(b.read(0, 2.5), 2.5f, 1e-6f);   // Hermite is exact on a line
  CHECK_NEAR(b.read(5, 3.0), 3.0f, 1e-6f);   // channel clamps to the last
  CHECK_NEAR(b.read(0, -4.0), 0.0f, 1e-6f);  // one-shot clamps to the ends
  CHECK_NEAR(b.read(0, 20.0), 7.0f, 1e-6f);

  // Looping: reads wrap, and the step from frame 7 back to frame 0 sees
  // both neighbours through the guards.
  b.setLooping(true);
  CHECK_NEAR(b.read(0, 11.0), 3.0f, 1e-6f);
  CHECK_NEAR(b.read(0, -1.0), 7.0f, 1e-6f);
  float across = b.read(0, 7.5);
  CHECK(across > 0.0f && across < 7.0f);
  // Writing frame 0 while looping updates the guard after frame 7.
  b.write(0, 0, 7.0f);
  CHECK_NEAR(b.channel(0)[8], 7.0f, 1e-6f);
  b.setLooping(false);
  CHECK_NEAR(b.channel(0)[8], 0.0f, 1e-6f);
  CHECK_NEAR(b.channel(0)[-1], 0.0f, 1e-6f);

  // Clear a range; positions past the end are clamped.
  b.clear(6, 100);
  CHECK_NEAR(b.read(0, 7.0), 0.0f, 1e-6f);
  CHECK_NEAR(b.read(0, 5.0), 5.0f, 1e-6f);

  // Long buffers: a double position keeps the fraction exact. As a float,
  // 1000000.3 would round to 1000000.3125 and read a different value.
  pt::AudioBuffer zigzag;
  zigzag.init(1 << 20);
  for (size_t i = 0; i < zigzag.frames(); ++i) zigzag.write(0, i, static_cast<float>(i % 2));
  CHECK_NEAR(zigzag.read(0, 1000000.3), zigzag.read(0, 2.3), 1e-6f);

  // --- WAV decoding ------------------------------------------------------
  pt::AudioBuffer w;

  // 16-bit stereo: de-interleaved, scaled to ±1.
  CHECK(decode(makeWav(1, 16, 2, 48000, {0x00, 0x40, 0x00, 0x80, 0x00, 0xC0, 0xFF, 0x7F}), w));
  CHECK(w.frames() == 2 && w.channels() == 2 && w.sampleRate() == 48000.0f);
  CHECK_NEAR(w.channel(0)[0], 0.5f, 1e-6f);
  CHECK_NEAR(w.channel(1)[0], -1.0f, 1e-6f);
  CHECK_NEAR(w.channel(0)[1], -0.5f, 1e-6f);
  CHECK_NEAR(w.channel(1)[1], 1.0f, 1e-4f);

  // 8-bit is unsigned.
  CHECK(decode(makeWav(1, 8, 1, 8000, {192, 0, 128}), w));
  CHECK_NEAR(w.channel(0)[0], 0.5f, 1e-6f);
  CHECK_NEAR(w.channel(0)[1], -1.0f, 1e-6f);
  CHECK_NEAR(w.channel(0)[2], 0.0f, 1e-6f);

  // 24-bit, with sign extension, inside an extensible header.
  CHECK(decode(makeWav(1, 24, 1, 96000, {0x00, 0x00, 0x40, 0x00, 0x00, 0xC0}, true), w));
  CHECK(w.frames() == 2);
  CHECK_NEAR(w.channel(0)[0], 0.5f, 1e-6f);
  CHECK_NEAR(w.channel(0)[1], -0.5f, 1e-6f);

  // 32-bit integer.
  CHECK(decode(makeWav(1, 32, 1, 48000, {0x00, 0x00, 0x00, 0xC0}), w));
  CHECK_NEAR(w.channel(0)[0], -0.5f, 1e-6f);

  // 32-bit float, after an odd-sized chunk that must be skipped with its pad.
  {
    float f = 0.25f;
    uint8_t raw[4];
    std::memcpy(raw, &f, 4);
    CHECK(decode(makeWav(3, 32, 1, 48000, {raw[0], raw[1], raw[2], raw[3]}, false, true), w));
    CHECK_NEAR(w.channel(0)[0], 0.25f, 1e-6f);
  }

  // 64-bit float.
  {
    double d = -0.125;
    uint8_t raw[8];
    std::memcpy(raw, &d, 8);
    CHECK(decode(makeWav(3, 64, 1, 48000, std::vector<uint8_t>(raw, raw + 8)), w));
    CHECK_NEAR(w.channel(0)[0], -0.125f, 1e-6f);
  }

  // Truncated data: decode the whole frames that are there.
  {
    std::vector<uint8_t> t = makeWav(1, 16, 1, 48000, {0x00, 0x40, 0x00, 0x40, 0x00, 0x40});
    t.resize(t.size() - 3);
    CHECK(decode(t, w));
    CHECK(w.frames() == 1);
  }

  // Rejected, leaving the buffer untouched.
  CHECK(!decode(makeWav(2, 4, 1, 48000, {0, 0}), w));  // ADPCM
  CHECK(!decode(std::vector<uint8_t>{'R', 'I', 'F', 'X', 0, 0, 0, 0, 'W', 'A', 'V', 'E'}, w));
  CHECK(!decode(std::vector<uint8_t>(), w));
  CHECK(w.frames() == 1);

  // loadWav: round trip through a file written by the tests' own writer.
  {
    std::string path = "build/test_sample.wav";
    CHECK(writeWav(path.c_str(), {0.0f, 0.5f, -0.5f}, 22050));
    pt::AudioBuffer f;
    CHECK(pt::loadWav(path.c_str(), f));
    CHECK(f.frames() == 3 && f.sampleRate() == 22050.0f);
    CHECK_NEAR(f.read(0, 1.0), 0.5f, 1e-6f);
    CHECK(!pt::loadWav("build/no_such_file.wav", f));
  }

  return testResult("sample");
}
