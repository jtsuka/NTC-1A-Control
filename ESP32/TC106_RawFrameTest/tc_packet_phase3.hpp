#pragma once
#include <Arduino.h>

namespace tc {

// --- Raspberry Pi 通信用の固定長定義 ---
constexpr uint8_t PI_LEN_SEND  = 6;   // 通常送信
constexpr uint8_t PI_LEN_SENS  = 8;   // センサ情報
constexpr uint8_t PI_LEN_RESET = 12;  // リセット/初期化
constexpr uint8_t PI_MAX       = 12;  // 最大パケット長

// --- TC106 内部処理用の定義 ---
constexpr uint8_t TC_DATA_LEN  = 5;
constexpr uint8_t RING_SIZE    = 64;  // 2の累乗前提

static inline uint8_t checksum7(const uint8_t* d, uint8_t n) {
  uint16_t s = 0;
  for (uint8_t i = 0; i < n; i++) s += (uint8_t)d[i];
  return static_cast<uint8_t>(s & 0x7F);
}

struct Packet {
  uint8_t buf[PI_MAX]{};
  uint8_t len{0};

  uint8_t cmd() const {
    return (len > 0) ? (buf[0] & 0x07) : 0;
  }

  void dumpTo(Stream& s) const {
    for (uint8_t i = 0; i < len; i++) {
      if (buf[i] < 0x10) s.print('0');
      s.print(buf[i], HEX);
      if (i + 1 != len) s.print(' ');
    }
  }
};

static inline uint64_t signature(const uint8_t* p, uint8_t len) {
  if (len < 2) return 0;
  uint64_t sig = 0;
  sig |= (uint64_t)len << 32;
  sig |= (uint64_t)p[len - 1] << 24;
  sig |= (uint64_t)p[0] << 16;
  sig |= (uint64_t)p[1] << 8;
  sig |= (uint64_t)p[len - 2];
  return sig;
}

class PacketFactory {
public:
  static bool tryParse(const uint8_t* ring, uint8_t head, uint64_t& lastSig, Packet& out) {
    const uint8_t lens[] = { PI_LEN_RESET, PI_LEN_SENS, PI_LEN_SEND };

    for (uint8_t l : lens) {
      uint8_t tmp[PI_MAX]{};

      for (uint8_t i = 0; i < l; i++) {
        tmp[i] = ring[(uint8_t)(head - l + i) & (RING_SIZE - 1)];
      }

      if (checksum7(tmp, l - 1) == tmp[l - 1]) {
        uint64_t sig = signature(tmp, l);
        if (sig != lastSig) {
          memcpy(out.buf, tmp, l);
          out.len = l;
          lastSig = sig;
          return true;
        }
      }
    }

    return false;
  }
};

struct TcFrames {
  uint8_t data[TC_DATA_LEN]{};
  uint8_t cmd{0};
  bool isTruncated{false};
};

static inline TcFrames toTcFrames(const Packet& p) {
  TcFrames out;
  out.cmd = (p.len > 0) ? p.buf[0] : 0;

  for (uint8_t i = 0; i < TC_DATA_LEN; i++) {
    out.data[i] = (1 + i < p.len) ? p.buf[1 + i] : 0;
  }

  out.isTruncated = (p.len > PI_LEN_SEND);
  return out;
}

struct Fixed6 {
  static constexpr uint8_t LEN = 6;
  uint8_t b[LEN]{};

  void dumpTo(Stream& s) const {
    for (uint8_t i = 0; i < LEN; i++) {
      if (b[i] < 0x10) s.print('0');
      s.print(b[i], HEX);
      if (i + 1 != LEN) s.print(' ');
    }
  }
};

} // namespace tc
