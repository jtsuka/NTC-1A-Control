#pragma once
#include <Arduino.h>

namespace tc {
  constexpr uint8_t PI_LEN_SEND  = 6;
  constexpr uint8_t PI_LEN_SENS  = 8;
  constexpr uint8_t PI_LEN_RESET = 12;
  constexpr uint8_t PI_MAX       = 12;
  constexpr uint8_t TC_DATA_LEN  = 5;
  constexpr uint8_t RING_SIZE    = 64;

  static inline uint8_t checksum7(const uint8_t* d, uint8_t n) {
    uint16_t s = 0;
    for (uint8_t i = 0; i < n; i++) s += d[i];
    return (uint8_t)(s & 0x7F);
  }

  struct Packet {
    uint8_t buf[PI_MAX]{};
    uint8_t len{0};
    uint8_t cmd() const { return buf[0] & 0x07; }
  };

  /**
   * 64bitシグネチャ：内容ベースの二重送信ガード
   * [Length(8bit) | Checksum(8bit) | Buf0(8bit) | Buf1(8bit) | BufLast-1(8bit)]
   */
  static inline uint64_t signature(const uint8_t* p, uint8_t len) {
    if (len < 2) return 0;
    uint64_t sig = 0;
    sig |= (uint64_t)len << 32;
    sig |= (uint64_t)p[len - 1] << 24; // Checksum (末尾)
    sig |= (uint64_t)p[0] << 16;       // CMD (先頭)
    sig |= (uint64_t)p[1] << 8;        // Data0 (2バイト目)
    sig |= (uint64_t)p[len - 2];       // Penultimate (末尾から2番目)
    return sig;
  }

  class PacketFactory {
  public:
    static const Packet* tryParse(const uint8_t* ring, uint8_t head, uint64_t &lastSig) {
      static Packet pkt;
      const uint8_t lens[] = { PI_LEN_RESET, PI_LEN_SENS, PI_LEN_SEND };
      for (uint8_t l : lens) {
        uint8_t tmp[PI_MAX];
        for (uint8_t i = 0; i < l; i++) {
          tmp[i] = ring[(uint8_t)(head - l + i) & (RING_SIZE - 1)];
        }
        if (checksum7(tmp, l - 1) == tmp[l - 1]) {
          uint64_t sig = signature(tmp, l);
          if (sig != lastSig) {
            memcpy(pkt.buf, tmp, l);
            pkt.len = l;
            lastSig = sig;
            return &pkt;
          }
        }
      }
      return nullptr;
    }
  };

  struct TcFrames {
    uint8_t data[TC_DATA_LEN]{};
    uint8_t cmd{0};
    bool isTruncated{false};
  };

  static inline TcFrames toTcFrames(const Packet& p) {
    TcFrames out;
    out.cmd = p.buf[0]; 
    for (uint8_t i = 0; i < TC_DATA_LEN; i++) out.data[i] = p.buf[1 + i];
    out.isTruncated = (p.len > PI_LEN_SEND);
    return out;
  }
}