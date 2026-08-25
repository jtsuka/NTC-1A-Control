#pragma once
#include <Arduino.h>
#include "tc_packet_phase3.hpp"

namespace tc {

enum class MsgSource : uint8_t {
  Pi     = 1,
  Tc     = 2,
  System = 3,
};

enum class MsgType : uint8_t {
  Command  = 1,
  Response = 2,
  Timeout  = 3,
  Error    = 4,
  Log      = 5,
};

enum MsgFlags : uint8_t {
  FLAG_NONE            = 0x00,
  FLAG_CHECKSUM_OK     = 0x01,
  FLAG_TIMEOUT         = 0x02,
  FLAG_TRUNCATED       = 0x04,
  FLAG_RECOVERY_RESET  = 0x08, // Stage5: fault中に受理したrecovery RESETを識別
};

struct TcMessage {
  uint32_t timestamp_us{0};
  MsgSource source{MsgSource::System};
  MsgType type{MsgType::Log};
  uint8_t len{0};
  uint8_t data[PI_MAX]{};
  uint8_t flags{FLAG_NONE};

  static TcMessage fromPacket(const Packet& p, MsgSource src, MsgType typ) {
    TcMessage m;
    m.timestamp_us = micros();
    m.source = src;
    m.type = typ;
    m.len = p.len;
    memcpy(m.data, p.buf, p.len);
    m.flags = FLAG_CHECKSUM_OK;
    return m;
  }

  static TcMessage timeout(MsgSource src) {
    TcMessage m;
    m.timestamp_us = micros();
    m.source = src;
    m.type = MsgType::Timeout;
    m.len = 6;
    memset(m.data, 0, m.len);
    m.flags = FLAG_TIMEOUT;
    return m;
  }

  void dumpTo(Stream& s) const {
    for (uint8_t i = 0; i < len; i++) {
      if (data[i] < 0x10) s.print('0');
      s.print(data[i], HEX);
      if (i + 1 != len) s.print(' ');
    }
  }
};

} // namespace tc
