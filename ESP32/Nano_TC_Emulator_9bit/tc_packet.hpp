#pragma once
#include <Arduino.h>

namespace tc {
    // パケット長定義
    constexpr uint8_t FRAME_LEN_SEND = 6;
    constexpr uint8_t FRAME_LEN_SENS = 8;
    constexpr uint8_t FRAME_LEN_RESET = 12;
    constexpr uint8_t FRAME_MAX = 12;
    constexpr uint8_t RING_SIZE = 64;

    // 7bit チェックサム計算
    static inline uint8_t checksum7(const uint8_t* data, uint8_t len) {
        uint16_t s = 0;
        for (uint8_t i = 0; i < len; i++) s += data[i];
        return (uint8_t)(s & 0x7F);
    }

    struct Packet {
        uint8_t buf[FRAME_MAX];
        uint8_t len{0};
        uint8_t cmd() const { return buf[0] & 0x07; }
    };

    class PacketFactory {
    public:
        static Packet* tryParse(const uint8_t* ring, uint8_t writeIdx) {
            static Packet pkt;
            const uint8_t lens[] = { 12, 8, 6 };
            for (uint8_t len : lens) {
                uint8_t tmp[FRAME_MAX];
                for (uint8_t i = 0; i < len; i++) {
                    tmp[i] = ring[(writeIdx - len + i + RING_SIZE) & (RING_SIZE - 1)];
                }
                if (checksum7(tmp, len - 1) == tmp[len - 1]) {
                    memcpy(pkt.buf, tmp, len);
                    pkt.len = len;
                    return &pkt;
                }
            }
            return nullptr;
        }
    };
}