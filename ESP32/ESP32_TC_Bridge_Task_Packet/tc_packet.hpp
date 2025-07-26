// =============================================================
// tc_packet.hpp  (place in same folder as .ino)
// =============================================================
#pragma once
#include <Arduino.h>

namespace tc {
/* ------------------------------------------------------------------
 *  Protocol constants (6‑byte SEND / 8‑byte SENS.ADJ / 12‑byte RESET)
 * ----------------------------------------------------------------*/
constexpr uint8_t  FRAME_LEN_SEND   = 6;
constexpr uint8_t  FRAME_LEN_SENS   = 8;
constexpr uint8_t  FRAME_LEN_RESET  = 12;
constexpr uint8_t  FRAME_MAX        = FRAME_LEN_RESET;
constexpr uint8_t  RING_SIZE        = 64;      // local ring‑buffer size

/* Command ID = byte0 LSB3 (参考) */
enum TcCmd : uint8_t { CMD_RESET=1, CMD_SENS_ADJ=2, CMD_SEND=3, CMD_SETTING=5, CMD_CONT_CONST=6 };

/* ---------------- utils ---------------- */
inline uint8_t checksum7(const uint8_t* d, size_t n) {
    uint8_t s = 0;  for(size_t i=0;i<n;++i) s += d[i];  return s & 0x7F;
}
inline uint8_t rev8(uint8_t v){
    v=(v>>4)|(v<<4);
    v=((v&0xCC)>>2)|((v&0x33)<<2);
    v=((v&0xAA)>>1)|((v&0x55)<<1);
    return v;
}

/* ---------------- packet struct -------- */
struct Packet{
    uint8_t buf[FRAME_MAX];
    uint8_t len{0};
    uint8_t cmd() const {
        return buf[0] & 0x07;
        }
    void toBytes(uint8_t* out,bool doRev=false) const{
        for(uint8_t i=0;i<len;++i)
            out[i]=doRev?rev8(buf[i]):buf[i];
        }
};

/* ---------------- packet factory -------- */
class PacketFactory{
public:
    // ring[writeIdx] に書き込む直前の writeIdx を渡す (0‑63)
    static Packet* tryParse(const uint8_t* ring, uint8_t writeIdx){
        static const uint8_t lens[] = {FRAME_LEN_SEND, FRAME_LEN_SENS, FRAME_LEN_RESET};
        static Packet pkt;
        uint8_t tmp[FRAME_MAX];
        for(uint8_t len: lens){
            /* len バイト取り出し → tmp[] */
            for(uint8_t i=0;i<len;++i){
                uint8_t pos = (writeIdx + RING_SIZE - len + i) & (RING_SIZE-1);
                tmp[i] = ring[pos];
            }
            if(checksum7(tmp,len-1) == tmp[len-1]){
                memcpy(pkt.buf,tmp,len); pkt.len=len; return &pkt;
            }
        }
        return nullptr;
    }
};

} // namespace tc