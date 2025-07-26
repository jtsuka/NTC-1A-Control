/*********************************************************************
 * tc_packet.hpp  –  Packet Framework (6-/2-/1-byte)   C++17
 *********************************************************************/
#pragma once
#include <Arduino.h>
#include <memory>

namespace tc {

enum class PktType : uint8_t { LENGTH_FORCE=0x80, FORCE_ONLY=0x81, RESET_CMD=0x82 };

inline uint8_t rev8(uint8_t b){
  b=(b&0xF0)>>4 | (b&0x0F)<<4; b=(b&0xCC)>>2 | (b&0x33)<<2; b=(b&0xAA)>>1 | (b&0x55)<<1; return b;
}
inline uint8_t calcChk(const uint8_t* d,size_t n){ uint16_t s=0;for(size_t i=0;i<n;++i)s+=d[i];return uint8_t(s&0x7F);}

class PacketBase{
public:
  virtual ~PacketBase()=default;
  virtual PktType type() const=0;
  virtual size_t  len()  const=0;          // 含CHK
  virtual void    fillRaw(uint8_t* raw)const=0;      // rev8 前データ
  /* rev8 の有無を切替できる toBytes */
  void toBytes(uint8_t* out,bool doRev=true)const{
    uint8_t r[8]; fillRaw(r);
    for(size_t i=0;i<len()-1;++i) out[i]= doRev? rev8(r[i]):r[i];
    out[len()-1]= doRev? rev8(calcChk(r,len()-1)): calcChk(r,len()-1);
  }
  bool isValid()const{
    uint8_t r[8]; fillRaw(r);
    return calcChk(r,len()-1)==r[len()-1];
  }
};

/* --- 6 byte --- */
class PktLenForce:public PacketBase{
  uint8_t h,m,l,f;
public:
  PktLenForce(uint32_t len,uint8_t force):h(len>>16),m(len>>8),l(len),f(force){}
  PktType type()const override{return PktType::LENGTH_FORCE;}
  size_t  len() const override{return 6;}
  void fillRaw(uint8_t*r)const override{ r[0]=uint8_t(type()); r[1]=h;r[2]=m;r[3]=l;r[4]=f; }
};
/* --- 2 byte --- */
class PktForceOnly:public PacketBase{
  uint8_t f;
public:
  explicit PktForceOnly(uint8_t force):f(force){}
  PktType type()const override{return PktType::FORCE_ONLY;}
  size_t  len() const override{return 2;}
  void fillRaw(uint8_t*r)const override{ r[0]=uint8_t(type()); r[1]=f; }
};
/* --- 1 byte --- */
class PktReset:public PacketBase{
public:
  PktType type()const override{return PktType::RESET_CMD;}
  size_t  len() const override{return 1;}
  void fillRaw(uint8_t*r)const override{ r[0]=uint8_t(type()); }
};

/* -------- Factory（バイト列から派生生成） -------- */
class PacketFactory{
public:
  static std::unique_ptr<PacketBase> tryParse(uint8_t*buf,size_t& idx){
    while(idx){
      switch(buf[0]){
        case 0x80: if(idx>=6 && calcChk(buf,5)==buf[5]){auto p=std::make_unique<PktLenForce>((buf[1]<<16)|(buf[2]<<8)|buf[3],buf[4]); idx-=6; memmove(buf,buf+6,idx); return p;} break;
        case 0x81: if(idx>=2 && calcChk(buf,1)==buf[1]){auto p=std::make_unique<PktForceOnly>(buf[1]); idx-=2; memmove(buf,buf+2,idx); return p;} break;
        case 0x82:                       {auto p=std::make_unique<PktReset>(); idx-=1; memmove(buf,buf+1,idx); return p;} 
        default : /*ゴミバイト*/         idx--; memmove(buf,buf+1,idx); break;
      }
      break;
    }
    return nullptr;
  }
};

} // namespace tc