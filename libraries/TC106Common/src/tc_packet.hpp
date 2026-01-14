#pragma once
#include <Arduino.h>

namespace tc {
  // --- Raspberry Pi 通信用の固定長定義 ---
  constexpr uint8_t PI_LEN_SEND  = 6;  // 通常送信
  constexpr uint8_t PI_LEN_SENS  = 8;  // センサ情報
  constexpr uint8_t PI_LEN_RESET = 12; // リセット/初期化
  constexpr uint8_t PI_MAX       = 12; // 最大パケット長
  
  // --- TC106(Nano) 内部処理用の定義 ---
  constexpr uint8_t TC_DATA_LEN  = 5;  // 9ビット通信のデータ部（5バイト）
  constexpr uint8_t RING_SIZE    = 64; // 受信バッファサイズ

  /**
   * @brief TC106 独自の7ビットチェックサム計算
   * 旧システムの仕様に基づき、合計値の下位7ビットを抽出する
   */
  static inline uint8_t checksum7(const uint8_t* d, uint8_t n) {
    uint16_t s = 0;
    for (uint8_t i = 0; i < n; i++) s += (uint8_t)d[i];
    return (uint8_t)(s & 0x7F);
  }

  /**
   * @brief Piからの受信パケット保持用
   */
  struct Packet {
    uint8_t buf[PI_MAX]{};
    uint8_t len{0};
    uint8_t cmd() const { return buf[0] & 0x07; } // 先頭バイトの下位3bitがコマンド
  };

  /**
   * @brief 同一パケットの連続処理を防止するためのシグネチャ生成
   */
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

  /**
   * @brief リングバッファから有効なパケットを解析・抽出する
   */
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
        // チェックサムが一致するか確認
        if (checksum7(tmp, l - 1) == tmp[l - 1]) {
          uint64_t sig = signature(tmp, l);
          if (sig != lastSig) { // 前回と異なるパケットのみ受理
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

  /**
   * @brief TC側(Nano Every)へ送る5データ+1コマンドの構造
   */
  struct TcFrames {
    uint8_t data[TC_DATA_LEN]{};
    uint8_t cmd{0};
    bool isTruncated{false};
  };

  /**
   * @brief PiパケットからTC送信用フレームへ変換
   */
  static inline TcFrames toTcFrames(const Packet& p) {
    TcFrames out;
    out.cmd = p.buf[0]; 
    for (uint8_t i = 0; i < TC_DATA_LEN; i++) out.data[i] = p.buf[1 + i];
    out.isTruncated = (p.len > PI_LEN_SEND);
    return out;
  }

  // ======================================================
  // 疎通テスト用：6バイト固定パケット構造体 (B案)
  // ======================================================
  struct Fixed6 {
    static constexpr uint8_t LEN = 6;
    uint8_t b[LEN]{};

    /**
     * @brief シリアルモニターへ16進数でダンプ出力
     */
    inline void dumpTo(Stream& s) const {
      for (uint8_t i = 0; i < LEN; i++) {
        if (b[i] < 0x10) s.print('0');
        s.print(b[i], HEX);
        if (i + 1 != LEN) s.print(' ');
      }
    }
  };
}