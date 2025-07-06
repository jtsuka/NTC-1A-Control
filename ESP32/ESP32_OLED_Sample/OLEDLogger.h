#ifndef OLED_LOGGER_H
#define OLED_LOGGER_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

/**
 * OLEDLogger - 上段2行＋下段2行に分かれた表示制御クラス
 * - FreeRTOSセーフ（内部にSemaphoreで排他制御）
 * - 上下段の個別更新が可能
 * - 描画内容を記憶して、再描画も可能
 */
class OLEDLogger {
public:
  OLEDLogger();

  /**
   * @brief OLEDとI2Cバスを初期化し、表示をクリアする
   */
  void begin();

  /**
   * @brief 上段2行（Y=0, Y=16）を更新表示
   * @param l1 1行目のテキスト
   * @param l2 2行目のテキスト
   */
  void updateTop(const String& l1, const String& l2);

  /**
   * @brief 下段2行（Y=32, Y=48）を更新表示
   * @param l1 3行目のテキスト
   * @param l2 4行目のテキスト
   */
  void updateBottom(const String& l1, const String& l2);

  /**
   * @brief OLED全体をクリア（内容も非表示に）
   */
  void clearAll();

  /**
   * @brief 最後に表示した上下段の内容を再描画
   */
  void redrawAll();

private:
  // 表示内容を保持する変数
  String topLine1, topLine2;
  String bottomLine1, bottomLine2;

  // OLED操作の排他制御用
  SemaphoreHandle_t oledMutex;

  /**
   * @brief 指定Y位置にテキストを描画（1行分）
   * @param y Y座標（例：0, 16, 32, 48）
   * @param text 描画するテキスト
   */
  void drawLine(int y, const String& text);

  /**
   * @brief 指定領域をクリア（fillRectで黒塗り）
   * @param yStart 開始Y座標
   * @param lines 何行分（1行=16ピクセル）
   */
  void clearLines(int yStart, int lines);
};

#endif // OLED_LOGGER_H
