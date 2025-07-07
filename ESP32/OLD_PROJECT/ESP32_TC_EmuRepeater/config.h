#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>

// OLED表示
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// GPIO定義
#define TEST_PIN   8
#define LED_PIN    10

// モード定義
enum ModeType { MODE_REPEATER, MODE_EMULATOR };
extern ModeType currentMode;

// MACアドレスによるモード自動切替
#define EMULATOR_MAC "D8:3B:DA:74:82:78"

#endif
