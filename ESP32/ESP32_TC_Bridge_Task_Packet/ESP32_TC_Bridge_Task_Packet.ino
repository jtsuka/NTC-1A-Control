// =============================================================
// ESP32_TC_Bridge_Task_Packet.ino  (main sketch)
// =============================================================
#include "tc_packet.hpp"
using namespace tc;
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/* ------------- pin / uart ----------------*/
#define UART_PI_TX 43   // (白)TX ESP32->Pi
#define UART_PI_RX 44   // (黄)RX Pi->ESP32
#define UART_TC_TX 2    // (白)TX ESP32->TC
#define UART_TC_RX 3    // (黄)RX TC->ESP32
#define BAUD_PI 9600
#define BAUD_TC 300
HardwareSerial SerialPi(1);
HardwareSerial SerialTC(2);
/* level‑shifter */
#define PIN_OE1 5       // (白)TX 出力有効 → 常時 LOW
#define PIN_DIR1 6      // (黄)TX方向：ESP32 → TC → HIGH 固定
#define PIN_OE2 7       // (白)RX 出力有効 → 常時 LOW
#define PIN_DIR2 8      // (黄)RX方向：TC → ESP32 → LOW 固定

/* ------------- RTOS queues ---------------*/
static QueueHandle_t qPi2Tc;
static QueueHandle_t qTc2Pi;

/* ------------- ring buffers --------------*/
static uint8_t bufPi[RING_SIZE]; static uint8_t idxPi = 0;
static uint8_t bufTc[RING_SIZE]; static uint8_t idxTc = 0;

/* ------------- ISR callbacks -------------*/
void IRAM_ATTR onUartPi(){
    BaseType_t hp=pdFALSE;
    while(SerialPi.available()){
        uint8_t b=SerialPi.read();
        xQueueSendFromISR(qPi2Tc,&b,&hp);
    }
    if(hp)
        portYIELD_FROM_ISR();
}
void IRAM_ATTR onUartTc(){
    BaseType_t hp=pdFALSE;
    while(SerialTC.available()){
        uint8_t b=SerialTC.read();
        xQueueSendFromISR(qTc2Pi,&b,&hp);
    }
    if(hp)
    portYIELD_FROM_ISR();
}

/* ------------- task Pi -> TC ------------*/
void taskPi2Tc(void*){
    for(;;){
        uint8_t b;
        if(xQueueReceive(qPi2Tc,&b,portMAX_DELAY)==pdTRUE){
            bufPi[idxPi]=b;
            idxPi=(idxPi+1)&(RING_SIZE-1);
            if(auto p=PacketFactory::tryParse(bufPi,idxPi)){
                SerialTC.write(p->buf,p->len);
                SerialTC.flush();
                idxPi=0;
            }
        }
    }
}
/* ------------- task TC -> Pi ------------*/
void taskTc2Pi(void*){
    for(;;){
        uint8_t b;
        if(xQueueReceive(qTc2Pi,&b,portMAX_DELAY)==pdTRUE){
            bufTc[idxTc]=b;
            idxTc=(idxTc+1)&(RING_SIZE-1);
            if(auto p=PacketFactory::tryParse(bufTc,idxTc)){
                uint8_t out[FRAME_MAX];
                p->toBytes(out,false);
                SerialPi.write(out,p->len);
                SerialPi.flush();
                idxTc=0;
            }
        }
    }
}

/* ------------- setup --------------------*/
void setup(){
    // for Debug serial monitor
    Serial.begin(115200);

    // ■ 追加：UART TX ピンを強制 idle HIGH してから begin()
    pinMode(UART_TC_TX, OUTPUT);
    digitalWrite(UART_TC_TX, LOW);

    // for Raspberry Pi & TC HW Serial init
    SerialPi.begin(BAUD_PI,SERIAL_8N1,UART_PI_RX,UART_PI_TX);
    // SerialTC.begin(BAUD_TC,SERIAL_8N1,UART_TC_RX,UART_TC_TX);
    // 反転 = true で TX/RX ともアイドル LOW に
    SerialTC.begin(BAUD_TC, SERIAL_8N1,
                    UART_TC_RX, UART_TC_TX,
                    /*invert = */ true);    
    // ── ここで少し待つ ──
    delay(100);
    // for AE-LLCNV-LVCH16T245 16-bit bidirectional level conversion module setup
    pinMode(PIN_OE1,OUTPUT); digitalWrite(PIN_OE1,LOW); pinMode(PIN_DIR1,OUTPUT); digitalWrite(PIN_DIR1,HIGH);
    pinMode(PIN_OE2,OUTPUT); digitalWrite(PIN_OE2,LOW); pinMode(PIN_DIR2,OUTPUT); digitalWrite(PIN_DIR2,LOW);

    // for Send paket Queue for Pi, for TC
    qPi2Tc=xQueueCreate(128,sizeof(uint8_t)); qTc2Pi=xQueueCreate(128,sizeof(uint8_t));

    // 2 task Create for Pi, TC
    xTaskCreatePinnedToCore(taskPi2Tc,"Pi2TC",4096,NULL,3,NULL,0);
    xTaskCreatePinnedToCore(taskTc2Pi,"TC2Pi",4096,NULL,4,NULL,0);

    // ─── ゴミ受信を全部捨てる ───
    while(SerialPi.available()) SerialPi.read();
    while(SerialTC.available()) SerialTC.read();

    // for ESP32S3 Setting for handling UART reception with interrupt + callback
    SerialPi.setRxFIFOFull(1);
    SerialPi.setRxTimeout(40);           // 5 ビット時間後に onReceive()
    SerialPi.onReceive(onUartPi);       // 受信割り込みで onUartPi()  を呼ぶ 
    SerialTC.setRxFIFOFull(1);          // 1バイト来るごとに割り込み発生
    SerialTC.setRxTimeout(40);
    SerialTC.onReceive(onUartTc);       // 受信割り込みで onUartTc()  を呼ぶ 
}

void loop(){
    vTaskDelay(pdMS_TO_TICKS(100));
}
