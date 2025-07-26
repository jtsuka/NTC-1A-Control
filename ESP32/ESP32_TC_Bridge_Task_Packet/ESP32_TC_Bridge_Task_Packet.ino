/*********************************************************************
   ESP32-S3 TC Repeater – Task & PacketClass version
*********************************************************************/
#include "tc_packet.hpp"
using namespace tc;

/* ---------- UART 定義 ---------- */
#define UART_PI_TX 43
#define UART_PI_RX 44
#define UART_TC_TX 2
#define UART_TC_RX 3
#define BAUD_PI 9600
#define BAUD_TC 300
HardwareSerial SerialPi(1);   // UART1
HardwareSerial SerialTC(2);   // UART2

/* ---------- LVC16T245 DIR / OE ---------- */
#define PIN_OE1 5
#define PIN_DIR1 6
#define PIN_OE2 7
#define PIN_DIR2 8

/* ---------- Queues ---------- */
QueueHandle_t qPi2Tc;
QueueHandle_t qTc2Pi;

/* ---------- RingBuffer for parser ---------- */
static uint8_t bufPi[64]; static size_t idxPi=0;
static uint8_t bufTc[64]; static size_t idxTc=0;

/* ---------- UART RX ISR → queue push ---------- */
void IRAM_ATTR onUartPi(void*){
  uint8_t b;
  while(SerialPi.readBytes(&b,1)==1){ xQueueSendFromISR(qPi2Tc,&b,nullptr);}
}
void IRAM_ATTR onUartTc(void*){
  uint8_t b;
  while(SerialTC.readBytes(&b,1)==1){ xQueueSendFromISR(qTc2Pi,&b,nullptr);}
}

/* ---------- Task: Pi -> TC ---------- */
void taskPi2Tc(void*){
  for(;;){
    uint8_t b;
    if(xQueueReceive(qPi2Tc,&b,portMAX_DELAY)==pdTRUE){
      bufPi[idxPi++]=b; if(idxPi>63) idxPi=0;
      if(auto pkt=PacketFactory::tryParse(bufPi,idxPi)){
        uint8_t out[8]; pkt->toBytes(out,false);   // TC側へは rev8 無し
        SerialTC.write(out,pkt->len());
        SerialTC.flush();
      }
    }
  }
}
/* ---------- Task: TC -> Pi ---------- */
void taskTc2Pi(void*){
  for(;;){
    uint8_t b;
    if(xQueueReceive(qTc2Pi,&b,portMAX_DELAY)==pdTRUE){
      bufTc[idxTc++]=b; if(idxTc>63) idxTc=0;
      if(auto pkt=PacketFactory::tryParse(bufTc,idxTc)){
        uint8_t out[8]; pkt->toBytes(out,true);    // Pi側へは rev8 有り
        SerialPi.write(out,pkt->len());
        SerialPi.flush();
      }
    }
  }
}

void setup(){
  Serial.begin(115200);

  /* UART init */
  SerialPi.begin(BAUD_PI,SERIAL_8N1,UART_PI_RX,UART_PI_TX);
  SerialTC.begin(BAUD_TC,SERIAL_8N1,UART_TC_RX,UART_TC_TX);

  /* Level-Shifter 固定 */
  pinMode(PIN_OE1,OUTPUT); digitalWrite(PIN_OE1,LOW);
  pinMode(PIN_DIR1,OUTPUT); digitalWrite(PIN_DIR1,HIGH); // A→B (ESP32→TC)
  pinMode(PIN_OE2,OUTPUT); digitalWrite(PIN_OE2,LOW);
  pinMode(PIN_DIR2,OUTPUT); digitalWrite(PIN_DIR2,LOW);  // B→A (TC→ESP32)

  /* Queues & Tasks */
  qPi2Tc = xQueueCreate(128,sizeof(uint8_t));
  qTc2Pi = xQueueCreate(128,sizeof(uint8_t));
  xTaskCreatePinnedToCore(taskPi2Tc,"Pi2TC",4096,nullptr,3,nullptr,0); // prio 3
  xTaskCreatePinnedToCore(taskTc2Pi,"TC2Pi",4096,nullptr,4,nullptr,0); // prio 4 (300bps側)

  /* UART IRQ hook */
  SerialPi.onReceive(onUartPi,nullptr);
  SerialTC.onReceive(onUartTc,nullptr);
}

void loop(){
  delay(100);  // main loop idle
}