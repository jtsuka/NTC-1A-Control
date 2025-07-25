void setup() {
  Serial.begin(115200);
  pinMode( 0, INPUT_PULLUP);                 // ←空きピン
  attachInterruptArg( 0, [](void*) {
        ets_printf("INT LOW!\n");
      }, nullptr, FALLING);

  pinMode(BITBANG_RX_PIN, INPUT_PULLUP);
  attachInterruptArg( BITBANG_RX_PIN, [](void*) {
        ets_printf("RXB LOW!\n");
      }, nullptr, FALLING);
}
void loop(){ delay(100); }
