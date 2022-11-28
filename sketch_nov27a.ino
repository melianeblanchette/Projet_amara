#include <Nextion.h>

NexSlider niv_emg = NexSlider(0,20,"niv_emg");
uint32_t pot;
uint32_t amp = 0;
void setup() {
  // put your setup code here, to run once:

nexInit();
}

void loop() {
  // put your main code here, to run repeatedly:
  pot = analogRead(A1);
  amp = map(pot,0,1023,0,100);
  niv_emg.setValue(amp);
  delay(200);
}
