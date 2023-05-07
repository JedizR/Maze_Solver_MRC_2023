void CountEn() {
  Count++;
}
void L() {
  Count = 0;
  attachInterrupt(0, CountEn, RISING);
  while (Count < 300) {
    motorControl(-200, 200);
  }
  Count = 0;
  motorControl(0,0);
  delay(200);
}
void R() {
  Count = 0;
  attachInterrupt(0, CountEn, RISING);
  while (Count < 500) {
    motorcontrol(40, -40);
  }
  Count = 0;
  motorcontrol(0,0);
  delay(200);
}
void T() {
  Count = 0;
  attachInterrupt(0, CountEn, RISING);
  while (Count < 210) {
    motorControl(255, -255);
  }
  Count = 0;
  motorControl(0,0);
  delay(200);
}
