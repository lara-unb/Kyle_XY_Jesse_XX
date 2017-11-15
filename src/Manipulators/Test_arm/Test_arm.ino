void setup() {
  // communication to slower microcontrollers: 9600bps
  Serial.begin(9600);
}
//Wiring:
//arduino - tx -> rx - SSC-32
//arduino - ground -> ground
//pin 0 - Base
//pin 1 - Shoulder
//pin 2 - Elbow
//pin 3 - Wrist
//pin 4 - Grip
//pin 5 - Wrist rotate
void loop() {
  move(4, 2400, 1000);
  move(4, 600, 1000);
  move(3, 1200, 1000);
  move(3, 1800, 1000);
  move(5, 1200, 1000);
  move(5, 1800, 1000);
  move(0, 1200, 1000);
  move(0, 1800, 1000);
  move(1, 1200, 1000);
  move(1, 1800, 1000);
  move(2, 1800, 1000);
  move(2, 2000, 1000);
}

void move(int servo, int position, int time) {
  Serial.print("#");
  Serial.print(servo);
  Serial.print(" P");
  Serial.print(position);
  Serial.print(" T");
  Serial.println(time);
  delay(time);
}
