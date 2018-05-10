
int incomingByte = 0;   // for incoming serial data

void setup() {
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
        while(!Serial){}

        Serial.println("Testing RS-232 Comm");
}

void loop() {

  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
     
    // say what you got:
    Serial.print("I received: ");
//    Serial.println((char)incomingByte);
    Serial.println(incomingByte, HEX);
  }
//  else { Serial.println("Got nothing");}
}