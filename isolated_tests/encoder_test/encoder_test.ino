#include <QuadratureEncoder.h>
// must also have enableInterrupt.h library

// Use any 2 pins for interrupt, this utilizes EnableInterrupt Library. 
// Even analog pins can be used. A0 = 14,A1=15,..etc for arduino nano/uno

// Max number of Encoders object you can create is 4. This example only uses 2.

Encoders leftFrontEncoder(2,3);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightFrontEncoder(A7,A6); // Encoder object name rightEncoder using analog pin A0 and A1 

Encoders leftBackEncoder(4,7);
Encoders rightBackEncoder(A5,A4);

void setup() {
  Serial.begin(9600);
}


unsigned long lastMilli = 0;

void loop() {
  // put your main code here, to run repeatedly:
  // print encoder count every 50 millisecond
  if(millis()-lastMilli > 50){ 
    
    long currentLeftFrontEncoderCount = leftFrontEncoder.getEncoderCount();
    long currentRightFrontEncoderCount = rightFrontEncoder.getEncoderCount();

    long currentLeftBackEncoderCount = leftBackEncoder.getEncoderCount();
    long currentRightBackEncoderCount = rightBackEncoder.getEncoderCount();
    
    Serial.print(currentLeftFrontEncoderCount);
    Serial.print(" , ");
    Serial.print(currentRightFrontEncoderCount);
    Serial.print(" , ");
    Serial.print(currentLeftBackEncoderCount);
    Serial.print(" , ");
    Serial.println(currentRightBackEncoderCount);
    lastMilli = millis();
  }
   
}
