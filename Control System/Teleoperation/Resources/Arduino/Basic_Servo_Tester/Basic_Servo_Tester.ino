// Include the Servo library 
#include <Servo.h> 
// Declare the Servo pin 
int servoPin = 3; 
// Create a servo object 
Servo Servo1; 
void setup() { 
   // We need to attach the servo to the used pin number 
   Servo1.attach(servoPin);
}
void loop(){ 
   // Make servo go to 0 degrees 
   int i = 00;
   Servo1.write(i); 
   
}
////   delay(10000); 
//   // Make servo go to 90 degrees 
//   for(i = 0; i<180;i++)
//   {
//    Servo1.write(i);
//    delay(5);
//    }
//    delay(1000);
//    for(i=180;i>=0;i--)
//    {
//      Servo1.write(i);
//      delay(5);
//    }
//}
