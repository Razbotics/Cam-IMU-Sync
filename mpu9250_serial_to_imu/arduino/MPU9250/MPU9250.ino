#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define TRIGGER_PIN 5
bool blinkState = false;
int status;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[25] = { '$', 0x03, 0,0, 0,0, 0,0,  0,0, 0,0, 0,0, 0,   0,0,0,0,   0,0,0,0, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile unsigned long irqTimestamp = 0;
volatile unsigned long triggerCounter = 0;
volatile byte irqCounter = 0;
int ax, ay, az, gx, gy, gz;

void Interrupt_ready() {
    irqTimestamp = millis();
    irqCounter++;
    if (irqCounter == 10){ // 20 Hz
        digitalWrite(TRIGGER_PIN, HIGH);
        //delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);
        triggerCounter++;
 
      irqCounter = 0;
    } 
    mpuInterrupt = true;
}


void setup() {
   
    Serial.begin(230400);
    while (!Serial); 
    digitalWrite(TRIGGER_PIN, LOW);  //  drive it low without temporarily driving it high
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT);
  
    status = IMU.begin();
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
     }
    
      IMU.setSrd(4);
      IMU.enableDataReadyInterrupt();
      // setting the accelerometer full scale range to +/-2G 
      IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
      // setting the gyroscope full scale range to +/-250 deg/s
      IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), Interrupt_ready, RISING);
      // configure LED for output
      pinMode(LED_PIN, OUTPUT);
    
    delay(4000);
}


void loop() {
        while (!mpuInterrupt);
        mpuInterrupt = false;
        IMU.readSensor();
        ax = IMU.getAccelX_mss() * 1000;
        ay = IMU.getAccelY_mss() * 1000;
        az = IMU.getAccelZ_mss() * 1000;

        gx = IMU.getGyroX_rads() * 1000;
        gy = IMU.getGyroY_rads() * 1000;
        gz = IMU.getGyroZ_rads() * 1000;

        //Serial.print(ax); Serial.print("  "); Serial.print(ay); Serial.print("  "); Serial.print(az); Serial.print("  ");
        //Serial.print(gx); Serial.print("  "); Serial.print(gy); Serial.print("  "); Serial.print(gz); Serial.println("");
        
        // accel values

        teapotPacket[2] = (ax >> 8) & 0xFF;
        teapotPacket[3] = ax & 0xFF;
        
        teapotPacket[4] = (ay >> 8) & 0xFF;
        teapotPacket[5] = ay & 0xFF;
        
        teapotPacket[6] = (az >> 8) & 0xFF;
        teapotPacket[7] = az & 0xFF;
        
        // gyro values

        teapotPacket[8] = (gx >> 8) & 0xFF;
        teapotPacket[9] = gx & 0xFF;
        
        teapotPacket[10] = (gy >> 8) & 0xFF;
        teapotPacket[11] = gy & 0xFF;

        teapotPacket[12] = (gz >> 8) & 0xFF;
        teapotPacket[13] = gz & 0xFF;
        
        //  timestamp
        teapotPacket[15]=(irqTimestamp >> 24) & 0xFF;
        teapotPacket[16]=(irqTimestamp >> 16) & 0xFF;
        teapotPacket[17]=(irqTimestamp >> 8) & 0xFF;        
        teapotPacket[18]=irqTimestamp & 0xFF;
        // trigger counter
        teapotPacket[19]=(triggerCounter >> 24) & 0xFF;
        teapotPacket[20]=(triggerCounter >> 16) & 0xFF;
        teapotPacket[21]=(triggerCounter >> 8) & 0xFF;        
        teapotPacket[22]=triggerCounter & 0xFF; 
        
        Serial.write(teapotPacket, 25);
        
        teapotPacket[14]++; // packetCount, loops at 0xFF on purpose
     
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
}
