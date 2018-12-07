/**
 * TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino
 *
 * Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
 * Based on Multiplex Code: https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
 * Based on IR Sensor Code: https://www.dfrobot.com/wiki/index.php/Positioning_ir_camera

 */
 
#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
 
#define TCAADDR 0x70
int XYSensor = 1;
int ZYSensor = 2;
int IRsensorAddress = 0xB0;
int slaveAddress;
int Ix1[4];
int Iy1[4];
int Ix2[4];
int Iy2[4];
byte data_buf[16];

int i;
int s;

/*this selects which sensor it gets data from*/ 
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
/*This Writes the bytes onto IR Sensors
*/
void Write_2bytes(byte d1, byte d2){
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}
 
void setup()
{
    //Giving i2c command to intialize the sensors
    Serial.begin(115200);
    Wire.begin();
    slaveAddress = IRsensorAddress >> 1;
    Serial.println("Intializing IR sensors");
    Serial.println("");
    Serial.println("Initializing XY coordinate...");
    Serial.println("");
    tcaselect(XYSensor);
    Wire.begin();
    Write_2bytes(0x30,0x01); delay(10);
    Write_2bytes(0x30,0x08); delay(10);
    Write_2bytes(0x06,0x90); delay(10);
    Write_2bytes(0x08,0xC0); delay(10);
    Write_2bytes(0x1A,0x40); delay(10);
    Write_2bytes(0x33,0x33); delay(10);
    delay(100);
    Serial.println("Initializing ZY coordinate...");
    Serial.println("");
    tcaselect(ZYSensor);
    Wire.begin();
    Write_2bytes(0x30,0x01); delay(10);
    Write_2bytes(0x30,0x08); delay(10);
    Write_2bytes(0x06,0x90); delay(10);
    Write_2bytes(0x08,0xC0); delay(10);
    Write_2bytes(0x1A,0x40); delay(10);
    Write_2bytes(0x33,0x33); delay(10);
    delay(100);
}

 
void loop() 
{
  //XY - Sensor Read
  tcaselect(XYSensor);
  Wire.beginTransmission(slaveAddress);
  Wire.write(0x36);
  Wire.endTransmission();
  
  Wire.requestFrom(slaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
  for (i=0;i<16;i++) { data_buf[i]=0; }
  i=0;
  while(Wire.available() && i < 16) { 
      data_buf[i] = Wire.read();
      i++;
  }
  Ix1[0] = data_buf[1];
  Iy1[0] = data_buf[2];
  s   = data_buf[3];
  Ix1[0] += (s & 0x30) <<4;
  Iy1[0] += (s & 0xC0) <<2;

  Ix1[1] = data_buf[4];
  Iy1[1] = data_buf[5];
  s   = data_buf[6];
  Ix1[1] += (s & 0x30) <<4;
  Iy1[1] += (s & 0xC0) <<2;

  Ix1[2] = data_buf[7];
  Iy1[2] = data_buf[8];
  s   = data_buf[9];
  Ix1[2] += (s & 0x30) <<4;
  Iy1[2] += (s & 0xC0) <<2;

  Ix1[3] = data_buf[10];
  Iy1[3] = data_buf[11];
  s   = data_buf[12];
  Ix1[3] += (s & 0x30) <<4;
  Iy1[3] += (s & 0xC0) <<2;
  
  //ZY - Sensor Read
  tcaselect(ZYSensor);
  Wire.beginTransmission(slaveAddress);
  Wire.write(0x36);
  Wire.endTransmission();
  Wire.requestFrom(slaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
  for (i=0;i<16;i++) { data_buf[i]=0; }
  i=0;
  while(Wire.available() && i < 16) { 
  data_buf[i] = Wire.read();
  i++;
  }
  Ix2[0] = data_buf[1];
  Iy2[0] = data_buf[2];
  s   = data_buf[3];
  Ix2[0] += (s & 0x30) <<4;
  Iy2[0] += (s & 0xC0) <<2;

  Ix2[1] = data_buf[4];
  Iy2[1] = data_buf[5];
  s   = data_buf[6];
  Ix2[1] += (s & 0x30) <<4;
  Iy2[1] += (s & 0xC0) <<2;

  Ix2[2] = data_buf[7];
  Iy2[2] = data_buf[8];
  s   = data_buf[9];
  Ix2[2] += (s & 0x30) <<4;
  Iy2[2] += (s & 0xC0) <<2;

  Ix2[3] = data_buf[10];
  Iy2[3] = data_buf[11];
  s   = data_buf[12];
  Ix2[3] += (s & 0x30) <<4;
  Iy2[3] += (s & 0xC0) <<2;


  for(i=0; i<1; i++)
    {
      Serial.print("X1:");
      if (Ix1[i] < 1000)
        Serial.print("");
      if (Ix1[i] < 100)  
        Serial.print("");
      if (Ix1[i] < 10)  
        Serial.print("");
      Serial.print( int(Ix1[i]) );
      Serial.print(",");
      
      Serial.print("Y1:");
      if (Iy1[i] < 1000)
        Serial.print("");
      if (Iy1[i] < 100)  
        Serial.print("");
      if (Iy1[i] < 10)  
        Serial.print("");
      Serial.print( int(Iy1[i]) );
      Serial.print(",");
      
      Serial.print("X2:");
      if (Ix2[i] < 1000)
        Serial.print("");
      if (Ix2[i] < 100)  
        Serial.print("");
      if (Ix2[i] < 10)  
        Serial.print("");
      Serial.print( int(Ix2[i]) );
      Serial.print(",");
      
      Serial.print("Y2:");
      if (Iy2[i] < 1000)
        Serial.print("");
      if (Iy2[i] < 100)  
        Serial.print("");
      if (Iy2[i] < 10)  
        Serial.print("");
      Serial.print( int(Iy2[i]) );
      Serial.print(",");    
    }
  
  Serial.println("");
  delay(15);
}
