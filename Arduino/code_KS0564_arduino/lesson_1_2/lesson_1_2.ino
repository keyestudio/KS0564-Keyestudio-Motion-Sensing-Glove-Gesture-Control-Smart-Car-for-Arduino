#include <MPU6050.h>

MPU6050lib mpu;

int16_t tempCount;              // Store the real internal chip temperature in degrees Celsius
float temperature;              // Store the actual temperature in degrees Celsius
float gyroBias[3] = {0, 0, 0};  // Correct gyroscope and acclerometer bias
float accelBias[3] = {0, 0, 0};
float SelfTest[6];              // Self-test value storing container

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  // Read the WHO_AM_I register, this is a good test of communication
  // Read WHO_AM_I register for MPU-6050
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  
  Serial.print("I AM ");
  Serial.print(c, HEX);
  //Set the minimum scale if the device is in self-test
      // Possible gyro scales (and their register bit settings) are: 
      // 250 DPS (0x00), 500 DPS (0x01), 1000 DPS (0x10), and 2000 DPS  (0x11).
      // Possible accelerometer scales (and their register bit settings) are:
      // 2 Gs (0x00), 4 Gs (0x01), 8 Gs (0x10), and 16 Gs  (0x11).
  mpu.settings(AFS_8G, GFS_250DPS);
  // version WHO_AM_I should always be 0x68 //MPU6050  address 1: 0x68, address 2: 0x98
  if (c == 0x68 || c == 0x98) {
    Serial.println("MPU6050 is online...");
    // Start by performing self test
    mpu.MPU6050SelfTest(SelfTest);
    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f 
    && SelfTest[5] < 1.0f) {
      Serial.println("Pass Selftest!");
      // Calibrate gyro and accelerometers, load biases in bias registers
      mpu.calibrateMPU6050(gyroBias, accelBias);
      mpu.settings(AFS_2G, GFS_250DPS);
      mpu.initMPU6050();
      // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      Serial.println("MPU6050 initialized for active data mode...."); 
    }
    else{
      Serial.print("Could not connect to MPU6050: 0x");
      Serial.println(c, HEX);
      // Loop forever if communication doesn't happen
      while (1) ; 
    }
  }
}

void loop()
{
  // If data ready bit set, all data registers have new data
  // check if data ready interrupt
  if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
    tempCount = mpu.readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }

  Serial.println("--------");
  // Temperature in degrees Centigrade
  Serial.print("TEMP values:");
  Serial.println(temperature);
  Serial.println("--------");
  delay(500);
}