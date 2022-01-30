#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "TCA9548A.h"
TCA9548A I2CMux;
#define TCAADDR 0x70

long testTimer = millis();
long testLimit = 5000;
MPU6050 accelgyro1(0x68);
MPU6050 accelgyro2(0x68);
MPU6050 accelgyro3(0x68);
MPU6050 accelgyro4(0x68);
MPU6050 accelgyro5(0x68);
MPU6050 accelgyro6(0x68);
MPU6050 accelgyro7(0x68);
MPU6050 accelgyro8(0x68);
MPU6050 accelgyro9(0x68);
MPU6050 accelgyro10(0x68);
MPU6050 accelgyro11(0x68);
MPU6050 accelgyro12(0x68);
MPU6050 accelgyro13(0x68);

#define AX 0
#define AY 1
#define AZ 2
#define RX 3
#define RY 4
#define RZ 5

//int useIMU[] = {2, 3, 4, 5, 6, 7, 8, 9, 14, 20, 21, 22, 23};
int useIMU[] = {6,  4, 9, 8, 22,  3, 7, 17,  1, 5, 2, 15,  16};
int sensorMux[13] = {6, 7, 6, 5, 4, 4, -1, -1, 5, 4, 6, 6, 7};
float netPulse[14];   // last is 200G
float sensor[14][6];  // last is 200G
float av[14] = {0.0}; // last is 200G

float maxECG = 6.5; // threshold for printing out data
bool ecg = true; // ecg = true for 13 plots adds an offset to look like ECG
bool calib = false; // calib = true to calibrate all sensors for gravity FIRST
bool runningNow = false; // running = true is setup for impact tests
int numSamples = 100;
int status;
// MPU control/status vars

bool dmpReady1 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU
uint8_t devStatus1;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer
bool dmpReady2 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus2;   // holds actual interrupt status byte from MPU
uint8_t devStatus2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer2[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t ax, ay, az;
int16_t gx, gy, gz;

long freqCheck = 5000;
uint32_t timer = millis();
long msTime = millis();
long ecgUpdate = millis();
void setup(void) {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Started");
  Wire.begin();
  Serial.println("Starting");
  I2CMux.begin(Wire);
  I2CMux.closeAll();
  Serial.println("\nI2CMux set up!");
  usePinMode();
  /*
    setup6050a();
    setup6050b();
    setup6050c();
    setup6050d();
    setup6050e();
  */
  setup6050(&accelgyro1, 1);
  setup6050(&accelgyro2, 2);
  setup6050(&accelgyro3, 3);
  setup6050(&accelgyro4, 4);
  setup6050(&accelgyro5, 5);
  setup6050(&accelgyro6, 6);
  setup6050(&accelgyro7, 7);
  setup6050(&accelgyro8, 8);
  setup6050(&accelgyro9, 9);
  setup6050(&accelgyro10, 10);
  setup6050(&accelgyro11, 11);
  setup6050(&accelgyro12, 12);
  setup6050(&accelgyro13, 13);
  delay(1000);
  if (calib)
  {
    maxECG = 0;
    Serial.print("Keep still and sample for ");
    Serial.print(numSamples);
    Serial.println(" measurements");
    for (int i = 0; i < numSamples; i++)
    {
      justRead6050(&accelgyro1, 1);
      justRead6050(&accelgyro2, 2);
      justRead6050(&accelgyro3, 3);
      justRead6050(&accelgyro4, 4);
      justRead6050(&accelgyro5, 5);
      justRead6050(&accelgyro6, 6);
      justRead6050(&accelgyro7, 7);
      justRead6050(&accelgyro8, 8);
      justRead6050(&accelgyro9, 9);
      justRead6050(&accelgyro10, 10);
      justRead6050(&accelgyro11, 11);
      justRead6050(&accelgyro12, 12);
      justRead6050(&accelgyro13, 13);
      read200G();
      for (int j = 0; j < 14; j++)
      {
        av[j] += netPulse[j];
      }
    }
    Serial.println("Calibration values");
    for (int j = 0; j < 13; j++)
    {
      maxECG += netPulse[j];
      Serial.print(av[j] / (float)numSamples);
      Serial.print(",");
    }
    maxECG *= 1.25;
    Serial.println(av[13] / (float)numSamples);
    delay(1000);
    Serial.println("Done Calibration");
  }
}

void loop() {

  //  /* Get a new normalized sensor event */
  msTime = millis();
  //Serial.print(millis());
  //Serial.print(" ");
  justRead6050(&accelgyro1, 1);
  justRead6050(&accelgyro2, 2);
  justRead6050(&accelgyro3, 3);
  justRead6050(&accelgyro4, 4);
  justRead6050(&accelgyro5, 5);
  justRead6050(&accelgyro6, 6);
  justRead6050(&accelgyro7, 7);
  justRead6050(&accelgyro8, 8);
  justRead6050(&accelgyro9, 9);
  justRead6050(&accelgyro10, 10);
  justRead6050(&accelgyro11, 11);
  justRead6050(&accelgyro12, 12);
  justRead6050(&accelgyro13, 13);
  read200G();
  if (runningNow)
    dataDump(false);
  if ((runningNow) && (millis() - testTimer > testLimit))
  {
    Serial.print("Heartbeat,");
    dataDump(true);
    testTimer = millis();
  }
  //result();
  if (ecg)
  {
    for (int i = 0; i < 13; i++)
    {
      //if((i==7) || (i==6))
      {
        Serial.print(netPulse[i] + (i + 1) * 15);
      }

      Serial.print(",");
    }
    Serial.println();
  }
}

void result()
{
  float x[] = {2, 9, 7, 0, -7, -9};
  float y[] = {6, 4, -5, -7, -5, 4};
  float sumX = 0.0;
  float sumY = 0.0;
  float sumN = 0.0;
  for (int i = 0; i < 13; i++)
  {
    sumX += netPulse[i] * x[i];
    sumY += netPulse[i] * y[i];
    sumN += netPulse[i];
  }
  sumX /= sumN;
  sumY /= sumN;
  /*
    Serial.print(", ");
    Serial.print(sumX);
    Serial.print(", ");
    Serial.print(sumY);
  */
  String side = "";
  String front = "";

  float thresh = 2;
  if (sumX > thresh) side = "Right";
  if (fabs(sumX) < thresh) side = "Mid";
  if (sumX < -thresh) side = "Left";

  if (sumY > thresh) front = "Front";
  if (fabs(sumY) < thresh) front = "Top";
  if (sumY < -thresh) front = "Back";
  /*
    Serial.print(", ");
    Serial.print(sumN);
    Serial.print(", ");
    /*
    if (sumN > 7.0)
    {
    Serial.print("\t");
    Serial.print(side);
    Serial.print(", ");
    Serial.print(front);
    delay(10000);
    }
    /*
    else
    {
    Serial.print("\t");
    Serial.print("Mid");
    Serial.print(", ");
    Serial.print("Top");

    }
  */
}

void resetFifos()
{
  Serial.println("\nResetting");
  accelgyro1.resetFIFO();
  accelgyro2.resetFIFO();
  accelgyro3.resetFIFO();
  accelgyro4.resetFIFO();
  accelgyro5.resetFIFO();
  accelgyro6.resetFIFO();
  accelgyro7.resetFIFO();
  accelgyro8.resetFIFO();
  accelgyro9.resetFIFO();
  accelgyro10.resetFIFO();
  accelgyro11.resetFIFO();
  accelgyro12.resetFIFO();
  accelgyro13.resetFIFO();
}

void use(int x)
{
  for (int i = 0; i < 13; i++)
  {
    digitalWrite(useIMU[i], 1);
  }
  //digitalWrite(useIMU[x - 1], 0);
  /*
    Serial.print("Sensor ");
    Serial.print(x);
    Serial.print(" = CS(");
    Serial.print(useIMU[x - 1]);
    Serial.print(" & Mux(");
    Serial.print(sensorMux[x - 1]);
    Serial.println(")");
  */
  tcaselect(sensorMux[x - 1]);
  digitalWrite(useIMU[x - 1], 0);
}

void usePinMode()
{
  for (int i = 0; i < 13; i++)
  {
    pinMode(useIMU[i], OUTPUT);
  }
}

void setup6050(MPU6050 *acc, int n)
{
  use(n);
  Serial.print("Initialising Accerlometer number ");
  Serial.println(n);
  acc->initialize();
  Serial.println("Initialised 6050a");
  acc->setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  Serial.println("setFullScaleGyroRange 2000 degrees/sec");
  acc->setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  Serial.println("setFullScaleAccelRange 16g");
}

void justRead6050(MPU6050 *acc, int n)
{
  long t = millis();

  use(n);
  acc->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // display tab-separated accel/gyro x/y/z values
  float Fax = (float)ax / 2048; //* 32 * 9.8 / 32768;
  float Fay = (float)ay / 2048; //* 32 * 9.8 / 32768;
  float Faz = (float)az / 2048; //* 32 * 9.8 / 32768;
  float Fgx = (float)gx / 16.4; //* 4000 / 32768;
  float Fgy = (float)gy / 16.4; //* 4000 / 32768;
  float Fgz = (float)gz / 16.4; //* 4000 / 32768;


  //delay(15);
  float pulse = sqrt(Fax * Fax + Fay * Fay + Faz * Faz) + n * 15;

  netPulse[n - 1] = sqrt(Fax * Fax + Fay * Fay + Faz * Faz);
  sensor[n - 1][AX] = Fax;
  sensor[n - 1][AY] = Fay;
  sensor[n - 1][AZ] = Faz;
  sensor[n - 1][RX] = Fgx;
  sensor[n - 1][RY] = Fgy;
  sensor[n - 1][RZ] = Fgz;
}

void dataDump(bool test)
{
  float sum = 0.0;

  for (uint8_t i = 0; i < 13; i++)
  {
    sum += netPulse[i];
  }
  if ((sum > maxECG) || test)
  {
    Serial.print(millis()); Serial.print(",");
    for (uint8_t i = 0; i < 13; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        Serial.print(sensor[i][j]);
        Serial.print(",");
      }
      Serial.print(netPulse[i]);
      if ((i == 3) || (i == 7) || (i == 11))
        Serial.println();
      else
        Serial.print(",");
    }
    for (int j = 0; j < 6; j++)
    {
      Serial.print(sensor[13][j]);
      Serial.print(",");
    }
    Serial.print(maxECG);
    Serial.println();
  }
}

void read200G()
{
  float Fax = (float)(analogRead(23) - 512) / 512 * 200;
  float Fay = (float)(analogRead(21) - 512) / 512 * 200;
  float Faz = (float)(analogRead(20) - 512) / 512 * 200;
  netPulse[13] = sqrt(Fax * Fax + Fay * Fay + Faz * Faz);
  sensor[13][AX] = Fax;
  sensor[13][AY] = Fay;
  sensor[13][AZ] = Faz;
  sensor[13][RX] = 0.0;
  sensor[13][RY] = 0.0;
  sensor[13][RZ] = 0.0;
  if (ecg)
  {
    Serial.print(",");
    netPulse[13];
  }
}

void tcaselect(int x)
{
  I2CMux.closeAll();
  if (x != -1)
  {
    I2CMux.openChannel(x);
  }
  else
  {
    I2CMux.closeAll();
  }
}
