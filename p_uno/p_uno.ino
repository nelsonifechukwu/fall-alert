#include <Wire.h>

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
double gyro_axis_cal[4];
double acc_axis_cal[4];

//int data[STORE_SIZE][5]; //array for saving past data
//byte currentIndex=0; //stores current data array index (0-255)
bool fall = false;      //stores if a fall has occurred
bool trigger1 = false;  //stores if first trigger (lower threshold) has occurred
bool trigger2 = false;  //stores if second trigger (upper threshold) has occurred
bool trigger3 = false;  //stores if third trigger (orientation change) has occurred

byte trigger1count = 0;  //stores the counts past since trigger 1 was set true
byte trigger2count = 0;  //stores the counts past since trigger 2 was set true
byte trigger3count = 0;  //stores the counts past since trigger 3 was set true
int angleChange = 0;
int cal_int = 0;

void setup() {

  Serial.begin(115200);
  Serial.println("Calibration DONE");
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  calibrate_imu();
  pinMode(5, OUTPUT);  

}

void loop() {
  mpu_read();

  ax = (AcX + acc_axis_cal[1]) / 16384.00;
  ay = (AcY + acc_axis_cal[2]) / 16384.00;
  az = (AcZ + acc_axis_cal[3]) / 16384.00;
  az = az - 1.5;

  Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.println(az);

  //270, 351, 136 for gyroscope
  gx = (GyX - gyro_axis_cal[1]) / 131.07;
  gy = (GyY - gyro_axis_cal[2]) / 131.07;
  gz = (GyZ - gyro_axis_cal[3]) / 131.07;

  // calculating Amplitute vactor for 3 axis
  float Raw_AM = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int AM = Raw_AM * 10;  // as values are within 0 to 1, I multiplied
                         // it by for using if else conditions
    Serial.println(AM);

  if (AM <= 6 && trigger2 == false) {  //if AM breaks lower threshold (0.4g)
  trigger1 = true;
  Serial.println("TRIGGER 1 ACTIVATED");
  fall = true;
  if (fall == true) {  //in event of a fall detection
  Serial.println("FALL DETECTED");
  digitalWrite(5, LOW);
  delay(20);
  digitalWrite(5, HIGH);
  fall = false;
  // exit(1);
}}}

void mpu_read() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);  //00 250, 08 500, 10 1000, 18 2000
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 14);        // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void calibrate_imu() {
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000; cal_int++) {  //Take 2000 readings for calibration.
    mpu_read();                                   //Read the gyro output.
    gyro_axis_cal[1] += GyX;                      //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += GyY;                      //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += GyZ;                      //Ad yaw value to gyro_yaw_cal.

    acc_axis_cal[1] += AcX;
      acc_axis_cal[2] += AcY;
        acc_axis_cal[3] += AcZ;
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;  //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;  //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;
 
     acc_axis_cal[1] /= 2000;
      acc_axis_cal[2] /= 2000;
        acc_axis_cal[3] /= 2000;

}