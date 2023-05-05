#include <Wire.h>
#include <Arduino.h>

//#define DEBUG_Acc

//define the Gyro variables
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;

//define the Acc variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float rad_to_deg = 180/3.141592654;

//define kalman filter variables
  //define the initial prediction and uncertainty
float KalmanAngleRoll=0, 
      KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch = 0,
      KalmanUncertaintyAnglePitch = 2 * 2;
// initialize the output of the filter
float Kalman1DOutput[] = {0, 0};
float kalmancst = 0.035;

void IMU_signals(){
  //switch on the low-pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);//bandwidth 10Hz
  Wire.endTransmission();

  //configure the Acc output
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08); //scale range 4g
  Wire.endTransmission();

 Wire.beginTransmission(0x68);
  Wire.write(0x1D);
  Wire.write(0x06); //DLPF 5.05Hz
  Wire.endTransmission();

  //access registres storing Acc measurments
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  // pull information from the 6 Acc's registres from 3B to 40
  int16_t AccXLSB = Wire.read()<<8|Wire.read();
  int16_t AccYLSB = Wire.read()<<8|Wire.read();
  int16_t AccZLSB = Wire.read()<<8|Wire.read();

  //set the sensitivity scale factor for gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  //access registres storing gyro measurments
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  // pull information from the 6 gyro's registres from 43 to 48
  int16_t GyroX = Wire.read()<<8|Wire.read();
  int16_t GyroY = Wire.read()<<8|Wire.read();
  int16_t GyroZ = Wire.read()<<8|Wire.read();

  //convert the Gyro's measurment units from LSB to °/s
  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  //convert the Acc's measurment units from LSB to g
  //make sure to substract or add the defrance between the Acc values and 1 for each axes(in all 3 direction)!!!!!!! 
  AccX = ((float)AccXLSB/8192)-0.08; //8192 is the LSB sensitivity for 4g
  AccY = ((float)AccYLSB/8192)-0.08;
  AccZ = ((float)AccZLSB/8192)+0.15;
  #ifdef DEBUG_Acc
  Serial.print("AccX: ");
  Serial.print(AccX);
  Serial.print(" AccY: ");
  Serial.print(AccY);
  Serial.print(" AccZ: ");
  Serial.println(AccZ);
  #endif
  // calculate the Absolute angels of the Acc
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*rad_to_deg;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*rad_to_deg;
}

void Gyro_calibration_values(){
  Serial.println("Start Gyro calibration");
  // calibrate the gyro measurments
  for(int i=0; i<2000; i++){
    IMU_signals();
    RateCalibrationPitch += RatePitch;
    RateCalibrationRoll += RateRoll;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  //calculate the calibration values
  RateCalibrationPitch /= 2000;
  RateCalibrationRoll /= 2000;
  RateCalibrationYaw /=2000;

  Serial.println("Gyro calibration Done");
}

//function for kalman filter
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement){
  KalmanState += kalmancst * KalmanInput;
  KalmanUncertainty += kalmancst * kalmancst * 20 * 20;
  float KalmanGain = KalmanUncertainty /(KalmanUncertainty+20*20);
  KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty*=(1-KalmanGain); 

  Kalman1DOutput[0]=KalmanState;
  Kalman1DOutput[1]=KalmanUncertainty;
}

float getPitch(){
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  return KalmanAnglePitch;
}
float getRoll(){
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  return KalmanAngleRoll;
}

void setup_mpu9250(){
  Serial.println("Start MPU9250 setting....");
  // According to the datasheet, the maximum I2C clock frequency supported by the MPU9250 is 400 kHz
 
  Wire.begin();
   Wire.setClock(400000);
  delay(250);

  //start the gyro in power mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Gyro_calibration_values();
  Serial.println("MPU9250 setting Done!");
}

//in your void loop() write those line to get the filtred pitch and roll

// void loop(){
//   IMU_signals();
//   RateRoll-=RateCalibrationRoll;
//   RatePitch-=RateCalibrationPitch;
//   RateYaw-=RateCalibrationYaw;

//   //now start the kalman filter function, and get roll and pitch.
//   float Roll=getRoll();
//   float Pitch=getPitch();
  
// }

