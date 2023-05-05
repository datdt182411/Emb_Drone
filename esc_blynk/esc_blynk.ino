
// Fill-in information from your Blynk Template here
//#define BLYNK_TEMPLATE_ID           "TMPLxxxxxx"
//#define BLYNK_DEVICE_NAME           "Device"
#define BLYNK_TEMPLATE_ID "TMPLx_9mwFQ4"
#define BLYNK_DEVICE_NAME "Drone"
#define BLYNK_AUTH_TOKEN "-j6TwRq8JHPiYSotM1zk6dyYnyrnlxIA"

#define BLYNK_FIRMWARE_VERSION "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
#define yaw_ 0
#define pitch_ 1
#define roll_ 2
#define APP_DEBUG
#define min_val 1000
#define max_val 2000
#define max_rp 100
#define max_rp_yaw 50
#define yaw_min -max_rp_yaw
#define yaw_max max_rp_yaw
#define roll_min -max_rp
#define roll_max max_rp
#define pitch_min -max_rp
#define pitch_max max_rp
#define debug 0
#define LED_PIN D3
#define stabi 0
#define flight_move 1
#define rad_to_deg(x) x * 180.0 / M_PI
#define use_blynk 1
bool mode_;

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI
#include "I2Cdev.h"

#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
#include "BlynkEdgent.h"
#include "Servo.h"
////

//
BlynkTimer timer;  // Announcing the timer

Servo esc_red_1;
Servo esc_black_1;
Servo esc_red_2;
Servo esc_black_2;
Servo *sv[4]{
  &esc_red_1,
  &esc_red_2,
  &esc_black_1,
  &esc_black_2
};
//////

bool blinkState = false;                                                                
WidgetTerminal terminal(V3);                                                            
// MPU control/status vars
bool dmpReady = false;   
uint8_t mpuIntStatus;    
uint8_t devStatus;      
uint16_t packetSize;     
uint16_t fifoCount;      
uint8_t fifoBuffer[64];  
unsigned int esc_value[4];
// orientation/motion vars
Quaternion q;         
VectorInt16 aa;       
VectorInt16 aaReal;   
VectorInt16 aaWorld;  
VectorFloat gravity;  
float euler[3];       
float ypr[3];         
double input_pid[3];
double ypr_sp[3] = { 0, 0, 0 };
double setpoint_set[3] = { 0, 0, 0 };
int16_t gyro[3];
double output_pid[3];
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };         
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;                                                            
float angle_pitch_output, angle_roll_output;                                                      
///
unsigned int throttle_val = 1000;
double p_pitch = 8.6, i_pitch = 0, d_pitch = 0;
double p_roll = p_pitch, i_roll = i_pitch, d_roll = d_pitch;
double p_yaw = 0, i_yaw = 0, d_yaw = 0;
PID pid_pitch(&input_pid[pitch_], &output_pid[pitch_], &ypr_sp[pitch_], p_pitch, i_pitch, d_pitch, DIRECT);
PID pid_roll(&input_pid[roll_], &output_pid[roll_], &ypr_sp[roll_], p_roll, i_roll, d_roll, DIRECT);
PID pid_yaw(&input_pid[yaw_], &output_pid[yaw_], &ypr_sp[yaw_], p_yaw, i_yaw, d_yaw, DIRECT);
//
///EEPROM
typedef union int16_ty {
  int16_t d;
  byte b[2];
};
typedef union float_ty {
  float d;
  uint8_t b[4];
};
void write_int16(int pos, int16_t d) {
  int16_ty loc;
  loc.d = d;
  EEPROM.write(pos++, loc.b[0]);
  EEPROM.write(pos++, loc.b[1]);
}
int16_t read_int16(int pos) {
  int16_ty loc;
  loc.b[0] = EEPROM.read(pos++);
  loc.b[1] = EEPROM.read(pos++);
  return loc.d;
}

void write_float(int pos, float d) {
  float_ty loc;
  loc.d = d;
  EEPROM.write(pos++, loc.b[0]);
  EEPROM.write(pos++, loc.b[1]);
  EEPROM.write(pos++, loc.b[2]);
  EEPROM.write(pos++, loc.b[3]);
}
float read_float(int pos) {
  float_ty loc;
  loc.b[0] = EEPROM.read(pos++);
  loc.b[1] = EEPROM.read(pos++);
  loc.b[2] = EEPROM.read(pos++);
  loc.b[3] = EEPROM.read(pos++);
  return loc.d;
}
void PID_Read() {
  int pos_begin = 110;
  if (EEPROM.read(132) == 0xAA) {
    p_pitch = read_float(pos_begin);
    pos_begin += 4;
    i_pitch = read_float(pos_begin);
    pos_begin += 4;
    d_pitch = read_float(pos_begin);
    // p_roll = p_pitch;
    // i_roll = i_pitch;
    // d_roll = d_pitch;
  }
  if (EEPROM.read(134) == 0xAA) {
    pos_begin += 4;
    p_yaw = read_float(pos_begin);
    pos_begin += 4;
    i_yaw = read_float(pos_begin);
    pos_begin += 4;
    d_yaw = read_float(pos_begin);
  }
  String c = "pitch:" + String(p_pitch) + "," + String(i_pitch) + "," + String(d_pitch) + "-yaw:" + String(p_yaw) + "," + String(i_yaw) + "," + String(d_yaw);
  if (use_blynk) send_terminal(c);
}
void PID_Store(int pitch_yaw) {
  int pos_begin = 110;
  if (pitch_yaw == pitch_) {
    write_float(pos_begin, p_pitch);
    pos_begin += 4;
    write_float(pos_begin, i_pitch);
    pos_begin += 4;
    write_float(pos_begin, d_pitch);
    EEPROM.write(132, 0xAA);
  } else {
    pos_begin += 12;
    write_float(pos_begin, p_yaw);
    pos_begin += 4;
    write_float(pos_begin, i_yaw);
    pos_begin += 4;
    write_float(pos_begin, d_yaw);  // 130
    EEPROM.write(134, 0xAA);
  }
  EEPROM.commit();
}

////
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void init_pid() {
  //PID_Read();
  p_roll = p_pitch / 3.0;
  //  i_roll = i_pitch;
  //  d_roll = d_pitch;
  String lm = "pitch:" + String(p_pitch) + "," + String(i_pitch) + "," + String(d_pitch) + "-yaw:" + String(p_yaw) + "," + String(i_yaw) + "," + String(d_yaw);
  Serial.println(lm);
  pid_pitch.SetTunings(p_pitch, i_pitch, d_pitch);
  pid_roll.SetTunings(p_roll, i_roll, d_roll);
  pid_yaw.SetTunings(p_yaw, i_yaw, d_yaw);
  pid_pitch.SetOutputLimits(pitch_min, pitch_max);
  pid_roll.SetOutputLimits(roll_min, roll_max);
  pid_yaw.SetOutputLimits(yaw_min, yaw_max);
  pid_yaw.SetSampleTime(10);
  pid_roll.SetSampleTime(10);
  pid_pitch.SetSampleTime(10);
  pid_yaw.SetMode(AUTOMATIC);
  pid_roll.SetMode(AUTOMATIC);
  pid_pitch.SetMode(AUTOMATIC);
}
void dmpDataReady() {
  mpuInterrupt = true;
}
void send_terminal(String data_send) {
  terminal.println(data_send);
  terminal.flush();
}
bool stop_ = 0;
BLYNK_WRITE(V3) {
  String c = param.asStr();
  int ind = c.indexOf("t");
  if (ind != -1) {
    int d = (int)(c[ind + 1] - '0');
    Serial.println(d);
    int p_ind = c.indexOf("p");
    int i_ind = c.indexOf("i");
    int d_ind = c.indexOf("d");
    if (d == 1) {
      p_pitch = c.substring(p_ind + 1, i_ind).toFloat();
      i_pitch = c.substring(i_ind + 1, d_ind).toFloat();
      d_pitch = c.substring(d_ind + 1).toFloat();
      // String c = String(p_pitch) + "," + String(i_pitch) + "," + String(d_pitch);
      // send_terminal(c);
      PID_Store(pitch_);
      PID_Read();
    } else {
      p_yaw = c.substring(p_ind + 1, i_ind).toFloat();
      i_yaw = c.substring(i_ind + 1, d_ind).toFloat();
      d_yaw = c.substring(d_ind + 1).toFloat();
      // String c = String(p_yaw) + "," + String(i_yaw) + "," + String(d_yaw);
      // send_terminal(c);
      PID_Store(yaw_);
      PID_Read();
    }
    //send_terminal(c);
  }
  // int s_ind = c.indexOf("s");
  // if (s_ind != -1) {
  //   String lm = "pitch:" + String(p_pitch) + "," + String(i_pitch) + "," + String(d_pitch) + "-yaw:" + String(p_yaw) + "," + String(i_yaw) + "," + String(d_yaw);
  //   send_terminal(lm);
  // }
  int l_ind = c.indexOf("l");
  if (l_ind != -1) {
    terminal.clear();
  }
  //send_terminal(c);
}
BLYNK_WRITE(V4) {
  throttle_val = param.asInt();

  if (debug) {
    for (int i = 0; i < 4; i++) {
      sv[i]->writeMicroseconds(throttle_val);
    }
  }
  Serial.println(throttle_val);
}
BLYNK_WRITE(V5) {
  setpoint_set[pitch_] = param.asFloat();  // -15 -> 15
  Serial.println(ypr_sp[pitch_]);
}
BLYNK_WRITE(V6) {
  setpoint_set[roll_] = param.asFloat();  // -15 -> 15
  Serial.println(ypr_sp[roll_]);
}
BLYNK_WRITE(V7) {
  mode_ = param.asInt();
}
BLYNK_WRITE(V8) {
  stop_ = param.asInt();
}
void myTimerEvent() {

  Blynk.virtualWrite(V1, ypr[1] * 180 / M_PI);
  Blynk.virtualWrite(V0, ypr[2] * 180 / M_PI);
}
void init_mpu() {
  //  join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize device

  mpu.initialize();

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection

    //  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(150);
  }
}
//
void init_esc() {
  esc_red_1.attach(D4, 500, 2400);
  esc_red_2.attach(D5, 500, 2400);
  esc_black_1.attach(D6, 500, 2400);
  esc_black_2.attach(D7, 500, 2400);
  for (int i = 0; i < 4; i++) {
    sv[i]->writeMicroseconds(2000);
  }
  delay(2000);
  for (int j = 0; j < 4; j++) {
    sv[j]->writeMicroseconds(1000);
  }
  delay(2000);
}
void calculate_pid() {
  // for (int i = 0; i < 3; i++) {
  //   if(mode_ == flight_mode || i ==0)input_pid[i] = (float)gyro[i]/100.0;  // 0-250 degree per second ->131* 0 -> 32750 -> /100-> 0->3275
  //   else{
  //     input_pid[i]=
  //   }
  //ypr_sp[i]
  input_pid[yaw_] = (float)gyro[yaw_] / 131.0;
  if (mode_ == flight_move) {
    ypr_sp[pitch_] = setpoint_set[pitch_] * 6.0;
    ypr_sp[roll_] = setpoint_set[roll_] * 6.0;
    input_pid[pitch_] = (float)gyro[pitch_] / 100.0;
    input_pid[roll_] = (float)gyro[roll_] / 100.0;
  } else {
    ypr_sp[pitch_] = setpoint_set[pitch_];
    ypr_sp[roll_] = setpoint_set[roll_];
    input_pid[pitch_] = rad_to_deg(ypr[pitch_]);
    input_pid[roll_] = rad_to_deg(ypr[roll_]);
  }
  
  pid_pitch.Compute();
  pid_roll.Compute();
  pid_yaw.Compute();
  int dir_ = 1;                                                                                       // pitch_angle = 40 -> roll = 0 -> output = -40
  esc_value[0] = throttle_val + dir_ * (-output_pid[pitch_] + output_pid[roll_] - output_pid[yaw_]);  //esc_red1 ccw right throttle =1500
  esc_value[1] = throttle_val + dir_ * (output_pid[pitch_] - output_pid[roll_] - output_pid[yaw_]);   //esc_red2 ccw left
  esc_value[2] = throttle_val + dir_ * (output_pid[pitch_] + output_pid[roll_] + output_pid[yaw_]);   //esc_black right ( roll = red1 yaw,pitch!)
  esc_value[3] = throttle_val + dir_ * (-output_pid[pitch_] - output_pid[roll_] + output_pid[yaw_]);  // cw left


  for (int i = 0; i < 4; i++) {
    if (esc_value[i] < 1050) esc_value[i] = 1050;
    else if (esc_value[i] > throttle_val + max_rp) esc_value[i] = throttle_val + max_rp;
  }

  if (!debug) {
    for (int i = 0; i < 4; i++) {
      if (stop_) {
        esc_value[i] = 1000;
      }
      sv[i]->writeMicroseconds(esc_value[i]);
    }
  }
  static unsigned long time_ = millis();
  if (millis() - time_ > 800) {
    String c = "";
    for (int i = 0; i < 4; i++) {
      c += String(esc_value[i]) + ",";
      Serial.print(esc_value[i]);
      Serial.print(",");
    }
    Serial.println();
    c += String(esc_value[3]);
    Serial.println();
    if (use_blynk) send_terminal(c);
    time_ = millis();
  }
}
void setup() {
  init_esc();
  Serial.begin(9600);
  delay(100);
  init_mpu();
  if (use_blynk) BlynkEdgent.begin();
  init_pid();
  if (use_blynk) timer.setInterval(200L, myTimerEvent);
}
void loop() {
  if (use_blynk) BlynkEdgent.run();
  if (use_blynk) timer.run();  // running timer every second
  if (!dmpReady) return;
  static unsigned long time_ = millis();
  if (millis() - time_ > 1000) {
    time_ = millis();
  }
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.getRotation(&gyro[roll_], &gyro[pitch_], &gyro[yaw_]);                                             // 131.0
    float acc_total_vector = sqrt((aaReal.x * aaReal.x) + (aaReal.y * aaReal.y) + (aaReal.z * aaReal.z));  //Calculate the total accelerometer vector
    if (abs(aaReal.y) < acc_total_vector) {                                                                //Prevent the asin function to produce a NaN
      angle_pitch_acc = asin((float)aaReal.y / acc_total_vector);                                          //Calculate the pitch angle.
    }
    if (abs(aaReal.x) < acc_total_vector) {                            //Prevent the asin function to produce a NaN
      angle_roll_acc = asin((float)aaReal.x / acc_total_vector) * -1;  //Calculate the roll angle.
    }
    ypr[1] = ypr[1] * 0.98 + angle_pitch_acc * 0.02;
    ypr[2] = ypr[2] * 0.98 + angle_roll_acc * 0.02;
    calculate_pid();
  }
}
