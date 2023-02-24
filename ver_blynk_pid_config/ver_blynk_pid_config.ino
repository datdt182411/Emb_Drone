
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
#define max_rp 400
#define max_rp_yaw 400
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
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
unsigned int esc_value[4];
// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double input_pid[3];
double input_cmd[3];
double ypr_sp[3] = { 0, 0, 0 };
double setpoint_set[3] = { 0, 0, 0 };
int16_t gyro[3];
double output_pid[3];
///
int16_t gyro_x, gyro_y, gyro_z, temperature;
float angle_pitch, angle_roll, angle_yaw;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float elapsedTime;
long Time, timePrev, time2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
///
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
long acc_x, acc_y, acc_z, acc_total_vector;
bool pitch_state = 0;
bool roll_state = 0;
bool esc_setup = 0;
bool esc_done = 0;

///
unsigned long time_loop;
unsigned int throttle_val = 1000;
unsigned int throttle_val_set = 1000;
double p_roll = 1.25, i_roll = 0.3, d_roll = 0.041;
double p_pitch = p_roll, i_pitch = i_roll, d_pitch = d_roll;
double p_yaw = 4, i_yaw = 0.02, d_yaw = 0;
bool get_data = 0;
String data_pid;
PID pid_pitch(&input_pid[pitch_], &output_pid[pitch_], &ypr_sp[pitch_], p_pitch, i_pitch, d_pitch, DIRECT);
PID pid_roll(&input_pid[roll_], &output_pid[roll_], &ypr_sp[roll_], p_roll, i_roll, d_roll, DIRECT);
PID pid_yaw(&input_pid[yaw_], &output_pid[yaw_], &ypr_sp[yaw_], p_yaw, i_yaw, d_yaw, DIRECT);
//
///EEPROM
bool start_pid = 0;
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
  // p_roll =0;
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
  pid_yaw.SetSampleTime(5);
  pid_roll.SetSampleTime(5);
  pid_pitch.SetSampleTime(5);
  pid_yaw.SetMode(AUTOMATIC);
  pid_roll.SetMode(AUTOMATIC);
  pid_pitch.SetMode(AUTOMATIC);
 
}
void pid_change() {

  pid_pitch.SetTunings(p_pitch, i_pitch, d_pitch);
  pid_roll.SetTunings(p_roll, i_roll, d_roll);
  pid_yaw.SetTunings(p_yaw, i_yaw, d_yaw);
  /*
  pid_roll.change_pid();
  pid_pitch.change_pid();
  pid_yaw.change_pid();
  */
  /*
  String lm = "pitch:" + String(p_pitch) + "-yaw:" + String(p_yaw);
  Serial.println(lm);
  double ki_get = pid_roll.GetKi();
  double kd_get = pid_roll.GetKd();
  Serial.println();
  Serial.print(ki_get, 5);
  Serial.print(",");
  Serial.print(kd_get, 4);
   Serial.println();
   */

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
  data_pid = param.asStr();
  get_data = 1;
}
BLYNK_WRITE(V4) {
  throttle_val_set = param.asInt();

  if (debug) {
    for (int i = 0; i < 4; i++) {
      sv[i]->writeMicroseconds(throttle_val_set);
    }
  }
  Serial.println(throttle_val_set);
}
BLYNK_WRITE(V5) {
  setpoint_set[pitch_] = param.asFloat();  // -15 -> 15
  // Serial.println(setpoint_set[pitch_]);
}
BLYNK_WRITE(V6) {
  setpoint_set[roll_] = param.asFloat();  // -15 -> 15
  // Serial.println(setpoint_set[roll_]);
}
BLYNK_WRITE(V7) {
  int throt_setup = param.asInt();
  esc_setup = 1;
}
BLYNK_WRITE(V8) {
  stop_ = param.asInt();
}
BLYNK_WRITE(V9) {
  setpoint_set[pitch_] = 0;
  setpoint_set[roll_] = 0;
}
void myTimerEvent() {
  static unsigned long time_ = millis();
  if (millis() - time_ > 1000) {
    time_ = millis();
    Blynk.virtualWrite(V1, ypr[1] * 180 / M_PI);
    Blynk.virtualWrite(V0, ypr[2] * 180 / M_PI);*                                                                                                                                     
  }
}
void init_mpu() {

  delay(500);
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
  mpu.setXGyroOffset(10);
  mpu.setYGyroOffset(-13);
  mpu.setZGyroOffset(29);
  mpu.setZAccelOffset(1556);
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();
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
  delay(3000);
  for (int j = 0; j < 4; j++) {
    sv[j]->writeMicroseconds(1000);
  }
  delay(2000);
  yield();
}

void calculate_pid() {
  // for (int i = 0; i < 3; i++) {
  //   if(mode_ == flight_mode || i ==0)input_pid[i] = (float)gyro[i]/100.0;  // 0-250 degree per second ->131* 0 -> 32750 -> /100-> 0->3275
  //   else{
  //     input_pid[i]=
  //   }
  //ypr_sp[i]
  input_pid[yaw_] = (float)gyro[yaw_] / 131.0;

 
    ypr_sp[pitch_] = setpoint_set[pitch_];
    ypr_sp[roll_] = setpoint_set[roll_];
    input_pid[pitch_] = rad_to_deg(ypr[pitch_]);
    input_pid[roll_] = rad_to_deg(ypr[roll_]);
  
  /// 2 mode : goc va toc do goc phan biet bang throttle>1500?goc:tocdo // tocdo -> -90->90 goc: -15->15 . yaw -> -45->45
  pid_pitch.Compute();
  pid_roll.Compute();
  pid_yaw.Compute();
  int dir_ = 1;
  throttle_val = 0.3 * throttle_val + 0.7 * throttle_val_set;                                         // pitch_angle = 40 -> roll = 0 -> output = -40
  esc_value[0] = throttle_val + dir_ * (-output_pid[pitch_] + output_pid[roll_] + output_pid[yaw_]);  //esc_red1 cw right throttle =1500
  esc_value[1] = throttle_val + dir_ * (output_pid[pitch_] - output_pid[roll_] + output_pid[yaw_]);   //esc_red2 cw left
  esc_value[2] = throttle_val + dir_ * (output_pid[pitch_] + output_pid[roll_] - output_pid[yaw_]);   //esc_black right ( roll = red1 yaw,pitch!)
  esc_value[3] = throttle_val + dir_ * (-output_pid[pitch_] - output_pid[roll_] - output_pid[yaw_]);  // ccw left
  for (int i = 0; i < 4; i++) {
    if (esc_value[i] < 1050) esc_value[i] = 1050;
    else if (esc_value[i] > 2000) esc_value[i] = 2000;
  }

  if (!debug) {
    for (int i = 0; i < 4; i++) {
      if (stop_) {
        esc_value[i] = 1000;
      }
      sv[i]->writeMicroseconds(esc_value[i]);
    }
    // unsigned long time_delay = millis() - time_loop;
    // time_loop = millis();
    // Serial.println("time_dl:" + String(time_delay));
  } else {
    static unsigned long time_ = millis();
    if (millis() - time_ > 800) {
      String c = "";
      for (int i = 0; i < 4; i++) {
        c += String(esc_value[i]) + ",";
        Serial.print(esc_value[i]);
        Serial.print(",");
      }
      Serial.println(""); 
      if (use_blynk) send_terminal(c);
      time_ = millis();
    }
    static int count_s = 0;
    if (count_s <= 1000) {
      static unsigned long time_delay = millis();
      unsigned long time_bb = millis() - time_delay;
      time_delay = millis();
      Serial.println(time_bb);
      count_s++;
    }
  }
}
void setup() {
  //init_esc();
  Serial.begin(9600);              // Debug-tim loi
  pinMode(LED_PIN, OUTPUT);       // Khoi tao Led (Out: Lam sang)
  digitalWrite(LED_PIN, LOW);     
  init_mpu();                     // Khoi tao giao thuc I2C ( tin hieu so)
  digitalWrite(LED_PIN, HIGH);    //Xuat tin hieu
  double angle_x = 0, angle_y = 0;
  int count_led = 0;
  bool led_state;
  float acc_total_vector;
  while (1) {  //
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      acc_total_vector = sqrt((aaReal.x * aaReal.x) + (aaReal.y * aaReal.y) + (aaReal.z * aaReal.z));  //Calculate the total accelerometer vector
      if (abs(aaReal.y) < acc_total_vector) {                                                          //Prevent the asin function to produce a NaN
        angle_pitch_acc = asin((float)aaReal.y / acc_total_vector);                                    //Calculate the pitch angle.
      }
      if (abs(aaReal.x) < acc_total_vector) {                            //Prevent the asin function to produce a NaN
        angle_roll_acc = asin((float)aaReal.x / acc_total_vector) * -1;  //Calculate the roll angle.
      }
      ypr[pitch_] = ypr[pitch_] * 0.98 + angle_pitch_acc * 0.02;
      ypr[roll_] = ypr[roll_] * 0.98 + angle_roll_acc * 0.02;
      angle_x += ypr[roll_];
      angle_y += ypr[pitch_];
      count_led++;
      if (count_led % 50 == 0) {
        digitalWrite(LED_PIN, led_state);
        led_state = 1 - led_state;
      }
      if (count_led >= 500) break;
    }
    yield();
  }
  setpoint_set[roll_] = (double)angle_x / 500.0;
  setpoint_set[pitch_] = (double)angle_y / 500.0;
  yield();
  for (int i = 0; i < 3; i++) {
    Serial.print(setpoint_set[i]);
    Serial.print(",");
  }
  BlynkEdgent.begin();
  if (start_pid == 1) {
    init_pid();
  }
}
void get_mpu() {
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
    Serial.print(ypr[1]);
     Serial.print(ypr[2]);
      Serial.println("");
    /*
    static unsigned long time_mpu = millis();
    unsigned long time_bw = millis() - time_mpu;
    time_mpu = millis();
    Serial.println("mpu:" + String(time_bw));

    */
    if (start_pid != 0) calculate_pid();
    else {
      if (stop_ == 1) {
        init_pid();
        start_pid = 1;
      }
    }
  }
}
void loop() {
  BlynkEdgent.run();
  if (esc_setup == 1 && esc_done == 0) {
    init_esc();
    esc_done = 1;
  }
  get_mpu();

  if (get_data == 1) {
    String c = data_pid;
    int ind = c.indexOf("t");
    if (ind != -1) {
      int d = (int)(c[ind + 1] - '0');
      Serial.println(d);
      int p_ind = c.indexOf("p");
      int i_ind = c.indexOf("i");
      int d_ind = c.indexOf("d");
      /*
      if (d == 1) {
        p_pitch = c.substring(p_ind + 1, i_ind).toFloat();
        i_pitch = c.substring(i_ind + 1, d_ind).toFloat();
        d_pitch = c.substring(d_ind + 1).toFloat();


        if (start_pid == 1) pid_change();

      }
      */
      if (d == 2) {
        p_roll = c.substring(p_ind + 1, i_ind).toFloat();
        i_roll = c.substring(i_ind + 1, d_ind).toFloat();
        d_roll = c.substring(d_ind + 1).toFloat();
        p_pitch = p_roll;
        i_pitch = i_roll;
        d_pitch = d_roll;

        if (start_pid == 1) pid_change();
      } else if (d == 3) {
        p_yaw = c.substring(p_ind + 1, i_ind).toFloat();
        i_yaw = c.substring(i_ind + 1, d_ind).toFloat();
        d_yaw = c.substring(d_ind + 1).toFloat();

        //send_terminal(c);
        if (start_pid == 1) pid_change();
      }
      /*
      String xa = String(p_pitch) + "," + String(i_pitch) + "," + String(d_pitch);
      String xy = String(p_roll) + "," + String(i_roll) + "," + String(d_roll);
      String xz = String(p_yaw) + "," + String(i_yaw) + "," + String(d_yaw);
      String sr = xa + xy + xz;
      Serial.println(sr);
      */
      //send_terminal(c);
    }
    int l_ind = c.indexOf("l");
    if (l_ind != -1) {
      terminal.clear();
    }
    //send_terminal(c);
    get_data = 0;
  }
  myTimerEvent();
}
