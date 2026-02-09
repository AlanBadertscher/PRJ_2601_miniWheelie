/* Usage:
 * - Select Tools->Board "ESP32S3 Dev Module" with default settings.
 * - Select the right Tools->Port.
 * - Place the robot in a fixture so the wheels don't touch ground. 
 * - Make sure the robot is horizontal in the XY plane.
 * - Upload the sketch SBRobot_IMU_Zero
 * - Open Serial Monitor, set baud rate to 115200
 * - Reset the robot and wait for the sketch to finish (takes a few minutes)
 * - Enter the offset values below (for X_OFFSET_ACCEL, etc.)
 * - Upload & run the sketch SBRobot_Motor_Compare. Follow the instructions at the
 *   top of the sketch to find the minimum motor speed (MOTOR_MIN_ABS_SPEED) and 
 *   motor speed corrections motor_speed_multiplier_left and motor_speed_multiplier_right.
 * - With the adjustments made, upload this sketch to the robot.
 * - Disconnect, switch off, switch on and place on the floor. 
 * - It may be surprising, but the robot balances best on a rough surface.
 * - A power monitor is available for monitoring battery voltage and motor power.
 *     This can be used to adapt PLC response to battery level.
 */
#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <DFRobot_INA219.h>
#define INA219_I2C_ADDRESS  (0x40)
DFRobot_INA219_IIC power_monitor(&Wire,INA219_I2C_ADDRESS);
boolean power_monitor_ok = false;

#define _HAS_DABBLE
#ifdef _HAS_DABBLE
  // Dabble stuff for Bluetooth control.
  #define CUSTOM_SETTINGS
  #define INCLUDE_GAMEPAD_MODULE
  #include <DabbleESP32.h>
  #define BLUETOOTH_NAME  "SBRobot"
#else
  #define BLUETOOTH_NAME  "SBRobot Sim"
#endif /* _HAS_DABBLE */

// Display & Graphics
#include <Arduino_GFX_Library.h> /* Use v1.4.6, higher versions may conflict with SPI driver! */
#include "SD_MMC.h"
#include <FS.h>
#include "JpegFunc.h"

#define SOFTWARE_VERSION  "SBRobot v17022025"

/* cpv - IMU x-axis is supposed to run forward/backward, y-axis is supposed to run left/right.
 * In case they are swapped, you can try defining SWAP_XY. */
//#define SWAP_XY 
/* cpv - Uncomment to run sketch without motors. This is practical when adding more functions or when debugging. */
//#define MOTORS_DISABLED
/* cpv - Uncomment to send debug & status information on the serial port. */
//#define __VERBOSE__

//#define LOG_INPUT  (1)
#define OK_K  "/ok-horizontal.jpg"
#define OK_V  "/ok-vertical.jpg"
#define ARROW_D  "/arrow_u.jpg"
#define ARROW_U  "/arrow_d.jpg"
#define ARROW_R  "/arrow_l.jpg"
#define ARROW_L  "/arrow_r.jpg"
uint8_t image_id = 0xff; // undefined.

typedef enum
{
  kIdle = 0,
  kForward,
  kBackward,
  kLeft,
  kRight,
}
joystick_t;

joystick_t joystick = kIdle;

// SD card connections.
#define PIN_SD_CMD  2
#define PIN_SD_CLK  42
#define PIN_SD_D0  41

// TFT connctions
#define TFT_CS  15
#define TFT_MOSI  13
#define TFT_MISO  12
#define TFT_SCLK  14
#define TFT_DC  21
Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(TFT_DC,TFT_CS,TFT_SCLK,TFT_MOSI,TFT_MISO,HSPI,true);
// Graphics defines
#define TFT_BLK  48
#define TFT_RES  -1
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RES, 0 /* rotation */, true /* IPS */);

// Ultrasonic transducer
#define ULTRASONIC_TRIG  (3)
#define ULTRASONIC_ECHO  (8)

/* cpv - Use the sketch SBRobot_IMU_Zero to find the offset values for the IMU.
 * Example final output (reformatted slightly for readability):
 * XAccel                    YAccel                ZAccel                        XGyro               YGyro                 ZGyro
 * [-2387,-2385] -->  [-9,4] [975,976] --> [-6,14] [1242,1243] --> [16379,16393] [102,103] --> [0,3] [-60,-59] --> [-2,1]  [72,73] --> [-1,1]
 * [-2387,-2386] -->  [-9,1] [975,976] --> [-9,14] [1242,1242] --> [16379,16385] [102,103] --> [0,3] [-60,-59] --> [-2,1]  [72,73] --> [-3,1]
 * [-2387,-2386] --> [-16,1] [975,976] --> [-8,14] [1242,1242] --> [16383,16385] [102,103] --> [0,3] [-60,-59] --> [-2,1]  [72,73] --> [-2,1]
 * -------------- done --------------*/
#define X_OFFSET_ACCEL (-115)
#define Y_OFFSET_ACCEL   (1140)
#define Z_OFFSET_ACCEL  (963)
#define X_OFFSET_GYRO    (80)
#define Y_OFFSET_GYRO    (-45)
#define Z_OFFSET_GYRO     (-37)

// MOTOR CONTROLLER
/* cpv - Set this to the largest minimum value of the two motors. */
#define MOTOR_MIN_ABS_SPEED  (50)
// Motor 0 - left motor when ESP32S3 antenna is pointing forward.
const int ENA = 46;
const int IN1 = 7;
const int IN2 = 18;
const int CA1 = 16;
const int CA2 = 17;
// Motor 1 - right motor when ESP32S3 antenna is pointing forward.
const int ENB = 45;
const int IN3 = 10;
const int IN4 = 11;
const int CB1 = 9;
const int CB2 = 47;
/* cpv - Set one of the speed multiplier values to 1.0 and the other to the ratio that is less than 1.0.
 * A motor cannot go faster than 1, so slow down the fastest motor.
 * Use the sketch SBRobot_Motor_Compare to find these values (e.g. L = 255, R = 247, R/L = 0.97, L/R = 1.03) */
double motor_speed_multiplier_left = 0.99; // R_max/L_max
double motor_speed_multiplier_right = 1.03; // L_max/R_max
LMotorController motor_controller(ENB,IN4,IN3,ENA,IN1,IN2,motor_speed_multiplier_left,motor_speed_multiplier_right);
uint8_t motors_enabled = 1;

/* cpv - SBRobot's I2C bus pin definitions. */
#define MPU6050_SDA  (5)
#define MPU6050_SCL  (4)
#define MPU6050_INT  (6)

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID
#define SPEED_OFFSET  (40) /* Controls left/right turning speed (in motor speed units). */
double speed_offset_l = 0;
double speed_offset_r = 0;
#define TILT_OFFSET  (2) /* This controls the forward/backward speed (in degrees!). */
double tilt_offset = 0;
double originalSetpoint = 180.0; //175.8;
double setpoint = originalSetpoint;
double input, output;
/* cpv - Kp, Kd & Ki are user-adjustable values. Try other values to see how they affect the robot's stability.
 * These values worked well for me. */
double Kp = 28;
double Kd = 40;
double Ki = 2;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void IRAM_ATTR dmpDataReady()
{
  mpuInterrupt = true;
}

//int offset_adjust(int offset, int delta)
//{
//  offset += delta;
//  offset = min(offset,255);
//  offset = max(offset,-255);
//  return offset;
//}


// pixel drawing callback
static int jpegDrawCallback(JPEGDRAW *pDraw)
{
  // Serial.printf("Draw pos = %d,%d. size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
}

void image_show(char *p_filename)
{
  jpegDraw(p_filename,jpegDrawCallback,true,0,0,gfx->width(),gfx->height()); // x, y, w, h
}

void show_float(float value)
{
  static float value_last = 0.0;
  uint8_t textsize = 7;
  uint8_t rotation = gfx->getRotation();
  gfx->setRotation(1);
  gfx->setTextSize(textsize);
  uint8_t x = (320-5*textsize*6)/2; // 5 chars including decimal point, 6 is default char width.
  uint8_t y = (240-textsize*8)/2; // 8 is default char height.
  //gfx->fillRect(x,y,5*textsize*6,textsize*8,WHITE);
  gfx->setCursor(x,y);
  gfx->setTextColor(WHITE);
  gfx->print(value_last,1);
  gfx->setCursor(x,y);
  gfx->setTextColor(BLUE);
  gfx->print(value,1);
  value_last = value;
  gfx->setRotation(rotation);
}

void print_center_x(char *p_str, uint8_t textsize, int y)
{
  int x = (gfx->width()-strlen(p_str)*6*textsize)/2;
  gfx->setTextSize(textsize);
  gfx->setCursor(x,y);
  gfx->print(p_str);
}

int center_y(uint8_t textsize)
{
  return (gfx->height()-8*textsize)/2;
}

#ifdef _HAS_DABBLE
void dabble_wait_for_connection(void)
{
  int y = 40;
  uint8_t rotation = gfx->getRotation();  

  gfx->setRotation(1);
  gfx->fillScreen(WHITE);

  gfx->setTextColor(BLUE);
  gfx->setTextSize(1);
  gfx->setCursor(5,5);
  gfx->print((char*)SOFTWARE_VERSION);

  print_center_x((char*)"Bluetooth ID:",3,20+y);
  gfx->setTextColor(RED);
  print_center_x((char*)BLUETOOTH_NAME,4,55+y);
  
  gfx->setTextColor(BLUE);
  print_center_x((char*)"Waiting for",2,110+y);
  print_center_x((char*)"connection...",2,130+y);

  if (motors_enabled==0)
  {
    gfx->setTextColor(RED);
    print_center_x((char*)"Motors are disabled",2,170+y);
  }

  Dabble.waitForAppConnection();

  // Restore rotation.
  gfx->setRotation(rotation);
  
  image_show((char*)OK_K);
}

#else /* _HAS_DABBLE */

void dabble_simulate_connection(void)
{
  int y = 40;
  uint8_t rotation = gfx->getRotation();  

  gfx->setRotation(1);
  gfx->fillScreen(WHITE);

  gfx->setTextColor(BLUE);
  gfx->setTextSize(1);
  gfx->setCursor(5,5);
  gfx->print((char*)SOFTWARE_VERSION);

  print_center_x((char*)"Bluetooth ID:",3,20+y);
  gfx->setTextColor(RED);
  print_center_x((char*)BLUETOOTH_NAME,4,55+y);
  
  gfx->setTextColor(BLUE);
  print_center_x((char*)"Waiting for",2,110+y);
  print_center_x((char*)"connection...",2,130+y);

  delay(2000);

  // Restore rotation.
  gfx->setRotation(rotation);
  image_show((char*)OK_K);
}
#endif /* _HAS_DABBLE */

void arrow_show(joystick_t dir)
{
  char *p_image = 0;
  
  switch (dir)
  {
    case kForward:
      p_image = (char*)ARROW_U; // up
      break;
      
    case kBackward:
      p_image = (char*)ARROW_D; // down
      break;
      
    case kLeft:
      p_image = (char*)ARROW_L; // left
      break;
      
    case kRight:
      p_image = (char*)ARROW_R; // right
      break;
  }
  if (p_image!=0) image_show(p_image);
  else gfx->fillScreen(WHITE);
}

void show_rectangle(joystick_t dir, int color)
{
  int x = 0;
  int y = 0;
  int h = 20;
  int w = 60;
  int _h = 0;
  int _w = 0;

  uint8_t rotation = gfx->getRotation();
  gfx->setRotation(1);
  
  switch (dir)
  {
    case kForward:
      x = (gfx->width()-w)/2;
      y = gfx->height()-h;
      _w = w;
      _h = h;
      break;
      
    case kBackward:
      x = (gfx->width()-w)/2;
      y = 0;
      _w = w;
      _h = h;
      break;
      
    case kLeft:
      x = gfx->width()-h;
      y = (gfx->height()-w)/2;
      _w = h;
      _h = w;
      break;
      
    case kRight:
      x = 0;
      y = (gfx->height()-w)/2;
      _w = h;
      _h = w;
      break;
  }

  if (_w!=0 || _h!=0) gfx->fillRect(x,y,_w,_h,color);
  gfx->setRotation(rotation);
}

#ifdef _HAS_DABBLE
void dabble_parse(void)
{
  joystick = kIdle;
  // Accept only one direction at a time.
  if (GamePad.isUpPressed())
  {
    joystick = kForward;
  }
  else if (GamePad.isDownPressed())
  {
    joystick = kBackward;
  }
  else if (GamePad.isLeftPressed())
  {
    joystick = kLeft;
  }
  else if (GamePad.isRightPressed())
  {
    joystick = kRight;
  }

  // Stop
  if (GamePad.isCrossPressed())
  {
    joystick = kIdle;
  }

// Currently unused handlers.
//  int a = GamePad.getAngle();
//  int b = GamePad.getRadius();
//  float dir = GamePad.getXaxisData();
//  float spd = GamePad.getYaxisData();
//  if (GamePad.isTrianglePressed())
//  {
//  }
//  if (GamePad.isSquarePressed())
//  {
//  }
//  if (GamePad.isCirclePressed())
//  {
//  }
//  if (GamePad.isStartPressed())
//  {
//  }
//  if (GamePad.isSelectPressed())
//  {
//  }
}

#else

void dabble_simulate(void)
{
  static uint8_t state = 0;
  static uint32_t t_prev = 0;
  uint32_t timeout = 2000;

  uint32_t t_new = millis();
  if (t_new>=t_prev+timeout)
  {
    t_prev = t_new;
    state++;
    if (state&1) joystick = kLeft;
    else joystick = kRight;

    //if (state&1) joystick = kBackward;
    //else joystick = kForward;
  }
}
#endif /* _HAS_DABBLE */

void direction_update(joystick_t dir)
{
  static joystick_t dir_last = kIdle;
  if (dir!=dir_last)
  {
    show_rectangle(dir_last,WHITE); // Remove current rectangle.
    dir_last = dir;
    show_rectangle(dir_last,RED); // Draw new rectangle.
    //arrow_show(dir); // Too slow, breaks PID control :-(

    speed_offset_l = 0;
    speed_offset_r = 0;
    tilt_offset = 0;
    
    switch (dir)
    {
      case kIdle:
        //Serial.println("idle");
        break;
        
      case kForward:
        //Serial.println("forward");
        tilt_offset = TILT_OFFSET;
        break;
        
      case kBackward:
        //Serial.println("backward");
        tilt_offset = -TILT_OFFSET;
        break;
        
      case kLeft:
        //Serial.println("left");
        speed_offset_l = SPEED_OFFSET;
        break;
        
      case kRight:
        //Serial.println("right");
        speed_offset_r = SPEED_OFFSET;
        break;
    }
  }
}

double speed_adjust(double output, double offset, joystick_t dir)
{
  if (dir==kLeft || dir==kRight)
  {
    if (output<0.0)
    {
      output = max(-255.0,output-offset);
    }
    else
    {
      output = min(255.0,output+offset);
    }
  }
  /*else if (dir==kForward || dir==kBackward)
  {
    // (forwards) -255 <= output <= 255 (backwards)
    // If the robot is moving in the opposite direction, accelerate to make it tilt the other way.
    // If it is moving in the right direction, make it go slightly faster.
    if ((dir==kForward && output>0) || (dir==kBackward && output<0))
    {
      // Moving in the opposite direction. Try to reverse tilt.
      output *= 0.1; // 10%?
    }
    else
    {
      // Moving in the right direction. Increase speed.
      if (output<0.0)
      {
        output = max(-255.0,output-offset);
      }
      else
      {
        output = min(255.0,output+offset);
      }
    }
    // TODO: handle output==0, i.e. perfectly balanced?
  }*/
  
  return output;
}

void sdcard_init(void)
{
  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0);
  if (!SD_MMC.begin("/sdcard", true, true))
  {
    Serial.println("SD card not found");
    gfx->println(F("SD card not found"));
  }
}

void display_init(void)
{
  // ILI9488 init
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, 1);

  gfx->begin();
  gfx->fillScreen(WHITE);
  gfx->setTextSize(1);
  gfx->setTextColor(RED);
}

void power_monitor_print(void)
{
  if (power_monitor_ok==true)
  {
    Serial.print(power_monitor.getBusVoltage_V(),2);
    Serial.print(" V\t");
    Serial.print((int)power_monitor.getCurrent_mA());
    Serial.print(" mA\t");
    Serial.print((int)power_monitor.getPower_mW());
    Serial.println(" mW");
  }
  else
  {
    Serial.println("power monitor not available");
  }
}

void setup(void)
{
  // Initialize serial communication.
  Serial.begin(115200);
  Serial.flush();

  // Display & SD card.
  display_init();
  sdcard_init();

  // Ultrasonic transducer (not used yet).
  pinMode(ULTRASONIC_TRIG,OUTPUT);
  digitalWrite(ULTRASONIC_TRIG,LOW);
  pinMode(ULTRASONIC_ECHO,INPUT_PULLUP);

#ifdef _HAS_DABBLE
  // Start Bluetooth.
  Dabble.begin(BLUETOOTH_NAME);
  dabble_wait_for_connection();
#else
  dabble_simulate_connection();
#endif

  #ifdef MOTORS_DISABLED
    // It is more comfy to add & debug code with the motors disabled
    // but don't forget to turn them back on when done.
    Serial.println("Motors are disabled.");
  #else
    if (motors_enabled==0) Serial.println("Motors are disabled.");
  #endif
  
  // Make sure this is the very first I2C call, even before Wire.begin.
  Wire.setPins(MPU6050_SDA,MPU6050_SCL); // For IMU
  // Join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(MPU6050_SDA,MPU6050_SCL,400000);
  pinMode(MPU6050_INT,INPUT);

  // initialize device
  #ifdef __VERBOSE__
    Serial.println(F("Initializing I2C devices..."));
  #endif /* __VERBOSE__ */
  
  power_monitor_ok = power_monitor.begin();
  if (power_monitor_ok==false)
  {
    #ifdef __VERBOSE__
      Serial.println(F("INA219 power monitor not found"));
    #endif /* __VERBOSE__ */
  }

  mpu.initialize();

  // verify connection
  #ifdef __VERBOSE__
    Serial.println(F("MPU6050 "));
    Serial.println(mpu.testConnection() ? F("found") : F("not found"));
    Serial.println(F("Initializing DMP..."));
  #endif /* __VERBOSE__ */
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(X_OFFSET_GYRO);
  mpu.setYGyroOffset(Y_OFFSET_GYRO);
  mpu.setZGyroOffset(Z_OFFSET_GYRO);
  mpu.setXAccelOffset(X_OFFSET_ACCEL);
  mpu.setYAccelOffset(Y_OFFSET_ACCEL);
  mpu.setZAccelOffset(Z_OFFSET_ACCEL);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    #ifdef __VERBOSE__
      Serial.println("Enabling DMP...");
    #endif /* __VERBOSE__ */
    mpu.setDMPEnabled(true);

    // enable interrupt detection
    #ifdef __VERBOSE__
      Serial.println("Enabling interrupt detection...");
    #endif /* __VERBOSE__ */
    // cpv attachInterrupt(0, dmpDataReady, RISING);
    attachInterrupt(digitalPinToInterrupt(MPU6050_INT), dmpDataReady, RISING);
    
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    #ifdef __VERBOSE__
      Serial.println("DMP ready! Waiting for first interrupt...");
    #endif /* __VERBOSE__ */
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = 0; // cpv
    
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);  
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop(void)
{
  static uint32_t t_prev = 0;
  static uint32_t seconds = 0;
  
  uint32_t t_now = millis();
  if (t_now>=t_prev+1000)
  {
    t_prev = t_now;
    #ifdef __VERBOSE__
      power_monitor_print();
    #endif /* __VERBOSE__ */
    seconds++;
  }

#ifdef _HAS_DABBLE
  if (Dabble.processInput_tick()==true)
  {
    dabble_parse();
    direction_update(joystick);
  }
#else
  dabble_simulate();
  direction_update(joystick);
#endif
  
  // if MPU programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors

    pid.Compute();

    // (forwards) -255 <= output <= 255 (backwards)

    #ifndef MOTORS_DISABLED
    if (motors_enabled!=0)
    {
      if (joystick==kLeft || joystick==kRight)
      {
        // Turn left or right.
        double speed_l = speed_adjust(output,speed_offset_l,joystick);
        double speed_r = speed_adjust(output,speed_offset_r,joystick);
        motor_controller.move(speed_l,speed_r,MOTOR_MIN_ABS_SPEED);
      }
      else
      {
        setpoint = originalSetpoint + tilt_offset;
        motor_controller.move(output,MOTOR_MIN_ABS_SPEED);
      }
    }
    #endif
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    #ifdef __VERBOSE__
      Serial.println(F("FIFO overflow!"));
    #endif /* __VERBOSE__ */
  
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    #ifndef SWAP_XY
      input = ypr[1] * 180/M_PI + 180; // use pitch
    #else
      input = ypr[2] * 180/M_PI + 180; // use roll
    #endif

    // input < 180: tilting backwards
    // input > 180: tilting forwards
    //show_float(input);
  }
}
