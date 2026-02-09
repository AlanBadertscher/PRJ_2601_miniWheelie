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
 */
#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* cpv - Use the sketch SBRobot_IMU_Zero to find the offset values for the IMU.
 * Example final output (reformatted slightly for readability):
 * XAccel                    YAccel                ZAccel                        XGyro               YGyro                 ZGyro
 * [-2387,-2385] -->  [-9,4] [975,976] --> [-6,14] [1242,1243] --> [16379,16393] [102,103] --> [0,3] [-60,-59] --> [-2,1]  [72,73] --> [-1,1]
 * [-2387,-2386] -->  [-9,1] [975,976] --> [-9,14] [1242,1242] --> [16379,16385] [102,103] --> [0,3] [-60,-59] --> [-2,1]  [72,73] --> [-3,1]
 * [-2387,-2386] --> [-16,1] [975,976] --> [-8,14] [1242,1242] --> [16383,16385] [102,103] --> [0,3] [-60,-59] --> [-2,1]  [72,73] --> [-2,1]
 * -------------- done --------------*/
#define X_OFFSET_ACCEL (-2387)
#define Y_OFFSET_ACCEL   (975)
#define Z_OFFSET_ACCEL  (1242)
#define X_OFFSET_GYRO    (102)
#define Y_OFFSET_GYRO    (-60)
#define Z_OFFSET_GYRO     (72)

// MOTOR CONTROLLER
/* cpv - Set this to the largest minimum value of the two motors. */
#define MOTOR_MIN_ABS_SPEED  (52)
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
double motor_speed_multiplier_left = 1.0; // R_max/L_max
double motor_speed_multiplier_right = 0.97; // L_max/R_max
LMotorController motor_controller(ENB,IN4,IN3,ENA,IN1,IN2,motor_speed_multiplier_left,motor_speed_multiplier_right);

/* cpv - IMU x-axis is supposed to run forward/backward, y-axis is supposed to run left/right.
 * In case they are swapped, you can try defining SWAP_XY. */
//#define SWAP_XY 
/* cpv - Uncomment to run sketch without motors. This is practical when adding more functions or when debugging. */
//#define MOTORS_OFF
/* cpv - Uncomment to send debug & status information on the serial port. */
//#define __VERBOSE__

//#define LOG_INPUT  (1)
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
double originalSetpoint = 180.0; //175.8;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth
/* cpv - Kp, Kd & Ki are user-adjustable values. Try other values to see how they affect the robot's stability.
 * These values worked for me. */
double Kp = 22;
double Kd = 2;
double Ki = 40;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void IRAM_ATTR dmpDataReady()
{
  mpuInterrupt = true;
}

int offset_adjust(int offset, int delta)
{
  offset += delta;
  offset = min(offset,255);
  offset = max(offset,-255);
  return offset;
}

void setup(void)
{
    // Initialize serial communication.
    Serial.begin(115200);
    Serial.flush();
  
    // Make sure this is the very first I2C call, even before Wire.begin.
    Wire.setPins(MPU6050_SDA,MPU6050_SCL); // For IMU
    // Join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin(MPU6050_SDA,MPU6050_SCL,400000);
    pinMode(MPU6050_INT,INPUT);

    // initialize device
    #ifdef __VERBOSE__
      Serial.println(F("Initializing I2C devices..."));
    #endif /* __VERBOSE__ */
    mpu.initialize();

    // verify connection
    #ifdef __VERBOSE__
      Serial.println(F("Testing device connections..."));
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
    // if MPU programming failed, don't try to do anything
    if (!dmpReady) return;
  
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors
        
        pid.Compute();
        #ifndef MOTORS_OFF
          motor_controller.move(output,MOTOR_MIN_ABS_SPEED);
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
   }
}
