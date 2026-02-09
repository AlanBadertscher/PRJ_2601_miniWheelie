/*
 * Purpose: Find the minimum & maximum motor speeds.
 * Board: ESP32S3 Dev Module / SBRobot (Elektpr project 240134)
 * IDE: 1.8.19
 *
 * Usage:
 * 
 * Find Minimum Speed
 * ==================
 * - Upload this sketch and open the Serial Monitor. 
 * - Use the 's', 'l'/'r' & 'u'/'d' commands to find the:
 *   - minimum value that keep a motor spinning (e.g. 49)
 *   - minimum value that makes a motor start spinning (e.g. 55)
 * - Write these values down
 *
 * Compare Motors
 * ==============
 * - Upload this sketch and open the Serial Monitor to flush the serial buffer.
 * - Close the Serial Monitor 
 * - Open the Serial Plotter
 * - Use the 'c' command to start
 * - Use the 'R'/'L' commands to slow down one of the motors. The corresponding 
 *   graph in the Serial Plotter will move up. Adjust the motor speeds this way 
 *   to make the graphs overlap eachother as good as possible.
 * - Use the 's' command to stop
 * - Close the Serial Plotter
 * - Open the Serial Monitor
 * - Use the 'P' command to display speeds & ratios (e.g. "L = 255, R = 247, R/L = 0.97, L/R = 1.03")
 * - Write these values down
 * 
 * By: Clemens Valens
 * Date: 26/06/2024
 */
#include "LMotorController.h"

//MOTOR CONTROLLER
const int ENA = 46;
const int IN1 = 7;
const int IN2 = 18;
const int CA1 = 16;
const int CA2 = 17;
double motorSpeedFactorLeft = 0.982;

const int ENB = 45;
const int IN3 = 10;
const int IN4 = 11;
const int CB1 = 9;
const int CB2 = 47;
double motorSpeedFactorRight = 1.0;

LMotorController motor_controller(ENB,IN4,IN3,ENA,IN1,IN2,motorSpeedFactorLeft,motorSpeedFactorRight);

typedef struct
{
  uint8_t pin;
  uint8_t state;
  uint32_t t0;
  uint32_t t1;
  uint32_t period0;
  uint32_t period1;
  boolean flag;
  uint32_t period;
}
hall_sensor_t;

#define HALL_SENSORS  (4)
volatile hall_sensor_t hall_sensor[HALL_SENSORS] =
{
  { CA1, 0, 0, 0, 0, false, 0 },
  { CA2, 0, 0, 0, 0, false, 0 },
  { CB1, 0, 0, 0, 0, false, 0 },
  { CB2, 0, 0, 0, 0, false, 0 },
};

int speed_adjust(int s, int ds)
{
  s += ds;
  s = min(s,255);
  s = max(s,-255);
  return s;
}

void hall_sensor_read(volatile hall_sensor_t &h)
{
  // Rising edge.
  uint32_t t = micros();
  h.period1 = t - h.t1;
  h.period = (15*h.period + h.period1)/16;
  h.t1 = t;
  h.flag = true;
}

void IRAM_ATTR hall_sensor_read_ca1(void)
{
  hall_sensor_read(hall_sensor[0]);
}

void IRAM_ATTR hall_sensor_read_ca2(void)
{
  hall_sensor_read(hall_sensor[1]);
}

void IRAM_ATTR hall_sensor_read_cb1(void)
{
  hall_sensor_read(hall_sensor[2]);
}

void IRAM_ATTR hall_sensor_read_cb2(void)
{
  hall_sensor_read(hall_sensor[3]);
}

void hall_sensor_print(volatile hall_sensor_t &h)
{
  if (h.flag==false)
  {
    Serial.print(0);
    return;
  }
  h.flag = false;
  /*     if (h.pin==CA1) Serial.print("CA1: ");
  else if (h.pin==CA2) Serial.print("CA2: ");
  else if (h.pin==CB1) Serial.print("CB1: ");
  else if (h.pin==CB2) Serial.print("CB2: ");
  else return; // Invalid sensor.*/
  //Serial.print("period0 = ");
  //Serial.print(h.period0);
  //Serial.print(" us, ");
  Serial.print(h.period);
  //Serial.println(" us");
}

void setup(void)
{
  Serial.begin(115200);
  pinMode(CA1,INPUT);
  pinMode(CA2,INPUT);
  pinMode(CB1,INPUT);
  pinMode(CB2,INPUT);
  attachInterrupt(digitalPinToInterrupt(CA1),hall_sensor_read_ca1,RISING);
  attachInterrupt(digitalPinToInterrupt(CA2),hall_sensor_read_ca2,RISING);
  attachInterrupt(digitalPinToInterrupt(CB1),hall_sensor_read_cb1,RISING);
  attachInterrupt(digitalPinToInterrupt(CB2),hall_sensor_read_cb2,RISING);
}

void loop(void)
{
  static char m = 's';
  static int speed_l = 128;
  static int speed_r = 128;
  int speed_d = 0;

  /*for (int i=0; i<HALL_SENSORS; i++)
  {
    hall_sensor_read(hall_sensor[i]);
  }*/
  if (m!='s' && m!='S' && (hall_sensor[2].flag==true || hall_sensor[0].flag==true))
  {
    if (m=='C')
    {
      hall_sensor_print(hall_sensor[2]);
      Serial.print(",");
      hall_sensor_print(hall_sensor[0]);
      Serial.println();
    }
    else
    {
      Serial.print("L=");
      Serial.print(speed_l);
      Serial.print("\t");
      hall_sensor_print(hall_sensor[2]);
      Serial.print("\tR=");
      Serial.print(speed_r);
      Serial.print("\t");
      hall_sensor_print(hall_sensor[0]);
      Serial.println();
    }
  }

  if (Serial.available())
  {
    int ch = Serial.read();
    if (ch=='0') { speed_l = 0; speed_r = 0; } // stop
    if (ch=='x') speed_d = 1000; // max
    if (ch=='w') speed_d = -1000; // max
    if (ch=='l') m = 'l'; // left motor
    if (ch=='r') m = 'r'; // right motor
    if (ch=='s') m = 's'; // stop both motors
    if (ch=='u' || ch=='+') speed_d = 1;
    if (ch=='d' || ch=='-') speed_d = -1;
    // Use with Serial Plotter, keep count yourself.
    if (ch=='c') m = 'c'; // compare motors at max speed
    if (ch=='R' || ch=='L')
    {
      // compare motors
      if (ch=='R') speed_r -= 1; // decrease R-motor speed
      if (ch=='L') speed_l -= 1; // decrease L-motor speed
      motor_controller.move(speed_l,speed_r,0);
    }
    if (ch=='P')
    {
      motor_controller.stopMoving();
      Serial.print("L = ");
      Serial.print(speed_l);
      Serial.print(", R = ");
      Serial.print(speed_r);
      Serial.print(", R/L = ");
      Serial.print((float)speed_r/(float)speed_l);
      Serial.print(", L/R = ");
      Serial.print((float)speed_l/(float)speed_r);
      Serial.println();
    }
  }

  if (m=='L' || m=='l')
  {
    if (m=='l')
    {
      Serial.println("Left - send 'u' | '+' or 'd' | '-' to start spinning.");
      m = 'L';
    }
    if (speed_d!=0)
    {
      speed_l = speed_adjust(speed_l,speed_d);
      Serial.print("L = ");
      Serial.println(speed_l);
      motor_controller.move(speed_l,0,0);
      speed_d = 0;
    }
  }
  else if (m=='R' || m=='r')
  {
    if (m=='r')
    {
      Serial.println("Right - send 'u' | '+' or 'd' | '-' to start spinning.");
      m = 'R';
      speed_d = 1;
    }
    if (speed_d!=0)
    {
      speed_r = speed_adjust(speed_r,speed_d);
      Serial.print("R = ");
      Serial.println(speed_r);
      motor_controller.move(0,speed_r,0);
      speed_d = 0;
    }
  }
  else if (m=='s')
  {
    Serial.println("Stopped - send 'l' or 'r' to select a motor.");
    motor_controller.stopMoving();
    speed_d = 0;
    m = 'S';
  }
  else if (m=='c')
  {
    speed_l = 255;
    speed_r = 255;
    motor_controller.move(speed_l,speed_r,0);
    m = 'C';
  }
}
