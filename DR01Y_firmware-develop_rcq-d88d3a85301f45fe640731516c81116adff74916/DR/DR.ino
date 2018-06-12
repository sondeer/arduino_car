   /*************************************************************************
  File Name          : Firmware_for_Auriga.ino
  Author             : myan
  Updated            : myan
  Version            : V09.01.


  Date               : 08/22/2016
  Description        : Firmware for Makeblock Electronic modules with Scratch.
  License            : CC-BY-SA 3.0
  Copyright (C) 2013 - 2016 Maker Works Technology Co., Ltd. All right reserved.
  http://www.makeblock.cc/
  History:
  <Author>         <Time>         <Version>        <Descr>
  Mark Yan         2016/03/12     09.01.001        build the new.
  Mark Yan         2016/05/03     09.01.002        Added encoder and compass driver and fix some bugs.
  Mark Yan         2016/05/24     09.01.003        Fix issue MBLOCK-1 and MBLOCK-12(JIRA issue).
  Mark Yan         2016/05/30     09.01.004        Add speed calibration for balanced car mode.
  Mark Yan         2016/06/07     09.01.005        Remove the boot animation.
  Mark Yan         2016/06/08     09.01.006        Add 1s blink function.
  Mark Yan         2016/06/25     09.01.007        Fix issue MBLOCK-38(limit switch return value).
  Mark Yan         2016/07/06     09.01.008        Fix issue MBLOCK-61(ultrasonic distance limitations bug).
  Mark Yan         2016/07/27     09.01.009        Add position parameters for encoder motor,fix issue MBLOCK-77.
  Mark Yan         2016/08/01     09.01.010        Fix issue MBLOCK-109 MBLOCK-110(encoder motor exception handling negative).
  Mark Yan         2016/08/10     09.01.011        Fix issue MBLOCK-128(ext encoder motor led to reset).
  Mark Yan         2016/08/24     09.01.012        Fix issue MBLOCK-171(Stepper online execution slow), MBLOCK-189(on board encoder motor reset issue).
**************************************************************************/
#include <Arduino.h>
#include <avr/wdt.h>
#include <MeAuriga.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

//#define DEBUG_INFO
//#define DEBUG_INFO1


class PWMBuzzer : public MePort
{
public:
  PWMBuzzer()
  {
    DDRH |= 0x20;   //port L direction
    TCCR4A = 0x0E;//0x32; //bit 3;2 compare output mode for Channel C
    TCCR4B = 0x1a;
    ICR4 = 20000;
    OCR4C = 0xffff;// OCR5B = 0xffff;
    maxPlayTime = 0;
    lastToneTime = 0;
  }
  PWMBuzzer(uint8_t port)
  {
    DDRH |= 0x20;   //port L direction
    TCCR4A = 0x0E;//0x32; //bit 3;2 compare output mode for Channel C
    TCCR4B = 0x1a;
    ICR4 = 20000;
    OCR4C = 0xffff;
    maxPlayTime = 0;
    lastToneTime = 0;
  }
  PWMBuzzer(uint8_t port, uint8_t slot)
  {
    DDRH |= 0x20;   //port L direction
    TCCR4A = 0x0E;//0x32; //bit 3;2 compare output mode for Channel C
    TCCR4B = 0x1a;
    ICR4 = 20000;
    OCR4C = 0xffff;// OCR5B = 0xffff;
    maxPlayTime = 0;
    lastToneTime = 0;
  }
  PWMBuzzer(int pin)
  {
    DDRH |= 0x20;   //port L direction
    TCCR4A = 0x0E;//0x32; //bit 3;2 compare output mode for Channel C
    TCCR4B = 0x1a;
    ICR4 = 20000;
    OCR4C = 0xffff;// OCR5B = 0xffff;
    maxPlayTime = 0;
    lastToneTime = 0;
  }
  void setpin(int pin)
  {
    DDRH |= 0x20;
    TCCR4A = 0x0E;
    TCCR4B = 0x1a;
    ICR4 = 20000;
    OCR4C = 0xffff;
    maxPlayTime = 0;
    lastToneTime = 0;
  }
  void tone(int pin, uint16_t frequency, uint32_t duration)
  {
     tone(frequency,duration);
  }
  void tone(uint16_t frequency, uint32_t duration = 0)
  {
     int pulse = 1000000L / frequency;
     int period = pulse * 2;
     
     OCR4C = pulse;
     ICR4 = period;
     //if(duration > 100)
     //{
        maxPlayTime = duration;
        lastToneTime = millis();
     //}
     //else
     //{
      // delay(duration);
     //  OCR5B = 0xffff;
     //}
  }

void delayTone(uint16_t frequency, uint32_t duration )
{
     int pulse = 1000000L / frequency;
     int period = pulse * 2;
     
     OCR4C = pulse;//OCR5B = pulse;
     ICR4 = period;

     delay(duration);
     
     OCR4C = 0xffff;
}
  
  void noTone(int pin)
  {
    //OCR4B = 0xffff;
  }
  void noTone()
  {
    //OCR4B = 0xffff;
  }

  void  updata(void)
  {
    if(maxPlayTime > 0)
    {
      if(millis() - lastToneTime >= maxPlayTime)
      {
        //OCR4B = 0xffff;
        OCR4C = 0xffff;
        maxPlayTime = 0;
      }
    }
  }
private:
  long lastToneTime;
  uint16_t maxPlayTime;
};

Servo servos[12];
MeDCMotor dc;
MeTemperature ts;
MeRGBLed led;
MeUltrasonicSensor *us = NULL;  //PORT_10
Me7SegmentDisplay seg;
MePort generalDevice;
MeLEDMatrix ledMx;
MeInfraredReceiver *ir = NULL;  //PORT_8
MeGyro gyro_ext(0, 0x68); //external gryo sensor
MeGyro gyro(1, 0x69);     //On Board external gryo sensor
MeCompass Compass;
MeJoystick joystick;
MeStepper steppers[4];
PWMBuzzer buzzer;
//MeBuzzer buzzer;
MeHumiture humiture;
MeFlameSensor FlameSensor;
MeGasSensor GasSensor;
MeTouchSensor touchSensor;
Me4Button buttonSensor;
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower line(PORT_10);
MeEncoderMotor encoders[2];
//Me7SegmentDisplay disp(PORT_8);


#define LOW_BRIGHTNSS(a) ((int16_t)a / 4)
#define TIME_OUT_COUNT  5000
uint16_t left_right_time;

uint16_t obstacle_count = 0;
unsigned long motor_no_rotate_time=0; //阻转计时
uint8_t motor_no_rotate_flag=0; //阻转的标识
uint8_t motor_stop_flag=0;
unsigned long motor_stop_time =0;
typedef struct MeModule
{
  int16_t device;
  int16_t port;
  int16_t slot;
  int16_t pin;
  int16_t index;
  float values[3];
} MeModule;

union
{
  uint8_t byteVal[4];
  float floatVal;
  long longVal;
} val;

union
{
  uint8_t byteVal[8];
  double doubleVal;
} valDouble;

union
{
  uint8_t byteVal[2];
  int16_t shortVal;
} valShort;
MeModule modules[12];
#if defined(__AVR_ATmega32U4__)
int16_t analogs[12] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
#endif
#if defined(__AVR_ATmega328P__) or defined(__AVR_ATmega168__)
int16_t analogs[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
#endif
#if defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
int16_t analogs[16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
#endif

int16_t len = 52;
int16_t servo_pins[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Just for Auriga
int16_t moveSpeed = 80;
int16_t avoidAnceSpeed = 100;
bool    runAvoidFlag = true;
int16_t turnSpeed = 90;
int16_t minSpeed = 45;
int16_t factor = 23;
int16_t distance = 0;
int16_t randnum = 0;
int16_t auriga_power = 0;
int16_t LineFollowFlag = 0;
#define MOVE_STOP       0x00
#define MOVE_FORWARD    0x01
#define MOVE_BACKWARD   0x02

#define BLUETOOTH_MODE                       0x00
#define AUTOMATIC_OBSTACLE_AVOIDANCE_MODE    0x01
#define BALANCED_MODE                        0x02
#define IR_REMOTE_MODE                       0x03
#define LINE_FOLLOW_MODE                     0x04
#define MAX_MODE                             0x05

#define POWER_PORT                           A4
#define BUZZER_PORT                          8//44 //45
#define RGBLED_PORT                          A3

uint8_t command_index = 0;
uint8_t auriga_mode = BLUETOOTH_MODE;
uint8_t old_auriga_mode = BLUETOOTH_MODE;
uint8_t index = 0;
uint8_t ser1index = 0;
uint8_t dataLen;
uint8_t ser1dataLen;
uint8_t modulesLen = 0;
uint8_t irRead = 0;
uint8_t prevc = 0;
uint8_t ser1prevc = 0;
uint8_t keyPressed = KEY_NULL;
uint8_t SerialRead;
uint8_t Ser1SerialRead;
uint8_t buffer[52];
uint8_t ser1buffer[52]; //用于usb串口的buf
int16_t check_digital_pins[15] = { 4, 5, 2, 3, 17, 16, 6, 7, 43, 42,19,18, 31 , 15, 14};

#define DEVICE_NORMAL_MODE  0
#define DEVICE_CHECK_MODE   1
uint8_t DeviceRunMode;

double  lastTime = 0.0;
double  currentTime = 0.0;
double  CompAngleY, CompAngleX, GyroXangle;
double  LastCompAngleY, LastCompAngleX, LastGyroXangle;
double  last_turn_setpoint_filter = 0.0;
double  last_speed_setpoint_filter = 0.0;
double  last_speed_error_filter = 0.0;
double  speed_Integral_average = 0.0;
double  angle_speed = 0.0;
double  balance_car_speed_offsets = 0.0;

float angleServo = 90.0;
float dt;

long lasttime_angle = 0;
long lasttime_speed = 0;
long update_sensor = 0;
long blink_time = 0;
long lasttime_receive_cmd = 0;
long last_Pulse_pos_encoder1 = 0;
long last_Pulse_pos_encoder2 = 0;

boolean isStart = false;
boolean isAvailable = false;

boolean isSer1Start = false;
boolean isSer1Available = false;

boolean leftflag;
boolean rightflag;
boolean start_flag = false;
boolean move_flag = false;
boolean boot_show_flag = true;
boolean blink_flag = false;

/*
//String mVersion = "09.01.012";
-- 10.01.001 to 10.01.002 change log:
-- gyro will wait calibration when enter balance mode
-- auto run mode to balance mode will stop
-- change the filter to decrease the car turn over
*/
String mVersion = "10.01.003";

uint8_t reset_gyro_flag = 0;
int getPWRVoltage = 0;   //由于电池电压不同，PID中的P值不同
#define PWR_ADC_PIN     A4

uint8_t led_mode, led_r, led_b, led_g;
uint16_t led_manual_on;

unsigned long power_state_update;

uint8_t power_key_filter;
uint8_t power_key_status_last;
uint8_t btn_status = 0;

unsigned long obs_state_update = 0;
long obs_distance;
uint8_t obs_enable = 0;
uint8_t obs_trigger = 0;

double teleop_vel_l;
double teleop_vel_a;

int16_t runTagDistance;
long    saveNowPos;
long    runAngleDistanceSpeed;

int16_t rotateTagAngle;
double  lastAngle;
double  totalRotateAngle;
long    runAngleBeginTime;
long    runAngleMaxTime;

long   lastReceiveRC;
 
enum {
  POWER_INIT,
  POWER_RISING,
  POWER_ON,
  POWER_OFF_RISING,
  POWER_OFF,
} power_state;

//////////////////////////////////////////////////////////////////////////////////////
//float RELAX_ANGLE = 2.7;
float RELAX_ANGLE = 10;//8;//10;
float saveRELAX_ANGLE =10;// 10;
#define PWM_MIN_OFFSET         2

#define VERSION                0
#define ULTRASONIC_SENSOR      1
#define TEMPERATURE_SENSOR     2
#define LIGHT_SENSOR           3
#define POTENTIONMETER         4
#define JOYSTICK               5
#define GYRO                   6
#define SOUND_SENSOR           7
#define RGBLED                 8
#define SEVSEG                 9
#define MOTOR                  10
#define SERVO                  11
#define ENCODER                12
#define IR                     13
#define PIRMOTION              15
#define INFRARED               16
#define LINEFOLLOWER           17
#define SHUTTER                20
#define LIMITSWITCH            21
#define BUTTON                 22
#define HUMITURE               23
#define FLAMESENSOR            24
#define GASSENSOR              25
#define COMPASS                26
#define TEMPERATURE_SENSOR_1   27
#define DIGITAL                30
#define ANALOG                 31
#define PWM                    32
#define SERVO_PIN              33
#define TONE                   34
#define BUTTON_INNER           35
#define ULTRASONIC_ARDUINO     36
#define PULSEIN                37
#define STEPPER                40
#define LEDMATRIX              41
#define TIMER                  50
#define TOUCH_SENSOR           51
#define JOYSTICK_MOVE          52
#define COMMON_COMMONCMD       60
//Secondary command
#define SET_STARTER_MODE     0x10
#define SET_AURIGA_MODE      0x11
#define SET_MEGAPI_MODE      0x12
#define GET_BATTERY_POWER    0x70
#define GET_AURIGA_MODE      0x71
#define GET_MEGAPI_MODE      0x72
#define ENCODER_BOARD          61
//Read type
#define ENCODER_BOARD_POS    0x01
#define ENCODER_BOARD_SPEED  0x02

#define ENCODER_PID_MOTION     62
//Secondary command
#define ENCODER_BOARD_POS_MOTION         0x01
#define ENCODER_BOARD_SPEED_MOTION       0x02
#define ENCODER_BOARD_PWM_MOTION         0x03
#define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
#define ENCODER_BOARD_CAR_POS_MOTION     0x05

#define ISKYU_RGBLED        100
#define ISKYU_RGBLED_COLOR  101
#define ISKYU_JOYSTICK      102
#define ISKYU_JOYSTICK_A    103
#define ISKYU_RGBLED_MANUAL 104
#define ISKYU_BUTTON        112
#define ISKYU_MOTION        113
#define ISKYU_LINE          114
#define ISKYU_OBS_ON_OFF    116

#define ISKYU_SHUTDOWN      117
#define ISKYU_GYRO          120
#define GYRO_RESET          121

#define ISKYU_ROTATE_ANGLE     128
#define ISKYU_MOVE_DISTANCE    129
#define ISKYU_ROTATE_ANGLE_A   130
#define ISKYU_MOVE_DISTANCE_A  131
#define ISKYU_SET_AUTOMATIC_SPD 132

#define GET 1
#define RUN 2
#define RESET 4
#define START 5


float angle_z_start;
float angle_z_current;
char steering_time = 0;
#define STEERING_TIMEOUT_S  5
typedef struct
{
  double P, I, D;
  double Setpoint, Output, Integral, differential, last_error;
} PID;

PID  PID_angle, PID_speed, PID_turn;

void parseSer1Data(void);
uint8_t Detect_IIC(void);
/**
   \par Function
      isr_process_encoder1
   \par Description
      This function use to process the interrupt of encoder1 drvicer on board,
      used to calculate the number of pulses.
   \param[in]
      None
   \par Output
      The number of pulses on encoder1 driver
   \return
      None
   \par Others
      None
*/
void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) != 0)
  {
      Encoder_1.pulsePosMinus();
  }
  else
  {
      Encoder_1.pulsePosPlus(); 
  }
}

/**
   \par Function
      isr_process_encoder2
   \par Description
      This function use to process the interrupt of encoder2 drvicer on board,
      used to calculate the number of pulses.
   \param[in]
      None
   \par Output
      The number of pulses on encoder2 driver
   \return
      None
   \par Others
      None
*/
void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) != 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

/**
   \par Function
      WriteBalancedDataToEEPROM
   \par Description
      This function use to write the balanced car configuration parameters to EEPROM.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void WriteBalancedDataToEEPROM(void)
{
  EEPROM.write(BALANCED_CAR_PARTITION_CHECK, EEPROM_IF_HAVEPID_CHECK1);
  EEPROM.write(BALANCED_CAR_PARTITION_CHECK + 1, EEPROM_IF_HAVEPID_CHECK2);
  EEPROM.write(BALANCED_CAR_START_ADDR, EEPROM_CHECK_START);

  EEPROM.put(BALANCED_CAR_NATURAL_BALANCE, RELAX_ANGLE);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR, PID_angle.P);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR + 4, PID_angle.I);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR + 8, PID_angle.D);

  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR, PID_speed.P);
  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR + 4, PID_speed.I);
  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR + 8, PID_speed.D);

  EEPROM.put(BALANCED_CAR_DIR_PID_ADDR, PID_turn.P);
  EEPROM.write(BALANCED_CAR_END_ADDR, EEPROM_CHECK_END);

  EEPROM.write(AURIGA_MODE_START_ADDR, EEPROM_CHECK_START);
  EEPROM.write(AURIGA_MODE_CONFIGURE, auriga_mode);
  EEPROM.write(AURIGA_MODE_END_ADDR, EEPROM_CHECK_END);
}

/**
   \par Function
      WriteAurigaModeToEEPROM
   \par Description
      This function use to write the Auriga Mode configuration parameter to EEPROM.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void WriteAurigaModeToEEPROM(void)
{
  EEPROM.write(AURIGA_MODE_PARTITION_CHECK, EEPROM_IF_HAVEPID_CHECK1);
  EEPROM.write(AURIGA_MODE_PARTITION_CHECK + 1, EEPROM_IF_HAVEPID_CHECK2);
  EEPROM.write(AURIGA_MODE_START_ADDR, EEPROM_CHECK_START);
  EEPROM.write(AURIGA_MODE_CONFIGURE, auriga_mode);
  EEPROM.write(AURIGA_MODE_END_ADDR, EEPROM_CHECK_END);
}

/**
   \par Function
      readEEPROM
   \par Description
      This function use to read the configuration parameters from EEPROM.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void readEEPROM(void)
{
  if ((EEPROM.read(BALANCED_CAR_PARTITION_CHECK) == EEPROM_IF_HAVEPID_CHECK1) && (EEPROM.read(BALANCED_CAR_PARTITION_CHECK + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    if ((EEPROM.read(BALANCED_CAR_START_ADDR)  == EEPROM_CHECK_START) && (EEPROM.read(BALANCED_CAR_END_ADDR)  == EEPROM_CHECK_END))
    {
      EEPROM.get(BALANCED_CAR_NATURAL_BALANCE, RELAX_ANGLE);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR, PID_angle.P);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR + 4, PID_angle.I);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR + 8, PID_angle.D);

      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR, PID_speed.P);
      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR + 4, PID_speed.I);
      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR + 8, PID_speed.D);

      EEPROM.get(BALANCED_CAR_DIR_PID_ADDR, PID_turn.P);
#ifdef DEBUG_INFO
      Serial3.println( "Read data from EEPROM:");
      Serial3.print(RELAX_ANGLE);
      Serial3.print( "  ");
      Serial3.print(PID_angle.P);
      Serial3.print( "  ");
      Serial3.print(PID_angle.I);
      Serial3.print( "  ");
      Serial3.print(PID_angle.D);
      Serial3.print( "  ");
      Serial3.print(PID_speed.P);
      Serial3.print( "  ");
      Serial3.print(PID_speed.I);
      Serial3.print( "  ");
      Serial3.print(PID_speed.D);
      Serial3.print( "  ");
      Serial3.println(PID_turn.P);
#endif
    }
    else
    {
      Serial3.println( "Data area damage on balanced car pid!" );
    }
  }
  else
  {
#ifdef DEBUG_INFO
    Serial3.println( "First written Balanced data!" );
#endif
    WriteBalancedDataToEEPROM();
  }

  if ((EEPROM.read(AURIGA_MODE_PARTITION_CHECK) == EEPROM_IF_HAVEPID_CHECK1) && (EEPROM.read(AURIGA_MODE_PARTITION_CHECK + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    if ((EEPROM.read(AURIGA_MODE_START_ADDR)  == EEPROM_CHECK_START) && (EEPROM.read(AURIGA_MODE_END_ADDR)  == EEPROM_CHECK_END))
    {
      EEPROM.get(AURIGA_MODE_CONFIGURE, auriga_mode);
#ifdef DEBUG_INFO
      Serial3.print( "Read auriga_mode from EEPROM:");
      Serial3.println(auriga_mode);
#endif
    }
    else
    {
      Serial3.println( "Data area damage on auriga mode!" );
    }
  }
  else
  {
#ifdef DEBUG_INFO
    Serial3.println( "First written auriga mode!" );
#endif
    WriteAurigaModeToEEPROM();
  }
}


/**
   \par Function
      Forward
   \par Description
      This function use to control the car kit go forward.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void Forward(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
  //Serial3.println("forward");
}

void Forward_Speed(int16_t speed)
{
  Encoder_1.runSpeed(speed);
  Encoder_2.runSpeed(-speed);
}

void TurnRight_Speed(int16_t speed)
{
  Encoder_1.runSpeed(speed);
  Encoder_2.runSpeed(speed);
}

/**
   \par Function
      Backward
   \par Description
      This function use to control the car kit go backward.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void Backward(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
  //Serial3.println("backward");
}

/**
   \par Function
      BackwardAndTurnLeft
   \par Description
      This function use to control the car kit go backward and turn left.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void BackwardAndTurnLeft(void)
{
  Encoder_1.setMotorPwm(-moveSpeed / 4);
  Encoder_2.setMotorPwm(moveSpeed);
  //Encoder_1.setMotorPwm(moveSpeed / 4);
  //Encoder_2.setMotorPwm(moveSpeed);
  //Serial3.println("backward and left");
}

/**
   \par Function
      BackwardAndTurnRight
   \par Description
      This function use to control the car kit go backward and turn right.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void BackwardAndTurnRight(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed / 4);
  
  //Encoder_1.setMotorPwm(moveSpeed);
  //Encoder_2.setMotorPwm(moveSpeed / 4);

  //Serial3.println("backward and right");
}

/**
   \par Function
      TurnLeft
   \par Description
      This function use to control the car kit go backward and turn left.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void TurnLeft(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed / 2);

  //Encoder_1.setMotorPwm(moveSpeed);
  //Encoder_2.setMotorPwm(moveSpeed / 2);


  //Serial3.println("left");
}
void TurnLeft_iskyu(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
 // Encoder_2.setMotorPwm(moveSpeed / 2);
  Encoder_2.setMotorPwm(0);
  //Serial3.println("left");
/*
  Encoder_1.setMotorPwm(moveSpeed);
 // Encoder_2.setMotorPwm(moveSpeed / 2);
  Encoder_2.setMotorPwm(0);
  */
}
/**
   \par Function
      TurnRight
   \par Description
      This function use to control the car kit go backward and turn right.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void TurnRight(void)
{
  Encoder_1.setMotorPwm(moveSpeed / 2);
  Encoder_2.setMotorPwm(-moveSpeed);

  //Encoder_1.setMotorPwm(moveSpeed / 2);
 // Encoder_2.setMotorPwm(moveSpeed);
  //Serial3.println("right");
}
void TurnRight_iskyu()
{
   Encoder_1.setMotorPwm(0);//Encoder_1.setMotorPwm(-moveSpeed / 2);
 // Encoder_1.setMotorPwm(-moveSpeed / 2);
   Encoder_2.setMotorPwm(moveSpeed);
}
/**
   \par Function
      TurnLeft1
   \par Description
      This function use to control the car kit go backward and turn left(fast).
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void TurnLeft1(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

/**
   \par Function
      TurnRight1
   \par Description
      This function use to control the car kit go backward and turn right(fast).
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void TurnRight1(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

/**
   \par Function
      Stop
   \par Description
      This function use to stop the car kit.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void Stop(void)
{
  Encoder_1.setMotionMode(DIRECT_MODE);
  Encoder_2.setMotionMode(DIRECT_MODE);
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

/**
   \par Function
      ChangeSpeed
   \par Description
      This function use to change the speed of car kit.
   \param[in]
      spd - the speed of car kit(-255 ~ 255)
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void ChangeSpeed(int16_t spd)
{
  moveSpeed = spd;
}

/**
   \par Function
      readBuffer
   \par Description
      This function use to read the Serial data from its buffer..
   \param[in]
      index - The first address in the array
   \par Output
      None
   \return
      The data need to be read.
   \par Others
      None
*/
uint8_t readBuffer(int16_t index)
{
  return buffer[index];
}

uint8_t readSer1Buffer(int16_t index)
{
  return ser1buffer[index];
}


/**
   \par Function
      writeBuffer
   \par Description
      This function use to write the Serial data to its buffer..
   \param[in]
      index - The data's first address in the array
    \param[in]
      c - The data need to be write.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void writeBuffer(int16_t index, uint8_t c)
{
  buffer[index] = c;
}

void writeSer1Buffer(int16_t index, uint8_t c)
{
  ser1buffer[index] = c;
}

/**
   \par Function
      writeHead
   \par Description
      This function use to write the head of transmission frame.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void writeHead(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
}

void writeSer1Head(void)
{
  writeSer1Serial(0xff);
  writeSer1Serial(0x56);
}
/**
   \par Function
      writeEnd
   \par Description
      This function use to write the terminator of transmission frame.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void writeEnd(void)
{
  Serial3.println();
}

void writeSer1End(void)
{
  Serial.println();
}

/**
   \par Function
      writeSerial
   \par Description
      This function use to write the data to Serial3.
   \param[in]
      c - The data need to be write.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void writeSerial(uint8_t c)
{
  Serial3.write(c);
}

void writeSer1Serial(uint8_t c)
{
  Serial.write(c);
}
/**
   \par Function
      readSerial
   \par Description
      This function use to read the data from Serial, and fill the data
      to its buffer.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void readSerial(void)
{
  isAvailable = false;
  if (Serial3.available() > 0)
  {
    isAvailable = true;
    SerialRead = Serial3.read();
  }
}

void readSer1Serial(void)
{
  isSer1Available = false;
  if (Serial.available() > 0)
  {
    isSer1Available = true;
    Ser1SerialRead = Serial.read();
  }
}

void checkSer1SerialData()
{
  uint8_t i=0; 
  
  wdt_disable();


  readSer1Serial();
  while (isSer1Available)
  {
    unsigned char c = Ser1SerialRead & 0xff;
      
    if ((c == 0x56) && (isSer1Start == false))
    {
      if (ser1prevc == 0xff)
      {
        ser1index = 1;
        isSer1Start = true;
      }
    }
    else
    {
      ser1prevc = c;
      if (isSer1Start)
      {
        if (ser1index == 2)
        {
          ser1dataLen = c;
        }
        else if (ser1index > 2)
        {
          ser1dataLen--;
        }
        writeSer1Buffer(ser1index, c);
      }
    }
    ser1index++;
    if (ser1index > 51)
    {
      ser1index = 0;
      isSer1Start = false;
    }
    if (isSer1Start && (ser1dataLen == 0) && (ser1index > 3))
    {
      isSer1Start = false;
      DeviceRunMode = DEVICE_CHECK_MODE;
      for(i=0;i<15;i++)
      {
        pinMode(check_digital_pins[i],OUTPUT);
        digitalWrite(check_digital_pins[i],LOW);
      }
      UCSR3B &= 0xE7;
      //DDRJ |= 0x03;
     // PORTJ &= 0xFC;
      
      parseSer1Data();
      ser1index = 0;
    }
    
    readSer1Serial();
    
  }
  
}

void parseSer1Data(void)
{
  uint8_t i=0;
  /*
   * 0xff 0x56 0x03 cmd xx checksum
  */

  int8_t get_value = 0;
  int getAdValue  = 0;
  int AdPin[] = {A0,A1,A3,A4,A6,A7,A8,A9,A11,A12,A13,A14};
  
  isSer1Start = false;
  //uint8_t ser1_idx = readBuffer(3);
  uint8_t ser1_action = readSer1Buffer(3);
  uint8_t ser1_value = readSer1Buffer(4);

  
  switch (ser1_action)
  {
    case 0xF0:    //beep
       if(ser1_value==0)    //on
      {
          buzzer.tone(1000, 0);         
      }
      else
      {
         buzzer.delayTone(1000,1);
      } 
      writeSer1Serial(0xff);
      writeSer1Serial(0x56);
      writeSer1End();
      break;
    case 0xF1:    //led
      if(ser1_value==0)    //on
      {
          digitalWrite(13, 1);
      }
      else      //off
      {
        digitalWrite(13, 0);
      }
      writeSer1Serial(0xff);
      writeSer1Serial(0x56);
      writeSer1End();
      break;
    case 0xF2:  //io
      if(readSer1Buffer(5)==0)    //on
      {
          //for(i=0;i<15;i++)
          //{
            pinMode(check_digital_pins[ser1_value],OUTPUT);
            digitalWrite(check_digital_pins[ser1_value],HIGH);
         // }
      }
      else      //off
      {
          //for(i=0;i<15;i++)
          //{
            pinMode(check_digital_pins[ser1_value],OUTPUT);
            digitalWrite(check_digital_pins[ser1_value],LOW);
          //}
      }
      writeSer1Serial(0xff);
      writeSer1Serial(0x56);
      writeSer1End();
      break;
    case 0xF3:  //DC motor
      pinMode(12, OUTPUT);
      digitalWrite(12, HIGH);
      if(ser1_value==0)    //forward
      {
          analogWrite(10, 0);  
          digitalWrite(11, HIGH);          
          digitalWrite(46, HIGH);
          analogWrite(45, 0);
      }
      else if(ser1_value==1)     //backward
      {
          digitalWrite(10, HIGH);
          analogWrite(11, 0);
          analogWrite(46, 0);  
          digitalWrite(45, HIGH);
      }
      else
      {
          digitalWrite(10, HIGH);  
          digitalWrite(11, HIGH);          
          digitalWrite(46, HIGH);
          digitalWrite(45, HIGH);
      }
      writeSer1Serial(0xff);
      writeSer1Serial(0x56);
      writeSer1End();
      break;
    case 0xF4:  //IIC      
      get_value =  Detect_IIC();
       
      if(get_value == 0)  //not found add
      {
          writeSer1Serial(0xff);
          writeSer1Serial(0x56);
          writeSer1Serial(0x00);
          writeSer1End();
          break;        
      }
      writeSer1Serial(0xff);
      writeSer1Serial(0x56);
      writeSer1Serial(get_value);
      writeSer1End();
      break;
    case 0xF5:      
      writeSer1Serial(0xff);
      writeSer1Serial(0x56);
      writeSer1Serial(12);
      for(i=0;i<12;i++)
      {          
          pinMode(AdPin[i],INPUT);
          getAdValue = analogRead(AdPin[i]);
          writeSer1Serial((getAdValue>>2));
          /*
          Serial.print("get AD Pin:");
          Serial.print(AdPin[i]);
          Serial.print("  value:  ");
          Serial.println(getAdValue);
            */  
      }
      writeSer1End();
      break;
    default:
      break;                                               
  }
}
/**parseData
   \par Function
      parseData
   \par Description
      This function use to process the data from the Serial port,
      call the different treatment according to its action.
      ff 55 len idx action device port  slot  data a
      0  1  2   3   4      5      6     7     8
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void parseData(void)
{
  isStart = false;
  uint8_t idx = readBuffer(3);
  uint8_t action = readBuffer(4);
  uint8_t device = readBuffer(5);
  command_index = (uint8_t)idx;
  lastReceiveRC = millis();
  switch (action)
  {
    case GET:
      {
        if ((device != ULTRASONIC_SENSOR) &&
            (device != HUMITURE) &&
            (device != ULTRASONIC_ARDUINO))
        {
          writeHead();
          writeSerial(idx);
        }
        readSensor(device);
        writeEnd();
      }
      break;
    case RUN:
      {
        runModule(device);
        callOK();
      }
      break;
    case RESET:
      {
        //reset
        /* Off on-Board LED lights */
        buzzer.setpin(BUZZER_PORT);
        led.setColor(0, 0, 0, 0);
        led.show();

        /* reset On-Board encoder driver */
        Encoder_1.setPulsePos(0);
        Encoder_2.setPulsePos(0);
        Encoder_1.moveTo(0, 10);
        Encoder_2.moveTo(0, 10);
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        Encoder_1.setMotionMode(DIRECT_MODE);
        Encoder_2.setMotionMode(DIRECT_MODE);

        /* reset dc motor on driver port */
        dc.reset(PORT_1);
        dc.run(0);
        dc.reset(PORT_2);
        dc.run(0);
        dc.reset(PORT_3);
        dc.run(0);
        dc.reset(PORT_4);
        dc.run(0);

        /* reset ext encoder driver */
        encoders[0].runSpeed(0);
        encoders[1].runSpeed(0);

        /* reset stepper motor driver */
        steppers[0].setCurrentPosition(0);
        steppers[1].setCurrentPosition(0);
        steppers[2].setCurrentPosition(0);
        steppers[3].setCurrentPosition(0);
        steppers[0].moveTo(0);
        steppers[1].moveTo(0);
        steppers[2].moveTo(0);
        steppers[3].moveTo(0);

        callOK();
      }
      break;
    case START:
      {
        //start
        callOK();
      }
      break;
    case 6:

      break;
  }
}

/**
   \par Function
      callOK
   \par Description
      Response for executable commands.
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void callOK(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}

/**
   \par Function
      sendByte
   \par Description
      Send byte data
   \param[in]
      c - the byte data need be sent.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void sendByte(uint8_t c)
{
  writeSerial(1);
  writeSerial(c);
}

/**
   \par Function
      sendString
   \par Description
      Send string data
   \param[in]
      s - the string data need be sent.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void sendString(String s)
{
  int16_t l = s.length();
  writeSerial(4);
  writeSerial(l);
  for (int16_t i = 0; i < l; i++)
  {
    writeSerial(s.charAt(i));
  }
}

/**
   \par Function
      sendFloat
   \par Description
      Sned float data
   \param[in]
      value - the float data need be sent.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void sendFloat(float value)
{
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

/**
   \par Function
      sendLong
   \par Description
      Sned long data
   \param[in]
      value - the long data need be sent.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void sendLong(long value)
{
  writeSerial(6);
  val.longVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

/**
   \par Function
      sendShort
   \par Description
      Sned short data
   \param[in]
      value - the short data need be sent.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void sendShort(int16_t value)
{
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}

/**
   \par Function
      sendDouble
   \par Description
      Sned double data, same as float data on arduino.
   \param[in]
      value - the double data need be sent.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void sendDouble(double value)
{
  writeSerial(5);
  valDouble.doubleVal = value;
  writeSerial(valDouble.byteVal[0]);
  writeSerial(valDouble.byteVal[1]);
  writeSerial(valDouble.byteVal[2]);
  writeSerial(valDouble.byteVal[3]);
}

/**
   \par Function
      readShort
   \par Description
      read the short data.
   \param[in]
      idx - The data's first address in the array.
   \par Output
      None
   \return
      the short data.
   \par Others
      None
*/
int16_t readShort(int16_t idx)
{
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx + 1);
  return valShort.shortVal;
}

/**
   \par Function
      readFloat
   \par Description
      read the float data.
   \param[in]
      idx - The data's first address in the array.
   \par Output
      None
   \return
      the float data.
   \par Others
      None
*/
float readFloat(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.floatVal;
}

/**
   \par Function
      readLong
   \par Description
      read the long data.
   \param[in]
      idx - The data's first address in the array.
   \par Output
      None
   \return
      the long data.
   \par Others
      None
*/
long readLong(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.longVal;
}

char _receiveStr[20] = {};
uint8_t _receiveUint8[16] = {};

/**
   \par Function
      readString
   \par Description
      read the string data.
   \param[in]
      idx - The string's first address in the array.
   \param[in]
      len - The length of the string data.
   \par Output
      None
   \return
      the address of string data.
   \par Others
      None
*/
char* readString(int16_t idx, int16_t len)
{
  for (int16_t i = 0; i < len; i++)
  {
    _receiveStr[i] = readBuffer(idx + i);
  }
  _receiveStr[len] = '\0';
  return _receiveStr;
}

/**
   \par Function
      readUint8
   \par Description
      read the uint8 data.
   \param[in]
      idx - The Uint8 data's first address in the array.
   \param[in]
      len - The length of the uint8 data.
   \par Output
      None
   \return
      the address of uint8 data.
   \par Others
      None
*/
uint8_t* readUint8(int16_t idx, int16_t len)
{
  for (int16_t i = 0; i < len; i++)
  {
    if (i > 15)
    {
      break;
    }
    _receiveUint8[i] = readBuffer(idx + i);
  }
  return _receiveUint8;
}

/**
   \par Function
      runModule
   \par Description
      Processing execute commands.
   \param[in]
      device - The definition of all execute commands.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void runModule(uint8_t device)
{
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  uint8_t port = readBuffer(6);
  uint8_t pin = port;
     
  switch (device)
  {
    case MOTOR:
      {
        int16_t speed = readShort(7);
        dc.reset(port);
        dc.run(speed);
      }
      break;
    case ENCODER_BOARD:
      if (port == 0)
      {
        uint8_t slot = readBuffer(7);
        int16_t speed_value = readShort(8);

        if (slot == SLOT_1)
        {
          Encoder_1.setTarPWM(speed_value);
        }
        else if (slot == SLOT_2)
        {
          Encoder_2.setTarPWM(speed_value);
        }
      }
      break;

    case JOYSTICK:
      {
        int16_t leftSpeed = readShort(6);
        Encoder_1.setTarPWM(leftSpeed);
        int16_t rightSpeed = readShort(8);
        Encoder_2.setTarPWM(rightSpeed);
      }
      break;

    case STEPPER:
      {
        int16_t maxSpeed = readShort(7);
        long distance = readLong(9);
        if (port == PORT_1)
        {
          steppers[0] = MeStepper(PORT_1);
          steppers[0].moveTo(distance);
          steppers[0].setMaxSpeed(maxSpeed);
          steppers[0].setSpeed(maxSpeed);
        }
        else if (port == PORT_2)
        {
          steppers[1] = MeStepper(PORT_2);
          steppers[1].moveTo(distance);
          steppers[1].setMaxSpeed(maxSpeed);
          steppers[1].setSpeed(maxSpeed);
        }
        else if (port == PORT_3)
        {
          steppers[2] = MeStepper(PORT_3);
          steppers[2].moveTo(distance);
          steppers[2].setMaxSpeed(maxSpeed);
          steppers[2].setSpeed(maxSpeed);
        }
        else if (port == PORT_4)
        {
          steppers[3] = MeStepper(PORT_4);
          steppers[3].moveTo(distance);
          steppers[3].setMaxSpeed(maxSpeed);
          steppers[3].setSpeed(maxSpeed);
        }
      }
      break;
    case ENCODER:
      {
        uint8_t slot = readBuffer(7);
        int16_t maxSpeed = readShort(8);
        float distance = readFloat(10);
        if (slot == SLOT_1)
        {
          encoders[0].move(distance, maxSpeed);
          delay(40);
        }
        else if (slot == SLOT_2)
        {
          encoders[1].move(distance, maxSpeed);
          delay(40);
        }
      }
      break;
    case RGBLED:
      {
        uint8_t slot = readBuffer(7);
        uint8_t idx = readBuffer(8);
        uint8_t r = readBuffer(9);
        uint8_t g = readBuffer(10);
        uint8_t b = readBuffer(11);
        if (port != 0)
        {
          led.reset(port, slot);
        }
        else
        {
          led.setpin(RGBLED_PORT);
        }
        if (idx > 0)
        {
          led.setColorAt(idx - 1, LOW_BRIGHTNSS(r), LOW_BRIGHTNSS(g), LOW_BRIGHTNSS(b));
        }
        else
        {
          led.setColor( LOW_BRIGHTNSS(r), LOW_BRIGHTNSS(g), LOW_BRIGHTNSS(b));
        }
        led_mode = 0;
        led.show();
      }
      break;
    case ISKYU_RGBLED:
      {
        uint8_t slot = readBuffer(7);
        uint8_t idx = readBuffer(8);
        uint8_t h = readBuffer(9);
        uint8_t s = readBuffer(10);
        uint8_t v = readBuffer(11);
        if (port != 0)
        {
          led.reset(port, slot);
        }
        else
        {
          led.setpin(RGBLED_PORT);
        }

        led_mode = idx;

        led_hsv_rgb(&led_r, &led_g, &led_b, h, s, v);
      }
      break;
    case ISKYU_RGBLED_COLOR:
      {
        uint8_t h = readBuffer(7);
        uint8_t s = readBuffer(8);
        uint8_t v = readBuffer(9);

        led_hsv_rgb(&led_r, &led_g, &led_b, h, s, v);
      }
      break;
    case ISKYU_RGBLED_MANUAL:
      {
        uint8_t manual_h = readBuffer(7);
        uint8_t manual_l = readBuffer(8);

        uint16_t temp = manual_h;
        temp = (temp << 8) | manual_l;

        led_manual_on = temp;

        for (int i = 0; i < 12; i++ ) {
          if (temp & (0x01 << (25 - i) % 12)) {
            led_manual_on |= (0x01 << i);
          } else {
            led_manual_on &= ~(0x01 << i);
          }
        }

        led_mode = 6;
        //        led_manual_on = manual_h;
        //        led_manual_on = (led_manual_on << 8) | manual_l;
      }
      break;
    case COMMON_COMMONCMD:
      {
        uint8_t subcmd = port;
        uint8_t cmd_data = readBuffer(7);
        if (SET_AURIGA_MODE == subcmd)
        {
          if ((cmd_data & 0x80) != 0) {
            obs_enable = 1;
          } else {
            obs_enable = 0;

          }

          cmd_data &= 0x7f;

          if (auriga_mode != cmd_data) 
          {  
          if(cmd_data == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE)
               runAvoidFlag = true;
           else if(cmd_data !=  BALANCED_MODE && cmd_data != IR_REMOTE_MODE)
              Stop();
     
          if ((cmd_data == BALANCED_MODE) ||
              (cmd_data == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE) ||
              (cmd_data == BLUETOOTH_MODE) ||
              (cmd_data == IR_REMOTE_MODE) ||
              (cmd_data == LINE_FOLLOW_MODE))
          { 
             
            if((auriga_mode==BLUETOOTH_MODE)&&(cmd_data==BALANCED_MODE))//进入平衡车必须从蓝牙模式进入
                reset_gyro_flag = 1;
            if(((old_auriga_mode==BLUETOOTH_MODE)&&(cmd_data==AUTOMATIC_OBSTACLE_AVOIDANCE_MODE))||(cmd_data==BLUETOOTH_MODE))//非平衡车模式可以结束自检
                reset_gyro_flag = 0;
            PID_speed.Setpoint = 0;
            PID_turn.Setpoint = 0;
            
            old_auriga_mode = auriga_mode;
            auriga_mode = cmd_data;
            if(auriga_mode == IR_REMOTE_MODE)
            {
              old_auriga_mode = BALANCED_MODE;
              auriga_mode = AUTOMATIC_OBSTACLE_AVOIDANCE_MODE;
            }
            if (EEPROM.read(AURIGA_MODE_CONFIGURE) != auriga_mode)
            {
              EEPROM.write(AURIGA_MODE_CONFIGURE, auriga_mode);
            }
          }
          else
          {
            auriga_mode = BLUETOOTH_MODE;
            if (EEPROM.read(AURIGA_MODE_CONFIGURE) != auriga_mode)
            {
              EEPROM.write(AURIGA_MODE_CONFIGURE, auriga_mode);
            }
          }
         }
        }
      }
      break;
    case SERVO:
      {
        uint8_t slot = readBuffer(7);
        pin = slot == 1 ? mePort[port].s1 : mePort[port].s2;
        uint8_t v = readBuffer(8);
        Servo sv = servos[searchServoPin(pin)];
        if (v >= 0 && v <= 180)
        {
          if (!sv.attached())
          {
            sv.attach(pin);
          }
          sv.write(v);
        }
      }
      break;
    case SEVSEG:
      {
        //if (seg.getPort() != port)
        //{
          seg.reset(port);
           //reset port
       // }
        float v = readFloat(7);
        seg.display(v);
      }
      break;
    case LEDMATRIX:
      {
        if (ledMx.getPort() != port)
        {
          ledMx.reset(port);
        }
        uint8_t action = readBuffer(7);
        if (action == 1)
        {
          int8_t px = buffer[8];
          int8_t py = buffer[9];
          int8_t len = readBuffer(10);
          char *s = readString(11, len);
          ledMx.drawStr(px, py, s);
        }
        else if (action == 2)
        {
          int8_t px = readBuffer(8);
          int8_t py = readBuffer(9);
          uint8_t *ss = readUint8(10, 16);
          ledMx.drawBitmap(px, py, 16, ss);
        }
        else if (action == 3)
        {
          int8_t point = readBuffer(8);
          int8_t hours = readBuffer(9);
          int8_t minutes = readBuffer(10);
          ledMx.showClock(hours, minutes, point);
        }
        else if (action == 4)
        {
          ledMx.showNum(readFloat(8), 3);
        }
      }
      break;
    case LIGHT_SENSOR:
      {
        if (generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
        }
        uint8_t v = readBuffer(7);
        generalDevice.dWrite1(v);
      }
      break;
    case SHUTTER:
      {
        if (generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
        }
        uint8_t v = readBuffer(7);
        if (v < 2)
        {
          generalDevice.dWrite1(v);
        }
        else
        {
          generalDevice.dWrite2(v - 2);
        }
      }
      break;
    case DIGITAL:
      {
        pinMode(pin, OUTPUT);
        uint8_t v = readBuffer(7);
        digitalWrite(pin, v);
      }
      break;
    case PWM:
      {
        pinMode(pin, OUTPUT);
        uint8_t v = readBuffer(7);
        analogWrite(pin, v);
      }
      break;
    case TONE:
      {
        pinMode(pin, OUTPUT);
        int16_t hz = readShort(7);
        int16_t ms = readShort(9);
      /*  hz /=3;
        if(hz == 0x03A3)
        {
          hz = 0x0370;
        }*/
        hz /=3;
        if (ms > 0)
        {
          buzzer.tone(pin, hz, ms);
        }
        else
        {
          buzzer.noTone(pin);
        }
      }
      break;
    case SERVO_PIN:
      {
        uint8_t v = readBuffer(7);
        if (v >= 0 && v <= 180)
        {
          Servo sv = servos[searchServoPin(pin)];
          if (!sv.attached())
          {
            sv.attach(pin);
          }
          sv.write(v);
        }
      }
      break;
    case TIMER:
      {
        lastTime = millis() / 1000.0;
      }
      break;
    case JOYSTICK_MOVE:
    case ISKYU_JOYSTICK_A:
      {
        if (port == 0)
        {
          int16_t joy_x = readShort(7);
          int16_t joy_y = readShort(9);
          teleop_vel_l = joy_y;
          teleop_vel_a = joy_x * abs(joy_x) / 100;

          if(teleop_vel_l < -50)
             teleop_vel_a = - teleop_vel_a;
          
          if(obs_trigger == 0 || teleop_vel_l <= 0)
          {
            PID_speed.Setpoint = teleop_vel_l;
            PID_turn.Setpoint = teleop_vel_a;
          }
         /* if (abs(PID_speed.Setpoint) > 1)
          {
            move_flag = true;
          }*/
        }
      }
      break;
    case ISKYU_JOYSTICK:
      {
        if (port == 0)
        {
          int16_t joy_x = readShort(7);
          int16_t joy_y = readShort(9);

          if(joy_x > 10)
          {
            joy_x = (joy_x - 10) * 10 / 9;
          }
          else if(joy_x < -10)
          {
            joy_x = (joy_x + 10) * 10 / 9;
          }
          else
          {
            joy_x = 0;
          }

           if(joy_y > 10)
          {
            joy_y = (joy_y - 10) * 10 / 9;
          }
          else if(joy_y < -10)
          {
            joy_y = (joy_y + 10) * 10 / 9;
          }
          else
          {
            joy_y = 0;
          }

       /*   if(joy_y != 0)
          {
            int16_t temp = sqrt(joy_y * joy_y + joy_x * joy_x);
            if(joy_y > 0)
            {
               teleop_vel_l = temp * 3;
            }
            else
            {
              teleop_vel_l = - temp * 3;
            }
            teleop_vel_a = joy_x;
            //teleop_vel_l = joy_y * 3;
           // teleop_vel_a = joy_x / 2;
          }
          else
          {
            teleop_vel_l = 0;
            teleop_vel_a = joy_x * 3;
          }*/
            teleop_vel_l = joy_y * 3;
            teleop_vel_a = joy_x;
          //teleop_vel_l = joy_y * 3;
          //teleop_vel_a = joy_x * 3;
          
          //teleop_vel_a = teleop_vel_a * abs(teleop_vel_a) / 300;

          if(teleop_vel_l < 0)
             teleop_vel_a = - teleop_vel_a;
          //teleop_vel_a = joy_x * abs(joy_x) * 3 / 100;//-(double)((double)joy_x * abs((double)joy_x) * 2.8) / 255.0f;
            //teleop_vel_l = (double)joy_y * 3.25;//2.25; //0.2

         
         if (auriga_mode == BLUETOOTH_MODE)
         {           
          if (obs_trigger == 0 || teleop_vel_l <= 0) {
             Encoder_1.runSpeed(teleop_vel_l + teleop_vel_a);
            Encoder_2.runSpeed(-teleop_vel_l + teleop_vel_a);
            /*
            Serial3.print("Encoder2 is :");
            Serial3.print(-teleop_vel_l + teleop_vel_a);
            Serial3.print("teleop_vel_l is :");
            Serial3.print(teleop_vel_l);
            Serial3.print("teleop_vel_a is :");
            Serial3.print(teleop_vel_a);
            */
          }
         }
         else if(auriga_mode == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE) 
         {
          if(joy_x == 0 && joy_y == 0)
          {
            runAvoidFlag = true;
          }
           else if(old_auriga_mode == BALANCED_MODE)
           {
              PID_speed.Setpoint = teleop_vel_l;
              PID_turn.Setpoint = teleop_vel_a;
           }
           else
           {
              Encoder_1.runSpeed(teleop_vel_l + teleop_vel_a);
              Encoder_2.runSpeed(-teleop_vel_l + teleop_vel_a);
              runAvoidFlag = false;
           }
           //avoidAnceSpeed = abs(joy_y * 3);
         }
        
          rotateTagAngle = 0;
          runTagDistance = 0;
        }
      }
      break;
    case ISKYU_SET_AUTOMATIC_SPD:
      if(auriga_mode == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE) 
      {
          int16_t temp = readShort(7);
          avoidAnceSpeed = abs(temp * 3);
      }
      break;
    case ISKYU_ROTATE_ANGLE:
    case ISKYU_ROTATE_ANGLE_A:
      if (port == 0)
       {
          int16_t angleSpeed = readShort(7);
          int16_t tagAngle = readShort(9);
          rotateTagAngle = 0;
          runTagDistance = 0;
          rotateToTagAngle(angleSpeed,tagAngle);
      }
      break;
    case ISKYU_MOVE_DISTANCE :
    case ISKYU_MOVE_DISTANCE_A:
       if (port == 0)
       {
          int16_t tagSpeed = readShort(7);
          int16_t tagDis = readShort(9);
          rotateTagAngle = 0;
          runTagDistance = 0;
          runToTagDistance(tagSpeed/2.0,tagDis);
       }
      break;
    case ISKYU_OBS_ON_OFF:
      {
        if (port == 0) {
          //obs_btn_state =  readShort(7);
        }
      }
      break;
    case ISKYU_SHUTDOWN:
    {
        power_state = POWER_OFF_RISING;
        buzzer.delayTone(1500, 100);
        buzzer.noTone();
        power_off();
    }
      break;
    case ENCODER_PID_MOTION:
      {
        uint8_t subcmd = port;
        uint8_t slot_num = readBuffer(7);
        if (ENCODER_BOARD_POS_MOTION == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          speed_temp = abs(speed_temp);
          if (slot_num == SLOT_1)
          {
            Encoder_1.move(pos_temp, (float)speed_temp);
          }
          else if (slot_num == SLOT_2)
          {
            Encoder_2.move(pos_temp, (float)speed_temp);
          }
        }
        else if (ENCODER_BOARD_SPEED_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);
          if (slot_num == SLOT_1)
          {
            Encoder_1.runSpeed((float)speed_temp);
          }
          else if (slot_num == SLOT_2)
          {
            Encoder_2.runSpeed((float)speed_temp);
          }
        }
        else if (ENCODER_BOARD_PWM_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);
          if (slot_num == SLOT_1)
          {
            Encoder_1.setTarPWM(speed_temp);
          }
          else if (slot_num == SLOT_2)
          {
            Encoder_2.setTarPWM(speed_temp);
          }
        }
        else if (ENCODER_BOARD_SET_CUR_POS_ZERO == subcmd)
        {
          if (slot_num == SLOT_1)
          {
            Encoder_1.setPulsePos(0);
          }
          else if (slot_num == SLOT_2)
          {
            Encoder_2.setPulsePos(0);
          }
        }
        else if (ENCODER_BOARD_CAR_POS_MOTION == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          if (slot_num == 1)
          {
            Encoder_1.move(-pos_temp, (float)speed_temp);
            Encoder_2.move(pos_temp, (float)speed_temp);
          }
          else if (slot_num == 2)
          {
            Encoder_1.move(pos_temp, (float)speed_temp);
            Encoder_2.move(-pos_temp, (float)speed_temp);
          }
          else if (slot_num == 3)
          {
            Encoder_1.move(-pos_temp, (float)speed_temp);
            Encoder_2.move(-pos_temp, (float)speed_temp);
          }
          else if (slot_num == 4)
          {
            Encoder_1.move(pos_temp, (float)speed_temp);
            Encoder_2.move(pos_temp, (float)speed_temp);
          }
        }
      }
      break;
      case GYRO_RESET:
      {
          reset_gyro_flag = 1;
          if(auriga_mode==BALANCED_MODE)
             Stop();
                
      }
      break;
  }
}

/**
   \par Function
      searchServoPin
   \par Description
      Check if the pin has been allocated, if it is not allocated,
      then allocate it.
   \param[in]
      pin - arduino gpio number
   \par Output
      None
   \return
      the servo number be assigned
   \par Others
      None
*/
int16_t searchServoPin(int16_t pin)
{
  for (uint8_t i = 0; i < 12; i++)
  {
    if (servo_pins[i] == pin)
    {
      return i;
    }
    if (servo_pins[i] == 0)
    {
      servo_pins[i] = pin;
      return i;
    }
  }
  return 0;
}

const int16_t TEMPERATURENOMINAL     = 25;    //Nominl temperature depicted on the datasheet
const int16_t SERIESRESISTOR         = 10000; // Value of the series resistor
const int16_t BCOEFFICIENT           = 3380;  // Beta value for our thermistor(3350-3399)
const int16_t TERMISTORNOMINAL       = 10000; // Nominal temperature value for the thermistor

/**
   \par Function
      calculate_temp
   \par Description
      This function is used to convert the temperature.
   \param[in]
      In_temp - Analog values from sensor.
   \par Output
      None
   \return
      the temperature in degrees Celsius
   \par Others
      None
*/
float calculate_temp(int16_t In_temp)
{
  float media;
  float temperatura;
  media = (float)In_temp;
  // Convert the thermal stress value to resistance
  media = 1023.0 / media - 1;
  media = SERIESRESISTOR / media;
  //Calculate temperature using the Beta Factor equation

  temperatura = media / TERMISTORNOMINAL;              // (R/Ro)
  temperatura = log(temperatura); // ln(R/Ro)
  temperatura /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  temperatura += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  temperatura = 1.0 / temperatura;                     // Invert the value
  temperatura -= 273.15;                               // Convert it to Celsius
  return temperatura;
}

/**
   \par Function
      readSensor
   \par Description
      This function is used to process query command.
   \param[in]
      device - The definition of all query commands.
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void readSensor(uint8_t device)
{
  /**************************************************
      ff 55 len idx action device port slot data a
      0  1  2   3   4      5      6    7    8
  ***************************************************/
  float value = 0.0;
  uint8_t port, slot, pin;
  port = readBuffer(6);
  pin = port;
  switch (device)
  {
    case ULTRASONIC_SENSOR:
      {
        if (us == NULL)
        {
          us = new MeUltrasonicSensor(port);
        }
        else if (us->getPort() != port)
        {
          delete us;
          us = new MeUltrasonicSensor(port);
        }
        value = obs_distance;
        writeHead();
        writeSerial(command_index);
        sendFloat(value);
      }
      break;
    case  TEMPERATURE_SENSOR:
      {
        slot = readBuffer(7);
        if (ts.getPort() != port || ts.getSlot() != slot)
        {
          ts.reset(port, slot);
        }
        value = ts.temperature();
        sendFloat(value);
      }
      break;
    case  LIGHT_SENSOR:
    case  SOUND_SENSOR:
    case  POTENTIONMETER:
      {
        //if (generalDevice.getPort() != port)
       // {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(), INPUT);
      //  }
        value = generalDevice.aRead2();
        sendFloat(value);
      }
      break;
    case  TEMPERATURE_SENSOR_1:
      {
        if (generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(), INPUT);
        }
        value = calculate_temp(generalDevice.aRead2());
        sendFloat(value);
      }
      break;
    case  JOYSTICK:
      {
        slot = readBuffer(7);
        if (joystick.getPort() != port)
        {
          joystick.reset(port);
        }
        value = joystick.read(slot);
        sendFloat(value);
      }
      break;
    case  INFRARED:
      {
        if (ir == NULL)
        
        {
          ir = new MeInfraredReceiver(port);
          ir->begin();
        }
        else if (ir->getPort() != port)
        {
          delete ir;
          ir = new MeInfraredReceiver(port);
          ir->begin();
        }
        irRead = ir->getCode();
        if ((irRead < 255) && (irRead > 0))
        {
          sendFloat((float)irRead);
        }
        else
        {
          sendFloat(0);
        }
      }
      break;
    case  PIRMOTION:
      {
        //if (generalDevice.getPort() != port)
        //{
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(), INPUT);
       // }
        value = generalDevice.dRead2();
        sendFloat(value);
      }
      break;
    case  LINEFOLLOWER:
      {
        if (generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin1(), INPUT);
          pinMode(generalDevice.pin2(), INPUT);
        }
        uint8_t tmp0 = generalDevice.dRead1();
        uint8_t tmp1 = generalDevice.dRead2();
        //        Serial3.print("line: ");
        //        Serial3.print(tmp0);
        //        Serial3.println(tmp1);
        value = generalDevice.dRead1() * 2 + generalDevice.dRead2();
        sendFloat(value);
      }
      break;
    case LIMITSWITCH:
      {
        slot = readBuffer(7);
        if (generalDevice.getPort() != port || generalDevice.getSlot() != slot)
        {
          generalDevice.reset(port, slot);
        }
        if (slot == 1)
        {
          pinMode(generalDevice.pin1(), INPUT_PULLUP);
          value = !generalDevice.dRead1();
        }
        else
        {
          pinMode(generalDevice.pin2(), INPUT_PULLUP);
          value = !generalDevice.dRead2();
        }
        sendFloat(value);
      }
      break;
    case COMPASS:
      {
        if (Compass.getPort() != port)
        {
          Compass.reset(port);
          Compass.setpin(Compass.pin1(), Compass.pin2());
        }
        double CompassAngle;
        CompassAngle = Compass.getAngle();
        sendFloat((float)CompassAngle);
      }
      break;
    case HUMITURE:
      {
        uint8_t index = readBuffer(7);
        //if (humiture.getPort() != port)
        //{
          humiture.reset(port);        
        //}
        uint8_t HumitureData;
        humiture.update();
        HumitureData = humiture.getValue(index);//由于传感器升级，底层驱动作修改，这里不计算小数和负温情况
        writeHead();
        writeSerial(command_index);
        sendByte(HumitureData);
      }
      break;
    case FLAMESENSOR:
      {
        //if (FlameSensor.getPort() != port)
        //{
          FlameSensor.reset(port);
          FlameSensor.setpin(FlameSensor.pin2(), FlameSensor.pin1());
       // }
        int16_t FlameData;
        FlameData = FlameSensor.readAnalog();
        sendShort(FlameData);
      }
      break;
    case GASSENSOR:
      {
        //if (GasSensor.getPort() != port)
        //{
          GasSensor.reset(port);
          GasSensor.setpin(GasSensor.pin2(), GasSensor.pin1());
       // }
        int16_t GasData;
        GasData = GasSensor.readAnalog();
        sendShort(GasData);
      }
      break;
    case  GYRO:
      {
        uint8_t axis = readBuffer(7);
        if ((port == 0) && (gyro_ext.getDevAddr() == 0x68))     //extern gyro
        {
          value = gyro_ext.getAngle(axis);
          ///writeHead();
          ///writeSerial(command_index);
          sendFloat(value);
        }
        else if ((port == 1) && (gyro.getDevAddr() == 0x69))
        {
          value = gyro.getAngle(axis);
         // writeHead();
          //writeSerial(command_index);
          sendFloat(value);
        }
      }
      break;
    case  VERSION:
      {
        sendString(mVersion);
      }
      break;
    case  DIGITAL:
      {
        pinMode(pin, INPUT);
        sendFloat(digitalRead(pin));
      }
      break;
    case  ANALOG:
      {
        pin = analogs[pin];
        pinMode(pin, INPUT);
        sendFloat(analogRead(pin));
      }
      break;
    case  PULSEIN:
      {
        int16_t pw = readShort(7);
        pinMode(pin, INPUT);
        sendLong(pulseIn(pin, HIGH, pw));
      }
      break;
    case ULTRASONIC_ARDUINO:
      {
        uint8_t trig = readBuffer(6);
        uint8_t echo = readBuffer(7);
        long pw_data;
        float dis_data;
        pinMode(trig, OUTPUT);
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        pinMode(echo, INPUT);
        pw_data = pulseIn(echo, HIGH, 30000);
        dis_data = pw_data / 58.0;
        delay(5);
        writeHead();
        writeSerial(command_index);
        sendFloat(pw_data);
      }
      break;

    case ISKYU_BUTTON:
      {
        if (btn_status) {
          sendByte(1);
          btn_status = 0;
        } else {
          sendByte(0);
        }
      }

      break;

    case ISKYU_LINE:
      {
        if (generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin1(), INPUT);
          pinMode(generalDevice.pin2(), INPUT);
        }

        sendByte(generalDevice.dRead2() * 2 + generalDevice.dRead1());
      }

      break;

    case ISKYU_MOTION:
      {
        if ((Encoder_2.getCurrentSpeed() - Encoder_1.getCurrentSpeed()) / 2 > 0) {
          sendByte(1);
        } else if ((Encoder_2.getCurrentSpeed() - Encoder_1.getCurrentSpeed()) / 2 < 0) {
          sendByte(2);
        } else {
          sendByte(0);
        }
      }
      break;

    case TIMER:
      {
        sendFloat((float)currentTime);
      }
      break;
    case TOUCH_SENSOR:
      {
        if (touchSensor.getPort() != port)
        {
          touchSensor.reset(port);
        }
        sendByte(touchSensor.touched());
      }
      break;
    case BUTTON:
      {
        if (buttonSensor.getPort() != port)
        {
          buttonSensor.reset(port);
        }
        sendByte(keyPressed == readBuffer(7));
      }
      break;
    case ENCODER_BOARD:
      {
        if (port == 0)
        {
          slot = readBuffer(7);
          uint8_t read_type = readBuffer(8);
          if (slot == SLOT_1)
          {
            if (read_type == ENCODER_BOARD_POS)
            {
              sendLong(Encoder_1.getCurPos());
            }
            else if (read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(Encoder_1.getCurrentSpeed());
            }
          }
          else if (slot == SLOT_2)
          {
            if (read_type == ENCODER_BOARD_POS)
            {
              sendLong(Encoder_2.getCurPos());
            }
            else if (read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(Encoder_2.getCurrentSpeed());
            }
          }
        }
      }
      break;
    case COMMON_COMMONCMD:
      {
        uint8_t subcmd = port;
        if (GET_BATTERY_POWER == subcmd)
        {
          sendFloat(get_power());
        }
        else if (GET_AURIGA_MODE == subcmd)
        {
          sendByte(auriga_mode);
        }
      }
      break;
  }//switch
}

/**
   \par Function
      get_power
   \par Description
      This function used to get the value of power supply
   \param[in]
      None
   \par Output
      None
   \return
      The power vlaue(unit is V)
   \par Others
      None
*/
float get_power(void)
{
  float power;
  auriga_power = analogRead(POWER_PORT);
  power = (auriga_power / 1024.0) * 15;
  return power;
}

/**
   \par Function
      PID_angle_compute
   \par Description
      The angle process for balance car
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
/*
void PID_angle_compute(void)   //PID
{
  CompAngleX = -gyro.getAngleX();
  //double error = CompAngleX - PID_angle.Setpoint;
  double error = CompAngleX - PID_angle.Setpoint;
  PID_angle.Integral += error;
  if (abs(CompAngleX - PID_angle.Setpoint) < 1)
  {
    PID_angle.Integral = 0;
  }

  PID_angle.differential = gyro.getGyroY();
  PID_angle.Output = PID_angle.P * error + PID_angle.I * PID_angle.Integral + PID_angle.D * PID_angle.differential;
  if (PID_angle.Output > 0)
  {
    PID_angle.Output = PID_angle.Output + PWM_MIN_OFFSET;
  }
  else
  {
    PID_angle.Output = PID_angle.Output - PWM_MIN_OFFSET;
  }

  double pwm_left = PID_angle.Output - PID_turn.Output;
  double pwm_right = - PID_angle.Output - PID_turn.Output;
  if (move_flag == true)
  {
    balance_car_speed_offsets = 0;
  }
  else
  {
    balance_car_speed_offsets = 1.1 * (abs(Encoder_1.getCurrentSpeed()) - abs(Encoder_2.getCurrentSpeed()));
  }

  if (balance_car_speed_offsets > 0)
  {
    if (pwm_left > 0)
    {
      pwm_right = pwm_right - abs(balance_car_speed_offsets);
    }
    else
    {
      pwm_right = pwm_right + abs(balance_car_speed_offsets);
    }
  }
  else if (balance_car_speed_offsets < 0)
  {
    if (pwm_right > 0)
    {
      pwm_left = pwm_left - abs(balance_car_speed_offsets);
    }
    else
    {
      pwm_left = pwm_left + abs(balance_car_speed_offsets);
    }
  }

#ifdef DEBUG_INFO
  Serial3.print("Relay: ");
  Serial3.print(PID_angle.Setpoint);
  Serial3.print(" AngX: ");
  Serial3.print(CompAngleX);
  Serial3.print(" Output: ");
  Serial3.print(PID_angle.Output);
  Serial3.print(" dif: ");
  Serial3.println(PID_angle.differential);
#endif

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  Encoder_1.setMotorPwm(pwm_left);
  Encoder_2.setMotorPwm(pwm_right);
}
*/
/**
   \par Function
      PID_speed_compute
   \par Description
      The speed process for balance car
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/

static float oldAngle;
void PID_angle_compute(void)   //PID
{
  CompAngleX =  -gyro.getAngleX();
  double error0 = PID_angle.Setpoint -  CompAngleX;
  double error = 2 * error0 - (CompAngleX - oldAngle);
  oldAngle = CompAngleX;
  //PID_angle.Integral += error;

  float outPutTemp = PID_angle.P  * error + PID_angle.I * PID_angle.Integral + PID_angle.D  * (error - PID_angle.last_error);


  if(outPutTemp >  PID_angle.Output + 40)
     PID_angle.Output += 40;
  else if(outPutTemp <  PID_angle.Output - 40)
     PID_angle.Output -= 40;
  else  
     PID_angle.Output = outPutTemp;

 #ifdef DEBUG_INFO
  Serial3.print("angle:");
  Serial3.print(CompAngleX);
  Serial3.print(" ");
  Serial3.print(error0);
  Serial3.print(" ");
  Serial3.print(error);
  Serial3.print(" ");
  Serial3.println(PID_angle.Output);
#endif
     
  PID_angle.last_error = error;

  error = PID_turn.Setpoint - (Encoder_1.getCurrentSpeed() + Encoder_2.getCurrentSpeed()) / 2;
  PID_turn.Integral += error;
  PID_turn.Integral = constrain(PID_turn.Integral,-1000,1000);
  
  PID_turn.Output = PID_turn.P  * error + PID_turn.I * PID_turn.Integral + PID_turn.D  * (error - PID_turn.last_error);
  PID_turn.last_error = error;

  
  double pwm_left = PID_angle.Output + PID_turn.Output;
  double pwm_right = - PID_angle.Output + PID_turn.Output;
  
#ifdef DEBUG_INFO
  Serial3.print("angle:");
  Serial3.print(error);
  Serial3.print(" ");
  Serial3.print(PID_angle.last_error);
  Serial3.print(" ");
  Serial3.print(error - PID_angle.last_error);
  Serial3.print(" ");
  Serial3.println(PID_angle.Output);
#endif
/*
  if(pwm_left > 0)
  {
    pwm_left += 10;
  }else if(pwm_left < 0)
  {
    pwm_left -= 10;
  }

  if(pwm_right > 0)
  {
    pwm_right += 10;
  }else if(pwm_right < 0)
  {
    pwm_right -= 10;
  }*/

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  Encoder_1.setMotorPwm(pwm_left);
  Encoder_2.setMotorPwm(pwm_right);
}

void PID_speed_compute(void)
{
  static float setPointFilter = 0;
  //static float oldSpeed = 0;
  double speed_now = (Encoder_1.getCurrentSpeed() - Encoder_2.getCurrentSpeed()) / 2;
  
  setPointFilter += (PID_speed.Setpoint - setPointFilter) /12; //10;

  /*
  if(setPointFilter > 100)
    setPointFilter = 100;
  else if(setPointFilter < -100)
    setPointFilter = -100;
  */
    if(setPointFilter > 80)
    setPointFilter = 80;
  else if(setPointFilter < -80)
    setPointFilter = -80;
     
  double error = setPointFilter -  speed_now;
  
  //double error = 0.01 * error0 - (speed_now - oldSpeed);
  //oldSpeed = speed_now;
  
  PID_speed.Integral += error;

  PID_speed.Integral = constrain(PID_speed.Integral , -1500, 1500);
  PID_speed.Output = PID_speed.P * error + PID_speed.I * PID_speed.Integral;

  PID_speed.Output = constrain( PID_speed.Output , -15.0, 15.0);
  PID_angle.Setpoint = RELAX_ANGLE - PID_speed.Output;
  saveRELAX_ANGLE += (PID_angle.Setpoint - saveRELAX_ANGLE) / 100;
  if(saveRELAX_ANGLE > 15)
     saveRELAX_ANGLE = 15;
  else if(saveRELAX_ANGLE < -15)
     saveRELAX_ANGLE = -15;
#ifdef DEBUG_INFO
  Serial3.print("\nspeed:");
//  Serial3.print(error0);
 // Serial3.print(" ");
  Serial3.print(speed_now);
  Serial3.print(" ");
  Serial3.print(error);
  Serial3.print(" ");
  Serial3.print(PID_speed.Integral);
  Serial3.print(" ");
  Serial3.println(PID_angle.Setpoint);
#endif
  
}

/*
void PID_speed_compute(void)
{
  double speed_now = (Encoder_1.getCurrentSpeed() - Encoder_2.getCurrentSpeed()) / 2;

  last_speed_setpoint_filter  = last_speed_setpoint_filter  * 0.8;
  last_speed_setpoint_filter  += PID_speed.Setpoint * 0.2;

  if ((move_flag == true) && (abs(speed_now) < 8) && (PID_speed.Setpoint == 0))
  {
    move_flag = false;
    last_speed_setpoint_filter = 0;
    PID_speed.Integral = speed_Integral_average;
  }

  double error = speed_now - last_speed_setpoint_filter;
  PID_speed.Integral += error;

  if (move_flag == true)
  {
    PID_speed.Integral = constrain(PID_speed.Integral , -1500, 1500);
    PID_speed.Output = PID_speed.P * error + PID_speed.I * PID_speed.Integral;
    PID_speed.Output = constrain(PID_speed.Output , -15.0, 15.0);
  }
  else
  {
    PID_speed.Integral = constrain(PID_speed.Integral , -1500, 1500);
    PID_speed.Output = PID_speed.P * speed_now + PID_speed.I * PID_speed.Integral;
    PID_speed.Output = constrain(PID_speed.Output , -15.0, 15.0);
    speed_Integral_average = 0.8 * speed_Integral_average + 0.2 * PID_speed.Integral;
  }

#ifdef DEBUG_INFO
  Serial3.print(Encoder_2.getCurrentSpeed());
  Serial3.print(",");
  Serial3.print(Encoder_1.getCurrentSpeed());
  Serial3.print(",");
  Serial3.print(speed_now);
  Serial3.print(",");
  Serial3.print(PID_speed.Setpoint);
  Serial3.print(",");
  Serial3.print(last_speed_error_filter);
  Serial3.print(",");
  Serial3.print(last_speed_setpoint_filter);
  Serial3.print(",");
  Serial3.print(PID_speed.Integral);
  Serial3.print(",");
  Serial3.println(PID_speed.Output);
#endif
  PID_angle.Setpoint =  RELAX_ANGLE + PID_speed.Output;
}*/

int16_t agx_start_count;

/**
   \par Function
      reset
   \par Description
      The exception process for balance car
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void reset(void)
{
  if ((start_flag == false) && (abs(-gyro.getAngleX() - RELAX_ANGLE) < 5))
  {
    agx_start_count++;
  }
  if ((start_flag == true) && (abs(-gyro.getAngleX() - RELAX_ANGLE) > 25))
  {
    agx_start_count = 0;
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
    PID_speed.Integral = 0;
    PID_angle.Setpoint = RELAX_ANGLE;
    PID_speed.Setpoint = 0;
    PID_turn.Setpoint = 0;
    Encoder_1.setPulsePos(0);
    Encoder_2.setPulsePos(0);
    PID_speed.Integral = 0;
    start_flag = false;
    last_speed_setpoint_filter = 0.0;
    last_turn_setpoint_filter = 0.0;
#ifdef DEBUG_INFO
    Serial3.println("> 32");
#endif
  }
  else if (agx_start_count > 20)
  {
    //RELAX_ANGLE = saveRELAX_ANGLE;
    agx_start_count = 0;
    PID_speed.Integral = 0;
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
    PID_angle.Setpoint = -gyro.getAngleX();//RELAX_ANGLE;
    Encoder_1.setPulsePos(0);
    Encoder_2.setPulsePos(0);
    lasttime_speed = lasttime_angle = 0;
    oldAngle = -gyro.getAngleX();
    PID_angle.last_error = 0;
     PID_turn.Integral = 0;
     PID_turn.last_error = 0;
     PID_angle.Output  = 0;
    start_flag = true;
#ifdef DEBUG_INFO
    Serial3.println("< 5");
#endif
  }
}

/**
   \par Function
      parseGcode
   \par Description
      The function used to configure parameters for balance car.
   \param[in]
      cmd - Gcode command
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void parseGcode(char * cmd)
{
  char * tmp;
  char * str;
  char g_code_cmd;
  float p_value = 0;
  float i_value = 0;
  float d_value = 0;
  float relax_angle = 0;
  str = strtok_r(cmd, " ", &tmp);
  g_code_cmd = str[0];
  while (str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if ((str[0] == 'P') || (str[0] == 'p')) {
      p_value = atof(str + 1);
    } else if ((str[0] == 'I') || (str[0] == 'i')) {
      i_value = atof(str + 1);
    } else if ((str[0] == 'D') || (str[0] == 'd')) {
      d_value = atof(str + 1);
    }
    else if ((str[0] == 'Z') || (str[0] == 'z')) {
      relax_angle  = atof(str + 1);
    }
    else if ((str[0] == 'M') || (str[0] == 'm')) {
      auriga_mode  = atof(str + 1);
    }
  }
  #ifdef DEBUG_INFO
  Serial3.print("PID: ");
  Serial3.print(p_value);
  Serial3.print(", ");
  Serial3.print(i_value);
  Serial3.print(",  ");
  Serial3.println(d_value);
  #endif
  if (g_code_cmd == '1')
  {
    PID_angle.P = p_value;
    PID_angle.I = i_value;
    PID_angle.D = d_value;
    EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR, PID_angle.P);
    EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR + 4, PID_angle.I);
    EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR + 8, PID_angle.D);
  }
  else if (g_code_cmd == '2')
  {
    PID_speed.P = p_value;
    PID_speed.I = i_value;
    PID_speed.D = d_value;
    EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR, PID_speed.P);
    EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR + 4, PID_speed.I);
    EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR + 8, PID_speed.D);
  }
  else if (g_code_cmd == '3')
  {
    RELAX_ANGLE = relax_angle;
    EEPROM.put(BALANCED_CAR_NATURAL_BALANCE, relax_angle);
  }
  else if (g_code_cmd == '4')
  {
    if (EEPROM.read(AURIGA_MODE_CONFIGURE) != auriga_mode)
    {
      EEPROM.write(AURIGA_MODE_CONFIGURE, auriga_mode);
    }
    Serial3.print("auriga_mode: ");
    Serial3.println(auriga_mode);
  }
}

/**
   \par Function
      parseCmd
   \par Description
      The function used to parse Gcode command.
   \param[in]
      cmd - Gcode command
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void parseCmd(char * cmd)
{
  if ((cmd[0] == 'g') || (cmd[0] == 'G'))
  {
    // gcode
    parseGcode(cmd + 1);
  }
}

/**
   \par Function
      balanced_model
   \par Description
      The main function for balanced car model
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void balanced_model(void)
{
  if(reset_gyro_flag)  //等待陀螺仪自检
  {
     //Serial.println("wait the GYRO Calibration()");
     start_flag = false;
     return;
  }
  reset();
  if (start_flag == true)
  {
     //Serial.println("reset finish,now start");
     if ((millis() - lasttime_speed) > 100)
     {
     PID_speed_compute();
  /*    last_turn_setpoint_filter  = last_turn_setpoint_filter * 0.8;
      last_turn_setpoint_filter  += PID_turn.Setpoint * 0.2;
      PID_turn.Output = last_turn_setpoint_filter;*/
      lasttime_speed = millis();
    }
    if ((millis() - lasttime_angle) > 10)
    {
      PID_angle_compute();
      lasttime_angle = millis();
    }
  }
  else
  {
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
  }
}

/**
   \par Function
      ultrCarProcess
   \par Description
      The main function for ultrasonic automatic obstacle avoidance
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/

void ultrCarProcess(void)
{
  //uint16_t = 0;
  static unsigned long last_turn_time = 0;
  static bool turn_flag = false;
  float value = 0;
 #if 1
  if (us == NULL)
  {
    us = new MeUltrasonicSensor(PORT_1);
  }
 /// moveSpeed = 150;
  if (us != NULL)
  {
    //distance = us->distanceCm();
  }
  else
  {
    return;
  }

  if(obs_distance < 20) //((obs_distance >= 20) && (obs_distance < 40))
  {
   // randnum = 91;//= random(300);
   if(old_auriga_mode == BALANCED_MODE)
   {
     PID_speed.Setpoint = - avoidAnceSpeed / 2;
     PID_turn.Setpoint = 0;
   }
   else
   {
     Forward_Speed(- avoidAnceSpeed);
   }
    last_turn_time = millis();
  }
  else if (obs_distance < 40)
  {
    if(millis() - last_turn_time > 2000)
    {
      randnum = random(millis());
      turn_flag = randnum % 2;
    }
    if(turn_flag)
    {
         if(old_auriga_mode == BALANCED_MODE)
         {
            PID_speed.Setpoint = 0;
            PID_turn.Setpoint = avoidAnceSpeed / 3;
          }
          else
          {
             TurnRight_Speed(avoidAnceSpeed);
          }
    }
    else 
    {
         if(old_auriga_mode == BALANCED_MODE)
         {
            PID_speed.Setpoint = 0;
            PID_turn.Setpoint = -avoidAnceSpeed / 3;
          }
          else
          {
             TurnRight_Speed(- avoidAnceSpeed);
          }
    }
    last_turn_time = millis();
  }
  else
  {  
         if(old_auriga_mode == BALANCED_MODE)
         {
            PID_speed.Setpoint = avoidAnceSpeed / 2;
            PID_turn.Setpoint = 0;
          }
          else
          {
             Forward_Speed(avoidAnceSpeed);
          }
  }
  #if 0
       Serial3.print("leftflag: ");
       Serial3.print(leftflag);
       Serial3.print("     ");
       Serial3.print("rightflag: ");
       Serial3.print(rightflag);
       Serial3.print("angle_z_start:");
       Serial3.print(angle_z_start);
       Serial3.print("     ");
       Serial3.print("angle_z_current:");
       Serial3.print(angle_z_current);
       Serial3.println();
       delay(10);
 #endif
 #else
 #endif
}

/*
void ultrCarProcess(void)
{
  //uint16_t = 0;
  float value = 0;
 #if 1
  if (us == NULL)
  {
    us = new MeUltrasonicSensor(PORT_1);
  }
  moveSpeed = 150;
  if (us != NULL)
  {
    //distance = us->distanceCm();
  }
  else
  {
    return;
  }

  if(obs_distance < 30) //((obs_distance >= 20) && (obs_distance < 40))
  {
   // randnum = 91;//= random(300);
    randnum = random(300);
    if(((randnum > 190)||(leftflag == true))&&(rightflag == false))  //((randnum > 190) && (!rightflag))
    {
      if(leftflag == false)
      {
          angle_z_start =  gyro.getAngleZ();
          left_right_time = TIME_OUT_COUNT;
          steering_time = STEERING_TIMEOUT_S;
      }
      leftflag = true;
   //   TurnLeft();
       TurnLeft_iskyu();
    }
    else
    {
      if(rightflag == false)
      {
           steering_time = STEERING_TIMEOUT_S;
           angle_z_start =  gyro.getAngleZ();
           left_right_time = TIME_OUT_COUNT;
      }
      rightflag = true;     
     // TurnRight();
      TurnRight_iskyu();
    }
  }
  else if ((obs_distance < 20) && (obs_distance >= 0))
  {
    randnum = random(300);
    if (randnum > 190)
    {
      BackwardAndTurnLeft();
      for (int16_t i = 0; i < 300; i++)
      {
        wdt_reset();
        if (read_Serial() == true)
        {
          break;
        }
        else
        {
          delay(2);
        }
      }
    }
    else
    {
      BackwardAndTurnRight();
      for (int16_t i = 0; i < 300; i++)
      {
        wdt_reset();
        if (read_Serial() == true)
        {
          break;
        }
        else
        {
          delay(2);
        }
      }
    }
  }
  else
  {  
    #if 1
       if(leftflag == true)
       {
          if(((angle_z_current - angle_z_start) < -100.0f)||((angle_z_current - angle_z_start) > 250.0f))
          {
              leftflag = false;
          }           
       }
       else if(rightflag == true)
       {
          if(((angle_z_current - angle_z_start) > 100.0f)||((angle_z_current - angle_z_start) < -250.0f))
          {
              rightflag = false;
          }
       }
      #endif
   //    leftflag = false;
   //    rightflag = false;



       if((leftflag == false)&&(rightflag == false))
       {
           steering_time = 0;
           Forward();
       }
    
  }
  #if 0
       Serial3.print("leftflag: ");
       Serial3.print(leftflag);
       Serial3.print("     ");
       Serial3.print("rightflag: ");
       Serial3.print(rightflag);
       Serial3.print("angle_z_start:");
       Serial3.print(angle_z_start);
       Serial3.print("     ");
       Serial3.print("angle_z_current:");
       Serial3.print(angle_z_current);
       Serial3.println();
       delay(10);
 #endif
 #else
     //BackwardAndTurnRight();
     value = gyro.getAngleZ();
    // TurnLeft();
     TurnLeft_iskyu();
    // sendFloat(value);
  //   gyro.getAngle(axis);
 #endif
}
*/
/**
   \par Function
      IrProcess
   \par Description
      The main function for IR control mode
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/
void IrProcess()
{
  if (ir == NULL)
  {
    ir = new MeInfraredReceiver(PORT_8);
    ir->begin();
  }
  ir->loop();
  irRead =  ir->getCode();
  if ((irRead != IR_BUTTON_TEST) && (auriga_mode != IR_REMOTE_MODE))
  {
    return;
  }
  switch (irRead)
  {
    case IR_BUTTON_PLUS:
      Forward();
      break;
    case IR_BUTTON_MINUS:
      Backward();
      break;
    case IR_BUTTON_NEXT:
      TurnRight();
      break;
    case IR_BUTTON_PREVIOUS:
      TurnLeft();
      break;
    case IR_BUTTON_9:
      ChangeSpeed(factor * 9 + minSpeed);
      break;
    case IR_BUTTON_8:
      ChangeSpeed(factor * 8 + minSpeed);
      break;
    case IR_BUTTON_7:
      ChangeSpeed(factor * 7 + minSpeed);
      break;
    case IR_BUTTON_6:
      ChangeSpeed(factor * 6 + minSpeed);
      break;
    case IR_BUTTON_5:
      ChangeSpeed(factor * 5 + minSpeed);
      break;
    case IR_BUTTON_4:
      ChangeSpeed(factor * 4 + minSpeed);
      break;
    case IR_BUTTON_3:
      ChangeSpeed(factor * 3 + minSpeed);
      break;
    case IR_BUTTON_2:
      ChangeSpeed(factor * 2 + minSpeed);
      break;
    case IR_BUTTON_1:
      ChangeSpeed(factor * 1 + minSpeed);
      break;
    case IR_BUTTON_0:
      Stop();
      break;
    case IR_BUTTON_TEST:
      Stop();
      while ( ir->buttonState() != 0)
      {
        ir->loop();
      }
      auriga_mode = auriga_mode + 1;
      if (auriga_mode == MAX_MODE)
      {
        auriga_mode = BLUETOOTH_MODE;
      }
      break;
    default:
      Stop();
      break;
  }
}

/**
   \par Function
      line_model
   \par Description
      The main function for Patrol Line navigation mode
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/

#define VEL_SPEED_DEF   50
#define VEL_ANGULAR_DEF 5



void line_model(void)
{
  /*static int16_t vel_linear = VEL_SPEED_DEF, vel_angular = 0;
  uint8_t val = line.readSensors();

  switch (val)
  {
    case S1_IN_S2_IN:
      //Forward();
      vel_linear = VEL_SPEED_DEF;
      vel_angular = 0;
      break;

    case S1_IN_S2_OUT:
      //Forward();
      if(vel_linear > 0)
        vel_linear -= 2;
      if (vel_angular > - 10)  vel_angular--;
      break;

    case S1_OUT_S2_IN:
      if(vel_linear > 0)
        vel_linear -= 2;
      if (vel_angular < 10) vel_angular++;
      break;

    case S1_OUT_S2_OUT:
    
        if(vel_linear >= 0)
        {
          vel_angular =  - vel_angular;
        }
         vel_linear =  - VEL_SPEED_DEF;

         
      break;
  }*/

    uint8_t val;
  int16_t vel_linear, vel_angular;
  val = line.readSensors();
  moveSpeed = 80;
  vel_linear = 40;

  switch (val)
  {
    case S1_IN_S2_IN:
      //Forward();
      vel_linear = 50;
      vel_angular = 0;
      LineFollowFlag = 10;
      break;

    case S1_IN_S2_OUT:
      //Forward();
      vel_linear = 50;
      vel_angular = 6;
      if (LineFollowFlag > 1) LineFollowFlag--;
      break;

    case S1_OUT_S2_IN:
      // Forward();
      vel_linear = 50; 
      vel_angular = 6;
      if (LineFollowFlag < 20) LineFollowFlag++;
      break;

    case S1_OUT_S2_OUT:
      if (LineFollowFlag == 10) {
        //Backward();
        vel_linear = -50;
        vel_angular = 0;
      }
      if (LineFollowFlag < 10) {
        //TurnLeft1();
        vel_linear = 0;
        if(LineFollowFlag>5)
            LineFollowFlag = 5;
          vel_angular = 5;
      }
      if (LineFollowFlag > 10) {
        //TurnRight1();
        if(LineFollowFlag<15)
            LineFollowFlag = 15;
        vel_linear = 0;
        vel_angular = 5;
      }
      break;
  }
  Encoder_1.setMotorPwm(vel_linear + vel_angular * (LineFollowFlag - 10));
  Encoder_2.setMotorPwm(-vel_linear + vel_angular * (LineFollowFlag - 10));
  //Encoder_1.setMotorPwm(vel_linear + vel_angular * VEL_ANGULAR_DEF);
  //Encoder_2.setMotorPwm(-vel_linear + vel_angular * VEL_ANGULAR_DEF);
/*
  Serial.print("state: ");
  Serial.print(val);
 
  Serial.print("speed: ");
  Serial.print(vel_linear);
  Serial.print(", ");
  Serial.print(LineFollowFlag); 
  Serial.print(", ");
  Serial.println(vel_angular);
  */
}

#if 0
void line_model(void)
{
  uint8_t val;
  int16_t vel_linear, vel_angular;
  val = line.readSensors();
  moveSpeed = 60;
  vel_linear = 40;

  switch (val)
  {
    case S1_IN_S2_IN:
      //Forward();
      vel_linear = 40;
      vel_angular = 0;
      LineFollowFlag = 10;
      break;

    case S1_IN_S2_OUT:
      //Forward();
      vel_linear = 40;
      vel_angular = 8;
      if (LineFollowFlag > 1) LineFollowFlag--;
      break;

    case S1_OUT_S2_IN:
      // Forward();
      vel_linear = 40;
      vel_angular = 8;
      if (LineFollowFlag < 20) LineFollowFlag++;
      break;

    case S1_OUT_S2_OUT:
     // if (LineFollowFlag == 10) {
        //Backward();
        if(vel_linear > 0)
          LineFollowFlag = 20 - LineFollowFlag;
        vel_linear = - 40;
        vel_angular = 8;
   /*   }
      if (LineFollowFlag < 10) {
        //TurnLeft1();
        LineFollowFlag = 0;
        vel_linear = 0;
        vel_angular = 6;
      }
      if (LineFollowFlag > 10) {
        //TurnRight1();
        vel_linear = 0;
        vel_angular = 6;
        LineFollowFlag = 20;
      }*/
      break;
  }

  //Serial3.print("speed ");

  //Serial3.print(vel_linear);
  //Serial3.print(", ");
  //Serial3.println(vel_angular);
  Encoder_1.setMotorPwm(vel_linear + vel_angular * (LineFollowFlag - 10));
  Encoder_2.setMotorPwm(-vel_linear + vel_angular * (LineFollowFlag - 10));
}

#endif

uint8_t buf[64];
uint8_t bufindex;

/**
   \par Function
      read_Serial
   \par Description
      The function used to process Serial data.
   \param[in]
      None
   \par Output
      None
   \return
      Is there a valid command
   \par Others
      None
*/
boolean read_Serial(void)
{
  boolean result = false;
  readSerial();
  if (isAvailable)
  {
    uint8_t c = SerialRead & 0xff;
    result = true;
    if ((c == 0x55) && (isStart == false))
    {
      if (prevc == 0xff)
      {
        index = 1;
        isStart = true;
      }
    }
    else
    {
      prevc = c;
      if (isStart)
      {
        if (index == 2)
        {
          dataLen = c;
        }
        else if (index > 2)
        {
          dataLen--;
        }
        writeBuffer(index, c);
      }
    }
    index++;
    if (index > 51)
    {
      index = 0;
      isStart = false;
    }
    if (isStart && (dataLen == 0) && (index > 3))
    {
      isStart = false;
      parseData();
      index = 0;
    }
    return result;
  }
}

void power_on() {
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
}

void power_off() {
  int i;
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
 // for(i = 0 ; i < 11 ; i ++)
  led.setColor(0, 0, 0, 0);
  led.show();
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
}


/**
   \par Function
      setup
   \par Description
      Initialization function for arduino
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/


void runToTagDistance(int16_t speed,int16_t tagDistance)
{
  int16_t tagSpeed = speed * 3;
  runTagDistance = tagDistance * 18;
  //Serial3.print("runToTagDistance ");
  
  saveNowPos = (Encoder_1.getCurPos() - Encoder_2.getCurPos()) / 2;
  if(tagDistance < 0 && tagSpeed > 0)
    tagSpeed = - tagSpeed;
  runAngleDistanceSpeed = tagSpeed;
  if(auriga_mode == BLUETOOTH_MODE)
  {
    Encoder_1.runSpeed(tagSpeed);
    Encoder_2.runSpeed(-tagSpeed);
  }
  else if(auriga_mode == BALANCED_MODE)
  {
    PID_speed.Setpoint = tagSpeed;
  }
  runAngleBeginTime = millis();
  runAngleMaxTime = abs( tagDistance / speed ) * 1000 + 100000;
  
}

void updateRunDistance(void)
{
  if(runTagDistance == 0)
     return;
 // Serial3.print("Encoder_1.getCurPos ");
 // Serial3.print(Encoder_1.getCurPos());
 // Serial3.print("Encoder_2.getCurPos ");
 // Serial3.print(Encoder_2.getCurPos());
  long nowPos = (Encoder_1.getCurPos() - Encoder_2.getCurPos()) / 2;
  long disPos = abs(runTagDistance) - abs(nowPos - saveNowPos);
  if(disPos <= 0 || ( millis() - runAngleBeginTime ) >= runAngleMaxTime)
  {
     if(auriga_mode == BLUETOOTH_MODE)
    {
      //Serial3.print("END updateRunDistance ");
     Encoder_1.setMotionMode(DIRECT_MODE);
     Encoder_2.setMotionMode(DIRECT_MODE);
     Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
    }
    else if(auriga_mode == BALANCED_MODE)
    {
      PID_speed.Setpoint = 0;
    }
    runTagDistance = 0;
  }
  else if(disPos < 360)
  {
      if(auriga_mode == BLUETOOTH_MODE)
      {
          long tagSpeed = runAngleDistanceSpeed * disPos / 360;
          if(tagSpeed == 0)
          {
             if(runAngleDistanceSpeed > 0)
               tagSpeed = 1;
             else 
               tagSpeed = -1;
          }
          Encoder_1.setSpeed(tagSpeed);
          Encoder_2.setSpeed(- tagSpeed);
      }
  }
}

void rotateToTagAngle(int16_t angleSpeed,int16_t tagAngle)
{
  //Serial3.print("rotateToTagAngle ");
  if(tagAngle == 0)
    return;
  rotateTagAngle = tagAngle;
  lastAngle = -gyro.getAngleZ();;
  totalRotateAngle = 0;
  if(auriga_mode == BLUETOOTH_MODE)
  {
    int16_t angleSpeed1 = ((long)angleSpeed * 183) / (65 * 6);
    if(tagAngle < 0 && angleSpeed1 > 0)
    angleSpeed1 = - angleSpeed1;

    if(abs(angleSpeed1) < 100)
    {
      if(angleSpeed1 < 0)
      {
        angleSpeed1 = - 100;
      }
      else 
      {
        angleSpeed1 = 100;
      }
    }
    runAngleDistanceSpeed = angleSpeed1;
    Encoder_1.runSpeed(angleSpeed1);
    Encoder_2.runSpeed(angleSpeed1);
  }
  else if(auriga_mode == BALANCED_MODE)
  {
       PID_turn.Setpoint = ((long)angleSpeed) * 183 / (65 * 6);
  }
  runAngleBeginTime = millis();
  runAngleMaxTime = abs( tagAngle / angleSpeed ) * 1000 + 10000;
}

void updateRotateToAngle(void)
{
  if(rotateTagAngle == 0)
    return;
  //Serial3.print("updateRotateToAngle ");
  double nowdiffAngle = -gyro.getAngleZ() - lastAngle;
  lastAngle = -gyro.getAngleZ();

  if(nowdiffAngle > 180)
    nowdiffAngle -= 360;
  else if(nowdiffAngle < -180) 
    nowdiffAngle += 360;

  totalRotateAngle += nowdiffAngle;
  
  if(abs(totalRotateAngle - rotateTagAngle) <= 1 || abs(totalRotateAngle) >= abs(rotateTagAngle) || ( millis() - runAngleBeginTime ) >= runAngleMaxTime)
  {
   /* Serial3.print("END updateRotateToAngle ");
    Serial3.print(rotateTagAngle);
    Serial3.print(",");
    Serial3.print(gyro.getAngleZ());*/
    if(auriga_mode == BLUETOOTH_MODE)
    {
     Encoder_1.setMotionMode(DIRECT_MODE);
     Encoder_2.setMotionMode(DIRECT_MODE);
     Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
    }
    else if(auriga_mode == BALANCED_MODE)
    {
      PID_turn.Setpoint = 0;
    }
     rotateTagAngle = 0;
  }
  else if(abs(totalRotateAngle - rotateTagAngle) <= 10)
  {
    if(auriga_mode == BLUETOOTH_MODE)
    { 
      long tagSpeed = runAngleDistanceSpeed * nowdiffAngle / 10;
      if(tagSpeed == 0)
      {
        if(runAngleDistanceSpeed > 0)
           tagSpeed = 1;
        else 
           tagSpeed = -1;
      }
      Encoder_1.setSpeed(tagSpeed);
      Encoder_2.setSpeed(tagSpeed);
    }
  }
}

uint8_t Detect_IIC(void)
{
  byte error, address;
  int nDevices;

  wdt_disable();
  
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      nDevices++;
      break;
    }  
  }
  if (nDevices == 0)
  {
   // Serial.println("No I2C devices foundn");
      return 0;
  }
  else
  {
    return address;
  }
}

uint8_t Detect_GyroCalibration() 
{
    double  get_gyrX, get_gyrY, get_gyrZ;
    static uint8_t  GyroCalibrationCount = 0;

     gyro.fast_update();
     get_gyrX = gyro.getGyroX();
     get_gyrY = gyro.getGyroY();
  
     #if 0
     Serial.print("Gyro X:");
     Serial.print(gyro.getGyroX() );
     Serial.print("Gyro Y:");
     Serial.println(gyro.getGyroY() );
    #endif
    if((fabs(get_gyrX)<1)&&(fabs(get_gyrY)<3))
    {
        //gyro.begin();  
        //Serial.println("static,alibration the gyro again!!!!!");
        //delay(300);
        GyroCalibrationCount = 0;

        return 1; 
    } 
    else
    {
        gyro.begin();
          
        //Serial.println("move!Calibration the gyro again!!!!!");
        //delay(200);
        return 0;              
    }
    return 0;
}

void checkMotoTurnState(void)
{
    if((fabs(Encoder_1.getCurPwm())>20)&&(fabs(Encoder_1.getCurrentSpeed())<10)||
       (fabs(Encoder_2.getCurPwm())>20)&&(fabs(Encoder_2.getCurrentSpeed())<10)
       )   //以此判断阻转
    {
      if(motor_no_rotate_flag==0)
      {
          motor_no_rotate_flag = 1;  
          motor_no_rotate_time = millis();      
          //Serial .print(motor_no_rotate_time);
          //Serial .println(",turn to no rotate state");
      }   
      //Serial .println(",turn to no rotate state");         
      motor_stop_flag=0;
      motor_stop_time = millis();
    }
    else
    {
      //没有阻转，开始计数，5s内仍然没有阻转，清除阻转的状态
      if(motor_no_rotate_flag==1)
      {
          if(motor_stop_flag==0)
          {    
               //Serial .println("no rotate,start to get the time!!!"); 
               motor_stop_flag=1;
               motor_stop_time = millis();
          } 
      }

      if((millis()-motor_stop_time)>5*1000)
      {
          //Serial .println("no rotate>5s,clear the rotate!!!"); 
          motor_stop_flag = 0;      
          motor_no_rotate_flag = 0;
          motor_no_rotate_time = millis();
      }
      #if 0
      //在此处需要判断是否有停止的情况,根据时间来判断，超过1s，清除判断阻转
      if((fabs(Encoder_1.getCurPwm())==0)&&(fabs(Encoder_2.getCurPwm())==0))
      {    
          Serial.print("POT1 PWM:");
          Serial.print(Encoder_1.getCurPwm());
          Serial.print(",Speed1:");
          Serial.print(Encoder_1.getCurrentSpeed());
          Serial.print(",POT2 PWM:");
          Serial.print(Encoder_2.getCurPwm());
          Serial.print(",Speed:");
          Serial.print(Encoder_2.getCurrentSpeed());
          Serial .print("\n");
          Serial .println("set speed run ===0!!!");     
          if(motor_stop_flag==0)
          {    
               Serial .println("set speed stop,start to count the time!!!"); 
               motor_stop_flag=1;
               motor_stop_time = millis();
          }

          if((millis()-motor_stop_time)>1*1000)
          {
             Serial .println("stop>1s,clear the rotate!!!"); 
             motor_stop_flag = 0;      
             motor_no_rotate_flag = 0;
             motor_no_rotate_time = millis();
          }
      }
      else  //不一定同时为0
      {
              /*
        Serial .print(motor_no_rotate_time);
        Serial .println(",turn state ok");
        */            
        Serial.print("POT1 PWM:");
        Serial.print(Encoder_1.getCurPwm());
        Serial.print(",Speed1:");
        Serial.print(Encoder_1.getCurrentSpeed());
        Serial.print(",POT2 PWM:");
        Serial.print(Encoder_2.getCurPwm());
        Serial.print(",Speed:");
        Serial.print(Encoder_2.getCurrentSpeed());
        Serial .print("\n");
      
        Serial .println("ok speed run!!!");
        motor_stop_flag = 0;      
        motor_no_rotate_flag = 0;
        motor_no_rotate_time = millis();
      }
      #endif     
    }

    if(((millis()-motor_no_rotate_time)>30*1000)&&(motor_no_rotate_flag == 1))  //30s
    {
      // turn off the power
       power_state = POWER_OFF_RISING;
        buzzer.delayTone(1500, 100);
        buzzer.noTone();
        power_off();
    }    
}

void setup()
{
  power_off();
  pinMode(9, INPUT);

  Serial.begin(19200); // 用于测试硬件的串口
  
  Serial3.begin(115200);

  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  led.setpin(RGBLED_PORT);
  buzzer.setpin(BUZZER_PORT);
  led.setColor(0, 0, 0, 0);
  led.show();

  Serial3.print("Version: ");
  Serial3.println(mVersion);

  // enable the watchdog
  wdt_enable(WDTO_8S);
  delay(5);
  gyro_ext.begin();                                                               
  delay(5);
  wdt_reset();
  gyro.begin();
  delay(5);
  pinMode(13, OUTPUT);
  encoders[0] = MeEncoderMotor(SLOT_1);
  encoders[1] = MeEncoderMotor(SLOT_2);
  encoders[0].begin();
  encoders[1].begin();
  wdt_reset();
  //  if(boot_show_flag == true)
  //  {
  //    init_form_power();
  //  }
  wdt_reset();
  encoders[0].runSpeed(0);
  encoders[1].runSpeed(0);

  //Set Pwm 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  TCCR5A = _BV(WGM50);
  TCCR5B = _BV(CS51) | _BV(WGM52);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  //Encoder_1.setRatio(39.267);
  //Encoder_2.setRatio(39.267);
  Encoder_1.setRatio(47.5);//35.8);
  Encoder_2.setRatio(47.5);//35.8);
  
  Encoder_1.setPosPid(2, 0, 1.2);
  Encoder_2.setPosPid(2, 0, 1.2);
  //Encoder_1.setPosPid(15.8, 0, 2.2);
  //Encoder_2.setPosPid(15.8, 0, 2.2);
  Encoder_1.setSpeedPid(0.4, 0, 0);
  Encoder_2.setSpeedPid(0.4, 0, 0);
  Encoder_1.setMotionMode(DIRECT_MODE);
  Encoder_2.setMotionMode(DIRECT_MODE);

  readEEPROM();
  pinMode(PWR_ADC_PIN,INPUT);

  leftflag = false;
  rightflag = false;
  RELAX_ANGLE = saveRELAX_ANGLE;
  PID_angle.Setpoint = RELAX_ANGLE;
/*  PID_angle.P = 20;          //17;
  PID_angle.I = 0;           //0;
  PID_angle.D = -0.2;        //-0.2  PID_speed.Setpoint = 0;
  PID_speed.P =- 0.06;        // -0.1
  PID_speed.I =- 0.008;      // -0.008
*/
  PID_turn.P = 0.2;          
  PID_turn.I = 0.1;           
  PID_turn.D = 0; 
  PID_angle.P = 8;//8;//10;          
  PID_angle.I = 0;           //0;
  PID_angle.D =0.5;//0.5;//-0.2;        //-0.2  PID_speed.Setpoint = 0;
  PID_speed.P = 0.08;//0.08;//- 0.06;        // -0.1
  PID_speed.I = 0.004;//- 0.008;    

  //readEEPROM();

  auriga_mode = BLUETOOTH_MODE;
  //auriga_mode = BALANCED_MODE;
  old_auriga_mode = BLUETOOTH_MODE;//BLUETOOTH_MODE;
  //auriga_mode = BALANCED_MODE;
  //auriga_mode = AUTOMATIC_OBSTACLE_AVOIDANCE_MODE;
  update_sensor = lasttime_speed = lasttime_angle = millis();
  blink_time = millis();
  power_key_status_last = 1;
  power_state_update = millis();
  rotateTagAngle = 0;
  runTagDistance = 0;

  DeviceRunMode = DEVICE_NORMAL_MODE;

  reset_gyro_flag = 0; //陀螺仪重新校准的标志
}

/**
   \par Function
      loop
   \par Description
      main function for arduino
   \param[in]
      None
   \par Output
      None
   \return
      None
   \par Others
      None
*/



void loop()
{  
  currentTime = millis() / 1000.0 - lastTime;
  keyPressed = buttonSensor.pressed();
  checkSer1SerialData();
  if(DeviceRunMode==DEVICE_NORMAL_MODE)  
  {
    if (millis() - blink_time > 1000)
    {
      blink_time = millis();
      blink_flag = !blink_flag;
      if(power_state == POWER_ON || power_state == POWER_RISING)
         digitalWrite(13, blink_flag);
      else 
         digitalWrite(13, 0);
  
      if(steering_time > 0)
      {
         steering_time--;
         
      }
      else
      {
          rightflag = false;
          leftflag = false;
      }
    }
    power_manager();
    wdt_reset();
    if (power_state != POWER_ON) {
      return;
    }
    /*
    getPWRVoltage  = analogRead(PWR_ADC_PIN);
    float vol = getPWRVoltage*(5.0 / 1023.0);  
    //Serial.print("The current voltage:");
    //Serial.println(vol);
    if(vol<4.10)  //<8.5V
        PID_angle.P = 10;
    else if(vol>4.35) //>9.0
        PID_angle.P = 8;
    else
       PID_angle.P = 9;
     */
    buzzer.updata();
    if (ir != NULL)
    {
      IrProcess();
    }
    steppers[0].runSpeedToPosition();
    steppers[1].runSpeedToPosition();
    steppers[2].runSpeedToPosition();
    steppers[3].runSpeedToPosition();
  
    get_power();
 
    Encoder_1.loop();
    Encoder_2.loop(); 
    
    readSerial();
    while (isAvailable)
    {
      unsigned char c = SerialRead & 0xff;
      if ((c == 0x55) && (isStart == false))
      {
        if (prevc == 0xff)
        {
          index = 1;
          isStart = true;
        }
      }
      else
      {
        prevc = c;
        if (isStart)
        {
          if (index == 2)
          {
            dataLen = c;
          }
          else if (index > 2)
          {
            dataLen--;
          }
          writeBuffer(index, c);
        }
      }
      index++;
      if (index > 51)
      {
        index = 0;
        isStart = false;
      }
      if (isStart && (dataLen == 0) && (index > 3))
      {
        isStart = false;
        parseData();
        index = 0;
      }
      readSerial();
    }
    
   if(reset_gyro_flag)
   {   
        Serial.println("Now go to  Gyro Calibration ");
        if(Detect_GyroCalibration())
        {       
          reset_gyro_flag = 0;        
          start_flag = false;      
          Serial.println("Have Gyro Calibration Done");
        }
        
   }
   
    if (Compass.getPort() != 0)
    {
      Compass.getAngle();
    }
    
    //auriga_mode = BALANCED_MODE;
    if (auriga_mode == BLUETOOTH_MODE)
    {
      if (millis() - update_sensor > 10)
      {
        update_sensor = millis();
        gyro.fast_update();
        gyro_ext.update();
  
        updateRotateToAngle();
        updateRunDistance();
        /*
        Serial.print("POT1:");
        Serial.print(Encoder_1.getCurPos());
          Serial.print(",V1:");
        Serial.print(Encoder_1.getCurrentSpeed());
        Serial.print(",POT2:");
        Serial.print(Encoder_2.getCurPos());
        Serial.print(",V2:");
        Serial.print(Encoder_2.getCurrentSpeed());
        Serial .print("\n");
        */
        long temp = millis() / 1000;
  /*
        if(temp % 2)
        {
          disp.display((uint16_t)Encoder_2.getCurrentSpeed());
         DDRG |= 0x20;
         DDRE |= 0x08;
  
         PORTE |= 0x08;
         PORTG &= 0xdf;
  
         led.setColor(1, 0, 0, 0);
         led.show();
        }
        else 
        {
   *     DDRG |= 0x20;
         DDRE |= 0x08;
  
         PORTE &= 0xf7;
         PORTG |= 0x20;
  
         led.setColor(1, 255, 0, 0);
         led.show();
          disp.display((uint16_t)Encoder_1.getCurrentSpeed());
        }
        */
        if(millis() - lastReceiveRC >= 5000)
        {
          Stop();
        }
      }
    }
    else if (auriga_mode == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE)
    {
      if (millis() - update_sensor > 10)
      {
        update_sensor = millis();
        gyro.fast_update();
        gyro_ext.update();
      }
      //Encoder_1.setMotionMode(DIRECT_MODE);
      //Encoder_2.setMotionMode(DIRECT_MODE);
     if((millis() - lastReceiveRC) >= 5000 && !runAvoidFlag)
     {
       runAvoidFlag = true;
     }
      
      if(runAvoidFlag) 
      {
         ultrCarProcess();
      }
    }
    else if (auriga_mode == LINE_FOLLOW_MODE)
    {
      Encoder_1.setMotionMode(DIRECT_MODE);
      Encoder_2.setMotionMode(DIRECT_MODE);
      line_model();
    }
  
    if (auriga_mode == BALANCED_MODE || (old_auriga_mode == BALANCED_MODE && auriga_mode == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE))
    {
      gyro.fast_update();
      Encoder_1.setMotionMode(DIRECT_MODE);
      Encoder_2.setMotionMode(DIRECT_MODE);
      updateRotateToAngle();
      updateRunDistance();
      balanced_model();
      if(millis() - lastReceiveRC >= 5000)
      {
          PID_speed.Setpoint = 0;
          PID_turn.Setpoint = 0;
      }
      /*
        Serial3.print("POT1:");
        //Serial3.print(Encoder_1.getCurPos());
        Serial3.print(",V1:");
        Serial3.print(Encoder_1.getCurrentSpeed());
        Serial3.print(",POT2:");
        //Serial3.print(Encoder_2.getCurPos());
        Serial3.print(",V2:");
        Serial3.print(Encoder_2.getCurrentSpeed());
        Serial3.print("\n");
        */
    }
  
    ledTask();
    button_trigger();
    obs_task();
    checkMotoTurnState();
  }
}


uint32_t update_led, led_mode_last;;
uint8_t led_index;

void led_face(uint16_t face, uint8_t r, uint8_t g, uint8_t b)
{
  //led.setColor(0, 0, 0);
  for (int i = 0; i < 12; i++) {
    if ((face >> i) & 0x01) {
      led.setColor(i + 1,  LOW_BRIGHTNSS(r), LOW_BRIGHTNSS(g), LOW_BRIGHTNSS(b));
    } else {
      led.setColor(i + 1, 0, 0, 0);
    }
  }
}

void led_hsv_rgb(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t h, uint8_t s, uint8_t v)
{
  if ( h > 180 || s > 100 || v > 100) {
    return;
  }

  uint8_t tmp_h = (h / 30);
  float tmp_s = s / 100.0f;
  float tmp_v = v / 100.0f;
  float f = (float)h / 30.0f - (float)tmp_h;
  float p = tmp_v * (1 - tmp_s);
  float q = tmp_v * (1 - f * tmp_s);
  float t = tmp_v * ( 1 - (1 - f) * tmp_s);

  switch (tmp_h) {
    case 0:
      *r = tmp_v * 255.0f;
      *g = t * 255.0f;
      *b = p * 255.0f;
      break;
    case 1:
      *r = q * 255.0f;
      *g = tmp_v * 255.0f;
      *b = p * 255.0f;
      break;
    case 2:
      *r = p * 255.0f;
      *g = tmp_v * 255.0f;
      *b = t * 255.0f;
      break;
    case 3:
      *r = p * 255.0f;
      *g = q * 255.0f;
      *b = tmp_v * 255.0f;
      break;
    case 4:
      *r = t * 255.0f;
      *g = p * 255.0f;
      *b = tmp_v * 255.0f;
      break;
    case 5:
      *r = tmp_v * 255.0f;
      *g = p * 255.0f;
      *b = q * 255.0f;
      break;
    case 6:
      *r = tmp_v * 255.0f;
      *g = t * 255.0f;
      *b = p * 255.0f;
      break;
    default:
      break;
  }

}

const uint16_t led_face_data[12] = {0xFFF, 0x7FF, 0x3FE, 0x1FC, 0xF8, 0x70, 0x20, 0x70, 0xF8, 0x1FC, 0x3FE, 0x7FF};

void ledTask(void)
{
  if ((led_mode_last != led_mode) || (millis() - update_led > 200 && led_mode > 0))
  {
    update_led = millis();
    led_mode_last = led_mode;

    switch (led_mode) {
      case 1:
        led.setColor(0, 0, 0);
        led.show();
        break;
      case 2:
        led.setColor(0, 0, 0);
        led.setColor(led_index  + 1, LOW_BRIGHTNSS(led_r), LOW_BRIGHTNSS(led_g), LOW_BRIGHTNSS(led_b));
        led_index = (led_index + 1) % 12;
        led.show();
        break;
      case 3:
        led_face(1273, led_r, led_g, led_b);
        led.show();
        break;
      case 4:
        if (led_index < 0 || led_index > 12) {
          led_index = 0;
        }
        led_face(led_face_data[led_index], led_r, led_g, led_b);
        led_index = (led_index + 1) % 12;
        led.show();
        break;
      case 5:
        led.setColor(LOW_BRIGHTNSS(led_r), LOW_BRIGHTNSS(led_g), LOW_BRIGHTNSS(led_b));
        led.show();
        break;
      case 6:
        led_face(led_manual_on,LOW_BRIGHTNSS(led_r), LOW_BRIGHTNSS(led_g), LOW_BRIGHTNSS(led_b));
        led.show();
        break;
      default:
        break;
    }
  }

}

void power_manager() {
  uint8_t power_key = digitalRead(31);
  
  switch (power_state) {
    case POWER_INIT:
/*
      if (!power_key) {
        if ((millis() - power_state_update) > 10) {
          power_state_update = millis();
          power_key_filter++;
        }
      } else {
        power_key_filter = 0;
      }

      if (power_key_filter > 50) {*/
        
        led.setColor(0, 0, 0, 0);
        led.show();
        power_state = POWER_RISING;
        buzzer.delayTone(1000, 100);
        buzzer.noTone();
    //  }
      
      break;
    case POWER_RISING:
      //release btn
      if (power_key) {
        power_state = POWER_ON;
      }
      break;
    case POWER_ON:
      power_on();

      if(power_key_status_last==0)    //上次为0
      {
        if (power_key) 
          power_key_filter = 1;
        else 
          power_key_filter = 0;   
      }   
                
      if(power_key_filter!=0)
      { 
          if ((millis() - power_state_update) > 10) {
            power_state_update = millis();
            power_key_filter++;
          }
      }
 
      if (power_key_filter > 150) {
        power_key_filter = 0;
        power_state = POWER_OFF_RISING;
        power_off();
        buzzer.delayTone(1500, 100);
        buzzer.noTone();
      }     
      break;
    case POWER_OFF_RISING:
      if (power_key) {
        power_state = POWER_OFF;
      }
      break;
    case POWER_OFF:
      if(power_key_status_last==0)    //上次为0
      {
        if (power_key) 
          power_key_filter = 1;
        else 
          power_key_filter = 0;   
      }  
       if(power_key_filter!=0)
      { 
          if ((millis() - power_state_update) > 10) {
            power_state_update = millis();
            power_key_filter++;
          }
      }
      /*
     if (power_key) {
        if ((millis() - power_state_update) > 10) {
          power_state_update = millis();
          power_key_filter++;
        }
      } else {
        power_key_filter = 0;
      }
      */
      if (power_key_filter > 50) {
        power_key_filter = 0;
        led.setColor(0, 0, 0, 0);
        led.show();
        power_state = POWER_RISING;
        buzzer.delayTone(1000, 100);
        buzzer.noTone();
      }
      break;
    default:
      break;
  }
  power_key_status_last = power_key;
}

uint8_t btn_flag=0;
uint8_t btn_last_status;
long btn_state_update;
void button_trigger() 
{
  if (digitalRead(31)) 
  {
    if((millis() - btn_state_update) > 300)
    {
      btn_state_update = millis();
      btn_flag = 1;      
    }
  } 
  else
  {
    //松手检测
    if(btn_flag==1)
    {
      btn_status = 1;
      btn_flag = 0;
    }
    if ((millis() - btn_state_update) > 3000) 
    {
      btn_state_update = millis();
      btn_status = 0;
    }
  }
}



void obs_task() {
  float value = 0;
  if (us == NULL)  {
    us = new MeUltrasonicSensor(PORT_1);
  }

  //

    if ( millis() - obs_state_update > 10 ) {
      obs_state_update = millis();
      obs_distance = us->distanceCm();

     // Serial3.print(obs_distance);
    //  Serial3.print("\n");
    if(obs_enable && teleop_vel_l > 0)
    {
      if(obs_distance < 20)
     {
      if(auriga_mode == BLUETOOTH_MODE)
      {
       //teleop_vel_l = 0;
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);
      }
      else if(auriga_mode == BALANCED_MODE)
      {
        PID_speed.Setpoint = 0;
      }
      obs_trigger = 1;
     }
     else if(obs_distance < 40)
     {
      if(auriga_mode == BLUETOOTH_MODE)
      {
        //Encoder_1.setTarPWM((teleop_vel_l + teleop_vel_a) * (obs_distance - 20) / 20);
        //Encoder_2.setTarPWM((-teleop_vel_l + teleop_vel_a) * (obs_distance - 20) / 20);
        Encoder_1.setSpeed((teleop_vel_l + teleop_vel_a) * (obs_distance - 20) / 20);
        Encoder_2.setSpeed((-teleop_vel_l + teleop_vel_a) * (obs_distance - 20) / 20);
      }
      else if(auriga_mode == BALANCED_MODE)
      {
            PID_speed.Setpoint = teleop_vel_l * (obs_distance - 20) / 20;
      }
      obs_trigger = 1;
     }
     else 
     {
       obs_trigger = 0;
     }
    }
  }
  else 
  {
     obs_trigger = 0;
  }
  /*if ( millis() - obs_state_update > 100 ) {
    obs_state_update = millis();
 //   Serial3.print("Z-ANGLE: ");
    obs_distance = us->distanceCm();
    angle_z_current = gyro.getAngleZ();
  //  angle_z_current = gyro.getAngle(3);
   // Serial3.print(angle_z_current);
   // Serial3.println(" ");
    
    if (obs_enable && obs_distance < 20 && teleop_vel_l > 0) {
      obs_trigger = 1;
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
    } else {
      obs_trigger = 0;
    }
  }*/
}



