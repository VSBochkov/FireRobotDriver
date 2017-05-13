#include <Servo.h>
#include <Multiservo.h>
#include <Wire.h>

#define    PIN_SERVO_FL  3
#define    PIN_SERVO_FR  2
#define    PIN_SERVO_BL  5
#define    PIN_SERVO_BR  6
#define    PIN_SERVO_GXY 7
#define    PIN_SERVO_GZ  8
#define    PUMP_PIN      12

#define    DRIVE_STOP    0
#define    DRIVE_FULL_FORW    1
#define    DRIVE_FULL_BACK    2
#define    DRIVE_LEFT    3
#define    DRIVE_RIGHT   4
#define    DRIVE_REVERT  5
#define    GUN_UP        6
#define    GUN_DOWN      7
#define    GUN_RIGHT     8
#define    GUN_LEFT      9
#define    PUMP_ON       10
#define    PUMP_OFF      11

#define    SERVO_FORW    0
#define    SERVO_STOP    90
#define    SERVO_BACK    180

#define    RS_FORW      SERVO_FORW
#define    RS_BACK      SERVO_BACK
#define    LS_FORW      SERVO_BACK
#define    LS_BACK      SERVO_FORW

Servo forw_left;
Servo forw_right;
Servo back_left;
Servo back_right;
Servo gun_xy;
Servo gun_z;
double gun_xy_ang;
double gun_z_ang;

char robot_forw = '1',
     robot_back = '2', 
     robot_right = '3', 
     robot_left = '4',
     gun_up = '5',
     gun_down = '6',
     gun_left = '7',
     gun_right = '8',
     pump_on = 'p',
     stop = 's',
     pump_off = 'e',
     auto_gun = 'a';

char command = stop;

double kGun_xy = 1., kGun_z = -1.;
unsigned char drive_sm;
unsigned char auto_gun_state = 0;
unsigned char purpose_command();
void doing_command();
void stop_servo(Servo *servo);
void run_servo(Servo *servo, int pin, int comm);
double getTemperature();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  drive_sm = DRIVE_STOP;
  pinMode(PUMP_PIN, OUTPUT);
  forw_left.attach(PIN_SERVO_FL);
  forw_right.attach(PIN_SERVO_FR);
  back_left.attach(PIN_SERVO_BL);
  back_right.attach(PIN_SERVO_BR);
  gun_xy.attach(PIN_SERVO_GXY);
  gun_z.attach(PIN_SERVO_GZ);
  gun_xy_ang = 90;
  gun_z_ang  = 90;
  gun_xy.write(gun_xy_ang);
  gun_z.write(gun_z_ang);
  digitalWrite(PUMP_PIN, LOW);
  doing_command();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(Serial.available()) {
    drive_sm = purpose_command();
    doing_command();
    Serial.write('o');
  } else {
    doing_command();
  }
  delay(1);
}

unsigned char purpose_command()
{
  command = (char)Serial.read();

  if (command == auto_gun) {
     auto_gun_state = auto_gun_state == 0 ? 1 : 0;
     return DRIVE_STOP;
  }
  if (command == robot_forw) {
     return DRIVE_FULL_FORW;
  }
  else if (command == robot_back) {
     return DRIVE_FULL_BACK;
  }
  else if (command == robot_left) {
     return DRIVE_LEFT;
  }
  else if (command == robot_right) {
     return DRIVE_RIGHT;
  }
  else if (command == gun_up) {
      if(gun_z_ang < 170)    return GUN_UP;
      else                   return DRIVE_STOP;
  }
  else if (command == gun_down) {
       if (gun_z_ang > 10)   return GUN_DOWN;
       else                  return DRIVE_STOP;
  }
  else if (command == gun_left) {
      if(gun_xy_ang < 170)   return GUN_LEFT;
      else                   return DRIVE_STOP;
  }
  else if (command == gun_right) {
      if(gun_xy_ang > 10)    return GUN_RIGHT;
      else                   return DRIVE_STOP;
  }
  else if (command == pump_on) {
    return PUMP_ON;
  } 
  else if (command == pump_off) {
    return PUMP_OFF;
  }
  else if (command == stop) {
    return DRIVE_STOP;
  }
}

void doing_command()
{
  int cfr = SERVO_STOP, cfl = SERVO_STOP, cbr = SERVO_STOP, cbl = SERVO_STOP;
    
  switch(drive_sm) {
    case DRIVE_FULL_FORW: 
    {
      cfr = RS_FORW;
      cfl = LS_FORW;
      cbr = RS_FORW;
      cbl = LS_FORW;
      break;
    }
    case DRIVE_FULL_BACK: 
    {
      cfr = RS_BACK;
      cfl = LS_BACK;
      cbr = RS_BACK;
      cbl = LS_BACK;
      break;
    }
    case DRIVE_LEFT: 
    {
      cbr = RS_FORW;
      cfr = RS_FORW;
      cbl = LS_BACK;
      cfl = LS_BACK;
      break;
    }
    case DRIVE_RIGHT: 
    {
      cbl = LS_FORW;
      cfl = LS_FORW;
      cbr = RS_BACK;
      cfr = RS_BACK;
      break;
    }
    case DRIVE_REVERT: 
    {
      cfl = LS_FORW;
      cbr = RS_FORW;
      break;
    }
    case GUN_UP:
    {
      if (auto_gun_state == 1) {
          gun_z_ang += 1.0;
          drive_sm = DRIVE_STOP;
      } else {
          gun_z_ang += kGun_z * 0.01;
      }
      gun_z.write(gun_z_ang);
      break;
    }
    case GUN_DOWN:
    {
      if (auto_gun_state == 1) {
          gun_z_ang -= 1.0;
          drive_sm = DRIVE_STOP;
      } else {
          gun_z_ang -= kGun_z * 0.01;
      }
      gun_z.write(gun_z_ang);
      break;
    }
    case GUN_LEFT:
    {
      if (auto_gun_state == 1) {
          gun_xy_ang += 1.0;
          drive_sm = DRIVE_STOP;
      } else {    
          gun_xy_ang += kGun_xy * 0.01;
      }
      gun_xy.write(gun_xy_ang);
      break;
    }
    case GUN_RIGHT:
    {
      if (auto_gun_state == 1) {
          gun_xy_ang -= 1.0;
          drive_sm = DRIVE_STOP;
      } else {    
          gun_xy_ang -= kGun_xy * 0.01;
      }
      gun_xy.write(gun_xy_ang);
      break;
    }
    case PUMP_ON:
    {
      digitalWrite(PUMP_PIN, HIGH);
      break;
    }
    case PUMP_OFF:
    {
      digitalWrite(PUMP_PIN, LOW);
      break;
    }
  }
    
  run_servo(&forw_left, PIN_SERVO_FL, cfl);
  run_servo(&forw_right, PIN_SERVO_FR, cfr);
  run_servo(&back_left, PIN_SERVO_BL, cbl);
  run_servo(&back_right, PIN_SERVO_BR, cbr);
}

void stop_servo(Servo *srv)
{
 if(srv->attached())
   srv->detach();
}

void run_servo(Servo *srv, int pin, int comm)
{
  if(comm == SERVO_STOP)
  {
    stop_servo(srv);
    return;
  }
  srv->attach(pin);
  srv->write(comm);
}
