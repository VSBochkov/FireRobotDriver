#include <BTCA2A.h>
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
unsigned char CS = 9, SO = 11, SCK_ = 13;
btca2a bluetooth;
double gun_xy_ang;
double gun_z_ang;
unsigned long cnt_max = 1600000;
unsigned long cnt = 0;

double kGun_xy = 1., kGun_z = -1.;
unsigned char drive_sm;
unsigned char drive_sm_prev;
unsigned char auto_pump;
unsigned char purpose_command();
void doing_command();
void stop_servo(Servo *servo);
void run_servo(Servo *servo, int pin, int comm);
double getTemperature();

void setup() {
  // put your setup code here, to run once:
//Serial.begin(9600);
  bluetooth.SetupHardwareSerial(0, 115200);
  pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);   // выход, выбор кристалла - "/CS" = 1
  pinMode(SO, INPUT);                            // вход, данные из термопреобразователя - "SO"      
  pinMode(SCK_, OUTPUT); digitalWrite(SCK_, LOW);// выход, тактирование считываемых данных - "SCK" = 0
  
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
  doing_command();
}

void loop() {
  // put your main code here, to run repeatedly:
  bluetooth.ReadCommand();
  drive_sm_prev = drive_sm;
  drive_sm = purpose_command();
  if(cnt == 0) {
    if(getTemperature() > 250)
      auto_pump = PUMP_ON; //if(drive_sm != drive_sm_prev)
    else
      auto_pump = PUMP_OFF;
  }
  doing_command();
  cnt = (cnt + 1) % 500;
}

unsigned char purpose_command()
{
  if(bluetooth.ButtonPressed(KEYCODE_DPAD_UP)) {
    if(bluetooth.ButtonPressed(KEYCODE_BUTTON_R1)) {
      if(gun_z_ang < 170)    return GUN_UP;
      else                   return DRIVE_STOP;
    }
    return DRIVE_FULL_FORW;
  }
  else if(bluetooth.ButtonPressed(KEYCODE_DPAD_DOWN)) {
    if(bluetooth.ButtonPressed(KEYCODE_BUTTON_R1)) {
       if (gun_z_ang > 10)   return GUN_DOWN;
       else                  return DRIVE_STOP;
    }
    return DRIVE_FULL_BACK;
  }
  else if(bluetooth.ButtonPressed(KEYCODE_DPAD_LEFT)) {
    if(bluetooth.ButtonPressed(KEYCODE_BUTTON_R1)) {
      if(gun_xy_ang < 170)   return GUN_LEFT;
      else                   return DRIVE_STOP;
    }
    return DRIVE_LEFT;
  }
  else if(bluetooth.ButtonPressed(KEYCODE_DPAD_RIGHT)) {
    if(bluetooth.ButtonPressed(KEYCODE_BUTTON_R1)) {
      if(gun_xy_ang > 10)    return GUN_RIGHT;
      else                   return DRIVE_STOP;
    }
    return DRIVE_RIGHT;
  }
  else if(bluetooth.ButtonPressed(KEYCODE_BUTTON_L1))
    return PUMP_ON;
  else
    return DRIVE_STOP;
}

void doing_command()
{
  int cfr = SERVO_STOP, cfl = SERVO_STOP, cbr = SERVO_STOP, cbl = SERVO_STOP;
  int pump = LOW;
  
  if (auto_pump == PUMP_ON)
    pump = HIGH;
    
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
      gun_z_ang += kGun_z * 0.01;
      gun_z.write(gun_z_ang);
      delay(1);
      break;
    }
    case GUN_DOWN:
    {
      gun_z_ang -= kGun_z * 0.01;
      gun_z.write(gun_z_ang);
      delay(1);
      break;
    }
    case GUN_LEFT:
    {
      gun_xy_ang += kGun_xy * 0.01;
      gun_xy.write(gun_xy_ang);
      delay(1);
      break;
    }
    case GUN_RIGHT:
    {
      gun_xy_ang -= kGun_xy * 0.01;
      gun_xy.write(gun_xy_ang);
      delay(1);
      break;
    }
    case PUMP_ON:
    {
      pump = HIGH;
      break;
    }
  }
    
  run_servo(&forw_left, PIN_SERVO_FL, cfl);
  run_servo(&forw_right, PIN_SERVO_FR, cfr);
  run_servo(&back_left, PIN_SERVO_BL, cbl);
  run_servo(&back_right, PIN_SERVO_BR, cbr);
  digitalWrite(PUMP_PIN, pump);
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

double getTemperature()
{
// -----------------------------------------------
 // Чтение бит D31...D16 из микросхемы MAX31855K
    digitalWrite(CS, LOW);                         // активация MAX31855K, "/CS" = 0
    digitalWrite(SCK_, HIGH);                      // задать "SCK" = 1
    byte Sign = digitalRead (SO);                       // чтение знака температуры термопары "Sign", бит D31
    digitalWrite(SCK_, LOW);                       // задать "SCK" = 0   
    word Temp = 0;                                      // Temp = 0
    byte Dxx = 13;                                 // задание Dxx, т.е. количества считываемых бит
    while (Dxx != 0)                               // повторять до тех пор, пока Dxx не станет равно нулю:
    {
       digitalWrite(SCK_, HIGH);                // задать "SCK" = 1
       Temp = Temp << 1;                        // сдвиг Temp на один разряд влево
       if(digitalRead(SO) == 1)                  // чтение бит температуры от D30 до D18 с
         bitSet(Temp, 0);                  // установкой в "1" нулевого бита переменной Temp (если это нужно)
       digitalWrite(SCK_, LOW);                 // задать "SCK" = 0              
       Dxx = Dxx - 1;                           // декрементирование Dxx
    }            
    digitalWrite(SCK_, HIGH);                      // задать "SCK" = 1, холостой такт, бит D17
    digitalWrite(SCK_, LOW);                       // задать "SCK" = 0, холостой такт
    digitalWrite(SCK_, HIGH);                      // задать "SCK" = 1
    byte Fault = digitalRead(SO);                       // чтение бита ошибки подключения термопары Fault, бит D16
    digitalWrite(SCK_, LOW);                       // задать "SCK" = 0       
    digitalWrite(CS, HIGH);                        // деактивация MAX31855K - "/CS" = 1
 // -----------------------------------------------
 // Анализ бита "Fault" и действия программы если есть ошибка подключения термопары   
    if(Fault == 1)
		return getTemperature();
 // -----------------------------------------------  
 // Преобразование отрицательной температуры в положительное число
    if (Sign == 1) 
    {
      Temp = ~ Temp;                 // инвертирование переменной Temp               
      Temp = Temp + 1;               // инкрементирование переменной Temp
      Temp = Temp & 8191;            // обнуление старших разрядов Temp
      if (Temp >= 1080)                            // ограничение отрицательной температуры
        Temp = 1080;                               // величиной "-270.0" град С (270 D = 100 0011 1000 B)                                       
    }
 // -----------------------------------------------                  
 // Разделение измеренной температуры на целые и десятые доли градуса 
    byte Temp_ = Temp & 3;                              // выделение десятых долей градуса температуры
    Temp = Temp >> 2;                              // выделение целых градусов температуры
    if (Temp >= 1372)                              // ограничение выводимой на график температуры
      Temp = 1372;                                 // величиной +1372 град С (в случае положительной
                                                   // температуры)

	// Вычисление температуры
    byte T_0 = ((5*bitRead(Temp_,1)) + (3*bitRead(Temp_,0))); // вычисление значения десятых
 	double temperature = (Sign == 1 ? -1. : 1.) * Temp + ((double) T_0 / 10.);
	return temperature;
}
