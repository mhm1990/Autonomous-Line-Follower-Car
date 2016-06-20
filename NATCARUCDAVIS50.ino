// Arduino Mega

#include <SoftwareSerial.h>
#include <Servo.h>


//RC CONTROL
#define mode 8
#define motor_switch 13
#define servo_switch 12
unsigned int motor_in;
unsigned int servo_in;

// For camera
uint8_t clockPin = 3;
uint8_t syncPin = 4;
uint8_t dataPin = A0; // must be in range 0 to 5
unsigned int lightVal[128];// to store 128 bits of camera data
unsigned int expose = 1000; // delay time in micro-sec for camera to capture new line pixels
unsigned long uptime, downtime; // to calculate one time cycle of entire code in microseconds

// For Motor
#define pmos 11
#define nmos 7
#define max_speed 50 //20 
#define turn_speed 70 //60
uint8_t motor_speed = 50;

// For servo
#define servo_left 1300  //2000
#define servo_center 1550
#define servo_right 1800 //1000
float _servo[128];
double k = 0;

Servo myservo;  // create servo object to control a servo
#define Filter_Const 0.25 //.20 = 20//0 to 1
#define kp      1.25// 1.12 = 20//1.2 = 40 //1.5 = 50//kp=1.25 for speed = 60 80 110
#define kd      0.10//0.08 = 20 //0.11 = 40   //0.07 = 50 //0.10 for 60 80
#define ki      0.00   //0.01 for 80

float p = 0, p_last = 0;
float _period = 0, _error = 0, _dterror = 0, _integral = 0, _lasterror = 0, _pid = 0, Filter_Out = 0, Filter_Out_Prev = 0;


//To increase increase analog read speed
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif




void setup() {

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(pmos, OUTPUT);
  pinMode(nmos, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(syncPin, OUTPUT);
  pinMode(dataPin, INPUT);
  pinMode(mode, INPUT);
  pinMode(motor_in, INPUT);
  pinMode(servo_in, INPUT);

//to map servo value to camera data
  for (int i = 0; i < 128; i++)
  {
    //_servo[i] = servo_left - (i*7.85);
    _servo[i] = servo_left + (i * 3.90625); //500
    //     _servo[i] = servo_left - (i*4.6875); //600
    // _servo[i] = servo_left - (i*5.46875); //700
  }
  Serial.begin(115200);

  cbi(ADCSRA, ADPS2) ; //sbi
  sbi(ADCSRA, ADPS1) ; //cbi
  cbi(ADCSRA, ADPS0) ;

}

void getCamera() // Read camera data
{
  // camera initialization sequence
  digitalWrite(clockPin, LOW);
  digitalWrite(syncPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(syncPin, LOW);
  digitalWrite(clockPin, LOW);
  
// delay to provide enough time for the camera to capture new data
// Varies according to desired max speed of the car
  delayMicroseconds(expose);
  
  k = 0;

  for (int j = 0; j < 128; j++)
  {
    //delayMicroseconds(20);
    lightVal[j] = analogRead(dataPin);
    k = k + lightVal[j];
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
  //Serial.println(lightVal);
  //delayMicroseconds(20);

  k = k / 128; //take average of camera raw data to to define threshold value

  k = k*1.15;//k * 1.25; //k * 1.07; //1.25 ;//+ 5; Filter to deect line in different lighting condition.
  //k = 1.3*k + 13;
  //k=(((0.1-k)*1)/4)+k;
  //Serial.println(k);

//For debugging the camera values. Check the line position
//      for (int j = 0; j < 128; j++)
//      {
//  
//           if (lightVal[j]>k)
//            {
//             // lightVal[j] =1 ;
//              //k=j;
//              Serial.print('1');
//            }
//            else
//            {
//             //lightVal[j] = 0;
//              Serial.print('0');
//            }
//  
////            Serial.print(lightVal[j]);
////            Serial.print(" ");
//  
//      }
//      Serial.println(" ");

  // Serial.println(k);

}


void Servo_out() // Servo Control
{

  for (int i = 0; i < 128; i++)
  {
    //Serial.println('1');
    if (lightVal[i] >= k && lightVal[i + 1] >= k && lightVal[i + 2] >= k && lightVal[i + 3] >= k)// && lightVal[i + 4] >= k ) //&& lightVal[i+5]>=k)//&& lightVal[i+1]==1 && lightVal[i+2]==1 && lightVal[i+3]==1&&lightVal[i+4]==1)
    {

      for (int j = 128; j > 0; j--)
      {

        if (lightVal[j] >= k && lightVal[j - 1] >= k && lightVal[j - 2] >= k && lightVal[j - 3] >= k )//&& lightVal[j - 4] >= k ) //&& lightVal[j-5]>=k)//&& lightVal[j-1]==1&& lightVal[j-2]==1&& lightVal[j-3]==1&& lightVal[j-4]==1)
        {

          if ((j - i) < 10)
          {
            //PID approach 1
            //p = servo_center+(_servo[(i+j)/2]-servo_center)*kp;
            _error =  servo_center - _servo[(i + j) / 2] ;
            _dterror = ( _error - _lasterror) / _period;
            _integral = _integral + _error * _period;
            _pid = _error * kp + _integral * ki + _dterror * kd;
            _lasterror = _error;
            p = servo_center - _pid;
            //                      Serial.print(p);
            //                      Serial.print(" ");

            //PID approach 2
//                                  _error = servo_center -  _servo[(i+j)/2];
//                                  //_dterror =( _error-_lasterror)/_period;
//                                  Filter_Out_Prev = Filter_Out; // save previous value
//                                  Filter_Out = Filter_Const*_error + (1-Filter_Const)* Filter_Out; // take a portion of the new input and a portion of the old value to average out over several samples
//                                  // simulate a first order exponential filter.
//                                  _dterror = (Filter_Out - Filter_Out_Prev) / _period; // derivative output based on filtered value
//                                  _integral = _integral + _error*_period;
//                                  _pid= _error*kp + _integral*ki +_dterror*kd;
//                                  _lasterror = _error;
//                                  p = servo_center - _pid;
          }
          else
          {
            p = p_last;
          }

//       Serial.println((i+j)/2); // check car center
//       Serial.print(" ");

          
          if (p > servo_right )
          {
            p = servo_right;
          }
          if (p < servo_left)
          {
            p = servo_left;
          }
          
          //motor control
          if (abs(p-p_last) <= 150)
          {
            motor_speed = turn_speed;
          }
          else if (abs(p-p_last) <=100)
          {
            motor_speed = (uint8_t)((turn_speed + max_speed)/2);
          }
          else
          {
            motor_speed = max_speed;
          }
          //Serial.println(p);
          //Serial.print(p);
          //Serial.print(" ");
          //Serial.println(motor_speed);
          p_last = p;
          myservo.writeMicroseconds(p); // send PWM signals to servo
          
//          downtime = micros();
//          _period = downtime - uptime;
          //Serial.println(_period);
          break;
        }
      }
      break;
    }


  }
  //  pc.printf("servo_end\n");

}


//Main Function
void loop()
{
  if (digitalRead(mode) == 1) //read RC Control status from Arduino Nano
  {
    //Autonomous Mode
    uptime = micros(); 
    getCamera( );
    Servo_out();
    analogWrite(pmos, motor_speed);
    digitalWrite(nmos, LOW);
    downtime = micros();
    _period = downtime - uptime;// calculate total loop time 
    //Serial.println(_period);
  }

  else
  {
    // RC Mode
    while (digitalRead(mode) == 0  )
    {
      //myservo.writeMicroseconds(servo_center);
      motor_in = pulseIn(motor_switch, HIGH); //RC controller Ch2
      //Serial.println(motor_in);
      if (motor_in>1030 && motor_in<1450)
      {
        //Serial.println(map(motor_in,1030,1450,20,80));
        analogWrite(pmos, map(motor_in,1060,1430,20,100));
        digitalWrite(nmos,LOW);
      }
      else
      {
        digitalWrite(pmos, HIGH);
        digitalWrite(nmos, HIGH);
      }
      servo_in = pulseIn(servo_switch, HIGH); //RC controller Ch1
      //Serial.println(servo_in);
      if (servo_in > 1500 && servo_in < 1950)
      {
        //Serial.println(servo_in);//map(val,1000,1570,80,255));
        myservo.writeMicroseconds( map(servo_in, 1500, 1850, servo_left, servo_right));
      }
    }
  }
}
