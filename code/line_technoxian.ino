#include "BluetoothSerial.h"

BluetoothSerial SerialBT;


bool lineLost = false;


#define AIN1 18  
#define AIN2 19
#define PWMA 23
#define BIN1 16  
#define BIN2 17
#define PWMB 5
#define STBY 4

#define S0 25
#define S1 26
#define S2 27

#define Z1 34
#define Z2 35

#define switch_pin 32

#define ir_pin 13

int sensor[16];  // saves raw readings from sensors
int sensorMin[16]; // stores min values of all the sensors
int sensorMax[16]; // stores max values of all the sensors
int sensorDigital[16]; // stores digital values of all the sensors/

int startIdx = 0;
int endIdx   = 16;

int base_speed = 150;
int const_speed = 150;
int min_speed = 60;

int check_rightspeed = 0;
int check_leftspeed = 0;

float error = 0;
float previous_error = 0;

float kp =50;
float kd =70;
float ki =0;
float ks =30;

float integral = 0;

bool inverted = false;
bool onloop = false;

int invertedCounter = 0; 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("LineFollower");

  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(STBY,OUTPUT);

  pinMode(S0,OUTPUT);
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);

  pinMode(Z1,INPUT);
  pinMode(Z2,INPUT);

  pinMode(switch_pin,INPUT_PULLUP);

  pinMode(ir_pin,INPUT_PULLUP);


digitalWrite(STBY,HIGH);


calibrateSensors();
generateDigital();

  // Wait for switch press
  while (digitalRead(switch_pin) == HIGH);

  delay(20);                    // debounce

  // Wait for release
  while (digitalRead(switch_pin) == LOW);


}

void loop() {

  if (digitalRead(ir_pin) != LOW)
  {
  follow_line();
  }

  else if (digitalRead(ir_pin) != HIGH)
  {
    stop_motors();
    delay(10);
  }

}


void move_forward(int right_speed, int left_speed)
{


  if (right_speed > 0 && left_speed > 0)
  {
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  analogWrite(PWMB,left_speed);
  analogWrite(PWMA,right_speed);
  }
  else if (right_speed < 0 && left_speed > 0 )
  {
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  analogWrite(PWMB,left_speed);
  analogWrite(PWMA,-right_speed);
  }
    else if (right_speed > 0 && left_speed < 0)
  {
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
  analogWrite(PWMB,-left_speed);
  analogWrite(PWMA,right_speed);
  }
  else if (right_speed < 0 && left_speed < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(PWMA, -right_speed);
    analogWrite(PWMB, -left_speed);
  }
}

void stop_motors()
{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);

    analogWrite(PWMA, 255);
    analogWrite(PWMB, 255);
}



void follow_line()
{
  get_value();
  generateDigital();

    error = find_error();

    float derivative = error - previous_error;
    integral += error;
    float correction = kp * error + kd * derivative + ki * integral;

    float absError = fabs(error);

// if (abs(error) > 5)
//     base_speed = 0;
// else if (abs(error) > 4)
//     base_speed = 30;
// else if (abs(error) > 3)
//     base_speed = 50;
// else if (abs(error) > 2)
//     base_speed = 120;
// else 
// {
//   base_speed = 170;
// }

    float e = abs(error);
    base_speed = min_speed +(const_speed - min_speed) * exp(-0.15 * e);

    int rightmotorspeed = base_speed + correction;
    int leftmotorspeed  = base_speed - correction;

    if (abs(error) <= 1 ){
  check_rightspeed = 0;
  check_leftspeed = 0;
}
else{
  check_rightspeed = (rightmotorspeed<255)?0:rightmotorspeed-255;
  check_leftspeed = (leftmotorspeed<255)?0:leftmotorspeed-255;
}

    // if (absError >= 6  || lineLost == true)
    // {
    //   rightmotorspeed = constrain(rightmotorspeed + check_leftspeed, -50, 50);
    //   leftmotorspeed  = constrain(leftmotorspeed + check_rightspeed, -50, 50);
    // }                 // sharp turn — hard floor
    // else if (absError >= 3)
    // {
    // rightmotorspeed = constrain(rightmotorspeed, -50, 120);
    //   leftmotorspeed  = constrain(leftmotorspeed, -50, 120); // ramp 3→6 maps to 170→100
    // }
    // else
    // {
    //   rightmotorspeed = constrain(rightmotorspeed, 0, 255);
    //   leftmotorspeed  = constrain(leftmotorspeed, 0, 255);
    // }                 // straight / gentle curve



      rightmotorspeed = constrain(rightmotorspeed + check_leftspeed, -255, 255);
      leftmotorspeed  = constrain(leftmotorspeed + check_rightspeed, -255, 255);
    move_forward(rightmotorspeed, leftmotorspeed);
    previous_error = error;
}


float find_error()
{
    float weights[16] = {-8,-6,-4.5,-3.5,-3,-2,-1.5,0,0,1.5,2,3,3.5,4.5,6,8};

    // onloop = check_in_loop();
    // int startIdx = onloop ? 2  : 0;
    // int endIdx   = onloop ? 13 : 15;

    float weightedSum = 0;
    float total = 0;

    // if (lineLost=true)
    // {
    //   startIdx = lineLost ? 2  : 0;
    //   endIdx   = lineLost ? 13 : 15;
    // }

    for (int i = 0; i < 16; i++)
    {
        float range = sensorMax[i] - sensorMin[i];
        float norm = (range > 0) ? (float)(sensor[i] - sensorMin[i]) / range : 0;
        norm = constrain(norm, 0.0, 1.0);
        // norm is already 1.0 = on line, 0.0 = off line, per your calibration polarity

        weightedSum += norm * weights[i];
        total += norm;
    }

    if (total < 2) // nothing sees the line
    {
      lineLost = true;

      return (previous_error > 0) ? 12 : -12;
    }

    lineLost = false;
    return weightedSum / total;
}



void selectChannel(int ch)
{
  digitalWrite(S0, ch & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
}

void get_value()
{


  for(int ch = 0; ch < 8; ch++)
  {
    selectChannel(ch);
    delayMicroseconds(50);

    sensor[ch] = analogRead(Z1);
    sensor[15-ch] = analogRead(Z2);
    
  }

}


void calibrateSensors()
{
    for(int i = 0; i < 16; i++)
    {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }

    // Rotate left
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    analogWrite(PWMA, 40);
    analogWrite(PWMB, 40);

    unsigned long start = millis();

    while(millis() - start < 3500)
    {
        get_value();

        for(int i = 0; i < 16; i++)
        {
            sensorMin[i] = min(sensorMin[i], sensor[i]);
            sensorMax[i] = max(sensorMax[i], sensor[i]);
        }
    }

    // Rotate right
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    start = millis();

    while(millis() - start < 3500)
    {
        get_value();

        for(int i = 0; i < 16; i++)
        {
            sensorMin[i] = min(sensorMin[i], sensor[i]);
            sensorMax[i] = max(sensorMax[i], sensor[i]);
        }
    }

    stop_motors();
}


void generateDigital()
{
    for(int i = 0; i < 16; i++)
    {
        int threshold = (sensorMin[i] + sensorMax[i]) / 2;

        sensorDigital[i] = (sensor[i] > threshold);
        sensorDigital[i] = !sensorDigital[i];     // on line = 0  && off line = 1
    }
}

void BT_connect()
{

    if (SerialBT.available())
    {
        String cmd = SerialBT.readStringUntil('\n');

        if(cmd.startsWith("KP="))
        {
            kp = cmd.substring(3).toFloat();
            SerialBT.print("KP = ");
            SerialBT.println(kp);
        }

        else if(cmd.startsWith("KD="))
        {
            kd = cmd.substring(3).toFloat();
            SerialBT.print("KD = ");
            SerialBT.println(kd);
        }

        else if(cmd.startsWith("KI="))
        {
            ki = cmd.substring(3).toFloat();
            SerialBT.print("KI = ");
            SerialBT.println(ki);
        }

        else if(cmd.startsWith("SPEED="))
        {
            base_speed = cmd.substring(6).toInt();
            SerialBT.print("SPEED = ");
            SerialBT.println(base_speed);
        }
    }
}

bool sharpRightTurn()
{
    // line detected on far-left sensors, nothing center/right
    return (sensorDigital[0] || sensorDigital[1] || sensorDigital[2])
        && !(sensorDigital[7] || sensorDigital[8])
        && !(sensorDigital[13] || sensorDigital[14] || sensorDigital[15]);
}

bool sharpLeftTurn()
{
    return (sensorDigital[13] || sensorDigital[14] || sensorDigital[15])
        && !(sensorDigital[7] || sensorDigital[8])
        && !(sensorDigital[0] || sensorDigital[1] || sensorDigital[2]);
}


// bool check_in_loop()
// {
//   if ((digitalRead(sensorDigital[4]) == digitalRead(sensorDigital[11]) == digitalRead(sensorDigital[8]) ) && (digitalRead(sensorDigital[11]) != digitalRead(sensorDigital[7])))
//   {
//     onloop = true;
//     return onloop;
//   }