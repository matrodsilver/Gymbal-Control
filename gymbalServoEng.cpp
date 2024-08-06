#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>


///////////
// Servo //
///////////

Servo myservos[2];
byte motorsPins[2] = {11, 10};

byte motorsAmmount = 2; // motors amount

/////////
// PID //
/////////

// angles and constants vars. //

float proportional;            // system error
const float Kp = 1;

float integral[2] = {0, 0};    // system correction accumulation
const float Ki = 0.01;

float derivative;                // system variation
const float Kd = 0;//135;


// vars. to store calculus values //

// PID's Proportional //
float position[2]; // {X, Y}    // var. to verify error

// PID's Integral //
// (same as Proportional)

// PID's Derivative //
// (same as Proportional)
float lastPosition[2]; // to verify position variation
float time;            // to verify current time
float lastTime[2];   // to verify time variation

// PID total //
float total; // final angle output


// minimum and maximum angle and distances //
int angles[] = {-45, 45, 135}; // {minimum angle (difference to default), maximum angle (difference to default), default angle}
int distances[] = {-135, 135}; // {minimum distance (difference to point 0), maximum distance (difference to point 0)}

int angMin = angles[0];     // minimum angle
int angMax = angles[1];     // maximum angle
int angDefault = angles[2]; // default angle
int distMin = distances[0]; // minimum distance
int distMax = distances[1]; // maximum distance


// function prototypes //
void Update(int currentPos, byte index);
void Proportion(int pos);
void Integration(int pos, byte index);
void Derivation(int pos, byte index);


// simulate ball position (for now) //
int X = 20;
int Y = -20;
  
void setup()
{
  Serial.begin(9600);


  // Servo setup //
  for (int i = 0; i < motorsAmmount; i++)
  {
    myservos[i].attach(motorsPins[i]);
  
    // servo initial test
    myservos[i].write(angDefault+angMin);
    delay(500);
    myservos[i].write(angDefault+angMax);
    delay(500);
    myservos[i].write(angDefault);
    delay(500);
  }
}

void loop()
{
  position[0] = X; // update ball X position
  position[1] = Y; // update ball Y position


  if (!(position[0] == 0 && position[1] == 0)) // if the ball is not on the center:
  {
    for (int i = 0; i < 2; i++)
    {
      Update(position[i], i); // calculate PID of given axis
    }
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      integral[i] = 0; // reset integral var. value
    }
  }
}


void Update(int currentPos, byte index) // update servos
{
  Proportion(currentPos);
  Integration(currentPos, index);
  Derivation(currentPos, index);

  total = proportional + integral[index] + derivative;

  total =  total + angDefault; // calculated angle + default angle

  if (total > angDefault + angMax)
  {
    total = angDefault + angMax;
  }
  else if (total < angDefault + angMin)
  {
    total = angDefault + angMin;
  }

  Serial.println("Output axis [" + String(index) + "]: " + String(total) + "°");
  myservos[index].write(total);
}

void Proportion(int pos) // verify system error
{
  proportional = Kp * map(pos, distMin, distMax, angMin, angMax);

  Serial.println("P: " + String(proportional));
}

void Integration(int pos, byte index) // verify error in relation to time
{
  integral[index] += Ki * map(pos, distMin, distMax, angMin, angMax);

  Serial.println("I: " + String(integral[index]));
}

void Derivation(int pos, byte index) // verify system variation
{
  time = millis();

  derivative = Kd * (pos - lastPosition[index]) / (time - lastTime[index]);

  Serial.println("D: " + String(derivative));

  lastTime[index] = time;

  if (position[index] != lastPosition[index])
  {
    Serial.println("Distance: " + String(position[index]));
  }

  lastPosition[index] = position[index];
}