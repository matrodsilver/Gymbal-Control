#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


////////////////////
// Servo (output) //
////////////////////

byte qntMotores = 2; // quantidade de motores

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)


/////////
// PID //
/////////

// Vars. para guardar resultados //

float proportional;            // Erro do sistema
const float Kp = 1;

float integral[2] = {0, 0};    // Acumulo da correção do sistema
const float Ki = 0.01;

float derivative;                // Variação do sistema
const float Kd = 0;//135;

// Vars. para guardar valores dos cálculos //

// Para P do PID //
float posicao[2]; // {X, Y}    // para verificar erro

// Para I do PID //
// (mesma variável de P)

// Para D do PID //
// (mesma variável de P)
float lastPosition[2]; // para verficar variação da posição
float tempo;                   // para verficar tempo atual
float lastTime[2];   // para verificar variação de tempo

// PID total //
float total; // graus a mudar no final dos cálculos


// definir angulos e distâncias //
int angulos[] = {-45, 45, 135}; // {ângulo mínimo (valor em relação ao padrão), ângulo máximo (valor em relação ao padrão), ângulo padrão}
int distances[] = {-135, 135};

int angMin = angulos[0];     // ângulo mínimo
int angMax = angulos[1];     // ângulo máximo
int angDefault = angulos[2]; // ângulo padrão
int distMin = distances[0]; // distância mínima
int distMax = distances[1]; // distância máxima


// prototipos das funções //
void atualizar(int posicao, byte index);
void Proportion(int pos);
void Integration(int pos, int index);
void Derivation(int pos, byte index);


// simular posição da bola //
int X = 20;
int Y = -20;
  
void setup()
{
  Serial.begin(9600);


  // Servo setup //
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  for (int i = 0; i < qntMotores; i++)
  {
    pwm.setPWM(i, 0, angleToPulse(angDefault+angMin) ); // angleToPulse(Numero do Motor, ponto inicial, ângulo desejado)
    delay(1000);
    pwm.setPWM(i, 0, angleToPulse(angDefault+angMax) );
    delay(1000);
    pwm.setPWM(i, 0, angleToPulse(angDefault) );
    delay(1000);
  }
}

void loop()
{
  posicao[0] = X; // atualizar posição X da bola
  posicao[1] = Y; // atualizar posição Y da bola

  if (!(posicao[0] == 0 && posicao[1] == 0)) // se a bola não está em inércia no meio da mesa:
  {
    for (int i = 0; i < 2; i++)
    {
      atualizar(posicao[i], i); // calcular PID no eixo do index
    }
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      integral[i] = 0;
    }
  }
}


void atualizar(int posicaoAtual, byte index) // atualiza o valor de ´ultimaPosicao´ se necessário
{
  Proportion(posicaoAtual);
  Integration(posicaoAtual, index);
  Derivation(posicaoAtual, index);

  total = proportional + integral[index] + derivative;

  total =  total + angDefault;

  if (total > angDefault + angMax)
  {
    total = angDefault + angMax;
  }
  else if (total < angDefault + angMin)
  {
    total = angDefault + angMin;
  }

  Serial.println("Output[" + String(index) + "]: " + String(total) + "°");

  pwm.setPWM(index, 0, angleToPulse(total) ); // mover o servo de acordo com os valore encontrados
}

void Proportion(int pos) // verificar o erro
{
  proportional = Kp/*ConstanteP MoverServo*/ * map(pos, distMin, distMax, angMin, angMax); // 0(mm) é o meio da mesa, 135 é a distância máxima em relação ao meio

  Serial.println("P: " + String(proportional));
}

void Integration(int pos, int index) // verificar o erro em relação ao tempo
{
  integral[index] += Ki/*ConstanteI MoverServo*/ * map(pos, distMin, distMax, angMin, angMax);

  Serial.println("I: " + String(integral[index]));
}

void Derivation(int pos, byte index) // verificar a variação
{
  tempo = millis();

  derivative = Kd/*ConstanteD MoverServo*/ * /*velocidade*/(pos - lastPosition[index]) / (tempo - lastTime[index]);

  Serial.println("D: " + String(derivative));

  lastTime[index] = tempo;

  if (posicao[index] != lastPosition[index])
  {
    Serial.println("Distância: " + String(posicao[index]));
  }

  lastPosition[index] = posicao[index];
}