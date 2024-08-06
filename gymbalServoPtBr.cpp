#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>


////////////////////
// Servo (output) //
////////////////////

Servo myservos[2];
byte motorsPins[2] = {11, 10};

byte qntMotores = 2; // quantidade de motores

/////////
// PID //
/////////

// Vars. para guardar resultados //

float proporcional;            // Erro do sistema
const float Kp = 1;

float integral[2] = {0, 0};    // Acumulo da correção do sistema
const float Ki = 0.01;

float derivada;                // Variação do sistema
const float Kd = 0;//135;

// Vars. para guardar valores dos cálculos //

// Para P do PID //
float posicao[2]; // {Y, Y}    // para verificar erro

// Para I do PID //
// (mesma variável de P)

// Para D do PID //
// (mesma variável de P)
float ultimaPosicao[2]; // para verficar variação da posição
float tempo;                   // para verficar tempo atual
float ultimoTempo[2];   // para verificar variação de tempo

// PID total //
float total; // graus a mudar no final dos cálculos


// definir angulos e distâncias //
int angulos[] = {-45, 45, 135}; // {ângulo mínimo (valor em relação ao padrão), ângulo máximo (valor em relação ao padrão), ângulo padrão}
int distancias[] = {-135, 135};

int angMin = angulos[0];     // ângulo mínimo
int angMax = angulos[1];     // ângulo máximo
int distMin = distancias[0]; // distância mínima
int distMax = distancias[1]; // distância máxima
int angDefault = angulos[2]; // ângulo padrão


// prototipos das funções //
void atualizar(int posicao, byte index);
void Proporcao(int pos);
void Integracao(int pos, int index);
void Derivacao(int pos, byte index);


// simular posição da bola //
int X = 20;
int Y = -20;
  
void setup()
{
  Serial.begin(9600);


  // Servo setup //
  for (int i = 0; i < qntMotores; i++)
  {
    myservos[i].attach(motorsPins[i]);
  
    // teste servo
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
  Proporcao(posicaoAtual);
  Integracao(posicaoAtual, index);
  Derivacao(posicaoAtual, index);

  total = proporcional + integral[index] + derivada;

  total =  total + angDefault; // default point

  if (total > angDefault + angMax)
  {
    total = angDefault + angMax;
  }
  else if (total < angDefault + angMin)
  {
    total = angDefault + angMin;
  }

  Serial.println("Output axis:[" + String(index) + "]: " + String(total) + "°");
  myservos[index].write(total); // mover o servo de acordo com os valore encontrados
  //myservo[i].write(default-total); // mover o servo de acordo com os valore encontrados
}

void Proporcao(int pos) // verificar o erro
{
  proporcional = Kp/*ConstanteP MoverServo*/ * map(pos, distMin, distMax, angMin, angMax); // 0(mm) é o meio da mesa, 135 é a distância máxima em relação ao meio

  Serial.println("P: " + String(proporcional));
}

void Integracao(int pos, int index) // verificar o erro em relação ao tempo
{
  integral[index] += Ki/*ConstanteI MoverServo*/ * map(pos, distMin, distMax, angMin, angMax);

  Serial.println("I: " + String(integral[index]));
}

void Derivacao(int pos, byte index) // verificar a variação
{
  tempo = millis();

  derivada = Kd/*ConstanteD MoverServo*/ * /*velocidade*/(pos - ultimaPosicao[index]) / (tempo - ultimoTempo[index]);

  Serial.println("D: " + String(derivada));

  ultimoTempo[index] = tempo;

  if (posicao[index] != ultimaPosicao[index])
  {
    Serial.println("Distância: " + String(posicao[index]));
  }

  ultimaPosicao[index] = posicao[index];
}