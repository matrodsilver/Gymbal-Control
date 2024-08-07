#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>


/////////////////////////////////////
// Servo (output) e sensor (input) //
/////////////////////////////////////

Servo myservo;


/////////
// PID //
/////////

// Vars. para guardar resultados //

float proporcional = 0; // Erro do sistema
const float Kp = 1;

float integral = 0;    // Acumulo da correção do sistema
const float Ki = 0;//.01;

float derivada = 0;    // Variação do sistema
const float Kd = 0;//135;


// Vars. para guardar valores dos cálculos //

// Para P do PID //
float distancia;         // para verificar erro

// Para I do PID //
// (mesma variável de P)

// Para D do PID //
// (mesma variável de P)
float ultimaDistancia;   // para verficar variação de distância
float tempo;
float ultimoTempo = 0;   // para verificar variação de tempo

// PID total //
float total;



// prototipos das funções //
void atualizar();
void verificarProporcao();
void verificarIntegracao();
void verificarDerivacao();

void planos();


///////////////////////////////////
// Vars. para cálculo nos planos //
///////////////////////////////////

// declaração vars. plano fixo
float X4 = 0;      // = Xc4                    //*
float Y4 = 1;      // = Yc4                    //*

float Xb = 0; // = Xcb
float Yb = 0; // = Ycb

float HIP4;                                    //*

float sinAngle4;
float angleAxis4;

// declaração vars. bola (móvel) //

float HIPb;                                    //!

float sinAngleHIPb;                            //!

float angleHIPb;                               //!


float Xb4;                                     //!
float Yb4;                                     //!

void setup()
{
  Serial.begin(9600);

  HIP4 = sqrt(pow(X4, 2) + pow(Y4, 2));
  // Condicinoais de posição no plano para plano fixo
  if (X4 >= 0 && Y4 >= 0)
  {
    Serial.println("+, +");
    sinAngle4 = Y4 / HIP4;
    angleAxis4 = asin(sinAngle4) * 180 / PI;             //*
  }
  else if (X4 < 0 && Y4 >= 0)
  {
    Serial.println("-, +");
    sinAngle4 = X4 / HIP4;
    angleAxis4 = 90 + (asin(sinAngle4) * 180 / PI)*-1;   //*
  }
  else if (X4 < 0 && Y4 < 0)
  {
    Serial.println("-, -");
    sinAngle4 = Y4 / HIP4;
    angleAxis4 = 180 + (asin(sinAngle4) * 180 / PI)*-1;  //*
  }
  else if (X4 >= 0 && Y4 < 0)
  {
    Serial.println("+, -");
    sinAngle4 = X4 / HIP4;
    angleAxis4 = 270 + asin(sinAngle4) * 180 / PI;       //*
  }

  Serial.println("HIP4 = "+String(HIP4));
  Serial.println("sinAngle4 = "+String(sinAngle4));
  Serial.println("angleAxis4 = "+String(angleAxis4));


  // Servo setup
  myservo.attach(11);
  // teste servo
  myservo.write(180);
  delay(500);
  myservo.write(0);
  delay(500);
  myservo.write(90);
  delay(500);
}

void loop() {
  planos();
  delay(500);

  distancia = Yb4;

  if (!(distancia == 135 && distancia == ultimaDistancia)) // se a bola não está em inércia no meio da mesa:
  {
    atualizar();
  }
  else
  {
    integral = 0;
  }

  delay(25);


  delay(500);
}


void planos()
{
  //Xb = 1; // = Xcb
  //Yb = -sqrt(3); // = Ycb
  Serial.println("Yb = "+String(Yb));


  HIPb = sqrt(pow(Xb, 2) + pow(Yb, 2));
  Serial.println("HIPb = "+String(HIPb));

  // condicionais da bola móvel
  if (HIPb == 0)
  {
    Serial.println("~ 0 ~");
    sinAngleHIPb = 0;
    angleHIPb = 0;  //*
  }
  else if (Xb >= 0 && Yb >= 0)
  {
    Serial.println("+, +");
    sinAngleHIPb = Yb / HIPb;
    angleHIPb = asin(sinAngleHIPb) * 180 / PI;            //*
  }
  else if (Xb < 0 && Yb >= 0)
  {
    Serial.println("-, +");
    sinAngleHIPb = Xb / HIPb;
    angleHIPb = 90 + (asin(sinAngleHIPb) * 180 / PI)*-1;  //*
  }
  else if (Xb < 0 && Yb < 0)
  {
    Serial.println("-, -");
    sinAngleHIPb = Yb / HIPb;
    angleHIPb = 180 + (asin(sinAngleHIPb) * 180 / PI)*-1; //*
  }
  else if (Xb >= 0 && Yb < 0)
  {
    Serial.println("+, -");
    sinAngleHIPb = Xb / HIPb;
    angleHIPb = 270 + asin(sinAngleHIPb) * 180 / PI;      //*
  }

  Serial.println("sinAngleHIPb = "+String(sinAngleHIPb));
  Serial.println("angleHIPb = "+String(angleHIPb));

  Serial.println(String(angleHIPb) + " - " + String(angleAxis4) + " = " + String(angleHIPb - angleAxis4));
  Serial.println("HIPb: " + String(HIPb) + " * " + String(cos((angleHIPb - angleAxis4) *PI/180)));

  Yb4 = HIPb * cos((angleHIPb - angleAxis4) *PI/180/*para rad (porque ´cos()´ funciona com rad*/);
  Serial.println("Yb4 = " + String(Yb4));
}

void atualizar() // atualiza o valor de ´ultimaDistancia´ se necessário
{
  verificarProporcao();
  verificarIntegracao();
  verificarDerivacao();

  total = proporcional + integral + derivada;

  total = 90 + total; // default point

  if (total > 180)
  {
    total = 180;
  }
  else if (total < 0)
  {
    total = 0;
  }

  Serial.println("Output: " + String(total) + "°");
  myservo.write(total); // mover o servo de acordo com os valore encontrados
}

void verificarProporcao() // verificar o erro
{
  proporcional = Kp/*ConstanteP MoverServo*/ * map((distancia), -135, 135, -90, 90); // 100(mm) é o meio da mesa

  Serial.println("P: " + String(proporcional));
}

void verificarIntegracao() // verificar o erro
{
  integral += Ki/*ConstanteI MoverServo*/ * map((distancia), -135, 135, -90, 90);

  Serial.println("I: " + String(integral));
}

void verificarDerivacao() // verificar a variação
{
  tempo = millis();

  derivada = Kd/*ConstanteD MoverServo*/ * /*velocidade*/(distancia - ultimaDistancia) / (tempo - ultimoTempo);

  Serial.println("D: " + String(derivada));

  ultimoTempo = tempo;

  if (distancia != ultimaDistancia)
  {
    Serial.println("Distância: " + String(distancia));
  }

  ultimaDistancia = distancia;
}


/*
  - Ver como se comporta com 0s
*/