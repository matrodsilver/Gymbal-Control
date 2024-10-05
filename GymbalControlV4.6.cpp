#include <Arduino.h>

#include <Wire.h>
#include <stdint.h>
#include "TouchScreen.h"
#include <Servo.h>

///////////////////
//* Touch Screen //
///////////////////
#define YP 26 // must be an analog pin, use "An" notation!  //' Fio branco
#define XM 27 // must be an analog pin, use "An" notation!  //! Fio Azul
#define YM 14                                               //# Fio Marrom
#define XP 12                                               //? Fio Vermelho

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 246); // susbstituir 246 pela resistência do touch

TSPoint p; // a point object that holds x and y coordinates

int min_max_touch[2][2] = {
  {920, -3072}, // valores mínimos e máximos de leitura do eixo X do touch
  {550, -2230}  // valores mínimos e máximos de leitura do eixo Y do touch
};

int touch_default[2] = {-3072, 1023}; // valores do padrão touch, quando não há nada o pressionando

/////////////////////
//* Servo (output) //
/////////////////////
byte qnt_servos = 2,           // quantidade de servos 
     servo_pins[2] = {25, 33}; // pinos dos servos

Servo servo[2];

//////////
//* PID //
//////////
int periodo = 50; // periodo pra cada iteração

float proporcional,         // erro no sistema
      integral[2] = {0, 0}, // acumulo da correção no sistema (de cada eixo)
      derivada;             // variação no sistema
      
float Kp[2] = {1.1, 1.1}, // constantes P
      Ki[2] = {0, 0}, // constantes I
      Kd[2] = {700, 700}; // constantes D

//' Vars. para guardar valores dos cálculos //
//# Para P do PID //
float posicao[2]; // X e Y no touch

//! Para I do PID //
// (mesma variável de P)

//* Para D do PID //
// (mesma variável de P) +
float ultima_posicao[2] = {0, 0},  // verificar variação da posição (de cada eixo)
      tempo_atual,        // guardar tempo atual
      ultimo_tempo[2] = {0, 0};    // variação de tempo  (de cada eixo)

//? PID total //
float total = 0; // angulo final dos cálculos


//' Constantes do hardware //
int PW_ang_default[2] = {145, 114}, // angulo padrão d cada eixo (mínimo e máximo são em relação ao padrão)
    PW_ang_min[2] = {30, 30},    // angulos máximo de cada eixo (padrão - mínimo) (não deve ser maior que 2400)
    PW_ang_max[2] = {-30, -30};      // angulos mínimo de cada eixo (máximo - padrão) (não deve ser menor que 600 )

//' Prototipos das funções //
void PegarPosicao(), LimparBuffer(), Calibrar();
float mapFloat(float x, float inp_min, float inp_max, float out_min, float out_max);
int PID_Calculations(int pos, byte index);

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Iniciando...");
  delay(500);

  //' Servos setup //
  servo[0].attach(servo_pins[0]);
  servo[1].attach(servo_pins[1]);

  for (int i = 0; i < qnt_servos; i++)
  {
    servo[i].write(PW_ang_default[i] + PW_ang_min[i]);
    delay(1000);

    servo[i].write(PW_ang_default[i] + PW_ang_max[i]);
    delay(1000);

    servo[i].write(PW_ang_default[i]);
    delay(1000);
  }
}

void loop()
{
  Calibrar();

  PegarPosicao();

  if (!(posicao[0] == touch_default[0] && posicao[1] == touch_default[1])) // se a bola está na touch:
  {
    for (int i = 0; i < qnt_servos; i++)
    {
      servo[i].write(PID_Calculations(posicao[i], i)); // calcular PID de cada eixo (0 = X, 1 = Y)
    }
  }

  else
  {
    for (int i = 0; i < qnt_servos; i++)
    {
      integral[i] = 0;
    }
  }

  delay(periodo);
}


void PegarPosicao()
{
  p = ts.getPoint();

  posicao[0] = p.x;  // atualizar posição X da bola
  posicao[1] = p.y;  // atualizar posição Y da bola
}

int PID_Calculations(int posicao_atual, byte index) // atualiza o valor de ´ultimaPosicao´ se necessário
{
  total = mapFloat(posicao_atual, min_max_touch[index][0], min_max_touch[index][1], PW_ang_min[index], PW_ang_max[index]);

  // Proportional
  proporcional = Kp[index] * total;
  
  // Integral
  integral[index] += Ki[index] * total;

  // Derivative
  tempo_atual = millis();

  derivada = Kd[index] * (posicao_atual - ultima_posicao[index]) / (tempo_atual - ultimo_tempo[index]);

  ultimo_tempo[index] = tempo_atual;
  ultima_posicao[index] = posicao_atual;


  // Total
  total = PW_ang_default[index] + proporcional + integral[index] + derivada;
  total = constrain(total, PW_ang_default[index] + PW_ang_max[index], PW_ang_default[index] + PW_ang_min[index]);

  int retorno = total;

  return retorno;
}

void Calibrar()
{
  if (Serial.available() > 0 && Serial.read() == 'c')
  {    
    for (int i = 0; i < qnt_servos; i++)
    {
      servo[i].write(PW_ang_default[i]);
      delay(550);
    }

    LimparBuffer();

    //# Selecionar eixo
    Serial.println("Selecione um eixo: \n1 para eixo X [0] \n2 para eixo Y [1]");

    while (Serial.available() == 0)
      ;

    byte axis = Serial.parseInt();

    if (axis != 1 && axis != 2)
    {
      Serial.println("Operação cancelada");
      return;
    }

    axis--;

    Serial.println("Eixo " + String(axis) + " selecionado");

    Serial.println("Kp <valor> → mudar valor da constante P \nKi <valor> → mudar valor da constante I \nKd <valor> → mudar valor da constante D");
    Serial.println("touch min <valor> → mudar valor de input mínimo do touch \ntouch max <valor> → mudar valor de input máximo do touch");
    Serial.println("delay <valor> → mudar o valor de atualização do sistema");
    Serial.println("ang → mudar os valores de pulso para o ângulo\n");

    LimparBuffer();

    while (Serial.available() == 0)
      ;

    //# selecionar variável
    while (Serial.available() > 0)
    {
      String inputed = Serial.readStringUntil('\n');
      // input.trim(); // Remove any leading or trailing whitespace

      //# constantes do PID
      if (inputed.startsWith("Kp"))
      {
        Kp[axis] = inputed.substring(3).toFloat();
      }
      else if (inputed.startsWith("Ki"))
      {
        Ki[axis] = inputed.substring(3).toFloat();
      }
      else if (inputed.startsWith("Kd"))
      {
        Kd[axis] = inputed.substring(3).toFloat();
      }

      //# valores do touch
      else if (inputed.startsWith("touch min"))
      {
        min_max_touch[axis][0] = inputed.substring(10).toInt();
      }
      else if (inputed.startsWith("touch max"))
      {
        min_max_touch[axis][1] = inputed.substring(10).toInt();
      }

      //# periodo
      else if (inputed.startsWith("delay"))
      {
        periodo = inputed.substring(6).toFloat();
      }

      //# ângulos
      else if (inputed.startsWith("ang"))
      {
        LimparBuffer();

        //' angulo padrão
        Serial.println("Digite o valor do pulso padrão");
        while (Serial.available() == 0)
          ;
        
        PW_ang_default[axis] = Serial.parseInt();


        LimparBuffer();

        //' angulo mínimo
        Serial.println("Digite o valor mínimo a adicionar ao pulso");
        while (Serial.available() == 0)
          ;
        
        PW_ang_min[axis] = Serial.parseInt();


        LimparBuffer();


        //' angulo máximo
        Serial.println("Digite o valor máximo a adicionar ao pulso");
        while (Serial.available() == 0)
          ;
        
        PW_ang_max[axis] = Serial.parseInt();
      }

      else
      {
        Serial.println("Operação encerrada");
      }


      Serial.println("Kp: " + String(Kp[axis]) + " no eixo " + String(axis));
      Serial.println("Ki: " + String(Ki[axis]) + " no eixo " + String(axis));
      Serial.println("Kd: " + String(Kd[axis]) + " no eixo " + String(axis));
      Serial.println("Touch min: " + String(min_max_touch[axis][0]) + " no eixo " + String(axis));
      Serial.println("Touch max: " + String(min_max_touch[axis][1]) + " no eixo " + String(axis));
      Serial.println("delay: " + String(periodo) + "ms no sistema");
      Serial.println("Angulos: \nmínimo: " + String(PW_ang_min[axis]) + "°, padrão: " + String(PW_ang_default[axis]) + "°, máximo: " + String(PW_ang_max[axis]) + "° no eixo " + String(axis));
    }
  }
}

void LimparBuffer()
{
  while (Serial.available() > 0)
  {
    Serial.read();
  }
}

float mapFloat(float x, float inp_min, float inp_max, float out_min, float out_max)
{
    const float range_inp = inp_max - inp_min;
    if(range_inp == 0)
    {
        Serial.println("mapFloat(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }

    const float range_out = out_max - out_min;
    const float delta = x - inp_min;
    return (delta * range_out) / range_inp + out_min;
}
