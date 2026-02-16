// Code using Robust Arduino Serial Protocol: https://github.com/araffin/arduino-robust-serial
#include <Arduino.h>

#include "order.h"
#include "slave.h"
#include "parameters.h"
#include <Servo.h>
#include <Metro.h>

#define LEFT 5
#define RIGHT 6

int coder[2] = {
  0,0};
int lastSpeed[2] = {
  0,0};

Metro measureDistance = Metro(50);
Metro sweepServo = Metro(20);

Servo frontServo, backServo;    // create servo objects
#define  frontPin 10
// #define  backPin 8
int pos = 90;    // initial position of the servos
int sweepFlag = 1;

int URPWM = 8; // PWM Output 0－25000US，Every 50US represent 1cm
int URTRIG = 9; // PWM trigger pin
uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};    // distance measure command

// Variables pour l'échantillonnage ADC
#define ADC_BUFFER_SIZE 50 // Taille du buffer pour stocker les échantillons
#define ADC_SAMPLE_INTERVAL 20 // Intervalle d'échantillonnage en microsecondes

volatile uint16_t adc_buffer[ADC_BUFFER_SIZE]; // Buffer circulaire pour les échantillons
volatile int adc_buffer_index = 0; // Index actuel dans le buffer
volatile uint32_t adc_sum = 0; // Somme pour calculer la moyenne
volatile uint16_t adc_avg = 0; // Dernière moyenne calculée
volatile unsigned long last_adc_sample_time = 0; // Temps du dernier échantillon

bool is_connected = false; ///< True if the connection with the master is available
int8_t motor_speed_front_right_av = 0;
int8_t motor_speed_front_left_av = 0;
int8_t motor_speed_rear_right_av = 0;
int8_t motor_speed_rear_left_av = 0;
int8_t motor_speed_rear_right = 0;

// Fonction pour initialiser l'échantillonnage ADC
void initADCSampling() {
  // Initialisation du buffer
  for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
    adc_buffer[i] = 0;
  }
}

// Fonction pour échantillonner l'ADC et mettre à jour la moyenne glissante
void sampleADC2() {
  // Lire la valeur actuelle
  uint16_t new_sample = analogRead(2);
  
  // Soustraire l'ancienne valeur et ajouter la nouvelle à la somme
  adc_sum = adc_sum - adc_buffer[adc_buffer_index] + new_sample;
  
  // Stocker la nouvelle valeur dans le buffer
  adc_buffer[adc_buffer_index] = new_sample;
  
  // Incrémenter l'index et revenir à zéro si nécessaire
  adc_buffer_index = (adc_buffer_index + 1) % ADC_BUFFER_SIZE;
  
  // Calculer la nouvelle moyenne
  adc_avg = adc_sum / ADC_BUFFER_SIZE;
}

void setup()
{
  frontServo.attach(frontPin);
  frontServo.write(pos);
//  backServo.attach(backPin);
  backServo.write(pos);
  // Init Serial
  Serial.begin(SERIAL_BAUD);
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3
  
  SensorSetup();
  initADCSampling(); // Initialiser l'échantillonnage ADC

  // Init Motor
  pinMode(SPEED_FRONT_LEFT, OUTPUT);
  pinMode(SPEED_FRONT_RIGHT, OUTPUT);
  pinMode(DIRECTION_FRONT_LEFT, OUTPUT);
  pinMode(DIRECTION_FRONT_RIGHT, OUTPUT);
  pinMode(SPEED_REAR_LEFT, OUTPUT);
  pinMode(SPEED_REAR_RIGHT, OUTPUT);
  pinMode(DIRECTION_REAR_LEFT, OUTPUT);
  pinMode(DIRECTION_REAR_RIGHT, OUTPUT);
  // Stop the car
  stop();
  

  // Wait until the arduino is connected to master
  while(!is_connected)
  {
    write_order(HELLO);
    wait_for_bytes(1, 1000);
    get_messages_from_serial();
  }

}

void SensorSetup(){
  pinMode(URTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG,HIGH);                  // Set to HIGH
  pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command
  for(int i=0;i<4;i++){
      Serial.write(EnPwmCmd[i]);
   }
}

int MeasureDistance(){        // a low pull on pin COMP/TRIG  triggering a sensor reading
    digitalWrite(URTRIG, LOW);
    digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses
    unsigned long distance=pulseIn(URPWM,LOW);
    if(distance==50000){              // the reading is invalid.
      Serial.print("Invalid");
    }else{
      distance=distance/50;           // every 50us low level stands for 1cm
    }
    return distance;
}

int16_t MeasureCurrent(){
  return adc_avg; // Retourner la dernière moyenne calculé
}

void loop()
{
  get_messages_from_serial();
  update_motors_orders();
  
  // Échantillonner l'ADC à haute fréquence
  if (micros() - last_adc_sample_time >= ADC_SAMPLE_INTERVAL) {
    last_adc_sample_time = micros();
    sampleADC2();
  }
}

void update_motors_orders()
{

  motor_speed_rear_right_av = constrain( motor_speed_rear_right_av, -SPEED_MAX, SPEED_MAX);
  motor_speed_rear_left_av = constrain(motor_speed_rear_left_av, -SPEED_MAX, SPEED_MAX);
  motor_speed_front_right_av = constrain(motor_speed_front_right_av, -SPEED_MAX, SPEED_MAX);
  motor_speed_front_left_av = constrain(motor_speed_front_left_av, -SPEED_MAX, SPEED_MAX);


  if (motor_speed_front_left_av > 0)
  {
    digitalWrite(DIRECTION_FRONT_LEFT, HIGH);
  }
  else
  {
    digitalWrite(DIRECTION_FRONT_LEFT, LOW);
  }
  
    if (motor_speed_front_right_av > 0)
  {
    digitalWrite(DIRECTION_FRONT_RIGHT, HIGH);
  }
  else
  {
       digitalWrite(DIRECTION_FRONT_RIGHT, LOW);
  }
    if (motor_speed_rear_left_av > 0)
  {
    digitalWrite(DIRECTION_REAR_LEFT, HIGH);
  }
  else
  {
    digitalWrite(DIRECTION_REAR_LEFT, LOW);
  }
  
    if (motor_speed_rear_right_av > 0)
  {
    digitalWrite(DIRECTION_REAR_RIGHT, HIGH);
  }
  else
  {
       digitalWrite(DIRECTION_REAR_RIGHT, LOW);
  }
  analogWrite(SPEED_FRONT_RIGHT, convert_to_pwm(float(motor_speed_front_right_av)));
  analogWrite(SPEED_FRONT_LEFT, convert_to_pwm(float(-motor_speed_front_left_av)));
  analogWrite(SPEED_REAR_LEFT, convert_to_pwm(float(-motor_speed_rear_left_av)));
  analogWrite(SPEED_REAR_RIGHT, convert_to_pwm(float(motor_speed_rear_right_av)));

}

void stop()
{

  digitalWrite(DIRECTION_REAR_LEFT, LOW);
  digitalWrite(DIRECTION_REAR_RIGHT, LOW);  
  digitalWrite(DIRECTION_FRONT_LEFT, LOW);
  digitalWrite(DIRECTION_FRONT_RIGHT, LOW);  

  analogWrite(SPEED_FRONT_RIGHT, 0);
  analogWrite(SPEED_FRONT_LEFT, 0);
  analogWrite(SPEED_REAR_LEFT, 0);
  analogWrite(SPEED_REAR_RIGHT, 0); 
}

int convert_to_pwm(float motor_speed)
{
  // TODO: compensate the non-linear dependency speed = f(PWM_Value)
  return (int) round(abs(motor_speed)*(255./100.));
}

void get_messages_from_serial()
{
  if(Serial.available() > 0)
  {
    // The first byte received is the instruction
    Order order_received = read_order();
    
    if(order_received == HELLO)
    {
      // If the cards haven't say hello, check the connection
      if(!is_connected)
      {
        is_connected = true;
        write_order(HELLO);
      }
      else
      {
        // If we are already connected do not send "hello" to avoid infinite loop
        write_order(ALREADY_CONNECTED);
      }
    }
    else if(order_received == ALREADY_CONNECTED)
    {
      is_connected = true;
    }
    else
    {
      switch(order_received)
      {
        case STOP:
        {
          motor_speed_front_right_av = 0;
          motor_speed_front_left_av = 0;
          motor_speed_rear_right_av = 0;
          motor_speed_rear_left_av = 0;
          stop();
          if(DEBUG)
          {
            write_order(STOP);
          }
          break;
        }
        case SERVO:
        {
          pos=read_i16();
          frontServo.write(pos);
          if(DEBUG)
          {
            write_order(SERVO);
          }
          break;
        }
        case MOTOR:
        {
          motor_speed_front_right_av = read_i8();
          if(DEBUG)
          {
            write_order(MOTOR);
            write_i8(motor_speed_front_right_av);
          }
          motor_speed_front_left_av = read_i8();
          if(DEBUG)
          {
            write_order(MOTOR);
            write_i8(motor_speed_front_left_av);
          }
          motor_speed_rear_right_av = read_i8();
          if(DEBUG)
          {
            write_order(MOTOR);
            write_i8(motor_speed_rear_right_av);
          }
          motor_speed_rear_left_av = read_i8();
          if(DEBUG)
          {
            write_order(MOTOR);
            write_i8(motor_speed_rear_left_av);
          }
          break;
        }        
        case READENCODERr:
        {
          write_i16(coder[RIGHT]);
          break; 
        }        
        case READENCODERl:
        {
          write_i16(coder[LEFT]);
          break; 
        }
        case RESETENC:
        {
          coder[LEFT] = 0;
          coder[RIGHT] = 0;
          break; 
        }
        case ULTRASONIC:
        {
          //Serial.flush();
          write_i16(MeasureDistance());
          break;
        }
        case CURRENT:
        {
          
          // Renvoyer la dernière moyenne calculée
          noInterrupts(); // Désactiver les interruptions pour éviter les modifications pendant la lecture
          uint16_t current_avg = adc_avg;
          interrupts(); // Réactiver les interruptions
          
          write_i16(current_avg);
          break;
        }

   // Unknown order
    default:
          //write_order(ERROR);
          //write_i32(404);
      return;
      }
    }
  //  write_order(RECEIVED); // Confirm the reception
  }
}


void LwheelSpeed()
{
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}


void RwheelSpeed()
{
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}


Order read_order()
{
  return (Order) Serial.read();
}

void wait_for_bytes(int num_bytes, unsigned long timeout)
{
  unsigned long startTime = millis();
  //Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t* buffer, size_t n)
{
  size_t i = 0;
  int c;
  while (i < n)
  {
    c = Serial.read();
    if (c < 0) break;
    *buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
    i++;
  }
}

int8_t read_i8()
{
  wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t) Serial.read();
}

int16_t read_i16()
{
  int8_t buffer[2];
  wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
  read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

int32_t read_i32()
{
  int8_t buffer[4];
  wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
  read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

void write_order(enum Order myOrder)
{
  uint8_t* Order = (uint8_t*) &myOrder;
  Serial.write(Order, sizeof(uint8_t));
}

void write_i8(int8_t num)
{
  Serial.write(num);
}

void write_i16(int16_t num)
{
  int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

void write_i32(int32_t num)
{
  int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  Serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}
