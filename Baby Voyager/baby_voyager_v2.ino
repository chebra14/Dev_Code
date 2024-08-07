#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include "BluetoothSerial.h"

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t lastIsrAt = 0;
volatile uint32_t isrCounter = 0;
uint32_t scaler = 4; //250ms
uint32_t connection_delay = 8000; //8s
uint32_t turn_delay = 10000 + connection_delay; // 13s
uint32_t max_On = turn_delay + 10000; //23s

BluetoothSerial SerialBT;

//IMU
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
//SDA = P21
//SCL = P22

//SLIP LEDs
const int LED_slip_1 = 17;
const int LED_slip_2 = 4;
// const int LED_slip_1 = 2;

//Motor
const int pwm_1 = 26;
const int pwm_2 = 16;

//Encoder
const int motor1_speed = 18;
const int motor2_speed = 19;
volatile uint32_t pwm_counter_1 = 0;
volatile uint32_t pwm_counter_2 = 0;
float disk_Slots = 20.00;

//Lateral Slip Test
float D_Wheels = 0.13; // Distance from rotation point
float R_wheel = 0.033; // Radius of wheels
float pi = (float) M_PI;
float encoder_max = 20.0;
float k = (((float)scaler) * R_wheel * 2.0 * pi)/encoder_max;

float yaw_mod = 100.0;
float threshold = 0.5;

void ISR_counter_1()
{
  pwm_counter_1 = pwm_counter_1 + 1;
}

void ISR_counter_2()
{
  pwm_counter_2 = pwm_counter_2 + 1;
}

bool slip_detect(uint32_t w_s_L, uint32_t w_s_R, float yaw_r)//, accel_x, accel_y)
{
  bool slip_test = false;

  float w_s_1 = 0; //Larger
  float w_s_2 = 0; //Smaller

  //float accel_xy = (accel_x ** 2 + accel_y ** 2) ** 0.5;

  w_s_1 = (float)(w_s_R) * k;
  w_s_2 = (float)(w_s_L) * k;

  //This order (1-2) to match 9DOF upside down
  yaw_mod = (w_s_1 - w_s_2)/D_Wheels;

  //Implement Threshold
  if(abs(yaw_mod - yaw_r) > threshold)
  {
    //Slipping
    slip_test = true;
  }

  return slip_test;

}

void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);

  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);

  isrCounter = isrCounter + 1;

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  isrCounter = 0;
}

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void setup() {

  //LED setup
  pinMode(LED_slip_1, OUTPUT);
  pinMode(LED_slip_2, OUTPUT);
  digitalWrite(LED_slip_1, LOW);
  digitalWrite(LED_slip_2, LOW);

  //MOTOR SETUP
  pinMode(pwm_1,OUTPUT) ;  	//we have to set PWM pin as output
  pinMode(pwm_2,OUTPUT) ;
  analogWrite(pwm_1,0);
  analogWrite(pwm_2,0);

  //SERIAL SETUP
  Serial.begin(115200);
  SerialBT.begin("Baby-Voyager");

  delay(connection_delay); // To connect bluetooth and put the robot down

  analogWrite(pwm_1,160);
  analogWrite(pwm_2,160);
  //TEST LED
  //digitalWrite(LED_slip_1, HIGH);

  //TIMER SETUP
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // Set timer frequency to 1Mhz
  timer = timerBegin(1000000);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, 1000000/scaler, true, 0);

  //ENCODER SETUP
  attachInterrupt(digitalPinToInterrupt(motor1_speed), ISR_counter_1, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2_speed), ISR_counter_2, RISING);

  //IMU SETUP
  /* Initialise the sensor */
  /* There was a problem detecting the LSM9DS0 ... check your connections */  
  while(!lsm.begin()){
    SerialBT.println(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    delay(1000);
  }
  /* Setup the sensor gain and integration time */
  configureSensor();
}

void loop() {
  // If Timer has fired after 200ms
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    
    uint32_t isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);

    /* Get a new sensor event */ 
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    if(isrTime > turn_delay)
    {
      if(isrTime > max_On)
      {
        analogWrite(pwm_1,0);
        analogWrite(pwm_2,0);

        //TEST LED
        //digitalWrite(LED_slip_1, LOW);
      }
      else{
        analogWrite(pwm_2,50);
      }
    }

    bool slipping = slip_detect(pwm_counter_1, pwm_counter_2, gyro.gyro.z);

    if(slipping)
    {
      digitalWrite(LED_slip_1, HIGH);
    }else{
      digitalWrite(LED_slip_1, LOW);
    }
    //PRINT DATA//

    if (SerialBT.connected()) {
      //TIME
      SerialBT.print(isrTime);
      SerialBT.print(", ");

      //IMU 
      // print out accelleration data
      SerialBT.print(accel.acceleration.x);
      SerialBT.print(", ");
      SerialBT.print(accel.acceleration.y);
      SerialBT.print(", ");
      // SerialBT.print(accel.acceleration.z);
      // SerialBT.print(", ");
      // print out gyroscopic data
      // SerialBT.print(gyro.gyro.x);
      // SerialBT.print(", ");
      // SerialBT.print(gyro.gyro.y);
      // SerialBT.print(", ");
      SerialBT.print(gyro.gyro.z);
      SerialBT.print(", ");

      //SPEED SENSOR (Pulses per 200ms)
      SerialBT.print(pwm_counter_1);
      SerialBT.print(", ");
      SerialBT.print(pwm_counter_2);
      SerialBT.print(", ");

      pwm_counter_1 = 0;
      pwm_counter_2 = 0;

      //Test yaw_model
      SerialBT.println(yaw_mod);
    }

  }

}