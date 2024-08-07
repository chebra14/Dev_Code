#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include "BluetoothSerial.h"

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore_1s;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t lastIsrAt = 0;
volatile uint32_t isrCounter = 0;
uint32_t scaler = 10; //100ms
uint32_t connection_delay = 8000; //8s
uint32_t turn_delay = 5000 + connection_delay; // 13s
uint32_t max_On = turn_delay + 10000; //23s

BluetoothSerial SerialBT;

//IMU
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

//Motor
const int pwm_1 = 26;
const int pwm_2 = 14;


//Encoder
const int motor1_speed = 18;
const int motor2_speed = 19;
volatile uint32_t pwm_counter_1 = 0;
volatile uint32_t pwm_counter_2 = 0;
float disk_Slots = 20.00;

void ISR_counter_1()
{
  pwm_counter_1 = pwm_counter_1 + 1;
}

void ISR_counter_2()
{
  pwm_counter_2 = pwm_counter_2 + 1;
}

void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);

  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);

  isrCounter = isrCounter + 1;

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore_1s, NULL);
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

  //TIMER SETUP
  // Create semaphore to inform us when the timer has fired
  timerSemaphore_1s = xSemaphoreCreateBinary();
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
  // If Timer has fired after 1 second
  if (xSemaphoreTake(timerSemaphore_1s, 0) == pdTRUE) {
    
    uint32_t isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);

    float rotation_1 = (pwm_counter_1 / disk_Slots) * scaler; //REV/s
    pwm_counter_1 = 0;
    float rotation_2 = (pwm_counter_2 / disk_Slots) * scaler; //REV/s
    pwm_counter_2 = 0;

    /* Get a new sensor event */ 
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    if(isrTime > turn_delay)
    {
      if(isrTime > max_On)
      {
        analogWrite(pwm_1,0);
        analogWrite(pwm_2,0);
      }
      else{
        analogWrite(pwm_2,50);
      }
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
      SerialBT.print(accel.acceleration.z);
      SerialBT.print(", ");
      // print out gyroscopic data
      SerialBT.print(gyro.gyro.x);
      SerialBT.print(", ");
      SerialBT.print(gyro.gyro.y);
      SerialBT.print(", ");
      SerialBT.print(gyro.gyro.z);
      SerialBT.print(", ");

      //SPEED SENSOR
      SerialBT.print(rotation_1);
      SerialBT.print(", ");
      SerialBT.println(rotation_2);
    }
  }

}