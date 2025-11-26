#include <Arduino.h>
#include <AccelStepper.h>
#include "ESP32Encoder.h"

//================= PIN DEFINITIONS =================//
const int stepPin        = 18;      
const int dirPin         = 19; 
const int encoderPinA    = 32;  //Green line
const int encoderPinB    = 33;  //White line
const int buttonPin      = 22;    // start/reset button
const int resetPin       = 23;  

//================= OBJECTS =================//
AccelStepper stepper(1, stepPin, dirPin);  // 1 = Driver mode (STEP, DIR)
ESP32Encoder encoder;

//================= VARIABLES =================//
double dt                = 1;
static float offset      = 0;
static float offset_dot  = 0;
static float offset_sum  = 0;
static float error       = 0;
static float error_dot   = 0;
static float error_sum   = 0;
float C2_u               = 0;
float cart_ac            = 0;
float cart_velo          = 0;
float cart_position      = 0;
float origin             = 0;
long counter             = 0;
double theta             = 0;
long counterP            = 0;
double thetaP            = 0;
float oldAngle           = 0;
float oldTheta_dot       = 0;
float dTheta             = 0;
float dTheta_dot         = 0;
float angular_velo       = 0;
float angular_ac         = 0;
float alpha              = 0.1;
bool start               = false;
bool begin               = false;
bool initSet             = true;
unsigned int helicopter  = 0;
bool stop                = false;
bool stopFlag            = true;
bool home                = false;
bool excite              = true;

unsigned long capturedTime = 0;
double oldTime           = 0;
double newTime           = 0;
unsigned long startMillis  = 0;
unsigned long timeCount    = 0;
const unsigned long interval = 10;   // For Serial.print() every 10ms

float tunerA             = 7;
float tunerV             = 0.45;//0.45;
//================= GAINS (LQR) =================//
float k1 = -1;  // example gain
float k2 = 24.3006;
float k3 = -1.7476;
float k4 = 3.3302;

TaskHandle_t ControlTaskHandle;
TaskHandle_t PrintTaskHandle;

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

void setup() {
  Serial.begin(115200);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  stepper.setMaxSpeed(2000000);
  encoder.attachFullQuad(encoderPinA, encoderPinB);
  encoder.clearCount();
  digitalWrite(dirPin, LOW);

  xTaskCreatePinnedToCore(
    printTask,
    "Print Task",
    5000,
    NULL,
    1,
    &PrintTaskHandle,
    0  // Core 0
  );
}

void loop() {
  while (true) {
    if (digitalRead(resetPin) == LOW){
      ESP.restart();
    }
    
    counter = encoder.getCount();
    theta = double(counter) * (2.0 * M_PI / 2400.0);
    // adjust theta to [0,2*pi]
    while (theta < 0) theta += 2.0 * M_PI;
    while (theta >= 2 * PI) theta -= 2.0 * M_PI;

    if (digitalRead(buttonPin) == LOW && !start) {
      start              = true;
      startMillis = millis();
      Serial.println("Start!");
      cart_ac            = 0;
      cart_velo          = 0;
      cart_position      = 0;
      angular_velo       = 0;
      angular_ac         = 0;
      oldAngle           = theta;
      oldTime            = micros();
    }

    //Pass if the button is not pushed
    if (!start) {
      return;
    }
    

///////////////limit x position////////////////////////////////
//****recommend to use limit switch to determine whether it need to home
    if (abs(-cart_position) >= 0.26){
      home = true;
    }
//////////////////////////////////////////////////////////////
    if ((160*M_PI/180 <= theta) && (theta <= 200*M_PI/180)){
      begin = true;
      if (initSet){
        // cart_ac            = 0;
        // cart_velo         = 0;
        // cart_position     = 0;
        offset      = 0;
        offset_dot  = 0;
        offset_sum  = 0;
        error       = 0;
        error_dot   = 0;
        error_sum   = 0;
        initSet     = false;
      }
    }
    else if (!((160*M_PI/180 <= theta) && (theta <= 200*M_PI/180))){
      begin = false;
      origin = cart_position;
      if (!initSet){
        initSet     = true;
        //stop        = true;
        //setzero
        // cart_ac            = 0;
        // cart_velo          = 0;
        // cart_position      = 0;
        // theta              = 0;
        // angular_velo       = 0;
        // angular_ac         = 0;
        // oldAngle           = 0;
        // oldTheta_dot       = 0;
      }
    } 
    newTime = micros();
    if (newTime - oldTime >= 1000){
      dt = (newTime - oldTime) / 1000;
      oldTime = newTime;

      float newAngle = theta;
      dTheta = (newAngle - oldAngle);
      oldAngle = newAngle;

      //angular_velo = (dTheta / dt) * 1000.0;  // rad/s
      float new_angular_velo = (dTheta / dt) * 1000.0;  // rad/s
      angular_velo = alpha * new_angular_velo + (1 - alpha) * angular_velo;

      float newTheta_dot = angular_velo;
      dTheta_dot = (newTheta_dot - oldTheta_dot);
      oldTheta_dot = newTheta_dot;

      float new_angular_ac = (dTheta / dt) * 1000.0;  // rad/s
      angular_ac = alpha * new_angular_ac + (1 - alpha) * angular_ac;  // rad/s^2
    //////////////////////////Control Part Begin//////////////////////////////////
      if (begin) {
        alpha = 0.15;
        float x1 = cart_position;
        float x2 = theta - M_PI;
        float x3 = cart_velo;
        float x4 = angular_velo;  
        cart_ac = constrain(-(k1 * x1 + k2 * x2 + k3 * x3 + k4 * x4), -tunerA, tunerA);
        cart_velo = constrain(cart_velo + cart_ac * (float)dt * 0.001, -tunerV, tunerV);
      }
    //////////////////////////Control Part End//////////////////////////////////
    //////////////////////////Swing Up Begin//////////////////////////////////
      else if (!begin){
        alpha = 1;
        float cart_ac_max = 1;//0.8; // m/s^2

        cart_ac = cart_ac_max * sgn(angular_velo) * sgn(-cosf(theta)); // m/s^2   From control rule

        // making the cart stay center point, + acc25% direction to center
        const float acc_offset = 0.4;//0.25;
        if (fabs(cart_position) >= 0.25){
          cart_ac_max = 0.3;
        }
        if (cart_position > 0.05){
          if (cart_ac > 0)
            cart_ac *= (1.0f - acc_offset);
          else
            cart_ac *= (1.0f + acc_offset);
        }
        else if (cart_position < -0.05){
          if (cart_ac < 0){
            cart_ac *= (1.0f - acc_offset);
          }
          else{
            cart_ac *= (1.0f + acc_offset);
          }
        }

        if (excite){
          float exite_a = 3;  // m/s
          float duration = 400;//ms
          if (millis() - startMillis < duration/2){
            cart_ac = exite_a;
          }
          else if (millis() - startMillis < duration){
            cart_ac = -exite_a;
          }

          else{
            excite = false;
          }
        }
        cart_velo = cart_velo + cart_ac * (float)dt * 0.001;  // m/s
      }
    //////////////////////////Swing Up End//////////////////////////////////
      if (stop){
        if (stopFlag){
          capturedTime = millis();
          stopFlag = false;
        }
        if(millis()-capturedTime <= 6000){
          cart_velo = 0;
          helicopter = 0;
        }
        else{
          stop = false;
          stopFlag = true;
          home = true;
        }
      }
      else{
        stop = false;
      }

      if (home){
        cart_velo = -sgn(cart_position)*(0.1);  // 100/30 step/mm
        if (abs(-cart_position)<=0.005){
          cart_velo = 0;
          cart_ac = 0;
          home = false;
        }
      }
      
      cart_position = cart_position + cart_velo * (float)dt * 0.001;  // m
    }
    stepper.setSpeed(-cart_velo * 1000.0f * 100.0f / 30.0f * 16);  // 100/30 step/mm, 16 micro step
    stepper.runSpeed();
  }
}


void printTask(void *parameter) {
  while (true) {
    counterP = encoder.getCount();
    thetaP = double(counterP) * (360 / 2400.0);
    while (thetaP < 0) thetaP += 360;
    while (thetaP >= 360) thetaP -= 360;
    if (start) {
      Serial.print(newTime/1000-startMillis);
      Serial.print(",");
      Serial.print(cart_position, 4);
      Serial.print(",");
      Serial.print(cart_velo, 4);
      Serial.print(",");
      Serial.print(cart_ac, 4);
      Serial.print(",");
      Serial.print(theta*180/M_PI, 4);
      Serial.print(",");
      Serial.print(angular_velo, 4);
      Serial.print(",");
      Serial.print(angular_ac, 4);
      // Serial.print(" oldTime: ");
      // Serial.print(oldTime);
      // Serial.print(" tuner a: ");
      // Serial.print(tunerA);
      // Serial.print(" tuner v: ");
      // Serial.print(tunerV);
      // Serial.print(" dt: ");
      // Serial.print(dt);
      // Serial.print(" e: ");
      // Serial.print(error);
      // Serial.print(" e_dot: ");
      // Serial.print(error_dot);
      // Serial.print(" e_sum: ");
      // Serial.print(error_sum);
      // Serial.print(" Kp: ");
      // Serial.print(pkp);
      // Serial.print(" Ki: ");
      // Serial.print(pki);
      // Serial.print(" Kd: ");
      // Serial.print(pkd);
      // Serial.print(" dTheta ");
      // Serial.print(dTheta, 4);
      // Serial.print(" Origin ");
      // Serial.print(origin);   
      // Serial.print(" u: ");
      // Serial.print(-cart_ac, 4);
      // Serial.print(" x: ");
      // Serial.print(cart_position, 4);
      // Serial.print(" x_dot: ");
      // Serial.print(cart_velo, 4);
      // Serial.print(" x_ddot: ");
      // Serial.print(cart_ac, 4);
      // Serial.print(" theta: ");
      // Serial.print(theta*180/M_PI, 4);
      // Serial.print(" theta_dot: ");
      // Serial.print(angular_velo, 4);
      // Serial.print(" theta_ddot: ");
      // Serial.print(angular_ac, 4);
      Serial.println("");
    }
    else{
      Serial.print(" theta: ");
      Serial.println(thetaP, 4); 
    }

    if (Serial.available()) {
      char temp = Serial.read();
      if (temp == 'a')
        tunerA += 0.05;
      else if (temp == 'z')
        tunerA -= 0.05;
      else if (temp == 'f')
        tunerV += 0.01;
      else if (temp == 'v')
        tunerV -= 0.01;
    }
    vTaskDelay(interval / portTICK_PERIOD_MS);
  }
  vTaskDelay(interval / portTICK_PERIOD_MS);  // Print every 10ms
}
