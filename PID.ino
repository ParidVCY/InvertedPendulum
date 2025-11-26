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
int count                = 0;
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
//================= GAINS (PID) =================//
float ckp = 0.1921;
float cki = 0.0586;
float ckd = 0.1465;


float pkp = 24.2;
float pki = 40.8;
float pkd = 3.58;

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

  // Create control task on Core 1
  // xTaskCreatePinnedToCore(
  //   controlTask,
  //   "Control Task",
  //   10000,
  //   NULL,
  //   1,
  //   &ControlTaskHandle,
  //   1  // Core 1
  // );

  // Create print task on Core 0
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
    if ((170*M_PI/180 <= theta) && (theta <= 190*M_PI/180)){
      begin = true;
      if (initSet){
        offset      = 0;
        offset_dot  = 0;
        offset_sum  = 0;
        error       = 0;
        error_dot   = 0;
        error_sum   = 0;
        initSet     = false;
      }
    }
    else if (!((170*M_PI/180 <= theta) && (theta <= 190*M_PI/180))){
      begin = false;
      origin = cart_position;
      if (!initSet){
        initSet     = true;
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
        static int counter, toggle, toggle_add = 1;
        static float ref_2;
        ref_2 = 0;

        counter++;
        if (counter >= 10) {
          float offset_temp = offset;
          offset = ref_2 + cart_position;
          offset_dot = (offset - offset_temp) * 1000 / dt / counter;
          offset_sum = offset_sum + offset / 1000 * dt * counter;

          C2_u = (offset * ckp) + (offset_dot * ckd) + (offset_sum * cki);
          counter = 0;
        }
      
        float error_temp = error;
        // error = (C2_u + M_PI - theta);  //r = PI
        error = (C2_u + 0-(theta - M_PI));  //r = 0
        error_dot = (error - error_temp) * 1000 / dt;
        error_sum = error_sum + error * dt / 1000;

        //================= For Experiment =================//
        //timeCount = millis() - startMillis;
        // cart_ac = 0.05;//0.4*sin(2*M_PI *timeCount/1000);
        // cart_velo = cart_velo + cart_ac * (float)dt * 0.001;
        //==================================================//
        cart_ac = constrain(pkp * error + pkd * error_dot + error_sum * pki, -tunerA, tunerA);
        cart_velo = constrain(cart_velo + cart_ac * (float)dt * 0.001, -tunerV, tunerV);
      }
    //////////////////////////Control Part End//////////////////////////////////
    //////////////////////////Swing Up Begin//////////////////////////////////
      else if (!begin){
        alpha = 1;
        float cart_ac_max = 0.7; // m/s^2

        cart_ac = cart_ac_max * sgn(angular_velo) * sgn(-cosf(theta)); // m/s^2   From control rule

        // making the cart stay center point, + acc25% direction to center
        const float acc_offset = 0.25;
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
          // else if (millis() - startMillis < 4*duration/4 + 100){
          //   cart_ac = exite_a;
          // }
          // else if (millis() - startMillis < 5*duration/5){
          //   cart_ac = 0;
          // }
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
      Serial.print(",");
      Serial.print(error);
      // Serial.print(" count: ");
      // Serial.print(count);
      // Serial.print(" theta_d: ");
      // Serial.print(C2_u);
      // Serial.print(" AngleLogicCheck: ");
      // Serial.print((170*M_PI/180 <= theta) && (theta <= 190*M_PI/180));
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
      else if (temp == 'p')
        pkp -= 1;
      else if (temp == 'i')
        pki -= 1;
      else if (temp == 'd')
        pkd -= 1;
      else if (temp == 'P')
        pkp += 1;
      else if (temp == 'I')
        pki += 1;
      else if (temp == 'D')
        pkd += 1;
    }
    vTaskDelay(interval / portTICK_PERIOD_MS);  // Print every 10ms
  }
  vTaskDelay(interval / portTICK_PERIOD_MS);  // Print every 10ms
}
