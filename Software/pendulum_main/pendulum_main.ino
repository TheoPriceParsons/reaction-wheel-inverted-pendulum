#include <SimpleFOC.h>
#include <math.h>
#include <stdlib.h>
//physical constants
#define M_PI     3.14159265358979f
//#define V_SUPPLY 12.0f
#define V_SUPPLY 3.0f//for set up
#define G        9.81f
#define M_PEND   0.5f
#define L        0.3f
#define IW       0.002f
#define B_DAMP   0.01f
//motor constants
#define R_M      2.4f
#define K_E      0.00954f
#define K_T      0.03f
//Gains from jupyter sims
#define DT       0.002f
const float Kd[2] = {-4.976694f, -1.404655f};
const float Ad[2][2] = {{1.000065f, 0.002000f}, {0.065401f, 1.000065f}};
const float Bd[2]    = {-0.000089f, -0.088891f};
const float Ld[2]    = {0.190094f,  4.418506f};
//switching threshholds
#define DELTA_SW     0.5f
#define DELTA_DOT_SW 4.0f
//FOC objects:
  BLDCMotor  motor(11);  // 11 pole pairs
  BLDCDriver3PWM driver(PA8, PA9, PA10);  // 3-phase

//Encoder on motor shaft
  Encoder enc_motor(PB4, PB5, 2048);
  void doMotorA() { enc_motor.handleA(); }
  void doMotorB() { enc_motor.handleB(); }

// Encoder on the pendulum gives us theta
  Encoder enc_pivot(PB6, PB7, 2048);
  void doPivotA() { enc_pivot.handleA(); }
  void doPivotB() { enc_pivot.handleB(); }

//initialising the observer vars
float x_hat[2]   = {0.0f, 0.0f};
float V_prev     = 0.0f;
float omega_prev = 0.0f;
//loop control we are running 500Hz so loop runs every 2ms
volatile uint32_t last_time = 0;


void setup() {
  Serial.begin(115200);
  //motor
  enc_motor.init();
  enc_motor.enableInterrupts(doMotorA, doMotorB);
  //pendulum
  enc_pivot.init();
  enc_pivot.enableInterrupts(doPivotA, doPivotB);
  //linking them up
  motor.linkDriver(&driver);
  motor.linkSensor(&enc_motor);
  motor.voltage_limit = V_SUPPLY;//caution start at 3V
  //Initializing...
  driver.init();
  motor.init();
  motor.initFOC();
  

}

//helper functions
float wrap(float angle) {
  return fmodf(angle+M_PI,2.0f * M_PI) - M_PI;
}
void loop() {
  motor.loopFOC();
  // put your main code here, to run repeatedly:(every 2ms)
  if (micros()-last_time>=2000) {
    last_time += 2000;
    //read the encoder motor and pendulum
    float theta = enc_pivot.getAngle();
    float delta_meas = wrap(theta-M_PI);
    float omega = enc_motor.getVelocity();
    float tau_prev = K_T * (V_prev - K_E * omega_prev) / R_M;
    float theta_dot = enc_pivot.getVelocity();
    //We estimate with sim calcs with model
    float x_hat_new[2];
    x_hat_new[0] = Ad[0][0]*x_hat[0] + Ad[0][1]*x_hat[1] + Bd[0]*tau_prev;
    x_hat_new[1] = Ad[1][0]*x_hat[0] + Ad[1][1]*x_hat[1] + Bd[1]*tau_prev;
    x_hat[0] = x_hat_new[0];
    x_hat[1] = x_hat_new[1];
    //we correct with our out of house L constant from sim
    float innov = delta_meas-x_hat[0];
    x_hat[0] = x_hat[0] + Ld[0]*innov;
    x_hat[1] = x_hat[1] + Ld[1] * innov;
    //LQR or Bang Bang?
    //energy for bang bang
    float E = 0.5f * M_PEND * L * L * theta_dot * theta_dot - M_PEND * G * L * cosf(theta);
    //our goal
    float E_star = M_PEND * L * G;
    //Bang Bang voltage:
    float V_su;
    if (fabsf(theta_dot)<1e-6f) {
      V_su = 0.3f*V_SUPPLY;
    }
    else if (E<E_star) {
      V_su = -V_SUPPLY*theta_dot/(fabsf(theta_dot));
    }
    else {
      V_su = 0;
    }
    //LQR
    float tau_lqr_des = -(Kd[0] * x_hat[0] + Kd[1] * x_hat[1]);
    float V_lqr = (tau_lqr_des/K_T)*R_M + K_E * omega;
    //we choose
    float V_cmd;
    if (fabsf(x_hat[0])<DELTA_SW && fabsf(x_hat[1])<DELTA_DOT_SW) {
      V_cmd=V_lqr;
    }
    else {
      V_cmd = V_su;
    }
    //we tell FOC what torque we want
    V_cmd = constrain(V_cmd, -V_SUPPLY, V_SUPPLY);
    motor.move(V_cmd);
    //we keep it for the next time around
    omega_prev = omega;
    V_prev = V_cmd;
  }

}
