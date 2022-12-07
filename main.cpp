#include "AnalogIn.h"
#include "DigitalIn.h"
#include "DigitalOut.h"
#include "InterfaceDigitalIn.h"
#include "InterruptIn.h"
#include "Mutex.h"
#include "PinNameAliases.h"
#include "PinNames.h"
#include "PinNamesTypes.h"
#include "PwmOut.h"
#include "ThisThread.h"
#include "UnbufferedSerial.h"
#include "mbed.h"
#include "mbed_power_mgmt.h"
#include <cmath>
#include <cstdint>
#include <cstdio>


#define MOTOR1 PA_6
#define MOTOR2 PC_7
#define PULSE_L PB_8
#define PULSE_R PC_6

#define ANALOG_PIN0 A0
#define ANALOG_PIN1 A1

#define REDUCTION 50
#define THREAD 0.48
#define WHELL_RADIUS 0.2
#define PULSE 18
#define PI 3.141592

PwmOut motor(MOTOR1);
PwmOut motor2(MOTOR2);
DigitalOut led(LED1);
DigitalIn rotation_direction_r(PB_9);
DigitalIn rotation_direction_l(PB_15);
Ticker ticker;

AnalogIn inputA0(ANALOG_PIN0);
AnalogIn inputA1(ANALOG_PIN1);


// main() runs in its own thread in the OS
using namespace std;
using namespace std::chrono_literals;

char buf[24];
Mutex mtx;

static UnbufferedSerial *pc;

InterruptIn pulse_l(PULSE_L);
InterruptIn pulse_r(PULSE_R);
int32_t cnt_l=0;
int32_t cnt_r=0;

int32_t cnt_l_prime=0;
int32_t cnt_r_prime=0;

float_t rpm_l = 0.0f;
float_t rpm_r = 0.0f;
float pulse_diff_l = 0.0f;
float pulse_diff_r = 0.0f;

// m/s
float velocity = 0.0f;
float omega = 0.0f;
float velocity_l = 0.0f;
float velocity_r = 0.0f;

// initialize odometry pose
// m
float odom_x = 0.0f;
float odom_x_prime = 0.0f;
float odom_y = 0.0f;
float odom_y_prime = 0.0f;

float theta = 0.0f;
float theta_prime = 0.0f;


void flip()
{
    //calculate velocity--------------------------
    //left
    pulse_diff_l = (cnt_l - cnt_l_prime) * 100 * 60;
    rpm_l = (pulse_diff_l / (REDUCTION * PULSE));
    velocity_l = (2 * PI * WHELL_RADIUS * rpm_l) / 60;

    pulse_diff_r = (cnt_r - cnt_r_prime) * 100 * 60;
    rpm_r = (pulse_diff_r / (REDUCTION * PULSE));
    velocity_r = (2 * PI * WHELL_RADIUS * rpm_r) / 60;
    //--------------------------------------------

    //calculate odometry for ros-------------------
    velocity = (velocity_r + velocity_l) * 0.5;
    omega = (velocity_r - velocity_l) / THREAD;
    theta = (omega * 0.01) + theta_prime;
    odom_x = ((velocity*cos(theta)) * 0.01) + odom_x_prime;
    odom_y = ((velocity*sin(theta)) * 0.01) + odom_y_prime;
    //--------------------------------------------

    motor.period_ms(5);
    motor.pulsewidth_ms(20);

    motor2.period_ms(5);
    motor2.pulsewidth_ms(10);

    //update--------------------------------------
    cnt_l_prime = cnt_l;
    cnt_r_prime = cnt_r;
    theta_prime = theta;
    odom_x_prime = odom_x;
    odom_y_prime = odom_y;
    //--------------------------------------------
}

void counter_l()
{
    //back
    if(rotation_direction_l.read() == 1)
    {
        cnt_l--;

    } else {
        cnt_l++;   
    }
}

void counter_r()
{
    //back
    if(rotation_direction_r.read() == 0)
    {
        cnt_r--;
    } else {
        cnt_r++;
    }
}

int main()
{
    led = 0;
    //パルスカウントのためのpullup
    pulse_l.mode(PullUp);
    pulse_r.mode(PullUp);
    rotation_direction_r.mode(PullUp);
    rotation_direction_l.mode(PullUp);
    // pc = new UnbufferedSerial(USBTX, USBRX, 115200);
    ticker.attach(&flip, 10ms);
    pulse_l.rise(&counter_l);
    pulse_r.rise(&counter_r);
    printf("start!!");
    while (1) {
        led = !led;
        printf("count_l:%d, count_r:%d\n", cnt_l, cnt_r);
        printf("velocity_left:%f, velocity_right:%f\n", velocity_l, velocity_r);
        printf("odometry_x:%f, odometry_y:%f theta:%f\n", odom_x, odom_y, theta);
        printf("rotation_r: %d ", rotation_direction_r.read());
        printf("rotation_l: %d\n", rotation_direction_l.read());
        ThisThread::sleep_for(100ms);
    }
}

