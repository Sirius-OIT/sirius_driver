#include "AnalogIn.h"
#include "DigitalIn.h"
#include "DigitalOut.h"
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

#include "mros2.h"
#include "EthernetInterface.h"
#include "std_msgs/msg/float32.hpp"
#include "tf2_msgs/msg/t_fmessage.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"

#define IP_ADDRESS ("192.168.11.2") /* IP address */
#define SUBNET_MASK ("255.255.255.0") /* Subnet mask */
#define DEFAULT_GATEWAY ("192.168.11.1") /* Default gateway */

#define MOTORL PA_6
#define MOTORR PC_7
#define PULSE_L PB_8
#define PULSE_R PC_6

#define ANALOG_PIN0 A0
#define ANALOG_PIN1 A1

#define REDUCTION 50
#define THREAD 0.48
#define HALF_THREAD 0.48 /2
#define EPSILON 0.01
#define WHELL_RADIUS 0.2
#define PULSE 18
#define PI 3.141592

#define MAXRPM 2500

PwmOut motor_l(MOTORL);
PwmOut motor_r(MOTORR);
DigitalOut led(LED1);
DigitalIn rotation_direction_r(PB_9);
DigitalIn rotation_direction_l(PB_15);
Ticker ticker;
Timer stm_clock;

AnalogIn inputA0(ANALOG_PIN0);
AnalogIn inputA1(ANALOG_PIN1);


// main() runs in its own thread in the OS
using namespace std;
using namespace std::chrono_literals;

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

float radius = 0.0f;
float v_l = 0.0f;
float v_r = 0.0f;
float pwm_l = 0.0f;
float pwm_r = 0.0f;

typedef struct
{
  uint32_t secs; // Transmit Time-stamp seconds.
} ntp_packet;


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
    //for caluculate back
    if(rotation_direction_l.read() == 1)
    {
        cnt_l--;

    } else {
        cnt_l++;   
    }
}

void counter_r()
{
    //for caluculate back
    if(rotation_direction_r.read() == 0)
    {
        cnt_r--;
    } else {
        cnt_r++;
    }
}

//control
void callback(geometry_msgs::msg::Twist *data)
{
    radius = data->linear.x / data->angular.z;
    if(radius == 0)
    {
        v_l = radius * data->angular.z;
        pwm_l = ((60 * v_l) / 2 * PI * WHELL_RADIUS) / MAXRPM;
        motor_l.write(pwm_l);
        motor_r.write(pwm_l);
    } else if(radius <= EPSILON)
    {
        v_l = radius * data->angular.z;
        pwm_l = ((60 * v_l) / 2 * PI * WHELL_RADIUS) / MAXRPM;
        motor_l.write(pwm_l);
        motor_r.write(-pwm_l);


    } else
    {
        v_l = (radius - HALF_THREAD) * data->angular.z;
        v_r = (radius + HALF_THREAD) * data->angular.z;
        pwm_l = ((60 * v_l) / 2 * PI * WHELL_RADIUS) / MAXRPM;
        pwm_r = ((60 * v_r) / 2 * PI * WHELL_RADIUS) / MAXRPM;
        motor_l.write(pwm_l);
        motor_r.write(pwm_r);
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

    EthernetInterface network;
    network.set_dhcp(false);
    network.set_network(IP_ADDRESS, SUBNET_MASK, DEFAULT_GATEWAY);
    nsapi_size_or_error_t result = network.connect();

    printf("mbed mros2 start!\r\n");
    mros2::init(0, NULL);
    MROS2_DEBUG("mROS 2 initialization is completed\r\n");

    mros2::Node node = mros2::Node::create_node("mros2_node");
    mros2::Publisher pub = node.create_publisher<geometry_msgs::msg::Pose>("wheel_odom", 10);
    mros2::Subscriber sub = node.create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, callback);
    osDelay(100);
    MROS2_INFO("ready to pub/sub message\r\n");
    printf("start!!");

    std_msgs::msg::Float32 msg;

    while (1) {
        led = !led;
        geometry_msgs::msg::Pose pose;
        pose.position.x = odom_x;
        pose.position.y = odom_y;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 1.0 * sin(theta/2);
        pose.orientation.w = cos(theta/2);

        pub.publish(pose);

        osDelay(100);
    }

    mros2::spin();
    return 0;
}

