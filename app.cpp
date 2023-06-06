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

#define MOTORL PF_7 
#define MOTORR PE_13
#define PULSE_L PE_6 
#define PULSE_R PE_9

#define ANALOG_PIN0 A0
#define ANALOG_PIN1 A1

#define REDUCTION 50
#define THREAD 0.48
#define HALF_THREAD 0.48 /2
#define EPSILON 0.01
#define WHELL_RADIUS 0.2
#define PULSE 18
#define PI 3.141592

#define MAXRPM 3000
#define BIAS 0.001

PwmOut motor_l(MOTORL);
PwmOut motor_r(MOTORR);
DigitalOut led(LED1);
DigitalIn rotation_direction_r(PF_14);
DigitalIn rotation_direction_l(PF_8);

DigitalOut forward_signal_r(PG_14);
DigitalOut back_signal_r(PF_15);
DigitalOut forward_signal_l(PG_1);
DigitalOut back_signal_l(PF_9);

Ticker ticker;
Timer stm_clock;

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
float rpm_control_l = 0.0f;
float rpm_control_r = 0.0f;

//debug
int frag;

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
    radius = data->linear.x / (data->angular.z + BIAS);
    //forward or back
    //if(radius >= 1.0 || radius <= -1.0)
    if(radius >= 100.0 || radius <= -100.0)
    {
        v_l = (radius) * (data->angular.z+BIAS);
        rpm_control_l = (60 * v_l) * 100.0 / 2 * PI * WHELL_RADIUS;
        if(rpm_control_l >= MAXRPM)
        {
            rpm_control_l = MAXRPM;
        } 
        else if(rpm_control_l <= -MAXRPM)
        {
            rpm_control_l = -MAXRPM;
        }

	    pwm_l = std::abs(rpm_control_l / MAXRPM);
        
        if(radius >= 1.0) //forward
        {
            forward_signal_l = 1;
            back_signal_l = 0;
            forward_signal_r = 0;
            back_signal_r = 1;
        }else
        {
            forward_signal_l = 0;
            back_signal_l = 1;
            forward_signal_r = 1;
            back_signal_r = 0;
        }
        motor_l.period_us(10);
        motor_l.write(pwm_l);
        motor_r.period_us(10);
        motor_r.write(pwm_l);

        frag = 1;

    //rotation only
    } else if(radius <= 0.0 + EPSILON && radius >= 0.0 - EPSILON)
    {
        v_l = data->angular.z;
        rpm_control_l = (60 * v_l) * 100.0 / 2 * PI * WHELL_RADIUS;
        if(rpm_control_l >= MAXRPM)
        {
            rpm_control_l = MAXRPM;
        }
        else if(rpm_control_l <= -MAXRPM)
        {
            rpm_control_l = -MAXRPM;
        }

        pwm_l = rpm_control_l / MAXRPM;
        pwm_l = std::abs(pwm_l + 0.02);

        if(rpm_control_l <= 0.0) //forward
        {
            forward_signal_l = 1;
            back_signal_l = 0;
            forward_signal_r = 1;
            back_signal_r = 0;
        }else
        {
            forward_signal_l = 0;
            back_signal_l = 1;
            forward_signal_r = 0;
            back_signal_r = 1;
        }
        motor_l.period_us(10);
        motor_l.write(pwm_l);
        motor_r.period_us(10);
        motor_r.write(pwm_l);

        frag = 2;

    } else
    {
        v_l = (radius - HALF_THREAD) * data->angular.z;
        v_r = (radius + HALF_THREAD) * data->angular.z;
        rpm_control_l = (60 * v_l) * 100.0 / 2 * PI * WHELL_RADIUS;
        if(rpm_control_l >= MAXRPM)
        {
            rpm_control_l = MAXRPM;
        }
        else if(rpm_control_l <= -MAXRPM)
        {
            rpm_control_l = -MAXRPM;
        }
        rpm_control_r = (60 * v_r) * 100.0 / 2 * PI * WHELL_RADIUS;
        if(rpm_control_r >= MAXRPM)
        {
            rpm_control_r = MAXRPM;
        }
        else if(rpm_control_r <= -MAXRPM)
        {
            rpm_control_r = -MAXRPM;
        }
        pwm_l = rpm_control_l / MAXRPM;
        pwm_r = rpm_control_r / MAXRPM;
        // motor_l = pwm_l;
        // motor_r = pwm_r;
        if(radius > 0.0) //forward
        {
            forward_signal_l = 1;
            back_signal_l = 0;
            forward_signal_r = 0;
            back_signal_r = 1;
        }else
        {
            forward_signal_l = 0;
            back_signal_l = 1;
            forward_signal_r = 1;
            back_signal_r = 0;
        }
        motor_l.period_us(10);
        motor_l.write(pwm_l);
        motor_r.period_us(10);
        motor_r.write(pwm_r);

        frag = 3;
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
        printf("radius: %f\n", radius);
        printf("rpm_control_l: %f, rpm_control_r: %f\n", rpm_control_l, rpm_control_r);
        printf("v_l: %f, v_r: %f\n", v_l, v_r);
        //printf("pwm_l : %f, pwm_r : %f\n", pwm_l);
        printf("pwm_l : %f, pwm_r : %f\n", pwm_l,pwm_r);

        switch (frag)
        {
        case 1:
            printf("forward\n");
            break;

        case 2:
            printf("rotation\n");
            break;

        case 3:
            printf("curve\n");
            break;

        default:
            break;
        }

        //printf("cnt_l : %d\ncnt_r : %d\n",cnt_l,cnt_r);
        //printf("L : %d\nR : %d\n",rotation_direction_l.read(),rotation_direction_r.read());

        osDelay(100);
    }

    mros2::spin();
    return 0;
}

