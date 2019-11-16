#include "hFramework.h"
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <Lego_Touch.h>
#include "ros.h"
#include "sensor_msgs/BatteryState.h"
#include <geometry_msgs/Twist.h>

using namespace hFramework;

// Engines
#define engine hMot1
#define steering hMot2

// Const
#define ENGINE_FORWARD -1
#define ENGINE_BACKWARD 1
#define STEERING_LEFT 1
#define STEERING_RIGHT -1

#define ENGINE_P_GAIN = 500
#define STEERING_P_GAIN = 1000
#define STEERING_MAX_ANGLE_RAD = 45 * 3.14/180.0 // 45 degrees

// Blocked steering turn
#define BLOCKED_NONE 0
#define BLOCKED_TURN_LEFT STEERING_LEFT
#define BLOCKED_TURN_RIGHT STEERING_RIGHT

// Definition core2 sensors & nodes
hLegoSensor_simple ls(hSens5);
hSensors::Lego_Touch sensor(ls);
sensor_msgs::BatteryState battery;

// Global variables
geometry_msgs::Twist cmd_vel;
int straightEncoderVal;
int leftMaxEncoderVal;
int rightMaxEncoderVal;

// ROS definitions
ros::NodeHandle nh;

// ROS communication
ros::Publisher *battery_pub;
ros::Publisher *cmd_vel_pub;
ros::Subscriber<geometry_msgs::Twist> *cmd_vel_sub;

void CmdVelSubscriber(const geometry_msgs::Twist& msg)
{
        cmd_vel = msg;
        cmd_vel_pub->publish(&cmd_vel);
}
void initBatteryPublisher()
{
        battery_pub = new ros::Publisher("/battery", &battery);
        nh.advertise(*battery_pub);
}
void initCmdVel()
{
        cmd_vel_pub = new ros::Publisher("/cmd_vel_echo", &cmd_vel);
        nh.advertise(*cmd_vel_pub);
        cmd_vel_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", CmdVelSubscriber);
        nh.subscribe(*cmd_vel_sub);
}

// Body
bool key_pressed(hSensors::Lego_Touch &device, int threshold=10)
{
        static int endurance = 0;
        if (device.isPressed())
                endurance++;
        else
                endurance = 0;

        if (endurance > threshold)
                return true;
        else
                return false;
}

int getAckermannEncoderReferenceValues()
{
    // Get data
    turn(STEERING_RIGHT, 500);
    sleep(2000*1000); // Wait minimum 2s until key press
    while(!key_pressed(sensor));
    rightMaxEncoderVal = steering.getEncoderCnt();

    turn(STEERING_LEFT, 500);
    sleep(2000*1000); // Wait minimum 2s until key press
    while(!key_pressed(sensor));
    leftMaxEncoderVal = steering.getEncoderCnt();

    // Calculate variables
    straightEncoderVal = (rightMaxEncoderVal + leftMaxEncoderVal) / 2
}

void turn_controller(int desiredAngle)
{
    // Get actual encoder value
    actualEncoderVal = steering.getEncoderCnt();

    // Calculate relative values
    relativeActualEncoderVal = actualEncoderVal - straightEncoderVal;
    relativeLeftMaxEncoderVal = leftMaxEncoderVal - straightEncoderVal;
    relativeRightMaxEncoderVal = rightMaxEncoderVal - straightEncoderVal;

    // Calculate actual angle
    if (abs(relativeActualEncoderVal - relativeLeftMaxEncoderVal) < abs(relativeActualEncoderVal - relativeRightMaxEncoderVal))
        actualAngle = (relativeActualEncoderVal / relativeLeftMaxEncoderVal) * STEERING_MAX_ANGLE_RAD * (1) // LEFT SIDE
    else
        actualAngle = (relativeActualEncoderVal / relativeRightMaxEncoderVal) * STEERING_MAX_ANGLE_RAD * (-1) // RIGHT SIDE

    // Calculate steering value and execute steering function
    steering_value = (desiredAngle - actualAngle) * STEERING_P_GAIN;
    if (steering_value > 0)
        turn(STEERING_LEFT, steering_value);
    else
        turn(STEERING_RIGHT, steering_value);
}

void turn(int turn_side, int power)
{
        static int blocked_turn = BLOCKED_NONE;
        static int can_block = true;

        /// INFO
        /*
        switch(blocked_turn)
        {
                case BLOCKED_NONE:
                        break;
                case BLOCKED_TURN_LEFT:
                        printf("Blocked left turn \r\n");
                        break;
                case BLOCKED_TURN_RIGHT:
                        printf("Blocked right turn \r\n");
                        break;
        }
        */

        /// BLOCKING TURN
        bool max_turn_reached = key_pressed(sensor);//sensor.isPressed();
        if(max_turn_reached)
        {
                if(turn_side == STEERING_LEFT && can_block)
                {
                        blocked_turn = STEERING_LEFT;
                }
                else if (turn_side == STEERING_RIGHT && can_block)
                {
                        blocked_turn = STEERING_RIGHT;
                }
                //printf("Max_turn_reached \r\n");
                can_block = false;
        }
        else
        {
                blocked_turn = BLOCKED_NONE;
                can_block = true;
        }

        /// STEERING
        if(blocked_turn != BLOCKED_NONE && turn_side == blocked_turn)
        {
                //printf("That action is blocked! \r\n");
                steering.setPower(0);
        }
        else
        {
                steering.setPower(turn_side*power);
                //if (abs(power) > 0)
                //      printf("Steering works! \r\n");
        }
}

void move(int power)
{
        engine.setPower(ENGINE_FORWARD*ENGINE_P_GAIN*power);
}

void hMain()
{
        Serial.init(9600, Parity::None, StopBits::One);
        nh.getHardware()->initWithDevice(&Serial);
        nh.initNode();
        initBatteryPublisher();
        initCmdVel();
        getAckermannEncoderReferenceValues()

        uint32_t t = sys.getRefTime();

        // int power;
        // int turn_side;

        while(true)
        {
                nh.spinOnce();

                // turn_side = 0;
                // power = 0;
                // if(hBtn2.isPressed())
                // {
                //         turn_side = STEERING_LEFT;
                //         power = 250;
                // }
                // if(hBtn1.isPressed())
                // {
                //         turn_side = STEERING_RIGHT;
                //         power = 250;
                // }
                // turn(turn_side, power);

                turn_controller(cmd_vel.angular.z);
                move(cmd_vel.linear.x);

                battery.voltage = sys.getSupplyVoltage();
                battery_pub->publish(&battery);
                sys.delaySync(t, 20);
        }
}
