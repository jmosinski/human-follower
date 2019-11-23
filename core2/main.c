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
#define STEERING_LEFT -1
#define STEERING_RIGHT 1

#define ENGINE_P_GAIN 1500.0
#define STEERING_P_GAIN 2000.0
#define STEERING_MAX_ANGLE_RAD 30.0 * 3.14/180.0 // 30 degrees

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
bool DEBUG = false;

// ROS definitions
ros::NodeHandle nh;

// ROS communication
ros::Publisher *battery_pub;
ros::Publisher *cmd_vel_pub;
ros::Subscriber<geometry_msgs::Twist> *cmd_vel_sub;

void CmdVelSubscriber(const geometry_msgs::Twist& msg)
{
        // NEW
        cmd_vel.linear.x = msg.linear.x;
        cmd_vel.angular.z = msg.angular.z;

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
bool key_pressed(hSensors::Lego_Touch &device, int threshold=3)
{
        static int endurance = 0;
        if (device.isPressed())
                endurance++;
        else
                endurance = 0;

        if (endurance > threshold)
		{
			hLED1.on();
			return true;
		}
        else
		{
			hLED1.off();
			return false;
		}
                
}

void turn(int turn_side, int power)
{
        static int blocked_turn = BLOCKED_NONE;
        static int can_block = true;

        /// INFO
        if (DEBUG)
        {
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
        }
        /// BLOCKING TURN
        bool max_turn_reached = key_pressed(sensor);
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
                if (DEBUG)
                        printf("Max_turn_reached \r\n");
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
                if (DEBUG)
                        printf("That action is blocked! %d \r\n", blocked_turn);
                steering.setPower(0);
        }
        else
        {
                steering.setPower(turn_side*min(800, abs(power)));
                if (DEBUG)
                {
                        if (turn_side == STEERING_LEFT)
                                printf("Steering left! \r\n");
                        else if (turn_side == STEERING_RIGHT)
                                printf("Steering right! \r\n");
                }
        }
}

float turn_controller(float desiredAngle)
{
    // Get actual encoder value
    int actualEncoderVal = steering.getEncoderCnt();

    // Calculate relative values
    int relativeActualEncoderVal = actualEncoderVal - straightEncoderVal;
    int relativeLeftMaxEncoderVal = leftMaxEncoderVal - straightEncoderVal;
    int relativeRightMaxEncoderVal = rightMaxEncoderVal - straightEncoderVal;
    float actualAngle;

    // Calculate actual angle
    if (abs(relativeActualEncoderVal - relativeLeftMaxEncoderVal) < abs(relativeActualEncoderVal - relativeRightMaxEncoderVal))
        actualAngle = (float(relativeActualEncoderVal) / relativeLeftMaxEncoderVal) * STEERING_MAX_ANGLE_RAD * (1.0); // LEFT SIDE
    else
        actualAngle = (float(relativeActualEncoderVal) / relativeRightMaxEncoderVal) * STEERING_MAX_ANGLE_RAD * (-1.0); // RIGHT SIDE

    // Calculate steering value and execute steering function
    cmd_vel.angular.x = actualAngle;
    cmd_vel.angular.y = desiredAngle;
	
    int steering_value = (desiredAngle - actualAngle) * STEERING_P_GAIN;
    if (DEBUG)
    {
        printf("Actual angle %f \r\n", actualAngle);
        printf("Steering value %d \r\n", steering_value);
    }
    if (steering_value > 0)
        turn(STEERING_LEFT, steering_value);
    else
        turn(STEERING_RIGHT, steering_value);

    return actualAngle;
}

void move(float power)
{
        engine.setPower(min(ENGINE_FORWARD*ENGINE_P_GAIN*power, 500.0));
}


void getAckermannEncoderReferenceValues()
{
    // Get data
    turn(STEERING_RIGHT, 400);
    sys.delay(1000); // Wait minimum 1s until key press
	while(!key_pressed(sensor))
		turn(STEERING_RIGHT, 400);
	turn(STEERING_RIGHT, 400);
    rightMaxEncoderVal = steering.getEncoderCnt();

    turn(STEERING_LEFT, 400);
    sys.delay(1000); // Wait minimum 1s until key press
	while(!key_pressed(sensor))
		turn(STEERING_LEFT, 400);
	turn(STEERING_LEFT, 400);
    leftMaxEncoderVal = steering.getEncoderCnt();

    // Calculate variables
    straightEncoderVal = (rightMaxEncoderVal + leftMaxEncoderVal) / 2.0;

    // INFO
    if (DEBUG)
    {
            printf("Left %d \r\n", leftMaxEncoderVal);
            printf("Right %d \r\n", rightMaxEncoderVal);
            printf("Straight %d \r\n", straightEncoderVal);
            sys.delay(2000);
    }

    // Position to straight wheels
    while(abs(turn_controller(0)) > 0.1)
        sys.delay(10);
    steering.setPower(0);
}

void hMain()
{
        // 1. Increase baud rate to 57600
        // 2. Look what is happening in subscribers
        // 3. Maybe moving publishing to main loop have already fixed problem
        // 4. Try to turn battery publisher
        // 5. Add ros rate
        // 6. Try to measure times in main loop and subscribers frequency
        Serial.init(57600, Parity::None, StopBits::One);
        nh.getHardware()->initWithDevice(&Serial);
        nh.initNode();
        initBatteryPublisher();
        initCmdVel();
        sys.delay(2000);
        getAckermannEncoderReferenceValues();

        uint32_t t = sys.getRefTime();
        int loop_cnt = 0;

        while(true)
        {
                // DEBUG
                if (DEBUG)
                {
                    int turn_side = 0;
                    int power = 0;
                    if(hBtn2.isPressed())
                    {
                            turn_side = STEERING_LEFT;
                            power = 1000;
                    }
                    if(hBtn1.isPressed())
                    {
                            turn_side = STEERING_RIGHT;
                            power = 1000;
                    }
                    turn(turn_side, power);
                }

                // MAIN LOOP
                nh.spinOnce();
                turn_controller(cmd_vel.angular.z);
                move(cmd_vel.linear.x);
                if((loop_cnt % 100) == 0)
                {
                        battery.voltage = sys.getSupplyVoltage();
                        battery_pub->publish(&battery);
						cmd_vel_pub->publish(&cmd_vel);
                }
                sys.delaySync(t, 10);
                loop_cnt++;
        }
}

