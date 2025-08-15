#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <cmath>
#include <String>

using namespace std;

#define USMIN 500
#define USMAX 2500
#define SERVO_FREQ 50
#define pi 3.1415926

int hip_right_front = 0;
int hip_left_front = 1;
int hip_right_back = 2;
int hip_left_back = 3;

int upper_joint_right_front = 4;
int upper_joint_left_front = 5;
int upper_joint_right_back = 6;
int upper_joint_left_back = 7;

int lower_joint_right_front = 8;
int lower_joint_left_front = 9;
int lower_joint_right_back = 10;
int lower_joint_left_back = 11;
int angles[3];
float a = 5.5;
float b = 11;
float c = 13.84;

Adafruit_PWMServoDriver pwm;

void dog_inverse_kinematics(double xf, double yf, double zf)
{
    // inverse equations
    double h = sqrt(pow(yf, 2) + pow(zf, 2));
    double m = sqrt(pow(xf, 2) + pow(yf, 2) + pow(zf, 2));
    double l = sqrt(pow(m, 2) - pow(a, 2));
    double alpha = atan(-yf / zf);
    double beta = acos(a / h);
    angles[0] = (alpha + beta) * (180 / pi); // hip angle
    double theta1 = atan(-xf / zf);
    double theta2 = acos((pow(l, 2) + pow(b, 2) - pow(c, 2)) / (2 * l * b));
    angles[1] = (pi / 2 - theta1 - theta2) * (180 / pi);                              // upper joint angle
    angles[2] = acos((pow(b, 2) + pow(c, 2) - pow(l, 2)) / (2 * b * c)) * (180 / pi); // lower joint angle
}

void set_servo_angles(int leg)
{

    switch (leg)
    {
    case 0:
        // left front leg
        pwm.writeMicroseconds(hip_left_front, map(angles[0], 0, 180, USMIN, USMAX) - 600);
        pwm.writeMicroseconds(upper_joint_left_front, map(180 - angles[1], 0, 180, USMIN, USMAX) - 100);
        pwm.writeMicroseconds(lower_joint_left_front, map(180 - angles[2], 0, 180, USMIN, USMAX) - 500);
        break;

    case 1:
        // left back leg
        pwm.writeMicroseconds(hip_left_back, map(180 - angles[0], 0, 180, USMIN, USMAX) - 650);
        pwm.writeMicroseconds(upper_joint_left_back, map(180 - angles[1], 0, 180, USMIN, USMAX));
        pwm.writeMicroseconds(lower_joint_left_back, map(180 - angles[2], 0, 180, USMIN, USMAX) - 500);
        break;
    case 2:
        // right front leg
        pwm.writeMicroseconds(hip_right_front, map(angles[0], 0, 180, USMIN, USMAX) - 300);
        pwm.writeMicroseconds(upper_joint_right_front, map(angles[1], 0, 180, USMIN, USMAX));
        pwm.writeMicroseconds(lower_joint_right_front, map(angles[2], 0, 180, USMIN, USMAX) - 650);
        break;
    case 3:
        // right back leg
        pwm.writeMicroseconds(hip_right_back, map(180 - angles[0], 0, 180, USMIN, USMAX) - 750);
        pwm.writeMicroseconds(upper_joint_right_back, map(angles[1], 0, 180, USMIN, USMAX) + 50);
        pwm.writeMicroseconds(lower_joint_right_back, map(angles[2], 0, 180, USMIN, USMAX) - 650);
        break;
    }
}

void callibrate()
{
    dog_inverse_kinematics(-b, -a, c);
    for (int i = 0; i < 4; i++)
    {
        set_servo_angles(i);
    }
}

void stand()
{
    for (int i = 10; i <= 20; i++)
    {
        dog_inverse_kinematics(0, -a, i);
        for (int j = 0; j < 4; j++)
        {
            set_servo_angles(j);
        }
        delay(80);
    }
}

void sit()
{
    for (int i = 20; i >= 10; i--)
    {
        dog_inverse_kinematics(0, -a, i);
        for (int j = 0; j < 4; j++)
        {
            set_servo_angles(j);
        }
        delay(80);
    }
}

void crawl()
{
    for (int i = 3; i >= 0; i--)
    {
        for (int x = 0; x <= 8; x++)
        {
            double z = sqrt(16 - pow(x - 4, 2));
            dog_inverse_kinematics(x, -a, 20 - z);
            set_servo_angles(i);
            dog_inverse_kinematics(-x, -a, 20);

            if (i == 0)
            {
                set_servo_angles(1);
                set_servo_angles(2);
                set_servo_angles(3);
            }
            else if (i == 1)
            {
                set_servo_angles(0);
                set_servo_angles(2);
                set_servo_angles(3);
            }
            else if (i == 2)
            {
                set_servo_angles(0);
                set_servo_angles(1);
                set_servo_angles(3);
            }
            else if (i == 3)
            {
                set_servo_angles(0);
                set_servo_angles(1);
                set_servo_angles(2);
            }
            delay(100);
        }
    }
}

void trot()
{
    for (int i = 0; i <= 1; i++)
    {
        for (int x = 0; x <= 8; x++)
        {
            double z = sqrt(16 - pow(x - 4, 2));
            if (i == 0)
            {
                dog_inverse_kinematics(x, -a, 20 - z);
                set_servo_angles(0);
                set_servo_angles(3);
                dog_inverse_kinematics(-x, -a, 20);
                set_servo_angles(1);
                set_servo_angles(2);
            }
            else if (i == 1)
            {
                dog_inverse_kinematics(x, -a, 20 - z);
                set_servo_angles(1);
                set_servo_angles(2);
                dog_inverse_kinematics(-x, -a, 20);
                set_servo_angles(0);
                set_servo_angles(3);
            }
            delay(100);
        }
    }
}

void setup()
{
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.print("Enter a command:");
}

void loop()
{

    while (Serial.available() > 0)
    {
        String input = Serial.readString();
        if (input == "stand")
        {
            stand();
        }

        else if (input == "sit")
        {
            sit();
        }

        else if (input == "callibrate")
        {
            callibrate();
        }

        else if (input == "crawl")
        {
            crawl();
        }

        else if (input == "trot")
        {

            trot();
        }

        Serial.print("Enter a command:");
    }
}