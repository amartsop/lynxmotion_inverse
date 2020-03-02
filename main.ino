

// #include "Arduino.h"
// #include "./include/lynxmotion.h"

// // #define servoNum 6

// // Robot instance
// LynxMotion robot;

// // Joint space vector
// int jointVec[servoNum] = {0};


// void setup()
// {
//     robot.init();
//     Serial.begin(9600);
// }



// void loop()
// {
//     /******************** Forward Kinematics ******************/


//     //Joystick
//     jointVec[robot.jointIndices.joint1] = 0;
//     jointVec[robot.jointIndices.joint2] = 80;
//     jointVec[robot.jointIndices.joint3] = 0;
//     jointVec[robot.jointIndices.joint4] = 0;
//     jointVec[robot.jointIndices.joint5] = 8000;


//     // Forward Kinematics
//     robot.forwardKinematics(jointVec);




//     delay(50);


//     // Serial.println(angle);

    

// }

#include "Arduino.h"
#include "./include/lynxmotion.h"
#include "BasicLinearAlgebra.h"


// Robot instance
LynxMotion robot;

// Joint space vector
int jointVec[servoNum] = {0};


void setup()
{
    robot.init();
    Serial.begin(9600);
}

int count = 0;

void loop()
{
    /******************** Inverse Kinematics ******************/
    // Joystick
    
    if (count > 7)
    {
        count = 0;
        delay(3000);
    }

    // Desired position
    BLA::Matrix<3, 8> trajectory = 
    {
        0.05, 0.13, 0.13, 0.05, 0.13, 0.13, 0.13, 0.05,
        -0.1, -0.1, 0.15, 0.15, -0.1, -0.1, 0.15, 0.15,
        0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.2
    };
    

    BLA::Matrix<3, 1> dFeF0 = {trajectory(0, count), 
        trajectory(1, count), trajectory(2, count)};
    
    // Desired orientation
    BLA::Matrix<3, 3> RFeF0{1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, -1.0};

    robot.inverseKinematics(dFeF0, RFeF0);
    
    count += 1;

    delay(1000);
}

