
#include "Arduino.h"
#include "./include/lynxmotion.h"
#include "BasicLinearAlgebra.h"
#include "./include/signal_processing.hpp"


// Robot instance
LynxMotion robot;

// Joint space vector
int jointVec[servoNum] = {0};

// Limits
const float upperCartesianLimits[3] = {0.16, 0.1, 0.15};
const float lowerCartesianLimits[3] = {0.07, -0.1, 0.0};

// Initial cartesian values
float xe = 0.1;
float ye = 0.1;
float ze = 0.0;


float count =-0.2;

void setup()
{
    robot.init();
    Serial.begin(9600);
}


void loop()
{

    ze = count;
    
    
    xe = SignalProcessing<float>::clipping(xe, lowerCartesianLimits[0],
        upperCartesianLimits[0]);

    ye = SignalProcessing<float>::clipping(ye, lowerCartesianLimits[1],
        upperCartesianLimits[1]);

    ze = SignalProcessing<float>::clipping(ze, lowerCartesianLimits[2],
    upperCartesianLimits[2]); 
    
    BLA::Matrix<3, 1> dFeF0 {xe, ye, ze} ;

    // Desired orientation
    BLA::Matrix<3, 3> RFeF0{1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, -1.0};

    robot.inverseKinematics(dFeF0, RFeF0);
    
    count = count + 0.001;

    delay(40);
}

