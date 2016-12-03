/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) Derek Cheyne                           */
/*                                   2016                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     vexuser.c                                                    */
/*    Author:     Derek Cheyne                                                 */
/*    Created:    11 November 2016                                             */
/*                                                                             */
/*    Revisions:                                                               */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as PiCo                    */
/*    or electronic mail using dkcheyne399_at_gmail_com                        */
/*    Mentor for team 2581P Phoenix Robotics, Battle Creek, Mi.                */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include <stdlib.h>

#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"        // vex library header
#include "smartmotor.h"
#include "vexgyro.h"


// Digi IO configuration
static  vexDigiCfg  dConfig[kVexDigital_Num] = {
        { kVexDigital_1,    kVexSensorDigitalOutput, kVexConfigOutput,      0 },
        { kVexDigital_2,    kVexSensorDigitalOutput, kVexConfigOutput,      0 },
        { kVexDigital_3,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_4,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_5,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_6,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_7,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_8,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_9,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_10,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_11,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_12,   kVexSensorDigitalInput,  kVexConfigInput,       0 }
};


/*-----------------------------------------------------------------------------*/
/** @brief      Motor Config Array                                             */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This is where all of the motors are defined as in their type and sensor along
 s* direction and channel
 */
static  vexMotorCfg mConfig[kVexMotorNum] = {
        { kVexMotor_1,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_2,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_3,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_4,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_1 },
        { kVexMotor_5,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_2 },
        { kVexMotor_6,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_7,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_8,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_9,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_10,     kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 }
};


#define LL1    kVexMotor_6
#define LL2    kVexMotor_3
#define LL3    kVexMotor_6
#define RL1    kVexMotor_7
#define RL2    kVexMotor_9
#define RL3    kVexMotor_10
#define LiftMotors kVexMotor_2


/* Some more trash that was added while Drickle was away */
#define RBW    kVexMotor_1
#define LBW    kVexMotor_8
#define RFW    kVexMotor_4
#define LFW    kVexMotor_5
/*------------------------------------------------------*/


int conversionConst = 16250;
int RBWPos = 0;
int LBWPos = 0;
int RFWPos = 0;
int LFWPos = 0;

int frontMotorDifference = 0;
bool loopAround = TRUE;
int loopsAround = 0;

int gyroMotorDifference = 0;
int motorSpeedGryo = 0;
float currentAngle = 0.0;

int autonLoop = 0;


/*-----------------------------------------------------------------------------*/
/** @brief      User setup                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  The digital and motor ports can (should) be configured here.
 */
void
vexUserSetup()
{
    vexDigitalConfigure( dConfig, DIG_CONFIG_SIZE( dConfig ) );
    vexMotorConfigure( mConfig, MOT_CONFIG_SIZE( mConfig ) );
}


/*-----------------------------------------------------------------------------*/
/** @brief      userDriveFoward                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function controls the bass of the robot
 */
void UserDriveForward(float forward, float sideways) 
{
     vexMotorSet(RBW, (forward - sideways));
     vexMotorSet(RFW, (forward - sideways));
     vexMotorSet(LFW, (forward + sideways));
     vexMotorSet(LBW, (forward + sideways)); 
}



/*-----------------------------------------------------------------------------*/
/** @brief      liftControl                                                    */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function controls the lift that makes the robot move up or  
 *  down on the pole
 */
void 
liftControl(void)
{
    // First need to make sure whether or not the lift motors should be on
    if((vexControllerGet(Btn6U) == 1) || (vexControllerGet(Btn5D) == 1))
    {
        if(vexControllerGet(Btn6U) == 1)
        {
            vexMotorSet(LiftMotors, 80);
        }

        if(vexControllerGet(Btn5D) == 1)
        {
            vexMotorSet(LiftMotors, -80);
        }
    }

    else 
    {
        vexMotorSet(LiftMotors, 0);
    }
    
}



/*-----------------------------------------------------------------------------*/
/** @brief      userArmControl                                                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function controls the arm of the robot. By looking for ch 1 for speed 
 *  It only works when button 8d is held down.
 */
 void 
 userArmControl(void)
 {
    if (vexControllerGet(Ch2) < 0)
    {
    vexMotorSet(LL1, (vexControllerGet(Ch2) * .4));
    vexMotorSet(LL2, (vexControllerGet(Ch2) * .4));
    vexMotorSet(LL3, (vexControllerGet(Ch2) * .4));
    vexMotorSet(RL1, (vexControllerGet(Ch2) * .4));
    vexMotorSet(RL2, (vexControllerGet(Ch2) * .4));
    vexMotorSet(RL3, (vexControllerGet(Ch2) * .4));
    }
    else 
    {
        vexMotorSet(LL1, vexControllerGet(Ch2));
        vexMotorSet(LL2, vexControllerGet(Ch2));
        vexMotorSet(LL3, vexControllerGet(Ch2));
        vexMotorSet(RL1, vexControllerGet(Ch2));
        vexMotorSet(RL2, vexControllerGet(Ch2));
        vexMotorSet(RL3, vexControllerGet(Ch2));
    }
    if (vexControllerGet(Btn6D) == 1)
    {
        vexMotorSet(LL1, (vexControllerGet(Ch2) * .4));
        vexMotorSet(LL2, (vexControllerGet(Ch2) * .4));
        vexMotorSet(LL3, (vexControllerGet(Ch2) * .4));
        vexMotorSet(RL1, (vexControllerGet(Ch2) * .4));
        vexMotorSet(RL2, (vexControllerGet(Ch2) * .4));
        vexMotorSet(RL3, (vexControllerGet(Ch2) * .4));
    }
        
 }

/*-----------------------------------------------------------------------------*/
/** @brief      AutonBase                                                      */
/*-----------------------------------------------------------------------------*/
/** @details                                                                   */
/*  AutonBase takes a paramter of distance, in cm, and makes the robot drive   */
/* that far.                                                                   */


void
autonForward(int distance)
{
    vexMotorPositionSet(RBW, 0);
    vexMotorPositionSet(LBW, 0);
    vexMotorPositionSet(RFW, 0);
    vexMotorPositionSet(LFW, 0);
    

    int RFWPos = (vexMotorPositionGet(RFW) * 1000);
                

    int ticksToGo = ((distance) * conversionConst);


    int motorSpeed = 0;

    while(loopAround == TRUE)
    {
        loopsAround = loopsAround + 1;

        while( (abs(RFWPos)) < ticksToGo  ) 
        {
            if (motorSpeed <= 70)
            {
                motorSpeed = motorSpeed + 5;
            }
            

            RFWPos = (vexMotorPositionGet(RFW) * 1000);
     
            frontMotorDifference = ( vexMotorPositionGet(RFW) + vexMotorPositionGet(LFW) );
            
            
            if (frontMotorDifference > 40)
            {
                frontMotorDifference = 40;
            }
            

            vexMotorSet(RFW, motorSpeed);  
            vexMotorSet(LFW, motorSpeed - 10);
            vexMotorSet(RBW, motorSpeed);  
            vexMotorSet(LBW, motorSpeed - 10);
 
            // This should keep going from zero speed to full speed from being instant.
            vexSleep ( 100 );
        }

        vexMotorSet(RFW, 0);
        vexMotorSet(LFW, 0);
        vexMotorSet(RBW, 0);
        vexMotorSet(LBW, 0);
        RFWPos = (vexMotorPositionGet(RFW) * 1000);
        loopAround = FALSE;
        

        
    }
    
}

/*-----------------------------------------------------------------------------*/
/** @brief      AutonArm                                                       */
/*-----------------------------------------------------------------------------*/
/** @details                                                                   */
/*  First the arm has to move out to pop open the robot                        */
/* that far.                                                                   */
void 
autoArm(int option)
{
    //Pop arm out
    if(option == 1)
    {
        // Turns robot arm on so that the robot arms pops out of its crimped 
        // Up position
        vexMotorSet(LL1, 30);
        vexMotorSet(LL2, 30);
        vexMotorSet(LL3, 30);
        vexMotorSet(RL1, 30);
        vexMotorSet(RL2, 30);
        vexMotorSet(RL3, 30);

        // Sleep for .8 seconds 
        vexSleep(800);

        vexMotorSet(LL1, -20);
        vexMotorSet(LL2, -20);
        vexMotorSet(LL3, -20);
        vexMotorSet(RL1, -20);
        vexMotorSet(RL2, -20);
        vexMotorSet(RL3, -20);
        
        // Sleep for .4 seconds
        vexSleep(400);

    }
    // Turn off the robot arm once it waits .8 seconds
    vexMotorSet(LL1, 0);
    vexMotorSet(LL2, 0);
    vexMotorSet(LL3, 0);
    vexMotorSet(RL1, 0);
    vexMotorSet(RL2, 0);
    vexMotorSet(RL3, 0);

    // Turns robot arm to up position to knock off the stars
    if(option == 2)
    {
        vexMotorSet(LL1, 80);
        vexMotorSet(LL2, 80);
        vexMotorSet(LL3, 80);
        vexMotorSet(RL1, 80);
        vexMotorSet(RL2, 80);
        vexMotorSet(RL3, 80);
        
        // Move the robot arm to 
        vexSleep(1000);
    }
    vexMotorSet(LL1, 0);
    vexMotorSet(LL2, 0);
    vexMotorSet(LL3, 0);
    vexMotorSet(RL1, 0);
    vexMotorSet(RL2, 0);
    vexMotorSet(RL3, 0);
}


/*-----------------------------------------------------------------------------*/
/** @brief      turnTo                                                         */
/*-----------------------------------------------------------------------------*/
/** @details
 *  To create a function to make the robot turn to a user defined angle. 
 */


void turnTo(int angle)
{
    /**
    int degreeTarget;

    // Because we con't reset the gyro's like the ime's we need to take an intial measurement 
    // Of the angle so that we know where to move against
    currentAngle = vexGyroGet();

    // Angle is here so that we can use the if statements to determine what way the 
    // Robot should be turning
    degreeTarget = angle;

    // The function vexGyroGet returns the change is degrees multiplied by ten
    // So in order to compaire the values properly degreeTarget must be mulplied by ten
    // So must every other thing else when dealing with the angle
    degreeTarget = ((degreeTarget * 10) + currentAngle);



    // This makes sure that the robot always turns the most effcient way
    // (multiplied by ten see note above)
    if (angle > 0)
    {
        while (vexGyroGet() < degreeTarget)
        {
            gyroMotorDifference = ( degreeTarget - vexGyroGet()); // While turning this way the target will have a higher value 
                                                                 // Than the current value of vexGyroGet()

            vexMotorSet(RFW, gyroMotorDifference);
            vexMotorSet(RBW, gyroMotorDifference);
            vexMotorSet(LFW, -gyroMotorDifference);
            vexMotorSet(LBW, -gyroMotorDifference);   

            

            vexSleep ( 25 ); // Don't hog the CPU
        }

        vexMotorSet(RFW, 0);
        vexMotorSet(RBW, 0);
        vexMotorSet(LFW, 0);
        vexMotorSet(LBW, 0);  
    }
    
    // Multiplied by ten
    if (angle < 0)
    {
        while (vexGyroGet() > degreeTarget)
        {
            gyroMotorDifference = (vexGyroGet() - degreeTarget);

            vexMotorSet(RFW, -gyroMotorDifference);
            vexMotorSet(RBW, -gyroMotorDifference);
            vexMotorSet(LFW, gyroMotorDifference);
            vexMotorSet(LBW, gyroMotorDifference);

            vexSleep ( 25 );
        }
    }
    */

    const float conversionTime = 18;

    int timeSleep = abs(angle) * conversionTime;

    if (angle > 0)
        {
            vexMotorSet(RFW, -40);
            vexMotorSet(RBW, -40);
            vexMotorSet(LFW, 40);
            vexMotorSet(LBW, 40);
            vexSleep (timeSleep);
        }
    if (angle < 0)
    {
        vexMotorSet(RFW, -40);
        vexMotorSet(RBW, -40);
        vexMotorSet(LFW, 40);
        vexMotorSet(LBW, 40);
        vexSleep (timeSleep);
    }
        
        




}

/*-----------------------------------------------------------------------------*/
/** @brief      Autonomous                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the autonomous period is started
 */
msg_t
vexAutonomous( void *arg )
{
    (void)arg;

    // Must call this
    vexTaskRegister("auton");

    while(1)
        {
           
            autonLoop = autonLoop + 1;

            if(autonLoop == 1 )
            {
            autoArm(1);
            autoArm(2);
            autonForward(75);
            }
            

        // Don't hog cpu
        vexSleep( 25 );
        }

    return (msg_t)0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Driver control                                                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the driver control period is started
 */
msg_t
vexOperator( void *arg )
{
    int16_t     blink = 0;

    (void)arg;

    // Must call this
    vexTaskRegister("operator");

    // Run until asked to terminate
    while(!chThdShouldTerminate())
        {

            /*
            if(vexControllerGet(Btn5U) == 1)
            {
                autonBase(10);
            }
            */

            

            if(vexControllerGet(Btn8D) == 1)
            {
                loopAround = TRUE;
                autonForward(100);
            }
            
            // Makes 
            if ((loopAround == FALSE || loopsAround == 0) && (vexControllerGet(Btn5U) == 0) && (vexControllerGet(Btn5D) == 0))
            {
                 UserDriveForward(vexControllerGet(Ch3), vexControllerGet(Ch1));
            }
            
            // This makes the base move slowly 
            if ((loopAround == FALSE || loopsAround == 0) && (vexControllerGet(Btn5U) == 0) && (vexControllerGet(Btn5D) == 1))
            {
                UserDriveForward((.4 * vexControllerGet(Ch3)), (.4 * vexControllerGet(Ch1)));
            }

            if(vexControllerGet(Btn8R) == 1)
            {
                turnTo(360);
            }

            if(vexControllerGet(Btn6U) == 0)
            {
                userArmControl();
            }

            if(vexControllerGet(Btn8U) == 1)
            {
               
                turnTo(5);
                autonForward(80);
                autoArm(1);
                autoArm(2);
            }



            


            

            //liftControl();


           
            

           


        // Don't hog cpu
        vexSleep( 25 );
        }

    return (msg_t)0;
}
