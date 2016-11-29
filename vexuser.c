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
        { kVexMotor_1,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_2},
        { kVexMotor_2,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_3,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_4,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_3 },
        { kVexMotor_5,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_4},
        { kVexMotor_6,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_7,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_8,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_1 },
        { kVexMotor_9,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_10,     kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 }
};


#define LL1    kVexMotor_2
#define LL2    kVexMotor_3
#define LL3    kVexMotor_6
#define RL1    kVexMotor_7
#define RL2    kVexMotor_9
#define RL3    kVexMotor_10


/* Some more trash that was added while Drickle was away */
#define RBW    kVexMotor_1
#define LBW    kVexMotor_8
#define RFW    kVexMotor_4
#define LFW    kVexMotor_5
/*------------------------------------------------------*/


int conversionConst = 3000;
int RBWPos = 0;
int LBWPos = 0;
int RFWPos = 0;
int LFWPos = 0;

int frontMotorDifference = 0;
bool loopAround = TRUE;
int loopsAround = 0;



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
/** @brief      User initialize                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function is called after all setup is complete and communication has
 *  been established with the master processor.
 *  Start other tasks and initialize user variables here
 */
void
vexUserInit()
{

}



void UserDriveForward(void) 
{
     vexMotorSet(RBW, (vexControllerGet(Ch3) - vexControllerGet(Ch1)));
     vexMotorSet(RFW, (vexControllerGet(Ch3) - vexControllerGet(Ch1)));
     vexMotorSet(LFW, (vexControllerGet(Ch3) + vexControllerGet(Ch1)));
     vexMotorSet(LBW, (vexControllerGet(Ch3) + vexControllerGet(Ch1))); 
}

/*-----------------------------------------------------------------------------*/
/** @brief      AutonBase                                                      */
/*-----------------------------------------------------------------------------*/
/** @details
/*  AutonBase takes a paramter of distance, in cm, and makes the robot drive 
/* that far.
/* Rev. [Future] - Include angle as part of function
*/

void
autonBase(int distance)
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
     
            frontMotorDifference = ( (vexMotorPositionGet(RFW) + vexMotorPositionGet(LFW)) );
            
            
            if (frontMotorDifference > 40)
            {
                frontMotorDifference = 40;
            }
            

            vexMotorSet(RFW, (motorSpeed + frontMotorDifference)  );  
            vexMotorSet(LFW, (motorSpeed + frontMotorDifference) );
            vexMotorSet(RBW, (motorSpeed + frontMotorDifference)  );  
            vexMotorSet(LBW, (motorSpeed + frontMotorDifference) );
 
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
            autonBase(30);
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

            if(vexControllerGet(Btn5U) == 1)
            {
                vexMotorSet(LL1, 80);
                vexMotorSet(LL2, 80);
                vexMotorSet(LL3, 80);
                vexMotorSet(RL1, 80);
                vexMotorSet(RL2, 80);
                vexMotorSet(RL3, 80);
            }
            if((vexControllerGet(Btn5U) == 0) && (vexControllerGet(Btn6D) == 0))
            {
                vexMotorSet(LL1, 0);
                vexMotorSet(LL2, 0);
                vexMotorSet(LL3, 0);
                vexMotorSet(RL1, 0);
                vexMotorSet(RL2, 0);
                vexMotorSet(RL3, 0);
            }

            if(vexControllerGet(Btn6D) == 1)
            {
                vexMotorSet(LL1, -80);
                vexMotorSet(LL2, -80);
                vexMotorSet(LL3, -80);
                vexMotorSet(RL1, -80);
                vexMotorSet(RL2, -80);
                vexMotorSet(RL3, -80);
            }

            if(vexControllerGet(Btn8D) == 1)
            {
                loopAround = TRUE;
                autonBase(100);
            }

            if (loopAround == FALSE || loopsAround == 0)
            {
                 UserDriveForward();
            }
           

           


        // Don't hog cpu
        vexSleep( 25 );
        }

    return (msg_t)0;
}
