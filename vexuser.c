/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     vexuser.c                                                    */
/*    Author:     Derek Cheyne                                                 */
/*    Created:    11 November 2016                                             */
/*    Vex Forum:  PiCo                                                         */
/*    Email:      dkcheyne399@gmail.com                                        */
/*-----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>

#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"        // vex library headerf
#include "smartmotor.h"
#include "vexgyro.h"
#include "main.h"
#include "chprintf.h"


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
        { kVexMotor_4,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_5,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_6,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_2},
        { kVexMotor_7,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_1},
        { kVexMotor_8,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_9,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_10,     kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 }
};


#define LL1    kVexMotor_9
#define LL3    kVexMotor_4
#define RL1    kVexMotor_5


#define LiftMotor1 kVexMotor_1
#define LiftMotor2 kVexMotor_10

#define claw kVexMotor_2


/* Some more trash that was added while Drickle was away */
#define RBW    kVexMotor_8
#define LBW    kVexMotor_3
#define RFW    kVexMotor_7
#define LFW    kVexMotor_6
/*------------------------------------------------------*/




/*-----------------------------------------------------------------------------*/
/** @brief      armLiftSpeed                                                   */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function was formed to reduce the clutter in the functions that are
 *  moving the arm. Whenever you move the arm all of the motors have to move
 *  in the same direction at the same time. So it was simple to basically 
 *  re-define them as one motor using this function. 
 */
void 
armLiftSpeed(int armMotorSpeed)
{
    vexMotorSet(LL1, -armMotorSpeed);
    vexMotorSet(LL3, -armMotorSpeed);
    vexMotorSet(RL1, -armMotorSpeed);
}


int armTarget = 0;
int offSetCurrent = 0;
int offSetPrev = 0;
int offSetDerivative = 0;


/*-----------------------------------------------------------------------------*/
/** @brief      armSet                                                         */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Even with the addition of the rubber bands to the robot arm it sometimes 
 *  gets jostled around. So the driver then has to fix this problem. This 
 *  tries to keep the arm in the same postion using a somewhat hacked together
 *  PID control system.  
 */
void armSet(int targetPositionArm) 
{

    int offSetPrev = offSetCurrent;

    // Calculate how far off the arm is from where it should be
    // RL1 is just whatever motor has the encoder on it
    int offTargetcurrent = (targetPositionArm - vexMotorPositionGet(RL1));

    int offSetDerivative = (offSetCurrent - offSetPrev);

    // Making the arm stay on target
    // This could be a problem because it has the ablitly to block out the other
    // Code that is going on. If that happens and I can't find a way to fix it
    // I'll have to create a new task for it :/
    if(abs(offTargetcurrent) > 10)
    {
        // This is for when the arm falls beneith its target
        // See the note from above
        if(vexMotorPositionGet(RL1) < targetPositionArm)
        {
            armLiftSpeed((-(offTargetcurrent) * 3) - (offSetDerivative * 3));
        }


        // This is for when the arm moves above its target
        // See the note from above
        if(vexMotorPositionGet(RL1) > targetPositionArm)
        {
            armLiftSpeed((offTargetcurrent) * 3 - (offSetDerivative * 3));
        }

        // Need to let the brain think
        vexSleep( 25 );

        // Intergrate the error over time 
        // This should update roughly 40 times a second(ish)


    }
    
}


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

void vexUserInit()
{
vexGyroInit(kVexAnalog_1);
}

/*-----------------------------------------------------------------------------*/
/** @brief      userDriveFoward                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function controls the bass of the robot
 */
void UserDriveForward(float sideways, float forward) 
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
    if((vexControllerGet(Btn8L) == 1) || (vexControllerGet(Btn8U) == 1))
    {
        if(vexControllerGet(Btn8L) == 1)
        {
            vexMotorSet(LiftMotor1, 120);
            vexMotorSet(LiftMotor2, 120);
        }

        if(vexControllerGet(Btn8U) == 1)
        {
            vexMotorSet(LiftMotor1, -120);
            vexMotorSet(LiftMotor2, -120);
        }
    }

    else 
    {
        vexMotorSet(LiftMotor1, 0);
        vexMotorSet(LiftMotor2, 0);
    }
    
}



/*-----------------------------------------------------------------------------*/
/** @brief      clawControl                                                    */
/*-----------------------------------------------------------------------------*/
/** @details
 *  To create a function to run the claw on the robot
 */
 void clawControl(void)
 {

    // Closes the Claw
    if(vexControllerGet(Btn5D) == 1)
    {
        vexMotorSet(claw, 100);
    }

    if(vexControllerGet(Btn5U) == 1)
    {
        vexMotorSet(claw, -100);
    }

    if( (vexControllerGet(Btn5U) == 0) && (vexControllerGet(Btn5D) == 0))
    {
        vexMotorSet(claw, 0);
    }

 }





// This is an incriment counter to be able to determine what target armSet() should get
// Right now all this needs to do is intialize it once. 
int armLoops = 0;
int lastState = 0;
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

    // This is for arm locking I only want it to pass a value to the loop once
    if ((vexControllerGet(Btn7L) != lastState) && (vexControllerGet(Btn7L) == 1))
    {
        armLoops = armLoops + 1;
        if(armLoops == 1)
        {
            // Need to Figure out how to pass this the initial position at which the button was pressed
             armSet(vexMotorPositionGet(RL1));
        }
        int lastState = (vexControllerGet(Btn7L));
    }

    // This is where the rest of the options are for armControl
    // The if line below is to make sure that these options don't overide armSet()
    if (vexControllerGet(Btn7L) == 0)
    {
        armLoops = 0;
         // This makes sure that you can't slam the arm down into the ground too fast
        if (vexControllerGet(Ch2) > 0)
        {

            armLiftSpeed( (vexControllerGet(Ch2) * .4));
        }

        // If your going up it's ok to go full speed
        else 
        {
            armLiftSpeed(vexControllerGet(Ch2) );  
        }

        // This allows for fine control
        if (vexControllerGet(Btn6D) == 1)
        {
            armLiftSpeed( (vexControllerGet(Ch2) * .4));
        }
    }

}












int frontMotorDifference = 0;
bool loopAround = TRUE;
int loopsAround = 0;
int conversionConst = 14250;
int RBWPos = 0;
int LBWPos = 0;
int RFWPos = 0;
int LFWPos = 0;

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

    //while(loopAround == TRUE)
    //{
      //  loopsAround = loopsAround + 1;
    bool forward;

    if( distance > 0);
    {
        forward = true;
    }

    if(distance < 0);
    {
        forward = false;
    }

        while( (abs(RFWPos)) < ticksToGo && distance == true) 
        {
            // The robot wants to move backwards
            if (motorSpeed <= 70 && distance > 0)
            {
                motorSpeed = motorSpeed + 5;
            }

            // The robot wants to move backwards
            if(motorSpeed > -70 && distance < 0)
            {
                motorSpeed = motorSpeed - 5;
            }


            

            RFWPos = (vexMotorPositionGet(RFW) * 1000);
     
             frontMotorDifference = 0; //( vexMotorPositionGet(RFW) - vexMotorPositionGet(LFW) );
            
            
            if (frontMotorDifference > 40)
            {
                frontMotorDifference = 40;
            }
            

            vexMotorSet(RFW,  -motorSpeed - frontMotorDifference);  
            vexMotorSet(LFW,  motorSpeed - 10 + frontMotorDifference);
            vexMotorSet(RBW, -motorSpeed - frontMotorDifference);  
            vexMotorSet(LBW,  motorSpeed - 10 + frontMotorDifference);
 
            // This should keep going from zero speed to full speed from being instant.
            vexSleep ( 25 );
            
            //if((vexControllerGet(Btn5U) == 1) && (vexControllerGet(Btn6U) == 1))
            //{
            //    break;
            //}

        }

        vexMotorSet(RFW, 0);
        vexMotorSet(LFW, 0);
        vexMotorSet(RBW, 0);
        vexMotorSet(LBW, 0);
        RFWPos = (vexMotorPositionGet(RFW) * 1000);
        loopAround = FALSE;

        vexSleep(25);
        
        while( ((RFWPos)) > ticksToGo && distance == false) 
        {
            // The robot wants to move backwards
            if (motorSpeed <= 70 && distance > 0)
            {
                motorSpeed = motorSpeed + 5;
            }

            // The robot wants to move backwards
            if(motorSpeed > -70 && distance < 0)
            {
                motorSpeed = motorSpeed - 5;
            }


            

            RFWPos = (vexMotorPositionGet(RFW) * 1000);
     
             frontMotorDifference = 0; //( vexMotorPositionGet(RFW) - vexMotorPositionGet(LFW) );
            
            
            if (frontMotorDifference > 40)
            {
                frontMotorDifference = 40;
            }
            

            vexMotorSet(RFW,  -motorSpeed - frontMotorDifference);  
            vexMotorSet(LFW,  motorSpeed - 10 + frontMotorDifference);
            vexMotorSet(RBW, -motorSpeed - frontMotorDifference);  
            vexMotorSet(LBW,  motorSpeed - 10 + frontMotorDifference);
 
            // This should keep going from zero speed to full speed from being instant.
            vexSleep ( 25 );
            
            //if((vexControllerGet(Btn5U) == 1) && (vexControllerGet(Btn6U) == 1))
            //{
            //    break;
            //}

        }

        vexMotorSet(RFW, 0);
        vexMotorSet(LFW, 0);
        vexMotorSet(RBW, 0);
        vexMotorSet(LBW, 0);
        RFWPos = (vexMotorPositionGet(RFW) * 1000);
        loopAround = FALSE;

        vexSleep(25);
        

        
    //}
    
}


/*-----------------------------------------------------------------------------*/
/** @brief      AutonArm                                                       */
/*-----------------------------------------------------------------------------*/
/** @details                                                                   */
/*  First the arm has to move out to pop open the robot                        */
/* that far.                                                                   */
void 
autoArm(int option, int target)
{
    //Pop arm out
    if(option == 1)
    {
        // Turns robot arm on so that the robot arms pops out of its crimped 
        // Up position
        armLiftSpeed(30);

        // Sleep for .8 seconds 
        vexSleep(800);

        armLiftSpeed(-20);
        
        // Sleep for .4 seconds
        vexSleep(400);

    }
    // Turn off the robot arm once it waits .8 seconds
    armLiftSpeed(0);

    // Turns robot arm to up position to knock off the stars
    if(option == 2)
    {
        while(vexMotorPositionGet(RL1) < target)
        {
            armLiftSpeed(target - RL1);

            //Sleep for a teenie tiny bit
            vexSleep(25);
        }
    }
    
    armLiftSpeed(0);
}




int gyroMotorDifference = 0;
int motorSpeedGryo = 0;
float currentAngle = 0.0;


/*-----------------------------------------------------------------------------*/
/** @brief      turnTo                                                         */
/*-----------------------------------------------------------------------------*/
/** @details
 *  To create a function to make the robot turn to a user defined angle. 
 */


void turnTo(int angle)
{
    
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
    
    // It was found that the current propotional control stops the robot just before it reaches it's target 
    // Keeping the motors on and locking up control of the robot. 
    // This is obvously undesirbiable so I added a intergral component to the control
    // Making it a PI control system
    int targetTurnIntergral = 0;
    int oldGyroMotorDifferent = 0;
    int newGyroMotorDifference = 0;
    int outPut = 0;
    int turnToDerivative = 0;

    // Tuning Varibles
    // See Eng Note Book for results its a result of Zieger-Nichols Methods 
    int Kp = 2.4;
    int Kd = .35;
    int Ki = .0875;

    if (angle > 0)
    {
        int loopTimer = 0;

        while ((vexGyroGet() != degreeTarget || (loopTimer < 200)))
        {
            // This is the start of the PID controller. I probably will only go with
            // a PD control for this function, but I may go with the whole blown thing
            // If you're not sure what this means please google this because it is
            // A Rather Important topic to cover

            // Swapping Values around to save the old one
            oldGyroMotorDifferent = newGyroMotorDifference;

            // Grabbing the new error
            newGyroMotorDifference = ( (degreeTarget / 10) - (vexGyroGet()/ 10) );

            // Accumulating the Error
            targetTurnIntergral = (targetTurnIntergral + (gyroMotorDifference * .025));

            // Finding the dervative
            turnToDerivative = ((newGyroMotorDifference - oldGyroMotorDifferent) / .025);

            // Combining them all together
            outPut = ( (Kp * newGyroMotorDifference) + (Ki * targetTurnIntergral) + (Kd * turnToDerivative) );
                                                                 
            // Than the current value of vexGyroGet()
            vexMotorSet(RFW, -outPut);
            vexMotorSet(RBW, -outPut);
            vexMotorSet(LFW, -outPut);
            vexMotorSet(LBW, -outPut);   

            //Incrimenting the timer
            loopTimer = loopTimer + 1;

            if(loopTimer > 200)
            {
                break;
            }

            
            // Don't hog the CPU
            vexSleep ( 25 );       
        }

        // Make sure that motors turn off when it hits it's target
        
        vexMotorSet(RFW, 0);
        vexMotorSet(RBW, 0);
        vexMotorSet(LFW, 0);
        vexMotorSet(LBW, 0);  
        
    }
    
}


int autonLoop = 0;
/*-----------------------------------------------------------------------------*/
/** @brief      Autonomous                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
*  This thread is started when the autonomous period is started
*/
 msg_t vexAutonomous( void *arg )
 {
     // (void)arg;
 
     // Must call this
     vexTaskRegister("auton");

    armLiftSpeed(100);
    vexSleep(200);
    armLiftSpeed(-120);
    vexSleep(600);
    armLiftSpeed(0);
    vexMotorSet(claw, 100);
    vexSleep(200);
    vexMotorSet(claw, 0);
    armLiftSpeed(100);
    vexSleep(250);
    armLiftSpeed(-100);
    vexSleep(250);
    armLiftSpeed(100);
    vexSleep(50);
    vexMotorSet(claw, 120);
    vexSleep(350);
    vexMotorSet(claw, 0);
    vexSleep(1110);

    //vexMotorSet(claw, 100);
    //vexSleep(100);
    /*
    vexMotorSet(claw, 0);
    autonForward(165);
    
    autonForward(-85);
    armLiftSpeed(-60);
    vexSleep(500);
    vexMotorSet(claw, -100);
    vexSleep(450);
    vexMotorSet(claw, 0);
    armLiftSpeed(0);
    turnTo(90);
    autonForward(100);
    vexMotorSet(claw, 100);
    armLiftSpeed(100);
    vexSleep(1000);
    turnTo(-90);
    autonForward(100);
    vexMotorSet(claw, -100);



    /*
    autonForward(-155);
    vexMotorSet(claw, -100);
    vexSleep(250);
    autonForward(155);
    vexMotorSet(claw, 100);
    autonForward(-155);
    vexMotorSet(claw, -100);
    vexSleep(250);
    autonForward(155);
    vexMotorSet(claw, 100);
    autonForward(-155);
    vexMotorSet(claw, -100);
    vexSleep(250);
    autonForward(155);
    vexMotorSet(claw, 100);
    autonForward(-155);
    vexMotorSet(claw, -100);
    vexSleep(250);
    autonForward(155);
    vexMotorSet(claw, 100);
    */
    
 
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
            

            /*
            if(vexControllerGet(Btn8D) == 1)
            {
                loopAround = TRUE;
                autonForward(100);
            }
            */
            // This is jus the normal base code
            if ((loopAround == FALSE || loopsAround == 0) && (vexControllerGet(Btn7U) == 0))
            {
                 UserDriveForward(vexControllerGet(Ch3), vexControllerGet(Ch1));
            }
            
            // This makes the base move slowly 
            if ((loopAround == FALSE || loopsAround == 0) && (vexControllerGet(Btn7U) == 1))
            {
                UserDriveForward((.4 * vexControllerGet(Ch3)), (.4 * vexControllerGet(Ch1)));
            }


            // Debug gryo code function
            // Need to get rid of this before competition
            /*
            if(vexControllerGet(Btn8R) == 1)
            {
                turnTo(360);
            }
            */
            
            liftControl();
            clawControl();

        
            // My auton debug button
            /*
             if(vexControllerGet(Btn8U) == 1)
            {
                armLiftSpeed(-60);
                vexSleep(400);
                armLiftSpeed(0);
                vexMotorSet(claw, 100);
                vexSleep(200);
                vexMotorSet(claw, 0);
                armLiftSpeed(100);
                vexSleep(250);
                armLiftSpeed(-100);
                vexSleep(250);
                armLiftSpeed(100);
                vexSleep(50);
                vexMotorSet(claw, 120);
                vexSleep(350);
                vexMotorSet(claw, 0);
                vexSleep(1110);
                //vexMotorSet(claw, 100);
                //vexSleep(100);
                vexMotorSet(claw, 0);
                autonForward(155);
    
                autonForward(-85);
                vexSleep(800);
                armLiftSpeed(-60);
                vexSleep(500);
                vexMotorSet(claw, -100);
                vexSleep(450);
                vexMotorSet(claw, 0);
                armLiftSpeed(0);
                turnTo(90);
                autonForward(100);
                vexMotorSet(claw, 100);
                armLiftSpeed(100);
                vexSleep(1000);
                turnTo(-90);
                autonForward(100);
                vexMotorSet(claw, -100);

            }
            */
            
            
            
            
            
            


            // User Arm Control will always work unless you press Btn6U
            // Then it will lock the arm into the last spot it was in "hopefully"
             userArmControl();

        // Don't hog cpu
        vexSleep( 25 );
        }


    return (msg_t)0;
}
