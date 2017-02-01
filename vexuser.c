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
 * direction and channel
 */
static  vexMotorCfg mConfig[kVexMotorNum] = {
        { kVexMotor_1,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_2,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_4 },
        { kVexMotor_3,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_4,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_5,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_6,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_1},
        { kVexMotor_7,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_2},
        { kVexMotor_8,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_9,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorIME,         kImeChannel_3},
        { kVexMotor_10,     kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 }
};

/* @cond 
*/
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
/* @endcond 
*/




/*-----------------------------------------------------------------------------*/
/** @brief      armLiftSpeed                                                   */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function was formed to reduce the clutter in the functions that are
 *  moving the arm. Whenever you move the arm all of the motors have to move
 *  in the same direction at the same time. So it was simple to basically 
 *  re-define them as one motor using this function. 
 *
 *  @param armMotorSpeed This is the speed at which the motors are to run
 */
void 
armLiftSpeed(int armMotorSpeed)
{
    // All of the arm motors have to turn at the same speed sso
    // There are actually four motors on the arm, but the right
    // Side shares a split wire (same logic and power)
    vexMotorSet(LL1, -armMotorSpeed);
    vexMotorSet(LL3, -armMotorSpeed);
    vexMotorSet(RL1, -armMotorSpeed);
}

/* @cond
*/
int armTarget = 0;
int offSetCurrent = 0;
int offSetPrev = 0;
int offSetDerivative = 0;
int offSetintigral = 0;
/* @endcond
 */

/*-----------------------------------------------------------------------------*/
/** @brief      armSet                                                         */
/*-----------------------------------------------------------------------------*/
/** @details
 *  Even with the addition of the rubber bands to the robot arm it sometimes 
 *  gets jostled around. So the driver then has to fix this problem. This 
 *  tries to keep the arm in the same postion using a somewhat hacked together
 *  PID control system. This system could techincally be used for auton where
 *  you would pass it a target. However this target would be in encoder counts
 *  so the units would be a hassel to deal with as they have not been nor will
 *  will they be converted to something usible i.e angle, % up, etc.
 *
 *
 *  @param targetPositionArm This is the target for the arm to move to in some
 *  arbitary positions. 
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
    /** @code */
    if(abs(offTargetcurrent) > 10)
    {
        // This is for when the arm falls beneith its target
        // See the note from above
        if(vexMotorPositionGet(RL1) < targetPositionArm)
        {
            armLiftSpeed((-(offTargetcurrent) * 3) - (offSetDerivative * 3) - (offSetintigral * .001) );
        }


        // This is for when the arm moves above its target
        // See the note from above
        if(vexMotorPositionGet(RL1) > targetPositionArm)
        {
            armLiftSpeed((offTargetcurrent) * 3 - (offSetDerivative * 3) - (offSetintigral * .001));
        }

        // Need to let the brain think
        vexSleep( 25 );

        // Intergrate the error over time 
        // This should update roughly 40 times a second(ish)
        offSetintigral = offSetintigral + offSetCurrent;
    }
    /** @endcode */
    
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
        // Lift the Robot up
        if(vexControllerGet(Btn8L) == 1)
        {
            vexMotorSet(LiftMotor1, 120);
            vexMotorSet(LiftMotor2, 120);
        }

        // Move the robot down
        if(vexControllerGet(Btn8U) == 1)
        {
            vexMotorSet(LiftMotor1, -120);
            vexMotorSet(LiftMotor2, -120);
        }
    }

    // Don't forget to turn off the motors
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
    // Claw opens 
    if(vexControllerGet(Btn5D) == 1)
    {
        vexMotorSet(claw, 100);
    }

    // Claw closes
    if(vexControllerGet(Btn5U) == 1)
    {
        vexMotorSet(claw, -100);
    }

    // The claw is off
    if( (vexControllerGet(Btn5U) == 0) && (vexControllerGet(Btn5D) == 0))
    {
        vexMotorSet(claw, 0);
    }

 }





// This is an incriment counter to be able to determine what target armSet() should get
// Right now all this needs to do is intialize it once. 
// @cond 
int armLoops = 0;
int lastState = 0;
// @endcond
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

// @cond
int frontMotorDifference = 0;
bool loopAround = TRUE;
int loopsAround = 0;
int conversionConst = 14250;
int RBWPos = 0;
int LBWPos = 0;
int RFWPos = 0;
int LFWPos = 0;
// @endcond

/*-----------------------------------------------------------------------------*/
/** @brief      AutonBase                                                      */
/*-----------------------------------------------------------------------------*/
/** @details                                                                   */
/*  AutonBase takes a paramter of distance, in cm, and makes the robot drive   */
/* that far.                                                                   */
/*                                                                             */
/** @param distance This is the distance in cm that you want the robot to move
*   in a straight line
*  
*/
void
autonForward(int distance)
{
    vexMotorPositionSet(RBW, 0);
    vexMotorPositionSet(LBW, 0);
    vexMotorPositionSet(RFW, 0);
    vexMotorPositionSet(LFW, 0);
    
    // In order to gain three decimals of percision while still staying an int everything was multiplied by 1000 
    int RFWPos = (vexMotorPositionGet(RFW) * 1000);
                
    // This converts the distance in cm to encoder counts on the robot
    // Remember that conversionConst will change depending on the 
    // Motor, gear, and wheel.
    int ticksToGo = ((distance) * conversionConst);

    // This sets the inital motor speed to zero so that they can slowly ramp up when this function is called
    int motorSpeed = 0;

        // The motors are to turn while they still have not hit their target. Abs returns the absolute value of
        // RFWPos this is here because sometimes you will want to pass this function a negative distance and
        // in order for the logic to work 
        while( (abs(RFWPos)) < ticksToGo  ) {
            
            // This ramps up the motor from its' stall speed to whatever it needs to move to based on the math below
            // slowly so that it won't create an unnecessarly large current spike heating up the thermal fuse.
            // The assumption is that at motor speed 70 it is safe to move to whatever speed it wants to drive at
            if (motorSpeed <= 70)
            {
                motorSpeed = motorSpeed + 5;
            }
            
            // In order to gain three decimals of percision while still staying an int everything was multiplied by 1000 
            // I know that we already did this in this function, but because RFWPos does not reference itself when it is
            // Changed you need to remultiple it each time
            RFWPos = (vexMotorPositionGet(RFW) * 1000);
            
            // This calculates how far off the motors are from each other while they are moving forward
            // It's a + and not a - because the LFW is moving in the oposite direction of RFW based on the motor mounting
            frontMotorDifference = ( vexMotorPositionGet(RFW) + vexMotorPositionGet(LFW) );
            
            // Caps the error at 40 so that in case something goes wrong. The robot doesn't move
            // Too fast in an unpredictable manor.
            if (frontMotorDifference > 40)
            {
                frontMotorDifference = 40;
            }
            
            // Depending on how the motors are mounted the sign of the correction needs to be changed
            // The -10 is there because for some reason the right side skips foward a little bit
            // When the function first boots up. It really should have a dervative contol in here
            // To first that, but I don't really have time. Maybe later I'll move to PD here. Right 
            // now it will stay as just P 
            vexMotorSet(RFW, -motorSpeed - frontMotorDifference);  
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
        
        // Don't forget to turn of the motors when you are done moving
        vexMotorSet(RFW, 0);
        vexMotorSet(LFW, 0);
        vexMotorSet(RBW, 0);
        vexMotorSet(LBW, 0);

}



/*-----------------------------------------------------------------------------*/
/** @brief      AutonArm                                                       */
/*-----------------------------------------------------------------------------*/
/** @details                                                                   */
/*  First the arm has to move out to pop open the robot                        */
/* that far.                                                                   */
/*                                                                             */
/** @param option There are multiple functionalities of this function \\-_-// 
*   The first being a simple routine to get the claw of the bot to unstick
*   Second option is to move the arm up to where ever the second parameter wants
*   it to go to.
*  
*  @param target This is the target position that option two uses to make the 
*  arm of the robot to move to. 
*/
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
        // If the arm isn't at it's target then it's positon needs to be changed.
        // The way it is written right now the arm will always overshoot the
        // target by a little bit. It also will not take a negative position.
        while(vexMotorPositionGet(RL1) < target) {

            // Again more proportional control being used. The farther
            // Off the arm is from its target the faster it moves
            armLiftSpeed(target - RL1);

            // Sleep to allow the other threads to think
            vexSleep(25);
        }
    }
    
    // Don't forget to turn off the motors when your done
    armLiftSpeed(0);
}



// @cond
int gyroMotorDifference = 0;
int motorSpeedGryo = 0;
float currentAngle = 0.0;
// @endcond


/*-----------------------------------------------------------------------------*/
/** @brief      turnTo                                                         */
/*-----------------------------------------------------------------------------*/
/** @details
 *  To create a function to make the robot turn to a user defined angle. 
 *
 *  @param angle This is the angle in degrees that robot is to move to. Relative
 *  to it's current angle. So that you don't have to keep track of it all while
 *  creating auton.
 */



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
    int Kp = 1;
    int Kd = 1;
    int Ki = 1;

    if (angle > 0)
    {

        while (vexGyroGet() != degreeTarget)
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
            vexMotorSet(RFW, outPut);
            vexMotorSet(RBW, outPut);
            vexMotorSet(LFW, outPut);
            vexMotorSet(LBW, outPut);   
            
            // Don't hog the CPU
            vexSleep ( 25 ); 

            

            
        }

        // Make sure that motors turn off when it hits it's target
        /*
        vexMotorSet(RFW, 0);
        vexMotorSet(RBW, 0);
        vexMotorSet(LFW, 0);
        vexMotorSet(LBW, 0);  
        */
    }
    
    // Multiplied by ten because vexGryoGet() multplies teh angle by ten
    if (angle < 0)
    {
        targetTurnIntergral = 0;

        // 
        while (vexGyroGet() > degreeTarget)
        {
            gyroMotorDifference = (vexGyroGet() - degreeTarget);
            vexMotorSet(RFW, targetTurnIntergral);
            vexMotorSet(RBW, targetTurnIntergral);
            vexMotorSet(LFW, targetTurnIntergral);
            vexMotorSet(LBW, targetTurnIntergral);
            vexSleep ( 25 );

            targetTurnIntergral = (targetTurnIntergral + (gyroMotorDifference * .1));


        }

        // Make sure that the motors turn of when the target is hit. 
        // Introducing it this way will not allow for continous error blocking
        // But I am fairly certain that it is not needed. 
        vexMotorSet(RFW, 0);
        vexMotorSet(RBW, 0);
        vexMotorSet(LFW, 0);
        vexMotorSet(LBW, 0);
    }


}




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
            
    vexMotorSet(claw, 100);
    vexSleep(400);
    vexMotorSet(claw, 0);
    armLiftSpeed(100);
    vexSleep(1300);
    armLiftSpeed(0);
    vexSleep(600);
    //vexMotorSet(claw, 100);
    //vexSleep(100);
    vexMotorSet(claw, 0);
    autonForward(155);
    // THis is for programming skils
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
 * 
 *  @param *arg I believe its a pointer to something that convex uses behind this
 *  more exploration is needed to determine it's purpose
 */
msg_t vexOperator( void *arg )
{
    // Again this is from convex the compiler hates it though
    int16_t     blink = 0;

    // 
    (void)arg;

    // Must call this so that this task can be killed by convex during a period of vexSleep() 
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

            
            // There needs to be some intialization in autonForward so that I can call it more than once 
            
            if(vexControllerGet(Btn8D) == 1)
            {
                loopAround = TRUE;
                autonForward(100);
            }
            
            // This is just the normal base code
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
            if(vexControllerGet(Btn8R) == 1)
            {
                turnTo(360);
            }
            
            //liftControl();
            clawControl();

        
            // My auton debug button
            /*
            if(vexControllerGet(Btn8U) == 1)
            {
               
                vexMotorSet(claw, 100);
                vexSleep(500);
                vexMotorSet(claw, 0);
                armLiftSpeed(100);
                vexSleep(1300);
                armLiftSpeed(0);
                vexSleep(1000);
                vexMotorSet(claw, 100);
                vexSleep(100);
                vexMotorSet(claw, 0);
                autonForward(155);
                vexMotorSet(claw, -100);
                vexSleep(500);
            }
            */
            
            


            // User Arm Control will always work unless you press Btn6U
            // Then it will lock the arm into the last spot it was in "hopefully"
             userArmControl();

        // Don't hog cpu allow time for other threads
        vexSleep( 25 );
        }

    // This messes with vexTaskRegister. I don't know what though. 
    return (msg_t)0;
}
