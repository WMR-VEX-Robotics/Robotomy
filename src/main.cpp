#include "vex.h"
#include <string>
#include <math.h>

using namespace vex;
competition Competition;

//hello
brain Brain;
//Other devices:
controller Controller1 = controller(primary);
//Motor Devices
motor Intake = motor(PORT6, ratio18_1, true);
motor Conveyor = motor(PORT16, ratio18_1, true);
motor Hood = motor(PORT3, ratio18_1, true);
motor Front_Left = motor(PORT10, ratio18_1, true);
motor Front_Right = motor(PORT9, ratio18_1, false);
motor Back_Left = motor(PORT18, ratio18_1, true);
motor Back_Right = motor(PORT17, ratio18_1, false);
motor Left_PTO = motor(PORT20, ratio18_1, true);
motor Right_PTO = motor(PORT19, ratio18_1, false);
motor_group Left_3 = motor_group(Front_Left, Back_Left, Left_PTO);
motor_group Right_3 = motor_group(Front_Right, Back_Right, Right_PTO);
motor_group Left_2 = motor_group(Front_Left, Back_Left);
motor_group Right_2 = motor_group(Front_Right, Back_Right);
rotation right_encoder = rotation(PORT14);
rotation left_encoder = rotation(PORT15);

//Pneumatics Devices
pneumatics Clamp = pneumatics(Brain.ThreeWirePort.H);
pneumatics PTO = pneumatics(Brain.ThreeWirePort.G);
pneumatics Lift = pneumatics(Brain.ThreeWirePort.F);

double left_delta = 0;
double right_delta = 0;
double old_left = 0;
double old_right = 0;
double left_lift_angle = 0;
double right_lift_angle = 0;
int liftstage = 0;

bool liftchanged = false;
bool liftchanged2 = false;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_NO_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".
// A global instance of competition

//Left Motors:
motor_group(Front_Left, Back_Left, Left_PTO),

//Right Motors:
motor_group(Front_Right, Back_Right, Right_PTO),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT7,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
999,     -12,

//LB:      //RB: 
87,     -53,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT1,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT5,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
0

);


int current_auton_selection = 0;
bool auto_started = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  left_encoder.resetPosition();
  right_encoder.resetPosition();
  left_encoder.setPosition(0,degrees);
  right_encoder.setPosition(0,degrees);
  left_lift_angle = 0;
  right_lift_angle = 0;
  vexcodeInit();
  default_constants();

  while(!auto_started){
    //Brain.Screen.clearScreen();
    //Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    //Brain.Screen.printAt(5, 40, "Battery Percentage:");
    //Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    //Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    //Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    //Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        //Brain.Screen.printAt(5, 140, "Auton 1");
        break;
      case 1:
        //Brain.Screen.printAt(5, 140, "Auton 2");
        break;
      case 2:
        //Brain.Screen.printAt(5, 140, "Auton 3");
        break;
      case 3:
        //Brain.Screen.printAt(5, 140, "Auton 4");
        break;
      case 4:
        //Brain.Screen.printAt(5, 140, "Auton 5");
        break;
      case 5:
        //Brain.Screen.printAt(5, 140, "Auton 6");
        break;
      case 6:
        //Brain.Screen.printAt(5, 140, "Auton 7");
        break;
      case 7:
        //Brain.Screen.printAt(5, 140, "Auton 8");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){ 
    case 0:
      //drive_test();
      break;
    case 1:         
      drive_test();
      break;
    case 2:
      turn_test();
      break;
    case 3:
      swing_test();
      break;
    case 4:
      full_test();
      break;
    case 5:
      odom_test();
      break;
    case 6:
      tank_odom_test();
      break;
    case 7:
      holonomic_odom_test();
      break;
 }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

//Clamp
void switchclamp() 
{
  if (Clamp.value() == true) {
    Clamp.close();
  } else {
    Clamp.open();
  }
}

void raiselift() 
{
  Lift.close();
}

void lowerlift() 
{
  Lift.open();
}

void activatePTO()
{
  if (PTO.value() == true) {
    PTO.close();
    chassis.DriveL = Left_3;
    chassis.DriveR = Right_3;
  } else {
    PTO.open();
    chassis.DriveL = Left_2;
    chassis.DriveR = Right_2;
  }
}

double delta_helper(double val1,  double val2)
{
  if(val1>val2)
  {
    if(fabs(val1-val2) < fabs((360-val1)+val2))
    {
      return val1-val2;
    }
    else
    {
      return -((360-val1)+val2);
    }
  }
  else if(val2>=val1)
  {
    if(fabs(val2-val1) < fabs((360-val2)+val1))
    {
      return -(val2-val1);
    }
    else
    {
      return ((360-val2)+val1);
    }
  }
  return 0;
}

void calculate_change()
{
    left_delta = delta_helper(left_encoder.angle(),old_left);
    right_delta = delta_helper(right_encoder.angle(),old_right);
    old_left = left_encoder.angle();
    old_right = right_encoder.angle();
    left_lift_angle += left_delta/12;
    right_lift_angle += right_delta/12;
}

void raiseLift()
{
  if(liftstage<=1)
  {
    liftstage ++;
  }
}

void lowerLift()
{
  if(liftstage > 0)
  {
    liftstage --;
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  //Brain.Screen.print("Hi Carleigh");
  calculate_change();
  left_lift_angle = 0;
  right_lift_angle = 0;
  wait(100,msec);
  while (1) {
    calculate_change();
    Brain.Screen.print("Left Encoder: ");
    Brain.Screen.print(left_lift_angle);
    Brain.Screen.print("\n");
    Brain.Screen.print("Right Encoder: ");
    Brain.Screen.print(right_lift_angle);
    Brain.Screen.print("\n");
    wait(20,msec);
    Brain.Screen.clearLine();
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();
    Controller1.ButtonB.pressed(switchclamp);
    Controller1.ButtonA.pressed(activatePTO);
//Run intake inwards/outwards bound l1,r1
  if (Controller1.ButtonL1.pressing()){
    Conveyor.spin(forward, 100, percent);
    Intake.spin(reverse, 100, percent);
    Hood.spin(reverse, 100, percent);
  } else if (Controller1.ButtonR1.pressing()){
    Conveyor.spin(reverse, 100, percent);
    Intake.spin(forward, 100, percent);
    Hood.spin(forward, 100, percent);
  } else {
    Conveyor.stop(hold);
    Intake.stop(hold);
    Hood.stop(hold);
  }

//Raise/lower lift r2,l2
  if (PTO.value()==true&&Controller1.ButtonR2.pressing()){
    Left_PTO.spin(forward, 100, pct);
    Right_PTO.spin(forward, 100, pct);
    raiselift();
  } else if (PTO.value()==true&&Controller1.ButtonL2.pressing()){
    Left_PTO.spin(reverse, 100, pct);
    Right_PTO.spin(reverse, 100, pct);
    lowerlift();
  } else if(PTO.value()==true){
    Left_PTO.stop(hold);
    Right_PTO.stop(hold);
  }

  if(Controller1.ButtonUp.pressing()&&!liftchanged&&liftstage<2)
  {
    liftstage++;
    liftchanged = true;
  }
  else if(!Controller1.ButtonUp.pressing())
  {
    liftchanged = false;
  }
  
  if(Controller1.ButtonDown.pressing()&&!liftchanged2&&liftstage>0)
  {
    liftstage--;
    liftchanged2 = true;
  }
  else if(!Controller1.ButtonDown.pressing())
  {
    liftchanged2 = false;
  }

  Brain.Screen.print(liftstage);

  if(liftstage == 0&&PTO.value()==true)
  {
    while(fabs(left_lift_angle-0)>15)
    {
      calculate_change();
      Left_PTO.spin(forward,100,pct);
      Right_PTO.spin(forward,100,pct);
      Brain.Screen.print("Left Encoder: ");
      Brain.Screen.print(left_lift_angle);
      Brain.Screen.print("\n");
      Brain.Screen.print("Right Encoder: ");
      Brain.Screen.print(right_lift_angle);
      Brain.Screen.print("\n");
      wait(20,msec);
      Brain.Screen.clearLine();
    }
    
  }
  else if(liftstage == 2&&PTO.value()==true)
  {
    while(left_lift_angle>-115)
    {
      calculate_change();
      Left_PTO.spin(reverse,100,pct);
      Right_PTO.spin(reverse,100,pct);
      Brain.Screen.print("Left Encoder: ");
      Brain.Screen.print(left_lift_angle);
      Brain.Screen.print("\n");
      Brain.Screen.print("Right Encoder: ");
      Brain.Screen.print(right_lift_angle);
      Brain.Screen.print("\n");
      wait(20,msec);
      Brain.Screen.clearLine();
    }
  }
  else if(liftstage == 1&&PTO.value()==true)
  {
    if(left_lift_angle>-55)
    {
      while(left_lift_angle>-40)
      {
        calculate_change();
        Left_PTO.spin(reverse,100,pct);
        Right_PTO.spin(reverse,100,pct);
        Brain.Screen.print("Left Encoder: ");
        Brain.Screen.print(left_lift_angle);
        Brain.Screen.print("\n");
        Brain.Screen.print("Right Encoder: ");
        Brain.Screen.print(right_lift_angle);
        Brain.Screen.print("\n");
        wait(20,msec);
        Brain.Screen.clearLine();
      }
    }
    else if(left_lift_angle<-55)
    {
      while(left_lift_angle<-70)
      {
        calculate_change();
        Left_PTO.spin(forward,100,pct);
        Right_PTO.spin(forward,100,pct);
        Brain.Screen.print("Left Encoder: ");
        Brain.Screen.print(left_lift_angle);
        Brain.Screen.print("\n");
        Brain.Screen.print("Right Encoder: ");
        Brain.Screen.print(right_lift_angle);
        Brain.Screen.print("\n");
        wait(20,msec);
        Brain.Screen.clearLine();
      }
    }
  }

}
}



//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
