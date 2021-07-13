#include <iostream>
#include <sstream>
#include <map>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Display.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 16
using namespace webots;

// define some global variables

static int lastError = 0;
static int I = 0;
static double leftSpeed;
static double rightSpeed;
static int turnTime;

// ------------------------------------------------------------------------------------------------

// limit the velocity of the wheels
double limit(double &val){
  if (val > 15){
    return 15;
  }else if(val < -15){
    return -15;
  }else{
    return val;
  }
}
// ------------------------------------------------------------------------------------------------

// line following function
void lineFollow(DistanceSensor* ir[],double &kp,double &kd,double &ki,double &baseSpeed){
  int error = 0; 
  for (int i=0;i<8;i++){
    if (ir[i]->getValue()<500){
      if (i<4){
        error += i+1;
      }else{
        error += -(i-3);
      }
    }
  }
  int P = error;
  int D = error - lastError;
  I = I + error;
  lastError = error;
  if (error == 0){
    I = 0;
  }
  double correction = kd*D + kp*P + ki*I;  
  leftSpeed = baseSpeed - correction;      leftSpeed = limit(leftSpeed);
  rightSpeed = baseSpeed + correction;     rightSpeed = limit(rightSpeed);
  return;
}
// ------------------------------------------------------------------------------------------------

// wallfollowing function
void wallFollow(int &ds,double &kp,double &kd,double &ki,double &baseSpeed, int direction){
  double error = (ds - 650)/100;   
  int P = error;
  int D = error - lastError;
  I = I + error;
  lastError = error;
  if (error == 0){
    I = 0;
  }
  double correction = kd*D + kp*P + ki*I;
  if (direction){
    correction = -correction;
  }
  leftSpeed = baseSpeed + correction;      leftSpeed = limit(leftSpeed);
  rightSpeed = baseSpeed - correction;     rightSpeed = limit(rightSpeed);
  return;
}
// ------------------------------------------------------------------------------------------------

// left turning function
void turnLeft(){
  turnTime--;
  leftSpeed = 0;
  rightSpeed = 15;
  return;
}
// ------------------------------------------------------------------------------------------------

// Right turning function
void turnRight(){
  turnTime--;
  leftSpeed = 15;
  rightSpeed = 0;
  return;
}
// ------------------------------------------------------------------------------------------------

// Back turning function
void turnBack(){
  turnTime--;
  leftSpeed = -10;
  rightSpeed = 10;
  return;
}
// ------------------------------------------------------------------------------------------------

// Go forward without turning function
void noTurn(){
  turnTime = turnTime-2;
  leftSpeed  = 10;
  rightSpeed = 10;
  return;
}
// ------------------------------------------------------------------------------------------------

int main(int argc, char **argv) {
  
  Robot *robot = new Robot();
  
  // linefollowing ir sensors
  DistanceSensor *ir[8];
  char irNames[8][5] = {"ir1", "ir2","ir3","ir4","ir5","ir6","ir7","ir8"};
  for (int i = 0; i < 8; i++) {
    ir[i] = robot->getDistanceSensor(irNames[i]);
    ir[i]->enable(TIME_STEP);
  }
  
  // wallfollowing distance sesors
  DistanceSensor *ds[3];
  char dsNames[3][10] = {"ir_left","ir_front","ir_right"};
  for (int i = 0; i<3; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  
  // junction detecting ir sensors
  DistanceSensor *bottom[4];
  char bottomNames[4][15] = {"bottomLeft","bottomFront","bottomRight","bottomBack"};
  for (int i = 0; i<4; i++){
    bottom[i] = robot->getDistanceSensor(bottomNames[i]);
    bottom[i]->enable(TIME_STEP);
  }
  
  // position sensors of arm motors
  PositionSensor *ps_arm[4];
  char ps_arm_names[4][18] = {"psArmBase","psRightGrab","psLeftGrab","psRightGrip"};
  for (int i = 0; i < 4; i++) {
    ps_arm[i] = robot->getPositionSensor(ps_arm_names[i]);
    ps_arm[i]->enable(TIME_STEP);
  }
  
  // position sensor of a wheel
  PositionSensor *ps;
  ps = robot->getPositionSensor("psSensor");
  
  // accelerometer for detecting the ramp
  Accelerometer *acc;
  acc = robot->getAccelerometer("accelerometer");
  
  // display to output some informations
  Display *display;
  display = robot->getDisplay("display");
  
  // camera as a colour decting sensor
  Camera *cam;
  cam = robot->getCamera("camera");
  
  // motors of the arm
  Motor *arm[4];
  char arm_names[4][18] = {"ArmBaseMotor","RightGrabMotor","LeftGrabMotor","RightGripMotor"};
  for (int i = 0; i < 4; i++) {
    arm[i] = robot->getMotor(arm_names[i]);
    arm[i]->setPosition(INFINITY);
    arm[i]->setVelocity(0.0);
  }
  
  // motors of wheels
  Motor *wheels[2];
  char wheels_names[2][18] = {"LeftMotor", "RightMotor"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  // define some variables
  
  int starting = 0;             // for the starting
   
  double kp = 1;                // line following parameters
  double kd = 0.05;
  double ki = 0.08;
  double baseSpeed = 10;
  
  turnTime = 0;                    // variables for taking turns
  int turns = 0;
  
  double start = 0;             // for calculating the diameter
  double end = 0;
  bool case1 = true;
  bool case2 = true;
  
  bool case5 = false;           // for ramp detection
  int duration = 0;
  int ramp = 0;
      
  int poles = 0;                // for counting the poles
  bool case3 = true;
  bool case4 = true;
  
  std::map<std::string,int> colour_mp = {{"Red",1},{"Green",2},{"Blue",3}};             // colour code
  
  int breakTime = 0;                // for arm movements and colour detection
  int timeLimit = 0;
  int side1 = 0;
  int side2 = 0;
  int side3 = 0;
  int difference = 0;
  bool case6 = false;
  bool case7 = true;
  
  bool break0  = false;         bool step0  = false;                // arm movements which have 14 steps and 14 break times
  bool break1  = false;         bool step1  = false;
  bool break2  = false;         bool step2  = false;
  bool break3  = false;         bool step3  = false;
  bool break4  = false;         bool step4  = false;
  bool break5  = false;         bool step5  = false;
  bool break6  = false;         bool step6  = false;
  bool break7  = false;         bool step7  = false;
  bool break8  = false;         bool step8  = false;
  bool break9  = false;         bool step9  = false;
  bool break10 = false;         bool step10 = false;
  bool break11 = false;         bool step11 = false;
  bool break12 = false;         bool step12 = false;  
  bool break13 = false;         bool step13 = false;
  
  bool case8 = true;            // for quadrant number displaying
  bool case9 = true;
  bool case10 = true;
  
  bool case11 = true;
  bool case12 = false;
  
  bool case13 = true;          // after reverse situation
  bool reverse = false;
  
  bool path1 = true;           // making three paths to get shortest path
  bool path2 = false;
  bool path3 = false;
  
  while (robot->step(TIME_STEP) != -1) {
    
    // distance sensor values
    int ds_left = ds[0]->getValue();
    int ds_front = ds[1]->getValue();
    int ds_right = ds[2]->getValue();
    
    // junction detecting ir sensor values
    int leftWay = bottom[0]->getValue();
    int frontWay = bottom[1]->getValue();
    int rightWay = bottom[2]->getValue();
    int backWay = bottom[3]->getValue();
      
    leftSpeed = 10;
    rightSpeed = 10;
    
    // starting time
    if (starting){
      leftSpeed = 10;
      rightSpeed = 10;
      starting--;
    }
    
    // stop & start conditions
    else if (leftWay<500 && frontWay<500 && rightWay<500 && backWay<500){
      if (ramp){
        leftSpeed = 0;
        rightSpeed = 0;
      }else{
        starting = 20;
        leftSpeed = 10;
        rightSpeed = 10;
      }
      
    }else{
      // check junctions
      if (((rightWay<500 && leftWay<500) || (leftWay<500 && frontWay<500) || (rightWay<500 && frontWay<500)) && turnTime==0 && ramp<2){
        turnTime = 26;
        turns++;
        //std::cout<<turns<<"\n";
      }
      // wall at right 
      if (ds_right < 900 && frontWay>500 && leftWay>500 && rightWay>500){
        wallFollow(ds_right,kp,kd,ki,baseSpeed,0);
      // wall at left
      }else if (ds_left < 900 && frontWay>500 && leftWay>500 && rightWay>500){
        wallFollow(ds_left,kp,kd,ki,baseSpeed,1);    
      }// check that there are no walls
      else{
        // junctions to turn left
        if (turnTime > 0 && ramp==1 && !(difference/2) && difference != 0){
          turnLeft();
          //turnRight();
        // junctions to turn right
        }else if (turnTime > 0 && ramp==1 && ((difference/2) || difference == 0)){
          turnRight();
          //turnLeft();
        // junctions to take no turn
        }else if (turnTime > 0 && ramp == 5){
          noTurn();
        }
        // -----------------------------------------------------------------------------------------------       
        // path 1
        
        // junctions to turn left
        else if (turnTime > 0 && (turns==2 || turns==4 || turns==5 || turns==8) && !ramp && path1){
          turnLeft();
        // junctions to turn right
        }else if (turnTime > 0 && (turns==1 || turns==3 || turns==6 || turns==7) && !ramp && path1){
          turnRight();        
        }
        // -----------------------------------------------------------------------------------------------       
        // path 2
        
        // junctions to turn left
        else if (turnTime > 0 && (turns==3 || turns==5) && !ramp && path2){
          turnLeft();
        // junctions to turn right
        }else if (turnTime > 0 && turns==4 && !ramp && path2){
          turnRight();  
        }
        // -----------------------------------------------------------------------------------------------        
        // path 3
        
        // junctions to turn left
        else if (turnTime > 0 && (turns==2 || turns==4) && !ramp && path3){
          turnLeft();
        // junctions to turn right
        }else if (turnTime > 0 && (turns==1 || turns==3 || turns==6) && !ramp && path3){
          turnRight();
        // junctions to take no turn
        }else if (turnTime > 0 && turns==5 && !ramp && path3){
          noTurn();
        }
        // ----------------------------------------------------------------------------------------------
        
        // junction to turn back
        else if (turnTime > 0 && poles){
          turnBack();
        }
        // line following
        else{
          turnTime = 0;
          lineFollow(ir,kp,kd,ki,baseSpeed);
        }
      }

// ------------------------------------------------------------------------------------------------------
      // box grabing and colour detection
    
   //-------------------------------------------------------------------------------------------
      // detect the box
      
      if (ds_front<133 && case7){
        // adjusting the distance
        if (ds_front<131){
          leftSpeed = -1;
          rightSpeed = -1;
        }else{
          case6 = true;
          case7 = false;
          step0 = true;
        }
        
      }
      
      if (case6){
        // camera enable
        if (breakTime==1){
          cam->enable(TIME_STEP);
        }
        // colour detection
        if (breakTime==18 || breakTime==23 || breakTime==28){
          int red = 0;
          int green = 0;
          int blue = 0;
          const unsigned char *image = cam->getImage();
          for (int x = 0; x < 64; x++){
            for (int y = 0; y < 64; y++) {
              red += cam->imageGetRed(image, 64, x, y);
              green += cam->imageGetGreen(image, 64, x, y);
              blue += cam->imageGetBlue(image, 64, x, y);
              }
            }
          if (red>blue && red>green){
            if (breakTime==18){
              side1 = colour_mp["Red"];
            }
            if (breakTime==23){
              side2 = colour_mp["Red"];
            }
            if (breakTime==28){
              side3 = colour_mp["Red"];
            }
            std::cout<<"Red"<<"\n";
          }else if (blue>green){
            if (breakTime==18){
              side1 = colour_mp["Blue"];
            }
            if (breakTime==23){
              side2 = colour_mp["Blue"];
            }
            if (breakTime==28){
              side3 = colour_mp["Blue"];
            }
            std::cout<<"Blue"<<"\n";
          }else{
            if (breakTime==18){
              side1 = colour_mp["Green"];
            }
            if (breakTime==23){
              side2 = colour_mp["Green"];
            }
            if (breakTime==28){
              side3 = colour_mp["Green"];
            }
            std::cout<<"Green"<<"\n";
          } 
        }
         // choosing the path and camera disable
        if (breakTime==31){
          cam->disable();
          if (turns == 2){
            path1 = false;
            path2 = true;
          }
          if (turns == 3){
            path1 = false;
            path3 = true;
          }
        }
        
     // ------------------------------------------------------------------------------
        // arm operation
        
        // arm wide
        if (step0 && ps_arm[1]->getValue()<0.35){
          arm[1]->setVelocity(1);
          arm[2]->setVelocity(-1);
        }else if (step0 && ps_arm[1]->getValue()>=0.35){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          step0 = false;
          break0 = true;
        }else if (break0 && breakTime<5){
          breakTime++;
        }else if (break0 && breakTime>=5){
          break0 = false;
          step1 = true;
        }
        // arm down
        else if (step1 && ps_arm[0]->getValue()<1.4){
          arm[0]->setVelocity(1);
        }else if (step1 && ps_arm[0]->getValue()>=1.4){
          arm[0]->setVelocity(0);
          step1 = false;
          break1 = true;
        }else if (break1 && breakTime<10){
          breakTime++;
        }else if (break1 && breakTime>=10){
          break1 = false;
          step2 = true;
        }
        // grab the box
        else if (step2 && timeLimit < 140){
          timeLimit++;
          arm[2]->setVelocity(0.2);
          if (ps_arm[1]->getValue()<0){
            arm[1]->setVelocity(0);
          }else{
            arm[1]->setVelocity(-0.2);
          }
        }else if (step2 && timeLimit >= 140){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          break2 = true;
          step2 = false;
        }else if (break2 && breakTime<15){
          breakTime++;
        }else if (break2 && breakTime>=15){
          break2 = false;
          step3 = true;
        }
        // arm up
        else if (step3 && ps_arm[0]->getValue()>1.09){
          arm[0]->setVelocity(-0.3);
        }else if (step3 && ps_arm[0]->getValue()<=1.09){
          arm[0]->setVelocity(0);
          break3 = true;
          step3 = false;
        }else if (break3 && breakTime<20){
          breakTime++;
        }else if (break3 && breakTime>=20){
          break3 = false;
          step4 = true;
        }
        // rotate the box
        else if (step4 && ps_arm[3]->getValue()>-1.6){
          arm[3]->setVelocity(-1);
        }else if (step4 && ps_arm[3]->getValue()<=-1.6){
          arm[3]->setVelocity(0);
          break4 = true;
          step4 = false;
        }else if (break4 && breakTime<25){
          breakTime++;
        }else if (break4 && breakTime>=25){
          break4 = false;
          step7 = true;
        }
        /*// arm down
        else if (step5 && ps_arm[0]->getValue()<1.1){
          arm[0]->setVelocity(0.3);
        }else if (step5 && ps_arm[0]->getValue()>=1.1){
          arm[0]->setVelocity(0);
          break5 = true;
          step5 = false;
        }else if (break5 && breakTime<30){
          breakTime++;
        }else if (break5 && breakTime>=30){
          break5 = false;
          step6 = true;
        }
        // arm up
        else if (step6 && ps_arm[0]->getValue()>1.09){
          arm[0]->setVelocity(-0.3);
        }else if (step6 && ps_arm[0]->getValue()<=1.09){
          arm[0]->setVelocity(0);
          break6 = true;
          step6 = false;
        }else if (break6 && breakTime<35){
          breakTime++;
        }else if (break6 && breakTime>=35){
          break6 = false;
          step7 = true;
        }*/
        // rotate the box
        else if (step7 && ps_arm[3]->getValue()>-3.2){
          arm[3]->setVelocity(-1);
        }else if (step7 && ps_arm[3]->getValue()<=-3.2){
          arm[3]->setVelocity(0);
          break7 = true;
          step7 = false;
        }else if (break7 && breakTime<30){
          breakTime++;
        }else if (break7 && breakTime>=30){
          break7 = false;
          step9 = true;
        }
        /*// arm down
        else if (step8 && ps_arm[0]->getValue()<1.1){
          arm[0]->setVelocity(0.3);
        }else if (step8 && ps_arm[0]->getValue()>=1.1){
          arm[0]->setVelocity(0);
          break8 = true;
          step8 = false;
        }else if (break8 && breakTime<45){
          breakTime++;
        }else if (break8 && breakTime>=45){
          break8 = false;
          step9 = true;
        }*/
        // arm side
        else if (step9 && ps_arm[1]->getValue()<0.6){
          arm[1]->setVelocity(1.5);
          arm[2]->setVelocity(1.5);
        }else if (step9 && ps_arm[1]->getValue()>=0.6){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          break9 = true;
          step9 = false;
        }else if (break9 && breakTime<35){
          breakTime++;
        }else if (break9 && breakTime>=35){
          break9 = false;
          step10 = true;
        }
        // release the box
        else if (step10 && ps_arm[1]->getValue()<0.85){
          arm[1]->setVelocity(1);
          arm[2]->setVelocity(-1);
        }else if (step10 && ps_arm[1]->getValue()>=0.85){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          break10 = true;
          step10 = false;
        }else if (break10 && breakTime<40){
          breakTime++;
        }else if (break10 && breakTime>=40){
          break10 = false;
          step11 =true;
        }
        // arm up
        else if (step11 && ps_arm[0]->getValue()>0){
          arm[0]->setVelocity(-1);
        }else if (step11 && ps_arm[0]->getValue()<=0){
          arm[0]->setVelocity(0);
          break11 = true;
          step11 = false;
        }else if (break11 && breakTime<45){
          breakTime++;
        }else if (break11 && breakTime>=45){
          break11 = false;
          step12 =true;
        }
        // arm middle
        else if (step12 && ps_arm[1]->getValue()>0.25){
          arm[1]->setVelocity(-1);
          arm[2]->setVelocity(-1);
        }else if (step12 && ps_arm[1]->getValue()<=0.25){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          step12 = false;
          break12 = true;
        }else if (break12 && breakTime<50){
          breakTime++;
        }else if (break12 && breakTime>=50){
          break12 = false;
          step13 = true;
        }
        // arm restore
        else if (step13 && ps_arm[1]->getValue()>0){
          arm[1]->setVelocity(-1);
          arm[2]->setVelocity(1);
        }else if (step13 && ps_arm[1]->getValue()<=0){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          break13 = true;
          step13 = false;
        }else if (break13 && breakTime<55){
          breakTime++;
        }else if (break13 && breakTime>=55){
          turnTime = 1;
          break13 = false;
          case6 = false;
          if (turns==2 || turns==5){
            difference = abs(side2-side3);
          }else{
            difference = abs(side2-side1);
          }
          std::cout<<"Difference: "<<difference<<"\n";
        }
        leftSpeed = 0;
        rightSpeed = 0;
      }
    // ---------------------------------------------------------------------------------------------
  
// -------------------------------------------------------------------------------------------------------------
      //get the diameter
          
      if (turns==2 && case1){
        start = ps->getValue();
        case1 = false;
      }
      if ((turns==4) && case2){
        end = ps->getValue();
        ps->disable();
        int dis = (end-start)*3.5;
        std::cout<<"Diameter: "<<dis<<"cm"<<"\n";
        
        // display the diameter
        std::string diam;
        std::stringstream ss;
        ss << dis;
        ss >> diam;
        diam = diam + "cm";
        display->drawText(diam,10,10);
        
        case2 = false;
      }

// ---------------------------------------------------------------------------------------------------------------
    // quadrant number display      
      
      if (turns==1 && case8){
        display->drawText("Q-01",13,25);
        std::cout<<"Q=01"<<"\n";
        ps->enable(TIME_STEP);              // enable the position sensor of the wheel
        case8 = false;
      }
      if (turns==4 && case9){
        if (!path2){
          display->drawText("Q-03",13,35);
          std::cout<<"Q=03"<<"\n";
          case9 = false;
        }else{
          display->drawText("Q-04",13,35);
          std::cout<<"Q=04"<<"\n";
          case5 = true;
          acc->enable(TIME_STEP);             // enable the accelerometer
          case9 = false;
        }
        
      }
      if (((turns==7 && path1) || (turns==5 && path3)) && case10){
        display->drawText("Q=04",13,45);
        std::cout<<"Q=04"<<"\n";
        case5 = true;
        acc->enable(TIME_STEP);             // enable the accelerometer
        case10 = false;
      }

// ----------------------------------------------------------------------------------------------------------------
      // recognizing the ramp
      
      if (case5){
        const double* vals = acc->getValues();
        if (vals[2]>3){
          duration++; 
        }else{
          duration = 0;
        }
        if (duration>10){
          ramp=1;
          turnTime = 1;
          acc->disable();
          case5 = false;
        }    
      }
      
// -----------------------------------------------------------------------------------------------------------------    
      // count the poles
      
      if (ramp){
        if (ds_right < 500 || ds_left < 500){
          if (case4){
            poles++;
            ramp++;
          }        
          case4 = false;
        }else{
          case4 = true;
        }     
      }
      // checking the path
      if (poles && (leftWay<500 || rightWay<500) && case3){
        std::cout<<"Pillars: "<<poles<<"\n";
        if (poles == 1){
          turnTime = 52;
          reverse = true;
        }
        ramp++;        
        case3 = false;
      }
      // after reversing, go forward on next junction
      if (reverse && (leftWay<500 || rightWay<500) && case13  && ramp == 4){
        turnTime = 26;
        ramp++;
        case13 = false;
      }

// ------------------------------------------------------------------------------------------------------------------
      // gate passing
      if (ramp && ir[0]->getValue()<500 && ir[7]->getValue()<500 && case11){
        if (ds_front<1000){
          case11 = false;
          case12 = true;
        }else{
          leftSpeed = 0;
          rightSpeed = 0;
        }
      }

      if (ramp && ir[0]->getValue()<500 && ir[7]->getValue()<500 && ds_front<1000 && case12){
        leftSpeed = 0;
        rightSpeed = 0;
      }
    }
// -------------------------------------------------------------------------------------------------------------------
    
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}