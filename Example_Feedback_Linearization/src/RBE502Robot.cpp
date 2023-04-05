#include <Arduino.h>
#include <RBE502Robot.h>
#include <string>


//Stuff for the Hardware Timer ISR
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#include <SAMDTimerInterrupt.h>

//Global variable to allow using ISR on enable/disable switch
bool disabled = true;
uint8_t enableSwitchPin;


//Instantiate the hardware timer
SAMDTimer ITimer0(TIMER_TC3);

//Define the timer handler function
void TimerHandler0(){
    // Serial.println("ISR");
}

//Define the timer interval in milliseconds (timer functions at microsecond level but is converted in the begin method)
#define TIMER0_INTERVAL_MS        500 




/*
    Creates an instance of an RBE502Robot Object.
    @param uint8_t txPin        : The tx serial pin of your microcontroller used to communicate with the servos.
    @param uint8_t txFlagPin    : The digital output pin being used to enable/disable communication TO the servos.
    @param uint8_t motor1ID     : The servo motor ID of motor1.
    @param uint8_t motor2ID     : The servo motor ID of motor2.
    @param uint8_t motor3ID     : The servo motor ID of motor3.
*/
RBE502Robot::RBE502Robot(uint8_t txPin, uint8_t txFlagPin, uint8_t enableSwitchPin, uint8_t motor1ID, uint8_t motor2ID, uint8_t motor3ID) : motor1(new LX16AServo(&servoBus, motor1ID)),
motor2(new LX16AServo(&servoBus, motor2ID)), motor3(new LX16AServo(&servoBus, motor3ID)){
    this->txFlagPin = txFlagPin;
    this->txPin = txPin;
    ::enableSwitchPin = enableSwitchPin;
    ::disabled = true;
}


/*
    Acts as an interpreter for the commands. To add a new command/state, add it to the enumeration, add an else if to 
    the conversion function below, and create the case in the state machine. Remember to add the command[0] = "N/A"; 
    statement when you wish to return to the default case. You should also add each function into the HELP menu. 
*/
enum Functions{DEFAULT, HELP, SETMOTORPOSITIONS, HOME, DISABLE,ENABLE,PRINTPOSITION,SETMOTORSPEEDS,STOP,FEEDBACKLINEARIZEPT, FEEDBACKLINEARIZETRAJ, FEEDBACKLINEARIZECALCTRAJ,PRINTVELOCITY,PDCONTROLLER,KP,KD,TEST};
static Functions convertToFunction(String str){
    str.toUpperCase();
    if(str == "SETMOTORPOSITIONS") {return SETMOTORPOSITIONS;}
    else if(str == "HELP"){return HELP;}
    else if(str == "HOME"){return HOME;}
    else if(str == "DISABLE"){return DISABLE;}
    else if(str == "ENABLE"){return ENABLE;}
    else if(str == "PRINTPOSITION"){return PRINTPOSITION;}
    else if(str == "SETMOTORSPEEDS"){return SETMOTORSPEEDS;}
    else if(str == "STOP"){return STOP;}
    else if(str == "FLPT"){return FEEDBACKLINEARIZEPT;}
    else if(str == "FLTRAJ"){return FEEDBACKLINEARIZECALCTRAJ;}//Calculates traj then reroutes to FLRUNTRAJ
    else if(str == "FLRUNTRAJ"){return FEEDBACKLINEARIZETRAJ;} //Runs the robot along given trajectory
    else if(str == "PRINTVELOCITY"){return PRINTVELOCITY;}
    else if(str == "PD"){return PDCONTROLLER;}
    else if(str == "KP"){return KP;}
    else if(str == "KD"){return KD;}
    else if(str == "TEST"){return TEST;}
    else if(str == "N/A"){return DEFAULT;}
    else {
        Serial.println("Unrecognized command: " + str);
        return DEFAULT;
    }
}


/*
    Runs the state machine: Monitors the serial inputs and follows the commands issued
*/
void RBE502Robot::stateMachine(){
    static String command[7] = {"N/A"};
    static String lastCommand = "N/A";
    static uint32_t lastTime = millis();
    int16_t des[3]; 

    //The time at which the trajectory is started
    static uint32_t startTime = 0; //Calculate start time

    //Set the K and D values
    //Kp = 0.5 and Kd = 0.15 for PD control
    static float Kp[3] = {0.5,0.5,0.5}; 
    static float Kd[3] = {0.15,0.15,0.15};

    //Values to hold trajectories
    static float A1[4];
    static float A2[4];
    static float A3[4];

    //Monitor for Serial Commands
    if(sr.monitorForCommands()){
        lastCommand = command[0];
        sr.getCommand(command);

        lastTime = 0; //Resets timing so next timed command sent is instantaneous after command

        //Can only reenable the robot from DISABLE if you type ENABLE
        if(lastCommand == "DISABLE" && command[0] != "ENABLE"){
            command[0] = "DISABLE";
        }
    }

    //Read motor position once per loop for use throughout state machine
    int16_t motorPos[3]; //Create var for motorPos
    this->getMotorPositions(motorPos); //Update motorPos
    
    //Determine new velocity every 50ms, output either new velocity or last one determined
    float motorVel[3]; // DEG/Second
    this->determineVelocity(motorPos,motorVel);

    //If the motors exceed their bounds, disable the robot.
    if(!this->checkWithinServoBounds(motorPos,true)){
        command[0] = "DISABLE";
        if(lastCommand != "DISABLE"){ //Only immediately print disabled the first time it exceeds bounds
            lastTime = 0; //Resets timing so next timed command sent is instantaneous after command
        }
    }

    switch(convertToFunction(command[0])){
        case HELP:
            Serial.println("\n------------------------------------------------------------------");
            Serial.println("See functions available below");

            Serial.println("\nTo servo to a given theta1, theta2, theta3:");
            Serial.println("\tSETMOTORPOSITIONS(m1CentiDegrees,m2CentiDegrees,m3CentiDegrees,timeToMovems)");

            Serial.println("\nTo move robot to its home position:");
            Serial.println("\tHOME");
            
            Serial.println("\nTo disable the robot:");
            Serial.println("\tDISABLE");

            Serial.println("\nTo enable the robot:");
            Serial.println("\tENABLE");

            Serial.println("\nTo print joint positions of robot:");
            Serial.println("\tPRINTPOSITION");

            Serial.println("\nTo print joint velocities:");
            Serial.println("\tPRINTVELOCITY");

            Serial.println("\nTo set joint speeds/efforts of robot -100 to 100:");
            Serial.println("\tSETMOTORSPEEDS(m1speed,m2speed,m3speed)");

            Serial.println("\nTo stop movement without disabling:");
            Serial.println("\tSTOP");

            Serial.println("\nTo move to given joint positions using a PD Controller:");
            Serial.println("\tPD(m1centidegrees,m2centidegrees,m3centidegrees)");

            Serial.println("\nTo move to given joint positions using Feedback Linearization:");
            Serial.println("\tFLPT(m1centidegrees,m2centidegrees,m3centidegrees)");

            Serial.println("\nTo move to given joint positions using Feedback Linearization and a cubic trajectory:");
            Serial.println("\tFLTRAJ(m1centidegrees,m2centidegrees,m3centidegrees,time)");

            Serial.println("\nTo change Kp values: (To just read them, type \"KD(READ)\")");
            Serial.println("\tKd(Kp1, Kp2, Kp3)");
            
            Serial.println("\nTo change Kd values: (To just read them, type \"KP(READ)\")");
            Serial.println("\tKd(Kd1, Kd2, Kd3)");

            Serial.println("\n------------------------------------------------------------------\n");
            command[0] = "N/A";
        break;
        case HOME:
            Serial.println("Moving robot to home position.");
            this->setMotorPositions(0,0,0,750);
            command[0] = "N/A";
        break;
        case SETMOTORPOSITIONS:
            Serial.println("Moving robot to Position: (" + command[1] + "," + command[2] + "," + command[3] + ") in " + command[4] + " milliseconds.");
            this->setMotorPositions(command[1].toInt(),command[2].toInt(),command[3].toInt(),command[4].toInt());
            command[0] = "N/A";
        break;
        case ENABLE:
            this->enableMotors();
            Serial.println("Robot Enabled");
            command[0] = "N/A";
        break;
        case DISABLE:
            // SET MOTORS TO DISABLE IN HERE
            this->disableMotors();
            if(millis() - 5000 > lastTime){ 
                Serial.println("Robot is DISABLED. Type \"ENABLE\" to reenable the robot.");
                lastTime = millis();
            }//Only way to exit disable is to type "ENABLE"
            lastCommand = command[0]; // Sets this so if robot disables itself, it does not continuously print that robot is disabled
        break;
        case PRINTPOSITION:
            Serial.print(motorPos[0]);
            Serial.print('\t');
            Serial.print(motorPos[1]);
            Serial.print('\t');
            Serial.print(motorPos[2]);
            Serial.print('\n');
        break;
        case PRINTVELOCITY:
            this->disableMotors();
            Serial.print(motorVel[0]);
            Serial.print('\t');
            Serial.print(motorVel[1]);
            Serial.print('\t');
            Serial.print(motorVel[2]);
            Serial.print('\n');
        break;
        case SETMOTORSPEEDS:
            Serial.println("Setting Motor Speeds to: (" + command[1] + "," + command[2] + "," + command[3] + ")");
            this->setMotorSpeeds(command[1].toInt(),command[2].toInt(),command[3].toInt());
            command[0] = "N/A";
        break;
        case STOP://disable the motors without going into disable mode
            this->disableMotors();
            this->enableMotors();//Reenable the motors right after so commands can work
            command[0] = "N/A";
        break;
        case PDCONTROLLER:
            //Print status every 5 seconds
            if(millis() - 5000 > lastTime){ 
                Serial.println("Running PD Controller to position: (" + command[1] + "," + command[2] + "," + command[3] + ")");
                lastTime = millis();
            }

            //Works with Kp = 0.5 and Kd = 3
            des[0] = command[1].toInt();
            des[1] = command[2].toInt();
            des[2] = command[3].toInt();

            this->PDController(Kp,Kd,des,motorPos,motorVel); 

        break;
        case KP:
            command[1].toUpperCase();
            if(command[1] == "READ"){ //Just print the values
                Serial.println("Kp is:\t" + String(Kp[0],3) + '\t' + String(Kp[1],3) + '\t' + String(Kp[2],3));
                command[0] = "N/A";
                return;
            } 
            
            Serial.println("Old Kp was:\t" + String(Kp[0],3) + '\t' + String(Kp[1],3) + '\t' + String(Kp[2],3));
            Kp[0] = command[1].toFloat();
            Kp[1] = command[2].toFloat();
            Kp[2] = command[3].toFloat();

            Serial.println("New Kp set:\t" + String(Kp[0],3) + '\t' + String(Kp[1],3) + '\t' + String(Kp[2],3));
            command[0] = "N/A";
        break;
        case KD:
            command[1].toUpperCase();
            if(command[1] == "READ"){ //Just print the values
                Serial.println("Kd is:\t" + String(Kd[0],3) + '\t' + String(Kd[1],3) + '\t' + String(Kd[2],3));
                command[0] = "N/A";
                return;
            }

            Serial.println("Old Kd was:\t" + String(Kd[0],3) + '\t' + String(Kd[1],3) + '\t' + String(Kd[2],3));
            
            Kd[0] = command[1].toFloat();
            Kd[1] = command[2].toFloat();
            Kd[2] = command[3].toFloat();

            Serial.println("New Kd set:\t" + String(Kd[0],3) + '\t' + String(Kd[1],3) + '\t' + String(Kd[2],3));
            command[0] = "N/A";
        break;
        case FEEDBACKLINEARIZEPT:
            //Print status every 5 seconds
            if(millis() - 5000 > lastTime){ 
                Serial.println("Running Feedback Linearization Controller to position: (" + command[1] + "," + command[2] + "," + command[3] + ")");
                lastTime = millis();
            }

            //Works with Kp = 0.5 and Kd = 3
            des[0] = command[1].toInt();
            des[1] = command[2].toInt();
            des[2] = command[3].toInt();

            this->feedbackLinearization(Kp,Kd,des,motorPos,motorVel); 
        break;
        case FEEDBACKLINEARIZECALCTRAJ: //Helper case that calculates the trajectories and then calls the case to execute that trajectory. 
            //Command = [command, q1des,q2des,q3des, tf-t0]
            Serial.println("Calculating Trajectory");

            if(command[4].toInt() < 1){
                command[0] = "N/A";
                Serial.println("Time value cannot be zero.");
                return;
            }

            this->cubicTraj(0,command[4].toFloat(),motorPos[0],command[1].toFloat(),0,0,A1);
            
            this->cubicTraj(0,command[4].toFloat(),motorPos[1],command[2].toFloat(),0,0,A2);
            
            this->cubicTraj(0,command[4].toFloat(),motorPos[2],command[3].toFloat(),0,0,A3);

            Serial.println("Done Calculating Trajectories");

            Serial.print(A1[0]);
            Serial.print('\t');
            Serial.print(A1[1]);
            Serial.print('\t');
            Serial.print(A1[2]);
            Serial.print('\t');
            Serial.print(A1[3]);
            Serial.print('\n');

            Serial.print(A2[0]);
            Serial.print('\t');
            Serial.print(A2[1]);
            Serial.print('\t');
            Serial.print(A2[2]);
            Serial.print('\t');
            Serial.print(A2[3]);
            Serial.print('\n');

            Serial.print(A3[0]);
            Serial.print('\t');
            Serial.print(A3[1]);
            Serial.print('\t');
            Serial.print(A3[2]);
            Serial.print('\t');
            Serial.print(A3[3]);
            Serial.print('\n');


            // float desPosTrash[3];
            // this->calcTrajPos(command[5].toFloat(),A2,desPosTrash);
            // Serial.println(desPosTrash[0]);

            

            startTime = millis();
            command[0] = "FLRUNTRAJ"; //After trajectories are calculated, feedback linearize along them

            // command[0] = "N/A";
        break;
        case FEEDBACKLINEARIZETRAJ:
            //Print status every 5 seconds
            if(millis() - 5000 > lastTime){ 
                Serial.println("Running Feedback Linearization Controller to position: (" + command[1] + "," + command[2] + "," + command[3] + ")");
                lastTime = millis();
            }

            this->feedbackLinearization(Kp,Kd,des,motorPos,motorVel, startTime, A1,A2,A3); 
            if(millis() - startTime > command[4].toFloat()*1000){// If the time has been exceeded, stop (otherwise robot will do weird shit along trajectory)
                command[0] = "STOP";
            }
            //Should add "when finished, do regular feedback linearization. At a min, need to stop after end time"
        break;
        case TEST:
            //Command = [command, q1des,q2des,q3des, tf-t0]

        break;
        default:
            //Reset command values
            command[0] = "N/A";
            command[1] = "0";
            command[2] = "0";
            command[3] = "0";
            command[4] = "0";
            command[5] = "0";
            command[6] = "0";
            
            //Print out instructions every 5 seconds
            if(max(millis(),5000) - 5000 > lastTime){ //max is there so the first 5 seconds arent spammed with "Waiting for input"
                Serial.println("Waiting for input");
                // Serial.println("Type \"HELP\" for options to print");
                lastTime = millis();
            }
            // this->disableMotors(); //This cant be uncommented because some functionality just sets a position and returns to idle state
        break;

    }
}

/*
    Drives the robot to a given position using Feedback Linearization control
    @param float* Kp            : An array of proportional gains for the motors
    @param float* Kd            : An array of differentiative gains for the motors
    @param int16_t* desiredPos  : An array of desired positions of the motors in centidegrees
    @param int16_t* curPos      : An array of current positions of the motors in centidegrees
    @param float curVel         : An array of current velocities in DEG/SEC
    @param float* A1            : Coefficients of trajectory for joint 1
    @param float* A2            : Coefficients of trajectory for joint 2
    @param float* A3            : Coefficients of trajectory for joint 3

*/
void RBE502Robot::feedbackLinearization(float* Kp, float* Kd, int16_t* desiredPos, int16_t* curPos, float* curVel, uint32_t startTime , float* A1, float* A2, float* A3){
    //Init the error values
    float err[3] = {0,0,0};
    float errDot[3] = {0,0,0};
    
    // Kp[0] = 2;
    // Kp[1] = 2;
    // Kp[2] = 2;



    //Init constants
    // float L1 = 0.04;
    float L2 = 0.1;
    // float L3 = 0.1;
    // float L1M = 0.02;
    float L2M = 0.05;
    float L3M = 0.05;
    // float m1 = 1;   // Mass of Link 1 [kg]
    float m2 = 0.063;   // Mass of Link 2 [kg]
    float m3 = 0.063;   // Mass of Link 3 [kg]
    float g = 9.81;
    float I1 = 0.05;
    float I2 = 0.05;
    float I3 = 0.05;

    
    //Calculate trajectories
    float qdd[3];
    if(A1 == 0 and A2 == 0 and A3 == 0){//Feedback linearize to point
        qdd[0] = 0;
        qdd[1] = 0;
        qdd[2] = 0;
        //Determine error from point in DEGREES and DEG/SEC
        this->determineError(desiredPos,curPos,err,errDot);

    }else{//Feedback linearize to trajectory
        //Trajectories in state machine are in CENTIDEGREES, but CALCTRAJPOS returns values in DEG, DEG/SEC, DEG/SEC^2 of plugging t into equations
        
        float joint1des[3];
        this->calcTrajPos(millis() - startTime,A1,joint1des);

        float joint2des[3];
        this->calcTrajPos(millis() - startTime,A2,joint2des);

        float joint3des[3];
        this->calcTrajPos(millis() - startTime,A3,joint3des);

        // Serial.print("Time: ");
        // Serial.println(millis() - startTime);
        // Serial.println(joint1des[0]);

        double desPosAtTime[3]; //IN DEGREES
        desPosAtTime[0] = joint1des[0];
        desPosAtTime[1] = joint2des[0];
        desPosAtTime[2] = joint3des[0];

        // Serial.println(joint1des[0]);

        float desVelAtTime[3]; //IN DEGREES/Second
        desVelAtTime[0] = joint1des[1];
        desVelAtTime[1] = joint2des[1];
        desVelAtTime[2] = joint3des[1];

        float desAccAtTime[3]; //IN DEGREES/Second^2
        qdd[0] = joint1des[1];
        qdd[1] = joint2des[1];
        qdd[2] = joint3des[1];
        // qdd[0] = 0;
        // qdd[1] = 0;
        // qdd[2] = 0;

        this->determineErrorFromTraj(desPosAtTime, curPos, desVelAtTime, curVel, err, errDot);
        //Determine error from trajectory in DEGREES and DEG/SEC
        // this->determineError(desiredPos,curPos,err,errDot); //  DONT FORGET TO CHANGE DESIREDPOS TO THE POINT IN THE TRAJ AT THE GIVEN TIME. ALSO NEED TO WRITE ANOTHER DETERMINE ERROR FUNCTION FOR TRAJS        
    
        Serial.print(float(millis() - startTime)/1000.0);
        Serial.print('\t');
        Serial.print(desPosAtTime[0]);
        Serial.print('\t');
        Serial.print(desVelAtTime[0]);
        Serial.print('\t');
        Serial.print(curPos[0]/100.0);
        Serial.print('\t');
        Serial.print(curVel[0]);
        Serial.print('\t');

        Serial.print(desPosAtTime[1]);
        Serial.print('\t');
        Serial.print(desVelAtTime[1]);
        Serial.print('\t');
        Serial.print(curPos[1]/100.0);
        Serial.print('\t');
        Serial.print(curVel[1]);
        Serial.print('\t');
        
        Serial.print(desPosAtTime[2]);
        Serial.print('\t');
        Serial.print(desVelAtTime[2]);
        Serial.print('\t');
        Serial.print(curPos[2]/100.0);
        Serial.print('\t');
        Serial.print(curVel[2]);
        Serial.print('\t');
    }

    



    //virtual control input
    float v[3];
    v[0] = err[0]*Kp[0] + errDot[0]*Kd[0] + qdd[0];
    v[1] = err[1]*Kp[1] + errDot[1]*Kd[1] + qdd[1];
    v[2] = err[2]*Kp[2] + errDot[2]*Kd[2] + qdd[2];


    float u[3];
    u[0] = v[0]*(I1 + I2 + I3 + (L2*L2*m3)/2 + (L2M*L2M*m2)/2 + (L3M*L3M*m3)/2 - (L2*L2*m3*cos(2*curPos[1]))/2 - (L2M*L2M*m2*cos(2*curPos[1]))/2 + (L3M*L3M*m3*cos(2*curPos[1] + 2*curPos[2]))/2 + L2*L3M*m3*sin(curPos[2]) - L2*L3M*m3*sin(2*curPos[1] + curPos[2])) + (curVel[0]*curVel[1]*(2*m3*sin(2*curPos[1])*L2*L2 - 4*m3*cos(2*curPos[1] + curPos[2])*L2*L3M + 2*m2*sin(2*curPos[1])*L2M*L2M - 2*m3*sin(2*curPos[1] + 2*curPos[2])*L3M*L3M))/2 - L3M*m3*curVel[0]*curVel[2]*(L3M*sin(2*curPos[1] + 2*curPos[2]) - L2*cos(curPos[2]) + L2*cos(2*curPos[1] + curPos[2]));
    u[1] = v[2]*(m3*L3M*L3M + L2*m3*sin(curPos[2])*L3M + I3) + v[1]*(m3*L2*L2 + 2*m3*sin(curPos[2])*L2*L3M + m2*L2M*L3M + m3*L3M*L3M + I2 + I3) + L3M*g*m3*cos(curPos[1] + curPos[2]) - L2*g*m3*sin(curPos[1]) - L2M*g*m2*sin(curPos[1]) - (L2*L2*m3*curVel[0]*curVel[0]*sin(2*curPos[1]))/2 - (L2M*L2M*m2*curVel[0]*curVel[0]*sin(2*curPos[1]))/2 + (L3M*L3M*m3*curVel[0]*curVel[0]*sin(2*curPos[1] + 2*curPos[2]))/2 + L2*L3M*m3*curVel[0]*curVel[0]*cos(2*curPos[1] + curPos[2]) + L2*L3M*m3*curVel[2]*cos(curPos[2])*(2*curVel[1] + curVel[2]);
    u[2] = I3*v[1] + I3*v[2] + L3M*L3M*m3*v[1] + L3M*L3M*m3*v[2] + L3M*g*m3*cos(curPos[1] + curPos[2]) + (L3M*L3M*m3*curVel[0]*curVel[0]*sin(2*curPos[1] + 2*curPos[2]))/2 - (L2*L3M*m3*curVel[0]*curVel[0]*cos(curPos[2]))/2 - L2*L3M*m3*curVel[1]*curVel[1]*cos(curPos[2]) + (L2*L3M*m3*curVel[0]*curVel[0]*cos(2*curPos[1] + curPos[2]))/2 + L2*L3M*m3*v[1]*sin(curPos[2]);




    Serial.print(u[0]);
    Serial.print('\t');
    Serial.print(u[1]);
    Serial.print('\t');
    Serial.print(u[2]);
    Serial.print('\n');
    // Serial.print(u[0]);
    // Serial.print('\t');
    // Serial.print(u[1]);
    // Serial.print('\t');
    // Serial.println(u[2]);
    //Set motor speeds based on control values
    this->setMotorSpeeds(u[0], u[1], u[2]);

}



/*
    Drives the robot to a given position using PD control
    @param float* Kp            : An array of proportional gains for the motors
    @param float* Kd            : An array of differentiative gains for the motors
    @param uint16_t* desiredPos : An array of desired positions of the motors (centidegrees)
    @param uint16_t* curPos     : An array of current positions of the motors (centidegrees)
*/
void RBE502Robot::PDController(float* Kp, float* Kd, int16_t* desiredPos, int16_t* curPos, float* curVel){
    //Init the error values
    float err[3] = {0,0,0};
    float errDot[3] = {0,0,0};
    
    //Get values of error
    this->determineError(desiredPos,curPos,err,errDot);

    Serial.print(float(millis())/1000.0);
    Serial.print('\t');
    Serial.print(desiredPos[0]/100);
    Serial.print('\t');
    Serial.print(curPos[0]/100.0);
    Serial.print('\t');
    Serial.print(curVel[0]);
    Serial.print('\t');

    Serial.print(desiredPos[1]/100);
    Serial.print('\t');
    Serial.print(curPos[1]/100.0);
    Serial.print('\t');
    Serial.print(curVel[1]);
    Serial.print('\t');
    
    Serial.print(desiredPos[2]/100);
    Serial.print('\t');
    Serial.print(curPos[2]/100.0);
    Serial.print('\t');
    Serial.print(curVel[2]);
    Serial.print('\t');

    Serial.print(err[0]*Kp[0] + errDot[0]*Kd[0]);
    Serial.print('\t');
    Serial.print(err[1]*Kp[1] + errDot[1]*Kd[1]);
    Serial.print('\t');
    Serial.print(err[2]*Kp[2] + errDot[2]*Kd[2]);
    Serial.print('\n');
   
    //Set motor speeds based on control values
    this->setMotorSpeeds(int16_t(err[0]*Kp[0] + errDot[0]*Kd[0]), int16_t(err[1]*Kp[1] + errDot[1]*Kd[1]), int16_t(err[2]*Kp[2] + errDot[2]*Kd[2]));
}


/*
    Calculates the error and errorDot to be used in control loops
    @param int16_t* desiredPos  : Desired position (centidegrees)
    @param int16_t* curPos      : current position (centidegrees)
    @param float*   err         : error (deg)
    @param float*   errDot      : errDot(deg/sec)
*/
void RBE502Robot::determineError(int16_t* desiredPos, int16_t* curPos, float* errOut, float* errDotOut){
    static float err[3] = {0,0,0};
    static float lastErr[3] = {0,0,0};
    float errDot[3] = {0,0,0};
    static uint32_t lastTimeCalculatedError = 0;

    //Determine error in DEGREES: (wanted - current)/100.
    err[0] = float(desiredPos[0] - curPos[0])/100.0;
    err[1] = float(desiredPos[1] - curPos[1])/100.0;
    err[2] = float(desiredPos[2] - curPos[2])/100.0;

    //Determine deg/second
    if(millis() - 50 > lastTimeCalculatedError){
        errDot[0] = (abs(err[0]) - abs(lastErr[0]))*20;
        errDot[1] = (abs(err[1]) - abs(lastErr[1]))*20;
        errDot[2] = (abs(err[2]) - abs(lastErr[2]))*20;

        // Serial.println(errDot[0]);

        //Erratic positive values caused when desiredPos changes. This eliminates them. Values should always be negative when converging to point, and are typically -300<x<300 anyways. Erratic values are > ~1000 
        //Also eliminates small jittering values
        if(abs(errDot[0]) > 300 or abs(errDot[0] <= 4.8)){
            errDot[0] = 0;
        }
        if(abs(errDot[1]) > 300 or abs(errDot[0] <= 4.8)){
            errDot[1] = 0;
        }
        if(abs(errDot[2]) > 300 or abs(errDot[0] <= 4.8)){
            errDot[2] = 0;
        }

        lastErr[0] = err[0];
        lastErr[1] = err[1];
        lastErr[2] = err[2];
        lastTimeCalculatedError = millis();
        // Serial.println(errDot[0]);
    }

    //Set return values
    errOut[0] = err[0];
    errOut[1] = err[1];
    errOut[2] = err[2];
    errDotOut[0] = errDot[0];
    errDotOut[1] = errDot[1];
    errDotOut[2] = errDot[2];
}


/*
    Calculates the error and errorDot to be used in control loops
    @param int16_t* desiredPos  : Desired position (DEGREES)
    @param int16_t* curPos      : Current position (centidegrees)
    @param float*   desVel      : Desired Velocity (DEG/SECOND)
    @param float*   curVel      : Current Velocity (DEG/SECOND)
    @param float*   err         : error (deg)
    @param float*   errDot      : errDot(deg/sec)
*/
void RBE502Robot::determineErrorFromTraj(double* desiredPos, int16_t* curPos, float* desVel, float* curVel, float* errOut, float* errDotOut){

    //Determine error in DEGREES: (wanted - current/100).
    errOut[0] = float(desiredPos[0] - curPos[0]/100.0);
    errOut[1] = float(desiredPos[1] - curPos[1]/100.0);
    errOut[2] = float(desiredPos[2] - curPos[2]/100.0);


    //Determine errorDot in DEGREES/Second
    errDotOut[0] = desVel[0] - curVel[0];
    errDotOut[1] = desVel[1] - curVel[1];
    errDotOut[2] = desVel[2] - curVel[2];
}


/*
    Determines the velocity at a set interval. curMotorVel[3] is an array that acts as the return value
    Max speed achieved on m1 with 50ms read time was 1296 (aka 1296 centidegrees per 50ms or 259.2 deg/sec)
    Max speed achieved on m1 with 25ms read time was  720 (aka  720 centidegrees per 25ms or 288.0 deg/sec)
    @param int16_t* curMotorPos : input of the current motor positions in CENTIDEGREES
    @param float* curMotorVel : output of the current motor velocities in DEG/SEC. Argument should be a [3] array to function as return value
*/
void RBE502Robot::determineVelocity(int16_t* curMotorPos, float* curMotorVel){
    const uint16_t readRate = 50; //milliseconds between each calculation
    static uint32_t lastTimeDeterminedVelocity = 0;
    static int16_t lastMotorPos[3] = {0,0,0};
    static float lastMotorVel[3] = {0,0,0};

    if(millis() - readRate > lastTimeDeterminedVelocity){
        //Ensure that not too much time has passed since this was last called to ensure very large/inaccurate velocities arent calculated. 
        if(millis() - 100 > lastTimeDeterminedVelocity){
            curMotorVel[0] = 0;
            curMotorVel[1] = 0;
            curMotorVel[2] = 0;
            Serial.println("ERROR CALCULATING VELOCITIES. LOOP NOT FAST ENOUGH. ERRATIC MOVEMENTS MAY OCCUR");
        }

        //Determine velocities
        curMotorVel[0] = curMotorPos[0] - lastMotorPos[0];
        curMotorVel[1] = curMotorPos[1] - lastMotorPos[1];
        curMotorVel[2] = curMotorPos[2] - lastMotorPos[2];
        
        //Ensure that random readings while stationary do not interfere with functionality (24 is the lowest value due to limitations of reading motor position)
        if(abs(curMotorVel[0]) <= 24){
            curMotorVel[0] = 0;
        }
        if(abs(curMotorVel[1]) <= 24){
            curMotorVel[1] = 0;
        }
        if(abs(curMotorVel[2]) <= 24){
            curMotorVel[2] = 0;
        }

        
        
        //Reset lastMotorPos to the current motor positions for next iteration
        lastMotorPos[0] = curMotorPos[0];
        lastMotorPos[1] = curMotorPos[1];
        lastMotorPos[2] = curMotorPos[2];

        //Convert to deg/sec
        curMotorVel[0] = curMotorVel[0]/5; // divide by 100 for degrees, multiply by 20 for seconds
        curMotorVel[1] = curMotorVel[1]/5;
        curMotorVel[2] = curMotorVel[2]/5;

        //Ensure no erroneous values
        if(abs(curMotorVel[0]) >= 5000){
            curMotorVel[0] = 0;
        }
        if(abs(curMotorVel[1]) >= 5000){
            curMotorVel[1] = 0;
        }
        if(abs(curMotorVel[2]) >= 5000){
            curMotorVel[2] = 0;
        }

        //Save the velocities so they can be returned until next 50ms is up
        lastMotorVel[0] = curMotorVel[0];
        lastMotorVel[1] = curMotorVel[1];
        lastMotorVel[2] = curMotorVel[2];

        //Reset timing to next calculate velocity
        lastTimeDeterminedVelocity = millis();
    }else{
        //return the last calculated velocity if waiting for next 50ms
        curMotorVel[0] = lastMotorVel[0];
        curMotorVel[0] = lastMotorVel[0];
        curMotorVel[0] = lastMotorVel[0];
    }
}



/*
    Static method ISR to read enable/disable switch
*/
void RBE502Robot::disableISR(){
    static uint32_t lastISRTime = millis();
    if(lastISRTime + 50 > millis()){return;}

    if(digitalRead(::enableSwitchPin)){
        Serial.println("Disabled");
        ::disabled = true;
    }else{
        Serial.println("Enabled");
        ::disabled = false;
    }
    lastISRTime = millis();
}




/*
    Method to be called in void setup(){} to initialize all necessary things.
*/
void RBE502Robot::begin(){
    //Initialize the servo Bus
    this->servoBus.begin(&Serial1,this->txPin,this->txFlagPin);

    //Set the pinmode of the enable/disable switch
    pinMode(::enableSwitchPin,INPUT_PULLUP);

    //Set the robot to disabled in the current switch position
    disabled = digitalRead(enableSwitchPin); //Set default position

    //Attach ISR to the switch changing position
    attachInterrupt(::enableSwitchPin,RBE502Robot::disableISR,CHANGE);

    //Setup hardware timer for monitoring the servo positions
    if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0)){
        Serial.println("Starting  ITimer0 OK, millis() = " + String(millis()));
    }else{
        Serial.println("Can't set ITimer0. Select another freq. or timer");  
    }
}


/*
    Commands the motors to servo to a given position within the given time in seconds, using the "home" offset (NOT raw values unless offset is not set/is zero). 
    If the given time is 0, moves at full speed.
    @param uint16_t motor1Pos : The position to move motor 1 to.
    @param uint16_t motor2Pos : The position to move motor 2 to.
    @param uint16_t motor3Pos : The position to move motor 3 to.
    @param uint16_t moveTime  : The time it will take to go from the starting position to the final position.
    @param bool     withOffset: Whether the input positions are meant to be raw values (false) or values relative to the given home offset (true)
*/
void RBE502Robot::setMotorPositions(int16_t motor1Pos, int16_t motor2Pos, int16_t motor3Pos, int16_t moveTime, bool withOffset){
    if(::disabled){this->disableMotors(); return;}
    int16_t motorPositions[3] = {motor1Pos, motor2Pos, motor3Pos};
    //Remove the offset before commanding the motors if necessary
    if(withOffset){
        int16_t* rawMotorPositions = this->removeZeroOffset(motorPositions);
        motor1Pos = rawMotorPositions[0];
        motor2Pos = rawMotorPositions[1];
        motor3Pos = rawMotorPositions[2];
    }

    motorPositions[0] = motor1Pos;
    motorPositions[1] = motor2Pos;
    motorPositions[2] = motor3Pos;
    
    //Check if the raw positions are within the bounds
    if(!this->checkWithinServoBounds(motorPositions, false)){db.println("command outside of servo bounds");return;}; //if trying to command to outside of bounds,  exit.

    

    this->motor1->move_time(motor1Pos,moveTime);
    this->motor2->move_time(motor2Pos,moveTime);
    this->motor3->move_time(motor3Pos,moveTime);
    db.println("Motors Set To Positions: \t" + String(motor1Pos) + '\t' + String(motor2Pos) + '\t' + String(motor3Pos) + '\t');
}

/*
    Disables the motors. Note that calling this will change the switch position for disabled to the current position. 
*/
void RBE502Robot::disableMotors(){
    ::disabled = true;
    this->setMotorSpeeds(0,0,0); //Motors cant be disabled if they are being driven by speed.
    this->motor1->disable();
    this->motor2->disable();
    this->motor3->disable();
    db.println("Motors Disabled");
}

void RBE502Robot::enableMotors(){
    ::disabled = false;
    this->motor1->enable();
    this->motor2->enable();
    this->motor3->enable();
    db.println("Motors Enabled");
}


/*
    Sets the Servo position limits for each servo in centidegrees (this is NOT offset).
    @param uint16_t m1Min   : Minimum value for motor 1 position
    @param uint16_t m1Max   : Maximum value for motor 1 position
    @param uint16_t m2Min   : Minimum value for motor 2 position
    @param uint16_t m2Max   : Maximum value for motor 2 position
    @param uint16_t m3Min   : Minimum value for motor 3 position
    @param uint16_t m3Max   : Maximum value for motor 3 position
*/
void RBE502Robot::setMotorLimits(uint16_t m1Min, uint16_t m1Max, uint16_t m2Min, uint16_t m2Max, uint16_t m3Min, uint16_t m3Max) {
    this->motor1MinPos = m1Min;
    this->motor1MaxPos = m1Max;
    this->motor2MinPos = m2Min;
    this->motor2MaxPos = m2Max;
    this->motor3MinPos = m3Min;
    this->motor3MaxPos = m3Max;
    db.println("Motor Maximum and Minimum Positions Set");
}




/*
    Returns the RAW positions of each motor in centidegrees in the form int[motor1Positon, motor2Position, motor3Position]
*/
void RBE502Robot::getRawMotorPositions(int16_t* rawMotorPositions){
    db.println("Getting raw motor positions");
    rawMotorPositions[0] = this->motor1->pos_read();
    rawMotorPositions[1] = this->motor2->pos_read();
    rawMotorPositions[2] = this->motor3->pos_read();
}

/*
    Returns the OFFSET positions of each motor in centidegrees in the form int16_t [motor1Positon, motor2Position, motor3Position]. Note that these values CAN be negative
*/
void RBE502Robot::getMotorPositions(int16_t* motorPositions){
    int16_t rawMotorPositions[3];
    this->getRawMotorPositions(rawMotorPositions);
    db.println("Converting raw motor positions to offset positions");
    motorPositions[0] = rawMotorPositions[0] - this->zeroPositons[0];
    motorPositions[1] = -(rawMotorPositions[1] - this->zeroPositons[1]);
    motorPositions[2] = -(rawMotorPositions[2] - this->zeroPositons[2]);
}


/*
    Returns the OFFSET positions of each motor in DEGREES in the form float [motor1Positon, motor2Position, motor3Position]. Note that these values CAN be negative
*/
void RBE502Robot::getMotorPositionsDeg(float* motorPositions){
    int16_t offsetMotorPos[3];
    this->getMotorPositions(offsetMotorPos);
    db.println("Converting raw motor positions to offset positions");
    motorPositions[0] = (offsetMotorPos[0] - this->zeroPositons[0])/100;
    motorPositions[1] = (offsetMotorPos[1] - this->zeroPositons[1])/100;
    motorPositions[2] = (offsetMotorPos[2] - this->zeroPositons[2])/100;
}


/*
    Sets the given positions to the zero positions of the motors
    @param uint16_t m1ZeroPos : raw position at which you want m1 to be considered at 0.
    @param uint16_t m2ZeroPos : raw position at which you want m2 to be considered at 0.
    @param uint16_t m3ZeroPos : raw position at which you want m3 to be considered at 0.
*/
void RBE502Robot::setMotorHomePositions(uint16_t m1ZeroPos, uint16_t m2ZeroPos, uint16_t m3ZeroPos){
    this->zeroPositons[0] = m1ZeroPos;
    this->zeroPositons[1] = m2ZeroPos;
    this->zeroPositons[2] = m3ZeroPos;
    db.println("Set motor zero/home positions");
}


/*
    Removes the motor position offsets from the given array of 3 values. (Converts from offset values to raw values)
    @param int16_t* offsetMotorPositons : The positions of the motors offset by the given zero position value.
*/
int16_t* RBE502Robot::removeZeroOffset(int16_t* offsetMotorPositions) {
    static int16_t retVal[3];
    retVal[0] = offsetMotorPositions[0] + this->zeroPositons[0];
    retVal[1] = -offsetMotorPositions[1] + this->zeroPositons[1];
    retVal[2] = -offsetMotorPositions[2] + this->zeroPositons[2];
    db.println("Removing zero offset (converting from offset positions to raw positions)");
    db.println(String(retVal[0]) + '\t' + String(retVal[1]) + '\t' + String(retVal[2]));
    return retVal;
}

/*
    Checks that the input values are within the bounds of the servos set using values set with .setMotorLimits()
    @param int16_t* motorPositons   : The positons of the motors in the form [motor1, motor2, motor3]
    @param bool    withOffset       : whether the values to check are offset, or raw. True is offset, False is raw. Defaults to offset (True)
*/
bool RBE502Robot::checkWithinServoBounds(int16_t* motorPositions, bool withOffset) {
    if(withOffset){//first remove the offset if necessary
        motorPositions = this->removeZeroOffset(motorPositions);
    }
    db.println("Checking if given motor position is within bounds");
    db.println(String(motorPositions[0]));
    db.println("");

    if( motorPositions[0] < this->motor1MaxPos && motorPositions[0] > this->motor1MinPos &&
        motorPositions[1] < this->motor2MaxPos && motorPositions[1] > this->motor2MinPos &&
        motorPositions[2] < this->motor3MaxPos && motorPositions[2] > this->motor3MinPos){
            db.println("---Motor Positions within bounds");
            return true;
        }else{
            db.println('---Motor Positions NOT WITHIN BOUNDS');
            return false;
        }
}




/*
    Calculates the q1,q2,q3 values of the motors assuming the standard home position of the robot. 
    @param int16_t x : X position to calculate motor positions for.
    @param int16_t y : Y position to calculate motor positions for.
    @param int16_t z : Z position to calculate motor positions for.
*/
int16_t* RBE502Robot::calcServoPosFromXYZ(int16_t x, int16_t y, int16_t z){
    db.println("Calculating Inverse Kinematics");
    static int16_t servoPos[3];
    servoPos[0] = degrees(atan2(y,x));
    servoPos[2] = 90 - degrees(acos((20000 - ((95 - z)*(95 - z)) - x*x - y*y)/20000));
    double theta1 = degrees(atan((sqrt(x*x + y*y))/(95 - z)));
    double L = sqrt(((95-z)*(95-z)) + x*x + y*y);
    double theta2 = degrees(asin((100*degrees(sin(90 - servoPos[2])))/L));
    servoPos[1] = 180 - theta1 - theta2;
    db.println("Q1: " + String(servoPos[0]) + "\tQ2: " + String(servoPos[1]) + "\tQ3: " + String(servoPos[2]));
    
    return servoPos;
}


/*
    Sets the raw speeds/efforts of the motors
    NOTE: Running this method outside of the state machine, the robot WILL exceed its bound and damage itself!
    @param
*/
void RBE502Robot::setMotorSpeeds(int16_t m1speed, int16_t m2speed, int16_t m3speed){
    // if(abs(m1speed) > 100 or abs(m2speed) > 100 or abs(m3speed) > 100){
    //     Serial.println("Motor control inputs are from -100 to 100. Enter a value within range");
    //     return;
    // }

    //Keep the motor speeds inside the allowed values. 
    m1speed = max(m1speed,-85);
    m1speed = min(m1speed,85);

    m2speed = max(m2speed,-85);
    m2speed = min(m2speed,85);

    m3speed = max(m3speed,-85);
    m3speed = min(m3speed,85);


    //Add 18 to allow movement at smallest input values (motors dont really move until ~20)
    const uint16_t minMotorVelocity = 15;
    if(m1speed < 0){
        m1speed = m1speed - minMotorVelocity;
    }else if(m1speed >0){
        m1speed = m1speed + minMotorVelocity;
    }

    if(m2speed < 0){
        m2speed = m2speed - minMotorVelocity;
    }else if(m2speed >0){
        m2speed = m2speed + minMotorVelocity;
    }

    if(m3speed < 0){
        m3speed = m3speed - minMotorVelocity;
    }else if(m3speed >0){
        m3speed = m3speed + minMotorVelocity;
    }

    //M1 min for movement is: 35
    //M2 min for movement is: 20 down and 40 up
    //M3 min for movement is: 20 down and 30 up

    this->motor1->motor_mode(m1speed * 10);
    this->motor2->motor_mode(-m2speed * 10);
    this->motor3->motor_mode(-m3speed * 10);
}




/*
    Plan a cubic trajectory from a given position to another position
    @param float  t0     : Starting time of traj (0)
    @param float  tf     : Ending time of traj
    @param float  q0     : starting joint position
    @param float  qf     : ending joint position
    @param float  q0d    : starting joint velocity
    @param float  q0f    : ending joint velocity
    @param float* A      : Output A array that fulfills the equation: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
*/
void RBE502Robot::cubicTraj(float t0, float tf, float q0, float qf, float q0d, float qfd, float* A){
    A[0] = -(q0*tf*tf*tf - qf*t0*t0*t0 - 3*q0*t0*tf*tf - q0d*t0*tf*tf*tf + 3*qf*t0*t0*tf + qfd*t0*t0*t0*tf + q0d*t0*t0*tf*tf - qfd*t0*t0*tf*tf)/((t0 - tf)*(t0*t0 - 2*t0*tf + tf*tf));
    A[1] = -(q0d*tf*tf*tf - qfd*t0*t0*t0 + q0d*t0*tf*tf - 2*q0d*t0*t0*tf + 2*qfd*t0*tf*tf - qfd*t0*t0*tf + 6*q0*t0*tf - 6*qf*t0*tf)/((t0 - tf)*(t0*t0 - 2*t0*tf + tf*tf));
    A[2] = (3*q0*t0 + 3*q0*tf - 3*qf*t0 - 3*qf*tf - q0d*t0*t0 + 2*q0d*tf*tf - 2*qfd*t0*t0 + qfd*tf*tf - q0d*t0*tf + qfd*t0*tf)/((t0 - tf)*(t0*t0 - 2*t0*tf + tf*tf));
    A[3] = -(2*q0 - 2*qf - q0d*t0 + q0d*tf - qfd*t0 + qfd*tf)/((t0 - tf)*(t0*t0 - 2*t0*tf + tf*tf));

}

/*
    Take an A matrix representing the coefficients of a (maximum of cubic) trajectory, and find the derivatives coefficients
    @param float* A     : An array of 4 values representing the coefficients of a trajectory: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    @param float* Ad    : An output array of 4 values representing the coefficients of derivative of the A trajectory.
*/
void RBE502Robot::cubicDeriv(float* A, float* Ad){
    Ad[0] = A[1];
    Ad[1] = A[2]*2;
    Ad[2] = A[3]*3;
    Ad[3] = 0;
}


/*
    Takes a relative time value, and plugs in T to the equation where A represents the coefficents: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    @param uint32_t  t      : The time value in milliseconds
    @param float*    A      : The A matrix of coeffients of the equation above
    @param float*    desired: The output, desired position, velocity and acceleration at that time [q;qdot;qdotdot] in DEG and Seconds
*/
void RBE502Robot::calcTrajPos(uint32_t t, float* A, float* desired){
    double tsec = t/1000.0;
    float Ad[4]; //A dot (velocity)
    float Add[4]; // A dot dot (acceleration)
    this->cubicDeriv(A, Ad);
    this->cubicDeriv(Ad, Add);

    // Serial.print("A Matrix: ");
    // Serial.print(A[0]);
    // Serial.print('\t');
    // Serial.print(A[1]);
    // Serial.print('\t');
    // Serial.print(A[2]);
    // Serial.print('\t');
    // Serial.print(A[3]);
    // Serial.print('\n');
    
    desired[0] = (A[0]*1 + A[1]*tsec + A[2]*tsec*tsec + A[3]*tsec*tsec*tsec)/100.0;
    desired[1] = (Ad[0]*1 + Ad[1]*tsec + Ad[2]*tsec*tsec + Ad[3]*tsec*tsec*tsec)/100.0;
    desired[2] = (Add[0]*1 + Add[1]*tsec + Add[2]*tsec*tsec + Add[3]*tsec*tsec*tsec)/100.0;
}



/*
    Enables/disables debugging print messages.
    @param bool enableDebug : Enables debugging if true, disables if false
*/
void RBE502Robot::debug(bool enableDebug){
    this->_debug = enableDebug;
    db.debug(this->_debug); 
}