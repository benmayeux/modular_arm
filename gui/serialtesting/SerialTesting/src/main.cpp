#include <Arduino.h>

// #include <MicroSpotLeg.h>
#include <serialReader.h>

String command[7] = {"N/A"};
serialReader sr;

// MicroSpotLeg LF(27,26,25,true,true); // -15, 5, 5  // Ankle + is forward. Knee + is forward. Hip + is up
// MicroSpotLeg RF(16,17,18,false,true); // 0, 5, -10
// MicroSpotLeg LR(14,12,13,true,false); // 5, 5, 0
// MicroSpotLeg RR(4,2,15,false,false); // 12, 0, -5

uint32_t lastTime = millis();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  // LF.setup(-15,5,5);
  // LR.setup(0,5,-3);
  // RF.setup(5,5,0);
  // RR.setup(12,0,-5);

  // LF.setup(-15,-7,-15);
  // LR.setup(0,-7,-23);
  // RF.setup(5,-7,-20);
  // RR.setup(12,-12,-25);
  sr.debug(false);
}

void loop()
{
  int numVars = 0;
  if (sr.monitorForCommands())
  {
    numVars = sr.getCommand(command);
    Serial.print("Received new command: ");
    Serial.println(command[0]);
    Serial.print("Values of: ");
    for (int i = 0; i < (numVars - 1); i++)
    {
      Serial.print(command[i + 1] + ", ");
    }
    if (numVars > 0)
      Serial.println(command[numVars]);
  }

  // if(command[0] == "LF"){
  //   LF.setAngles(command[1].toInt(), command[2].toInt(), command[3].toInt());
  //   Serial.println("Setting LF");
  // }
  // if(command[0] == "LR"){
  //   LR.setAngles(command[1].toInt(), command[2].toInt(), command[3].toInt());
  //   Serial.println("Setting LR");
  // }
  // if(command[0] == "RF"){
  //   RF.setAngles(command[1].toInt(), command[2].toInt(), command[3].toInt());
  //   Serial.println("Setting RF");
  // }
  // if(command[0] == "RR"){
  //   RR.setAngles(command[1].toInt(), command[2].toInt(), command[3].toInt());
  //   Serial.println("Setting RR");
  // }
  // if(command[0] == "ALL"){
  //   LF.setAngles(command[1].toInt(), command[2].toInt(), command[3].toInt());
  //   LR.setAngles(command[1].toInt(), command[2].toInt(), command[3].toInt());
  //   RF.setAngles(command[1].toInt(), command[2].toInt(), command[3].toInt());
  //   RR.setAngles(command[1].toInt(), command[2].toInt(), command[3].toInt());
  //   Serial.println("Setting All");
  // }
  // if(command[0] == "CUBICTRAJANGLE"){
  //   int8_t angles[3];
  //   angles[0] = command[1].toInt();
  //   angles[1] = command[2].toInt();
  //   angles[2] = command[3].toInt();
  //   LF.cubicTrajectoryAngle(angles,command[4].toInt());
  //   LR.cubicTrajectoryAngle(angles,command[4].toInt());
  //   RF.cubicTrajectoryAngle(angles,command[4].toInt());
  //   RR.cubicTrajectoryAngle(angles,command[4].toInt());
  //   Serial.println("Starting Cubic Trajectory");
  // }
  // if(command[0] == "IK"){
  //   float XYZ[3];
  //   XYZ[0] = command[1].toFloat();
  //   XYZ[1] = command[2].toFloat();
  //   XYZ[2] = command[3].toFloat();
  //   RF.solveIK(XYZ[0], XYZ[1], XYZ[2]);
  //   RF.setAngles(RF.IKSol[0], RF.IKSol[1], RF.IKSol[2]);
  // }
  // if(command[0] == "HOME"){
  //   RF.setAngles(0, 0, 0);
  //   LF.setAngles(0, 0, 0);
  //   LR.setAngles(0, 0, 0);
  //   RR.setAngles(0, 0, 0);
  // }

  command[0] = "";
  // command[1] = "";
  // command[2] = "";
  // command[3] = "";
  for (int i = 0; i < (numVars); i++)
  {
    command[i + 1] = "";
  }

  // put your main code here, to run repeatedly:
  // LF.stateMachine();
  // LR.stateMachine();
  // RF.stateMachine();
  // RR.stateMachine();

  if (millis() - lastTime >= 50) // test whether the period has elapsed
  {
    // Testing reading from the arduino
    int value = random(10); // Generate a random value between 0 and 10

    // Send the random value through serial communication
    Serial.print("Torque: ");
    Serial.println(value);
    lastTime = millis(); // IMPORTANT to save the start time
  }
}