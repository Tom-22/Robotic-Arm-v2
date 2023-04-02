#include <Stepper.h>

// M1 Stepper ##############################################################################################################################
const int M1stepsPerRevolution = 200 * 64 * 2;  // number of steps per full motor revolution (1.8 degrees * 16 microsteps * 2 gear ratio)
const int M1dirPin = 6;  // direction pin
const int M1pulPin = 7;  // pulse pin
const int M1enPin = 9;   // enable pin
// #########################################################################################################################################

// M2 Stepper ##############################################################################################################################
const int M2stepsPerRevolution = 200 * 64 * 2;  // number of steps per full motor revolution (1.8 degrees * 16 microsteps * 2 gear ratio)
const int M2dirPin = 3;  // direction pin
const int M2pulPin = 4;  // pulse pin
const int M2enPin = 5;   // enable pin
// #########################################################################################################################################

// M3 Stepper ##############################################################################################################################
const int M3stepsPerRevolution = 200 * 64 * 2;  // number of steps per full motor revolution (1.8 degrees * 16 microsteps * 2 gear ratio)
const int M3dirPin = 10;  // direction pin
const int M3pulPin = 11;  // pulse pin
const int M3enPin = 12;   // enable pin
// #########################################################################################################################################

Stepper M1stepper(M1stepsPerRevolution, M1dirPin, M1pulPin); // initialize the stepper motor object
Stepper M2stepper(M2stepsPerRevolution, M2dirPin, M2pulPin); // initialize the stepper motor object
Stepper M3stepper(M3stepsPerRevolution, M3dirPin, M3pulPin); // initialize the stepper motor object

int M1xCoord = 0; // variable to keep track of the current x coordinate
int M1xTarget = 0; // variable to store the target x coordinate

int M2xCoord = 0; // variable to keep track of the current x coordinate
int M2xTarget = 0; // variable to store the target x coordinate

int M3xCoord = 0; // variable to keep track of the current x coordinate
int M3xTarget = 0; // variable to store the target x coordinate

int direction;

void setup() {


  Serial.begin(9600);
  Serial.println("System initialization complete.");

  pinMode(M1enPin, OUTPUT);  // set enable pin as output
  pinMode(M2enPin, OUTPUT);  // set enable pin as output
  pinMode(M3enPin, OUTPUT);  // set enable pin as output

  digitalWrite(M1enPin, HIGH); // disable the stepper motor
  digitalWrite(M2enPin, HIGH); // disable the stepper motor
  digitalWrite(M3enPin, HIGH); // disable the stepper motor
  Serial.println("Activating motors ...");
  delay(1000);
  Serial.println("Motor 1 activated");
  digitalWrite(M1enPin, LOW); // enable the stepper motor
  delay(1250);
  Serial.println("Motor 2 activated");
  digitalWrite(M2enPin, LOW); // enable the stepper motor
  delay(1500);
  Serial.println("Motor 3 activated");
  digitalWrite(M3enPin, LOW); // enable the stepper motor
  delay(1750);
  Serial.println("Motors activated successfully.");
  delay(2000);
  Serial.println("Robotic Arm start-up complete");
  Serial.println();
  delay(1000);
  Serial.println("Waiting for Axis (M1, M2 or M3)...");
}

void loop() {
  checkSerial();
  moveMotor();
}

void checkSerial() {

  if (Serial.available() > 0) {   // Check if there's serial data available
    //Serial.println();
    //Serial.println("Reading input ...");
    String input = Serial.readString();   // Read the serial input as a string
    input.trim();

    if (input == "0") {
      goto Clear;
    }

    else {


      if (input == "M1") {   // Check if the input is "M1"
        // Wait for a numerical input
        Serial.println("Waiting for M1 X coordinate: ");
        while (Serial.available() == 0) {}   // Wait until there's serial data available
        int numInput = Serial.parseInt();   // Read the numerical input

        int M1input = numInput;  // read the X coordinate value
        M1xTarget = M1input; // update the target coordinate if input is not zero
        //Serial.print("M1 Target acuired ");
        //Serial.println(M1xTarget);
        goto Clear;
      }

      else if (input == "M2") {   // Check if the input is "M2"
        // Wait for a numerical input
        Serial.println("Waiting for M2 X coordinate: ");
        while (Serial.available() == 0) {}   // Wait until there's serial data available
        int numInput = Serial.parseInt();   // Read the numerical input

        int M2input = numInput;  // read the X coordinate value
        M2xTarget = M2input; // update the target coordinate if input is not zero
        //Serial.print("M2 Target acuired ");
        //Serial.println(M2xTarget);
        goto Clear;
      }

      else if (input == "M3") {   // Check if the input is "M3"
        // Wait for a numerical input
        Serial.println("Waiting for M3 X coordinate: ");
        while (Serial.available() == 0) {}   // Wait until there's serial data available
        int numInput = Serial.parseInt();   // Read the numerical input

        int M3input = numInput;  // read the X coordinate value
        M3xTarget = M3input; // update the target coordinate if input is not zero
        //Serial.print("M3 Target acuired ");
        //Serial.println(M3xTarget);
        goto Clear;
      }

      else if (input == "Home") {   // Check if the input is "Home"
        Serial.println("Homing in 5 seconds");
        delay(5000);
        homeMotor();
        goto Clear;
      }

      else if (input == "On") {   // Check if the input is "On"
        Serial.println("Initiating ...");
        delay(1000);
        onMotor();
        goto Clear;
      }

      else if (input == "Off") {   // Check if the input is "Off"
        Serial.println("Initiatin ...");
        delay(1000);
        offMotor();
        goto Clear;
      }

      else if (input == "Help") {   // Check if the input is "Help"
        helpArm();
        goto Clear;
      }

      else if (input == "Shut Down") {   // Check if the input is "Shut Down"
        shutDown();
        goto Clear;
      }

      else if (input == "Test") {   // Check if the input is "Test"
        testMotor();
        goto Clear;
      }

      //Serial.print("Invalid Input: ");
      //Serial.print(input);
      //Serial.println();


    }
Clear:
    //Serial.println();
    //Serial.println("Finished.");

    Serial.flush();
    memset(&input, 0, sizeof(input));  // clear input, set it to 0
    input = ("0");
    input.trim();

    delay(100);
  }
}

void moveMotor() {

  int M1distance = M1xTarget - M1xCoord;
  int M2distance = M2xTarget - M2xCoord;
  int M3distance = M3xTarget - M3xCoord;

  if (M1distance != 0) { // check if the target coordinate is different from the current coordinate
    direction = M1distance > 0 ? 1 : -1; // determine the direction to move the motor
    int M1stepsToMove = abs(M1distance) / 360.0 * M1stepsPerRevolution; // calculate the number of steps to move based on the distance between the target and current coordinates

    M1stepper.setSpeed(5); // set the motor speed (5 RPM)
    digitalWrite(M1dirPin, direction > 0 ? HIGH : LOW); // set the direction to move clockwise or counter-clockwise
    M1stepper.step(M1stepsToMove * direction); // move the stepper motor to the desired position

    delay(100); // wait for 1 second
    M1xCoord = M1xTarget; // update the current x coordinate to the target x coordinate

    Serial.println();
    Serial.print("M1 target  : ");
    Serial.println(M1xTarget);
    Serial.print("M1 position: ");
    Serial.println(M1xCoord);
  }

  if (M2distance != 0) { // check if the target coordinate is different from the current coordinate
    direction = M2distance > 0 ? 1 : -1; // determine the direction to move the motor
    int M2stepsToMove = abs(M2distance) / 360.0 * M2stepsPerRevolution; // calculate the number of steps to move based on the distance between the target and current coordinates

    M2stepper.setSpeed(5); // set the motor speed (5 RPM)
    digitalWrite(M2dirPin, direction > 0 ? HIGH : LOW); // set the direction to move clockwise or counter-clockwise
    M2stepper.step(M2stepsToMove * direction); // move the stepper motor to the desired position

    delay(100); // wait for 1 second
    M2xCoord = M2xTarget; // update the current x coordinate to the target x coordinate
    Serial.println();
    Serial.print("M2 target  : ");
    Serial.println(M2xTarget);
    Serial.print("M2 position: ");
    Serial.println(M2xCoord);
  }

  if (M3distance != 0) { // check if the target coordinate is different from the current coordinate
    direction = M3distance > 0 ? 1 : -1; // determine the direction to move the motor
    int M3stepsToMove = abs(M3distance) / 360.0 * M3stepsPerRevolution; // calculate the number of steps to move based on the distance between the target and current coordinates

    M3stepper.setSpeed(5); // set the motor speed (5 RPM)
    digitalWrite(M3dirPin, direction > 0 ? HIGH : LOW); // set the direction to move clockwise or counter-clockwise
    M3stepper.step(M3stepsToMove * direction); // move the stepper motor to the desired position

    delay(100); // wait for 1 second
    M3xCoord = M3xTarget; // update the current x coordinate to the target x coordinate
    Serial.println();
    Serial.print("M3 target  : ");
    Serial.println(M3xTarget);
    Serial.print("M3 position: ");
    Serial.println(M3xCoord);
  }
}

void moveMotorSimultan() {
  
  int M1distance = M1xTarget - M1xCoord;
  int M2distance = M2xTarget - M2xCoord;
  int M3distance = M3xTarget - M3xCoord;

  int M1stepsToMove = abs(M1distance) / 360.0 * M1stepsPerRevolution; // calculate the number of steps to move based on the distance between the target and current coordinates
  int M2stepsToMove = abs(M2distance) / 360.0 * M2stepsPerRevolution; // calculate the number of steps to move based on the distance between the target and current coordinates
  int M3stepsToMove = abs(M3distance) / 360.0 * M3stepsPerRevolution; // calculate the number of steps to move based on the distance between the target and current coordinates

  M1stepper.setSpeed(3); // set the motor speed (5 RPM)
  M2stepper.setSpeed(3); // set the motor speed (5 RPM)
  M3stepper.setSpeed(3); // set the motor speed (5 RPM)

  int M1direction = M1distance > 0 ? 1 : -1; // determine the direction to move the motor
  digitalWrite(M1dirPin, M1direction > 0 ? HIGH : LOW); // set the direction to move clockwise or counter-clockwise
  int M2direction = M2distance > 0 ? 1 : -1; // determine the direction to move the motor
  digitalWrite(M2dirPin, M2direction > 0 ? HIGH : LOW); // set the direction to move clockwise or counter-clockwise
  int M3direction = M3distance > 0 ? 1 : -1; // determine the direction to move the motor
  digitalWrite(M3dirPin, M3direction > 0 ? HIGH : LOW); // set the direction to move clockwise or counter-clockwise

  int M1stepsMoved = 0;
  int M2stepsMoved = 0;
  int M3stepsMoved = 0;

  while (M1stepsMoved < M1stepsToMove || M2stepsMoved < M2stepsToMove || M3stepsMoved < M3stepsToMove) {
    if (M1stepsMoved < M1stepsToMove) {
      M1stepper.step(M1direction);
      M1stepsMoved++;
    }
    if (M2stepsMoved < M2stepsToMove) {
      M2stepper.step(M2direction);
      M2stepsMoved++;
    }
    if (M3stepsMoved < M3stepsToMove) {
      M3stepper.step(M3direction);
      M3stepsMoved++;
    }
  }

  delay(250); // wait for 250ms
  M1xCoord = M1xTarget; // update the current x coordinate to the target x coordinate
  M2xCoord = M2xTarget; //
  M3xCoord = M3xTarget; //
}

void testMotor() {
  Serial.println("Starting test in 5 seconds");
  delay(5000);
  Serial.println("Simultanious axis movement activated ");

  M1xTarget = 1; // update the target coordinate
  M2xTarget = 1; // update the target coordinate
  M3xTarget = 1; // update the target coordinate
  delay(1000);
  moveMotorSimultan();
  M1xTarget = 90; // update the target coordinate
  M2xTarget = 90; // update the target coordinate
  M3xTarget = 90; // update the target coordinate
  moveMotorSimultan();
  M1xTarget = 1; // update the target coordinate
  M2xTarget = 1; // update the target coordinate
  M3xTarget = 1; // update the target coordinate
  moveMotorSimultan();
  delay(1000);
  Serial.println("Simultanious axis movement deactivated ");
  Serial.println("Test finished.");
  delay(1000);
}

void offMotor() {
  Serial.println("!WARNING!");
  delay(2000);
  Serial.println("Robot Arm will deactivate!");
  delay(3000);
  Serial.println("Arm will loose all power and fall!");
  delay(4000);
  Serial.println("Deactivation in 10 seconds ...");
  delay(1000);
  Serial.println("9 seconds ...");
  delay(1000);
  Serial.println("8 seconds ...");
  delay(1000);
  Serial.println("7 seconds ...");
  delay(1000);
  Serial.println("6 seconds ...");
  delay(1000);
  Serial.println("5 seconds ...");
  delay(1000);
  Serial.println("4 seconds ...");
  delay(1000);
  Serial.println("3 seconds ...");
  delay(1000);
  Serial.println("2 seconds ...");
  delay(1000);
  Serial.println("1 seconds ...");
  delay(1000);
  Serial.println("!POWER DOWN!");
  delay(1000);

  digitalWrite(M1enPin, HIGH); // disable the stepper motor
  digitalWrite(M2enPin, HIGH); // disable the stepper motor
  digitalWrite(M3enPin, HIGH); // disable the stepper motor
  delay(1000);
  Serial.println("... Robot arm powered down ...");
  delay(5000);
}

void onMotor() {
  Serial.println("!WARNING!");
  delay(2000);
  Serial.println("Robot Arm will activate!");
  delay(3000);
  Serial.println("Arm will gain power and lock itself in place!");
  delay(4000);
  Serial.println("Activation in 5 seconds ...");
  delay(5000);
  Serial.println("!POWER ON!");
  delay(1000);

  digitalWrite(M1enPin, LOW); // disable the stepper motor
  digitalWrite(M2enPin, LOW); // disable the stepper motor
  digitalWrite(M3enPin, LOW); // disable the stepper motor
  delay(1000);
  Serial.println("... Robot arm fully powered ...");
  delay(5000);
}

void homeMotor() {
  Serial.println("Moving to home position");
  delay(2000);
  M1xTarget = 1; // update the target coordinate
  M2xTarget = 1; // update the target coordinate
  M3xTarget = 1; // update the target coordinate
}

void shutDown() {
  Serial.println("!WARNING!");
  delay(2000);
  Serial.println("Robot Arm will shut down!");
  delay(3000);
  Serial.println("Arm will move to home position, and power down completely!");
  Serial.println("System will be unresponsive!");
  Serial.println("Reboot is required to power on");
  delay(5000);
  Serial.println("Shutdown sequence initing ...");
  delay(1000);
  Serial.println("5 seconds ...");
  delay(1000);
  Serial.println("4 seconds ...");
  delay(1000);
  Serial.println("3 seconds ...");
  delay(1000);
  Serial.println("2 seconds ...");
  delay(1000);
  Serial.println("1 seconds ...");
  delay(1000);
  Serial.println("!SHUTTING DOWN!");
  delay(1000);
  homeMotor();
  delay(5000);
  Serial.println("Good night.");
  delay(1000);

  digitalWrite(M1enPin, HIGH); // disable the stepper motor
  delay(500);
  digitalWrite(M2enPin, HIGH); // disable the stepper motor
  delay(500);
  digitalWrite(M3enPin, HIGH); // disable the stepper motor
  delay(500);
ZZZ:
  Serial.println("... zzz ...");
  delay(60000);
  goto ZZZ;
}

void helpArm() {
  Serial.println();
  Serial.println();
  Serial.println("INPUT:| DESCRIPTION:");
  Serial.println();
  Serial.println("Help  : Help");
  Serial.println("On    : Turns on all axis");
  Serial.println("Off   : Turns off all axis");
  Serial.println("Home  : Drives all axis to home position");
  Serial.println("Homing: Homes all axis (not implemented yet)");
  Serial.println("Test  : Test cycle.");
  Serial.println("M1    : Designate arm horizontal rotation axis");
  Serial.println("M2    : Designate arm first vertical rotation axis");
  Serial.println("M3    : Designate arm second vertical rotation axis");
  Serial.println();
  Serial.println("To input a coordinate, first designate an axis (M1, M2 or M3");
  Serial.println("Press enter ..");
  Serial.println("Then input a coordinate (number). i.e : 180");
  Serial.println("Press enter ..");
  Serial.println("The axis will drive to that coordinate");
  Serial.println();
  Serial.println();
}
