/*
 Name:		Python_Arduino_Joystick.ino
 Created:	11/14/2020 11:07:48 AM
 Author:	sar
*/

#include <MultiStepper.h>
#include<AccelStepper.h>

int Xpin = 2;
int Ypin = 3;
int ButtonPin = 12;
int buttonState1 = 1;
int buttonState2 = 1;
bool X_status = false;
bool Y_status = false;

int initialSensorValueX = 0;
int initialSensorValueY = 0;
int SensorValueY = 0;
int SensorValueX = 0;
int sensitivity = 50;
int directionX = 0;
int directionY = 0;
long VelocityX = 0;
long VelocityY = 0;
int MaxSpeed(250);



AccelStepper StepperY(AccelStepper::FULL4WIRE, 4, 5, 6, 7);
AccelStepper StepperX(AccelStepper::FULL4WIRE, 8, 9, 10, 11);


const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
boolean action = false;
boolean Joystick = false;
boolean capture = false;
char messageFromPC[buffSize] = { 0 };

int cap_x = 0;
int cap_y = 0;
int temp_coord = 0;
int newDirection = 0;
int newStepInterval = 0;
int newSpeed = 0;


void setup() {
    Serial.begin(9600);
    pinMode(Xpin, INPUT);
    pinMode(Ypin, INPUT);
    pinMode(ButtonPin, INPUT_PULLUP);
Serial.begin(9600);
Serial.println("Homing");
//homeseq(StepperX, Xpin, X_status);
Serial.println("X-Homed");
StepperX.setCurrentPosition(0);
//homeseq(StepperY, Ypin, Y_status);
StepperY.setCurrentPosition(0);
Serial.println("Y-Homed");

StepperX.setMaxSpeed(MaxSpeed);
StepperX.setAcceleration(200);
StepperY.setMaxSpeed(MaxSpeed);
StepperY.setAcceleration(200);
delay(1000);
initialSensorValueY = analogRead(A0);
initialSensorValueX = analogRead(A1);
PrintPosition();
Serial.println("<Arduino is ready>"); // tell the PC we are ready
}

void loop() {
    getProgramFromPC();
    JoystickProgram();
    CaptureProgram();
}
void getProgramFromPC() {
    if (Serial.available() > 0) {
        char x = Serial.read();
        if (x == endMarker) {
            readInProgress = false;
            newDataFromPC = true;
            inputBuffer[bytesRecvd] = 0;
            parseProgram();
        }
        if (readInProgress) {
            inputBuffer[bytesRecvd] = x;
            bytesRecvd++;
            if (bytesRecvd == buffSize) {
                bytesRecvd = buffSize - 1;
            }
        }
        if (x == startMarker) {
            bytesRecvd = 0;
            readInProgress = true;
        }
    }
}
void parseProgram() {
    if (newDataFromPC) {
        newDataFromPC = false;
        int Indx = atoi(inputBuffer);
        if (Indx == 0) {
            Joystick = true;
        }
        if (Indx == 1) {
            Serial.println("Capture Mode");
            capture = true;

        }
    }

}
void JoystickProgram() {
    while (Joystick) {
        DirectionSenseX();
        DirectionSenseY();
        MapVelocity();
        StepperX.setSpeed(VelocityX);
        StepperY.setSpeed(VelocityY);
        MotorMoveX();
        MotorMoveY();
        GetPosition();
    }
}
void GetPosition() {
    buttonState2 = digitalRead(ButtonPin);
    if (digitalRead(ButtonPin) == 0 && buttonState2 != buttonState1) {
        Serial.print("<");
        Serial.print(StepperX.currentPosition());
        Serial.println(">");
        Serial.print("<");
        Serial.print(StepperY.currentPosition());
        Serial.println(">");
        Joystick = false;
    }
    buttonState1 = buttonState2;
}
void CaptureProgram() {
    while (capture) {
        while(Serial.peek() == -1) {
        }
        ReceivePosition();
    }
}
void ReceivePosition() {
    //Serial.println("received");
    if (Serial.available() > 0) {
        char x = Serial.read();
        if (x == endMarker) {
            readInProgress = false;
            newDataFromPC = true;
            inputBuffer[bytesRecvd] = 0;
            parseLocation();
        }
        if (readInProgress) {
            inputBuffer[bytesRecvd] = x;
            bytesRecvd++;
            if (bytesRecvd == buffSize) {
                bytesRecvd = buffSize - 1;
            }
        }
        if (x == startMarker) {
            bytesRecvd = 0;
            readInProgress = true;
        }
    }
}
void parseLocation() {
    char* strtokIndx; 

    strtokIndx = strtok(inputBuffer, ","); 
    cap_x = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ","); 
    cap_y = atoi(strtokIndx);
    MotorDriver();
}
void MotorDriver() {
    StepperX.setMaxSpeed(500);
    StepperX.setMaxSpeed(500);
    StepperY.setAcceleration(100);
    StepperY.setAcceleration(100);
    StepperDrive(StepperX, cap_x);
    StepperDrive(StepperY, cap_y);
    if (StepperX.distanceToGo() == 0 && StepperY.distanceToGo() == 0) {
        capture = false;
        Serial.println("<Moved>");
    }
}
void StepperDrive(AccelStepper& stepper, int x) {
    stepper.moveTo(x);
    while (stepper.distanceToGo()!= 0) {
        stepper.run();
    }
    stepper.disableOutputs();
    
}


//
//Movement Algorithms
//


void homeseq(AccelStepper& stepper, int pin, bool status) {
    while (status == 0) {
        stepper.setMaxSpeed(100);
        stepper.setSpeed(100);
        status = home(stepper, pin, status);
    }
    delay(1000);
    backup(stepper, pin);
}
bool home(AccelStepper& stepper, int pin, bool status) {
    while (digitalRead(pin)) {
        stepper.runSpeed();
    }
    stepper.disableOutputs();
    return true;
}
void backup(AccelStepper& stepper, int pin) {
    while (!digitalRead(pin)) {
        stepper.setSpeed(-50);
        stepper.runSpeed();
    }
    stepper.disableOutputs();
}
void PrintPosition() {
    Serial.print("Current Position is ");
    Serial.print(StepperX.currentPosition());
    Serial.print(", ");
    Serial.println(StepperY.currentPosition());
}
void MotorMoveX() {
    if (directionX == 0) {
        StepperX.disableOutputs();
    }
    else {
        StepperX.runSpeed();
    }
}
void DirectionSenseX() {
    SensorValueX = analogRead(A0);
    if (SensorValueX >= initialSensorValueX + sensitivity) {
        directionX = 1;
    }
    else if (SensorValueX <= initialSensorValueX - sensitivity) {
        directionX = -1;
    }
    else {
        directionX = 0;
    }
}
void MotorMoveY() {
    if (directionY == 0) {
        StepperY.disableOutputs();
    }
    else {
        StepperY.runSpeed();
    }
}
void DirectionSenseY() {
    SensorValueY = analogRead(A1);
    if (SensorValueY >= initialSensorValueY + sensitivity) {
        directionY = 1;
    }
    else if (SensorValueY <= initialSensorValueY - sensitivity) {
        directionY = -1;
    }
    else {
        directionY = 0;
    }
}
void MapVelocity() {
    VelocityX = map(SensorValueX, initialSensorValueX, (2 * initialSensorValueX), 0, MaxSpeed);
    VelocityY = map(SensorValueY, initialSensorValueY, (2 * initialSensorValueY), 0, MaxSpeed);
}

