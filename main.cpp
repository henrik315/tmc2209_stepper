//TMC2209, ESP32: Sensorless homing in both directions with AccelStepper -> based on 

#include <TMCStepper.h>
#include <AccelStepper.h>

#define EN           33 //Enable pin for the stepper motor driver (active LOW)
#define DIR          13 //Direction pin for controlling the step direction
#define STEP         14 //Step pin for controlling step pulses
#define STALL      16   //Connected to DIAG pin on the TMC2209 for stall detection

// Hardware Serial Port Configuration for TMC2209
#define SERIAL_PORT Serial1 //TMC2209 HardwareSerial port

// TMC2209 Driver Configuration
#define driver_ADDRESS 0b00 //Pins MS1 and MS2 connected to GND.
#define STALL_VALUE 20      //Stallguard values for each driver(0-255), higher number -> higher sensitivity.
#define RA_SENSE 0.11f      //Sense resistor value, match to your driver

// Initialize TMC2209Stepper object
TMC2209Stepper driver(&SERIAL_PORT, RA_SENSE, driver_ADDRESS);

// Constants for stepper configuration
constexpr uint32_t steps_per_round = 750; // Number of steps per motor revolution

AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP, DIR); // Initialize AccelStepper in driver mode

// Global variables for homing and position tracking
bool startup = true; // set false after homing
bool isStalled = false;
bool dir = true;
long closedPosition = 0;

void stallInterruptX(){ // flag set for motor A when motor stalls
isStalled = true;
}

void resetError() {
  Serial.print("Disable and enable ENN Pin to reset DIAG.");
  isStalled = false;
  digitalWrite(EN, HIGH);
  delay(1000);
  digitalWrite(EN, LOW);
}

void motorAHome()
{
  //-----------move to completely open----------
  stepper.move(300000); // Move 3000000 steps 
  while(1)
    {
      stepper.run();

      if(isStalled){
        stepper.stop();
        stepper.setCurrentPosition(0);
        Serial.print("Stalled 1");
        break; 
      }
      
    }
    isStalled = false;
    delay(1000);  //Important so it doesnt stall again immediately

  //-----------move to completely close----------

  stepper.move(-300000); // Move -3000000 steps 
  while(1)
    {
      stepper.run();

      if(isStalled){
        stepper.stop();
        closedPosition = stepper.currentPosition();
        Serial.print(closedPosition);
        Serial.print("Stalled 2");
        break; 
      }
      
    }
    
    isStalled = false;
    delay(100);

    stepper.setMaxSpeed(10*steps_per_round);
    stepper.setAcceleration(50*steps_per_round);
    stepper.move(10000);                           //back up from stall position


    Serial.println("Homing finished. Range: ");
    Serial.println(closedPosition);
    delay(5000);
}

void setup() {
    
    SERIAL_PORT.begin(115200);      //HW UART drivers
    Serial.begin(115200);           //Init Serial Output

    attachInterrupt(digitalPinToInterrupt(STALL), stallInterruptX, RISING);
 
    driver.begin();             // Initiate pins and registeries
    driver.rms_current(700);    // Set stepper current to 700mA.
    //driver.en_pwm_mode(1);    // Enable extremely quiet stepping
    driver.pwm_autoscale(1);
    driver.microsteps(16);
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.SGTHRS(STALL_VALUE);

    stepper.setMaxSpeed(5*steps_per_round);  //set initial max speed
    stepper.setAcceleration(30*steps_per_round); //set initial acceleration
    stepper.setEnablePin(EN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
    
    motorAHome();
}

void loop() {
  if(!stepper.isRunning())
  {
    Serial.print(stepper.currentPosition());
    if(dir){stepper.moveTo(closedPosition *(0.75));} //move between 75% of max
    else {stepper.moveTo(closedPosition *(0.25));}   //and 25% of max
    dir=!dir;
    
  }
  stepper.run();
}