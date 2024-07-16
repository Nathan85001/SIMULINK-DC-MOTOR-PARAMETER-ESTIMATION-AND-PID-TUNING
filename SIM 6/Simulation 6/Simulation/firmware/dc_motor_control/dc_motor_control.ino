/******************************************
   COURSE  : MTE504 MECHATRONICS II
   PROJECT : MONITOR SPEED CONTROL
   DATE    : 13/08/2021
 *****************************************/

/***** LIBRARIES *****/
#include <Encoder.h>
#include <TimerOne.h>
#include <PID_v1.h>
/********************/

/***** VARIABLES *****/

// Serial Port Used
#define PORT Serial

// PWM value
double valuePWM = 0;

// Motor PWM Pin
byte pinMotor = 5;
byte dirMotor = 7;

// ENCODER
byte pinCHA = 2;

// setpoint
double sp = 0;

// Encoder number  of pulses per revolution
float PulsesPerRevolution = 420;

// Calculated speed in RPM
volatile double speedRPM = 0;

// Current count of incoming encoder pulses
volatile float PulseCount = 0.0;

// TIMER
float timerInterval = 0.1;

// PID
double Kp =4.59580301063121, Ki = 3.37849921341494, Kd = -0.353378872422161;
PID myPID(&speedRPM, &valuePWM, &sp, Kp, Ki, Kd, DIRECT);

/********************/


// PID controller action
void setMotor()
{
  myPID.Compute();

  // Windup
  if (valuePWM > 255)
    valuePWM = 255;
  if (valuePWM < 0)
    valuePWM = 0;

  analogWrite(pinMotor, valuePWM);
}

// Timer Interrupt Service Routine
void tmrISR()
{
  // Disable interrupt
  noInterrupts();

  speedRPM = ((PulseCount / timerInterval) / PulsesPerRevolution) * 60.0;

  // PID Action
  setMotor();

  PORT.print(sp);
  PORT.print(",");
  PORT.println(speedRPM);
  PulseCount = 0;




  // Enable Interrupt
  interrupts();
}

// External Interrupt Service Routine
void externalISR()
{
  PulseCount++;
}

void setup()
{
  // Setup serial baudrate
  PORT.begin(115200);

  // Set pins as output
  pinMode(pinMotor, OUTPUT);

  // Default to 0
  digitalWrite(pinMotor, LOW);

  pinMode(dirMotor, OUTPUT);

  digitalWrite(dirMotor, LOW);


  // Encoder PIN
  pinMode(pinCHA, INPUT); // Set encoder pin as input to receive pulse train
}

void loop()
{
}

void serialEvent()
{
  // Read command characters until \n is received
  auto IncomingCommand = PORT.readStringUntil('\n');
  const auto Command = IncomingCommand.substring(0, IncomingCommand.indexOf(','));
  auto StringData = IncomingCommand.substring(IncomingCommand.indexOf(',') + 1, IncomingCommand.length());
  PORT.flush();
  // Parsing command
  if (Command == "GO")
  {
    // Start streaming data
    Timer1.initialize(timerInterval * 1000000);
    // Attach ISR
    Timer1.attachInterrupt(tmrISR);

    // Enable external interrupt (Rising Edge)
    attachInterrupt(digitalPinToInterrupt(pinCHA), externalISR, RISING);

    // PID
    myPID.SetMode(AUTOMATIC);

  }
  if (Command == "END")
  {
    // Stop Motor
    analogWrite(pinMotor, 0.0);
    //Detach ISR
    Timer1.detachInterrupt();
    detachInterrupt(digitalPinToInterrupt(pinCHA));
  }

  // Change PWM
  if (Command == "PWM")
  {
    // Send PWM value
    valuePWM = StringData.toInt();
    analogWrite(pinMotor, valuePWM);
  }

  // Set speed
  if (Command == "SP")
  {
    // Set point
    sp = StringData.toInt();
    setMotor();
  }

}
