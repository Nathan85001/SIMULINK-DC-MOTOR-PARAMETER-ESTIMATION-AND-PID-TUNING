/******************************************
   COURSE  : MTE504 MECHATRONICS II
   PROJECT : MONITOR SPEED
   DATE    : 10/08/2021
 *****************************************/

/***** LIBRARIES *****/
#include <Encoder.h>
#include <TimerOne.h>
/********************/

/***** VARIABLES *****/

// Serial Port Used
#define PORT Serial

// PWM value
int valuePWM = 0;

// Motor PWM Pin
byte pinMotor = PA10;

// Direvction Pin
byte pinDir = PB3;

// ENCODER
byte pinCHA = PA1;
byte pinCHB = PA0;

// Encoder number  of pulses per revolution
float PulsesPerRevolution = 420;

// Calculated speed in RPM
volatile float speedRPM = 0;

// Current count of incoming encoder pulses
volatile float PulseCount = 0.0;

// TIMER
float timerInterval = 0.05;

// Logging
float sampleCount = 0.0;
float switchingPeriod = 0.5;
float numOfSamples = switchingPeriod / timerInterval;
float overallSampleCount = 0;
float overallNumOfSamples = numOfSamples * 20 * 2 + numOfSamples;
float t = 0.0;
/********************/

// Timer Interrupt Service Routine
void tmrISR()
{
  // Disable interrupt
  noInterrupts();

  speedRPM = ((PulseCount / timerInterval) / PulsesPerRevolution) * 60.0;
  PORT.print(t);
  PORT.print(",");
  PORT.print(valuePWM);
  PORT.print(",");
  PORT.println(speedRPM);
  PulseCount = 0;
  
  // For logging
  if (sampleCount == numOfSamples)
  {
    if (valuePWM > 0)
    {
      valuePWM = 0;
      analogWrite(pinMotor, valuePWM);
    }
    else
    {
      valuePWM = (255/4);
      analogWrite(pinMotor, valuePWM);
    }
    sampleCount = 0;
  }
  else
    // Increment pulse count
    sampleCount++;

  if (overallSampleCount >= overallNumOfSamples)
  {
    // End connection
    // Stop Motor
    analogWrite(pinMotor, 0.0);
    sampleCount = 0;
    PulseCount = 0;
    overallSampleCount = 0;
    t = 0;
    
    //Detach ISR
    Timer1.detachInterrupt();
    detachInterrupt(digitalPinToInterrupt(pinCHA));
  }
  else
    overallSampleCount++;

  // Increment timestamp
  t+=timerInterval;

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
  pinMode(pinDir, OUTPUT);


  // Default to 0
  digitalWrite(pinMotor, LOW);
  digitalWrite(pinDir, LOW);


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
    //Attach ISR
    Timer1.attachInterrupt(tmrISR);

    //Enable external interrupt (Rising Edge)
    attachInterrupt(digitalPinToInterrupt(pinCHA), externalISR, RISING);
  }
  if (Command == "END")
  {
    // Stop Motor
    analogWrite(pinMotor, 0.0);
    //Detach ISR
    Timer1.detachInterrupt();
    detachInterrupt(digitalPinToInterrupt(pinCHA));
  }

  // New command for PWM
  if (Command == "PWM")
  {
    // Send PWM value
    valuePWM = StringData.toInt();
    analogWrite(pinMotor, valuePWM);
  }

  // New command for direction
  if (Command == "DIR")
  {
    // Set Motor Direction
    int Direction = StringData.toInt();
    if (Direction == 0)
    {
      digitalWrite(pinDir, 1);
    }
    if (Direction == 1)
    {
      digitalWrite(pinDir, 0);
    }
  }

  if (Command == "GOLOG")
  {
    // Start streaming data
    Timer1.initialize(timerInterval * 1000000);
    //Attach ISR
    Timer1.attachInterrupt(tmrISR);

    //Enable external interrupt (Rising Edge)
    attachInterrupt(digitalPinToInterrupt(pinCHA), externalISR, RISING);

    // Set PWM to 0
    analogWrite(pinMotor, 0);
  }
}
