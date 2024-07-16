/******************************************
   COURSE  : MTE504 MECHATRONICS II
   PROJECT : SPEED CHANGER
   DATE    : 04/08/2021
 *****************************************/

// Serial Port Used
#define ser Serial

// Motor PWM Pin
byte pinMotor = 5;

// Direvction Pin
byte pinDir = 7;

void setup() {
  // Setup serial baudrate
  ser.begin(9600);

  // Set pins as output
  pinMode(pinMotor, OUTPUT);
  pinMode(pinDir, OUTPUT);

  // Default to 0
  digitalWrite(pinMotor, LOW);
  digitalWrite(pinDir, LOW);
}

void loop() {
}

void serialEvent() {
  if (ser.available()) {
    char cmd = ser.read();
    switch (cmd) {
      case 's':
        // Set Speed to 100%
        analogWrite(pinMotor, 1.0 * 255.0);
        break;

      case 'c':
        // Clockwise
        digitalWrite(pinDir, HIGH);
        break;

      case 'a':
        // Anti Clockwise
        digitalWrite(pinDir, LOW);
        break;

      case 'p':
        // Set Speed to 0%
        analogWrite(pinMotor, 0.0);
        break;

      default:
        break;
    }
  }
}
