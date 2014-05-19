//
// PiBot arduino code
// James Torbett 2014
// v. 0.1

// EARLY ALPHA RELEASE
//
// Todo:
// ADC implementation
//  Temperature (thermistor)
//  Voltages
// Neopixel display
// Wheel odometry
// Better stepper timings
// Serial terminal
// i2c interface


#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <NewPing.h>
#include <SPI.h>


#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define SPI_IDLE 0          // initial state of SPI bus
#define SPI_READ 1     // next byte is a READ from here
#define SPI_WRITE 2    // next byte is a WRITE to here

// vars to hold register value and write data. Read data is passed directly out on next clock cycle.
byte spiRegister = 0;
byte spiData = 0;


// pin definitions
byte pin_motor1dir = 2;   // direction of motor 1
byte pin_motor2dir = 4;   // direction of motor 2
byte pin_motor1pwm = 3;   // PWM (speed) of motor 1
byte pin_motor2pwm = 5;   // PWM (speed) of motor 2

byte pin_stepperDir = 7;  // direction of stepper motor
byte pin_stepperStep = 6; // cycle this pin to perform 1/32 of step
byte pin_stepperDisable = 8; // low = stepper enable, high = disable to save power
byte pin_neopixelData = 14; // neopixel data in pin

byte pin_uSoundTrig = 16;  // trigger pin of the ultrasound (ping) module
byte pin_uSoundEcho = 15;  // echo pin of the ultrasound module

byte pin_servoData = 17;  // pin for the servo data

byte spiReceived = 0; // flag to say we've received an SPI byte
byte spiByte = 0;     // value of the byte received over SPI

const int SPI_BUFFER_SIZE = 128;    // AB: This may need to be a power of 2 for the producer consumer algorithm to work
volatile unsigned int gSpiProduceCount = 0;
volatile unsigned int gSpiConsumeCount = 0;
volatile byte gSpiBuffer[ SPI_BUFFER_SIZE ];
volatile byte gSpiResult = 0;

volatile byte gSpiClash = 0;


unsigned int sonarInterval = 100; // number of loops per sonar measurement
unsigned int stepperInterval = 100; // number of loops per stepper step
unsigned int neoPixelInterval = 1000; // number of loops per neopixel array update

byte stepperValue = 0;   // increments. Bit 0 used for stepper pin value.

// internal data registers

byte motor1dir = 0;
int motor2dir = 0;

byte motor1pwm = 0;
byte motor2pwm = 0;

int servoPos = 0;

byte stepperSpeed = 0;
byte stepperDir = 0;

int uSoundDistance = 0;

const int NUM_NEOPIXELS = 8;
byte neoPixelData[51] = { 0 };

// count of hex 0x55 character received on SPI (01010101)
// if this is received 3 times in a row, reset the SPI state machine
byte spi55count = 0;

unsigned int controlCounter = 0;

byte spiState = SPI_IDLE;

// define some library objects
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_NEOPIXELS, pin_neopixelData, NEO_GRB + NEO_KHZ800);
Servo tiltServo;
NewPing sonar(pin_uSoundTrig, pin_uSoundEcho, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// setup: set pin modes and any other associated stuff
void setup()
{
    Serial.begin(9600);
    pinMode(pin_motor1dir, OUTPUT);
    pinMode(pin_motor2dir, OUTPUT);
    pinMode(pin_motor1pwm, OUTPUT);
    pinMode(pin_motor2pwm, OUTPUT);
    pinMode(pin_stepperDir, OUTPUT);
    pinMode(pin_stepperStep, OUTPUT);
    pinMode(pin_stepperDisable, OUTPUT);
    pinMode(pin_neopixelData, OUTPUT);

    // start the servo
    tiltServo.attach(pin_servoData);

    // start SPI
    enableSPI();

}

// SPI interrupt routine, activates when a byte is received.
ISR (SPI_STC_vect)
{
    byte c = SPDR;  // grab byte from SPI Data Register

    if ( gSpiProduceCount - gSpiConsumeCount == SPI_BUFFER_SIZE )
    {
        // SPI buffer is full
        gSpiClash = 1;
    }
    else
    {
        gSpiBuffer[ gSpiProduceCount % SPI_BUFFER_SIZE ] = c;
        gSpiProduceCount++;
    }

    SPDR = gSpiResult;

}  // end of interrupt routine SPI_STC_vect


// enable the SPI bus as slave device (PI is master)
void enableSPI()
{
    // have to send on master in, *slave out*
    pinMode(MISO, OUTPUT);

    // turn on SPI in slave mode
    SPCR |= _BV(SPE);

    // Set SPI mode
    SPI.setDataMode( SPI_MODE0 );

    // now turn on interrupts
    SPI.attachInterrupt();
}



// set the motor drives to values defined in the variables
void controlMotors(void)
{
    digitalWrite(pin_motor1dir, motor1dir);
    analogWrite(pin_motor1pwm, motor1pwm);

    digitalWrite(pin_motor2dir, motor2dir);
    analogWrite(pin_motor2pwm, motor2pwm);

}

// write to the servo
void controlServo(void)
{
    tiltServo.write(servoPos);
}

// toggle the stepper pin
void doStep(void)
{

    if (stepperSpeed > 0)
    {
        stepperInterval = 256 - stepperSpeed;
        digitalWrite(pin_stepperDisable, 0);
        digitalWrite(pin_stepperDir, stepperDir);
        digitalWrite(pin_stepperStep, stepperValue & 0x01);
        stepperValue++;

    }
    else
    {
        digitalWrite(pin_stepperDisable, 1);
    }


}

// read the distance reported by the ultrasound module.
// make sure to leave at least 50mS between calls to this
void readDistance()
{
    unsigned int uS = sonar.ping();
    uSoundDistance = uS / US_ROUNDTRIP_CM;
}

void processSPI(void)
{

    // finished clocking in a byte

    // process the sync byte regardless of state
    if(spiByte == 0x55)
    {
        //Serial.println( &quot;Sync byte&quot; );
        // increment sync counter
        spi55count++;
        if (spi55count > 2)
        {
            // go back to the idle state
            spiState = SPI_IDLE;
            spiReceived = 0;
            spi55count = 0; // Reset sync conter
            return;         // yes, it's an unconditional jump out of subroutine
        }

    }

    if (spiState == SPI_IDLE)
    {
        if (spiByte < 60)
        {
            // Write operation
            spiState = SPI_WRITE;
            spiRegister = spiByte;
        }
        else if (spiByte >= 100)
        {
            spiState = SPI_READ;
            spiRegister = spiByte;
            processSPIRead();

        }
    }
    else if (spiState == SPI_WRITE)
    {
        // we've got a data value now
        spiData = spiByte;
        processRegister();
        spiState = SPI_IDLE;
    }
    else if (spiState == SPI_READ)
    {
        // SPDR data register will have been clocked out now. Return to idle.
        spiState = SPI_IDLE;
    }


    spiReceived = 0;



}
void processSPIRead()
{
    // only one case for now: to read the ultrasound distance.
    switch (spiRegister)
    {
        case 100:
            gSpiResult = uSoundDistance;
            break;
        default:
            break;
    }

}

void processRegister()
{
    switch(spiRegister)
    {
        case 01:
            motor1dir = spiData;
            break;
        case 02:
            motor2dir = spiData;
            break;
        case 03:
            motor1pwm = spiData;
            break;
        case 04:
            motor2pwm = spiData;
            break;
        case 05:
            stepperDir = spiData;
            break;
        case 06:
            stepperSpeed = spiData;
            break;
        case 07:
            servoPos = spiData;
            break;
        case 8 ... 59:
            neoPixelData[spiRegister-8] = spiData;    // wow departure from ANSI-C
            break;

        default:
            break;
    }
}

void updateNeoPixel()
{
    // todo: actually copy the register data into the neopixel strip

    for ( int pixelIdx = 0; pixelIdx < NUM_NEOPIXELS; pixelIdx++ ) //sizeof( neoPixelData ) / 3; pixelIdx++ )
    {
        strip.setPixelColor( pixelIdx, neoPixelData[ 3*pixelIdx ], neoPixelData[ 3*pixelIdx+1 ], neoPixelData[ 3*pixelIdx+2 ] );
    }

    strip.show();
}


// main loop
void loop(void)
{
    // do stuff.
    controlCounter++;

    if ( gSpiProduceCount - gSpiConsumeCount > 0 )
    {
        spiByte = gSpiBuffer[ gSpiConsumeCount % SPI_BUFFER_SIZE ];
        spiReceived = 1;
        gSpiConsumeCount++;
    }

    if (spiReceived) processSPI();

    if ( gSpiClash )
    {
        //Serial.println( &quot;Serial clash occured&quot; );
        gSpiClash = 0;
    }

    controlMotors();
    controlServo();

    //Serial.println( controlCounter );

    // only read distance every sonarInterval loops
    if((controlCounter % neoPixelInterval) == 0)
    {
        updateNeoPixel();
    }

    // only read distance every sonarInterval loops
    if((controlCounter % sonarInterval) == 0)
    {
        readDistance();
    }

    // only step stepper based on interval (derived from speed)
    if((controlCounter % stepperInterval) == 0)
    {
        doStep();
    }
}
