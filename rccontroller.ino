/*
 * Carson Unimog U300 alternative controller firmware
 * to replace the original EM78P540NS024J controller with an "Arduino pro mini"
 *
 * 2015-08-17 by Sönke J. Peters
 */

//#define DEBUG
// serial output of values
#define SEROUTPUT

// if tx has switches modded to use a R2R resistor ladder on chan 5
//#define MODDEDTX

// Main motor
#define MOTOR1_HBRIDGEPIN0 11
#define MOTOR1_HBRIDGEPIN1 3
// Winch
#define MOTOR2_HBRIDGEPIN0 12
#define MOTOR2_HBRIDGEPIN1 2
// Platform lift motor
#define MOTOR3_HBRIDGEPIN0 5
#define MOTOR3_HBRIDGEPIN1 6

// channel in pins from the receiver
#define CHAN1_IN_PIN A6
#define CHAN2_IN_PIN A1
#define CHAN3_IN_PIN A2
#define CHAN4_IN_PIN A3
#define CHAN5_IN_PIN A4
#define CHAN6_IN_PIN A5

// This pin outputs chan6 on a modded tx
#define STEERING_OUT_PIN 8

#define HORN_OUT_PIN 7
#define LIGHTS_OUT_PIN 9
// Note: D13 is also routed to LED on the Arduino
#define BLINK_OUT_PIN 13
#define STARTUP_OUT_PIN 4
// Note: D13 is also routed to LED on the Arduino
#define BACKUPBEEPER_OUT_PIN 10
#define BREAKLIGHT_OUT_PIN A0

#define ANALOG_IN_PIN A7

// Number of switches coded into the AUX channel
#define NUMAUXSW 5

// minimal duration of servo pulse (ms)
#define SERVOMSMIN 1000
// minimal duration of servo pulse (ms)
#define SERVOMSMAX 2000
// middle position
#define SERVOMSMID SERVOMSMIN + ((SERVOMSMAX - SERVOMSMIN)/2)
// neutral range [SERVOMSMID - SERVONEUTR .. SERVOMSMID+SERVONEUTR]
#define SERVONEUTR 50
// Minimal pulse value which determines a receiver alive state
#define SERVOFAILSAFEMIN 50

#define HBRIDGEDELAY 20
// Minimal PWM value to make motor spin
#define HBRIDGEMIN 40

#include <Servo.h>

// See https://github.com/GreyGnome/EnableInterrupt
// prevent clashes when attachInterrupt() is used as is in RCArduinoFastLib.cpp
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>

uint8_t  AUXSwitches[NUMAUXSW] = {0};

uint16_t analogVal = 0;

Servo steeringServo;

// holds the update flags
volatile uint8_t bUpdateFlagsShared;

#define RX_ISR(x) \
	volatile uint16_t rc_value_shared ##x; \
	void rx_isr ##x () { \
		static uint16_t rc_value_start ##x; \
		if (digitalRead(CHAN##x ##_IN_PIN)) \
		{ \
			rc_value_start ##x = TCNT1; \
		} \
		else \
		{ \
			rc_value_shared ##x = (TCNT1 - rc_value_start ##x)>>1; \
			bUpdateFlagsShared |= (1 << ( x - 1)) ; \
		} \
	}

RX_ISR(1);
RX_ISR(2);
RX_ISR(3);
RX_ISR(4);
RX_ISR(5);
RX_ISR(6);

unsigned long lastdelaytime;

inline uint8_t dohbridge(uint16_t ThrottleIn, uint8_t hbridgepin0, uint8_t hbridgepin1, int8_t motorspeed)
{		// Motor 1 (main motor) H-bridge
	if ((ThrottleIn >= (SERVOMSMID + SERVONEUTR)) && (ThrottleIn <= SERVOMSMAX + 50) && motorspeed >= 0)
	{
		// drive forwards
		digitalWrite(hbridgepin0, 0);
		analogWrite(hbridgepin1, map(ThrottleIn, (SERVOMSMID + SERVONEUTR), SERVOMSMAX, HBRIDGEMIN, 255));
		//Serial.print("#0=0,#1=");Serial.println(map(ThrottleIn, (SERVOMSMID + SERVONEUTR), SERVOMSMAX, 0, 255));
		motorspeed = 1;
	} else if ((ThrottleIn <= (SERVOMSMID - SERVONEUTR)) && (ThrottleIn >= SERVOMSMIN -50) && motorspeed <= 0)
	{
		// drive backwards
		digitalWrite(hbridgepin1, 0);
		analogWrite(hbridgepin0, map(ThrottleIn, SERVOMSMIN, (SERVOMSMID - SERVONEUTR), 255, HBRIDGEMIN));
		//Serial.print("#1=0,#0=");Serial.println(map(ThrottleIn, SERVOMSMIN, (SERVOMSMID - SERVONEUTR), 255, 0));
		motorspeed = -1;
	} else
	{
		// stop
		digitalWrite(hbridgepin0, 0);
		digitalWrite(hbridgepin1, 0);

		motorspeed = 0;
	}

	//Serial.print(motorspeed); Serial.print(" ");Serial.println(ThrottleIn);
	return motorspeed;
}

void setup()
{
#ifdef SEROUTPUT
	Serial.begin(115200);
#endif

	digitalWrite(MOTOR1_HBRIDGEPIN0, 0);
	digitalWrite(MOTOR1_HBRIDGEPIN1, 0);
	pinMode(MOTOR1_HBRIDGEPIN0, OUTPUT);
	pinMode(MOTOR1_HBRIDGEPIN1, OUTPUT);

	digitalWrite(MOTOR2_HBRIDGEPIN0, 0);
	digitalWrite(MOTOR2_HBRIDGEPIN1, 0);
	pinMode(MOTOR2_HBRIDGEPIN0, OUTPUT);
	pinMode(MOTOR2_HBRIDGEPIN1, OUTPUT);

	digitalWrite(MOTOR3_HBRIDGEPIN0, 0);
	digitalWrite(MOTOR3_HBRIDGEPIN1, 0);
	pinMode(MOTOR3_HBRIDGEPIN0, OUTPUT);
	pinMode(MOTOR3_HBRIDGEPIN1, OUTPUT);

	pinMode(CHAN1_IN_PIN, INPUT);
	pinMode(CHAN2_IN_PIN, INPUT);
	pinMode(CHAN3_IN_PIN, INPUT);
	pinMode(CHAN4_IN_PIN, INPUT);
	pinMode(CHAN5_IN_PIN, INPUT);
	pinMode(CHAN6_IN_PIN, INPUT);

	//pinMode(STEERING_OUT_PIN, OUTPUT);
	pinMode(HORN_OUT_PIN, OUTPUT);
	pinMode(LIGHTS_OUT_PIN, OUTPUT);
	pinMode(BLINK_OUT_PIN, OUTPUT);
	pinMode(BACKUPBEEPER_OUT_PIN, OUTPUT);
	pinMode(BREAKLIGHT_OUT_PIN, OUTPUT);
	pinMode(STARTUP_OUT_PIN, OUTPUT);

	pinMode(ANALOG_IN_PIN, INPUT);

	// TODO: Check what signal is needed for sound module. A simple ENable doesn't seem to do it...
	digitalWrite(STARTUP_OUT_PIN, 1);

	steeringServo.attach(STEERING_OUT_PIN);

	// using the EnableInterrupt library, attach the interrupts
	// used to read the channels
	enableInterrupt(CHAN1_IN_PIN, rx_isr1, CHANGE);
	enableInterrupt(CHAN2_IN_PIN, rx_isr2, CHANGE);
	enableInterrupt(CHAN3_IN_PIN, rx_isr3, CHANGE);
	enableInterrupt(CHAN4_IN_PIN, rx_isr4, CHANGE);
	enableInterrupt(CHAN5_IN_PIN, rx_isr5, CHANGE);
	enableInterrupt(CHAN6_IN_PIN, rx_isr6, CHANGE);

	lastdelaytime = millis();
}


void loop()
{
	// create local variables to hold a local copies of the channel inputs
	// these are declared static so that their values will be retained
	// between calls to loop.

	static uint16_t Motor1ThrottleIn = 0;
	// determines the motor state to insert a delay between switching fwd/back, backward < 0 < forward
	static int8_t motor1_speed = 0;

	static uint16_t Motor2ThrottleIn = 0;
	// determines the motor state to insert a delay between switching fwd/back, backward < 0 < forward
	static int8_t motor2_speed = 0;

	static uint16_t Motor3ThrottleIn = 0;
	// determines the motor state to insert a delay between switching fwd/back, backward < 0 < forward
	static int8_t motor3_speed = 0;

	static uint16_t SteeringIn;
#ifndef MODDEDTX
	static uint16_t Switch1In;
	static uint16_t Switch2In;
#else
	static uint16_t AuxIn;			// Value to be converted back into switch settings
	static uint16_t Aux2In;
#endif

	// local copy of update flags
	static uint8_t bUpdateFlags;

	static uint32_t lasthbridgedelay = -1;


	// check shared update flags to see if any channels have a new signal
	if (bUpdateFlagsShared)
	{
		uint8_t oldSREG;
		oldSREG = SREG;	// save status register
		cli();	// disable interrupts

		// take a local copy of which channels were updated in case we need to use this in the rest of loop
		bUpdateFlags = bUpdateFlagsShared;

		// in the current code, the shared values are always populated
		// so we could copy them without testing the flags
		// however in the future this could change, so lets
		// only copy when the flags tell us we can.

		#define RC_VALUE_TO_VAR(x, variable) \
			if (bUpdateFlags & (1 << (x - 1 ))) { variable = rc_value_shared##x ;};

		RC_VALUE_TO_VAR(1, SteeringIn);
		RC_VALUE_TO_VAR(2, Motor1ThrottleIn);
		RC_VALUE_TO_VAR(3, Motor2ThrottleIn);
		RC_VALUE_TO_VAR(4, Motor3ThrottleIn);
#ifndef MODDEDTX
		RC_VALUE_TO_VAR(5, Switch1In);
		RC_VALUE_TO_VAR(6, Switch2In);
#else
		RC_VALUE_TO_VAR(5, AuxIn);
		RC_VALUE_TO_VAR(5, Aux2In); // TODO: To be used for mode switching?
#endif

		// clear shared copy of updated flags as we have already taken the updates
		// we still have a local copy if we need to use it in bUpdateFlags
		bUpdateFlagsShared = 0;

		SREG = oldSREG;	// restore interrupt state
	}

#ifndef MODDEDTX
	if (bUpdateFlags & (1 << (5 - 1 )))
	{
		if ((Switch1In > (SERVOMSMID + 2*SERVONEUTR)) || (Switch1In < (SERVOMSMID - 2*SERVONEUTR)))
		{
			digitalWrite(LIGHTS_OUT_PIN, 1);
		}
		else
		{
			digitalWrite(LIGHTS_OUT_PIN, 0);
		}
	}
	if (bUpdateFlags & (1 << (6 - 1 )))
	{
		if ((Switch2In > (SERVOMSMID + 2*SERVONEUTR)) || (Switch2In < (SERVOMSMID - 2*SERVONEUTR)))
		{
			digitalWrite(HORN_OUT_PIN, 1);
		}
		else
		{
			digitalWrite(HORN_OUT_PIN, 0);
		}
	}

	/*
	// TODO: something useful ;-)
	// Simple pass through of a servo signal, think of it as a placeholder
	if (bUpdateFlags & (1 << (1 - 1 )))
	{
		steeringServo.writeMicroseconds(SteeringIn);
	}
	*/
#else
	if ((bUpdateFlags & (1 << (5 - 1 ))) && AuxIn > SERVOFAILSAFEMIN)
	{
		// See http://aeroquad.com/entry.php?59-15-Transmitter-Modification-Replacing-AUX-or-MODE-VRs-with-discrete-switch-inputs
		// Decode AUX switches
		uint8_t idx;
		uint16_t astep, aval;
		aval = map(AuxIn, SERVOMSMIN, SERVOMSMAX, 0, 1000);	// Start with raw received value, normalize to 0..1000
		for (idx=NUMAUXSW; idx>0; idx--)
		{
			// Loop through each of the BITS (5..0), starting with MSB
			astep = (1000 / (2<<(NUMAUXSW-idx))) - (idx > 1 ? 0 : 5);	// (1000 / 2^idx) determines step value.  Last bit needs a bit more range.
			AUXSwitches[idx-1] = (aval > astep);	// Set bit to 1 in array if current value is > step, otherwise 0
			aval -= AUXSwitches[idx-1] * astep;	// If bit was set, remove corresponding amount from value for next interation
		}

		if (AUXSwitches[0] > 0)
		{
			digitalWrite(LIGHTS_OUT_PIN, 1);
		}
		else
		{
			digitalWrite(LIGHTS_OUT_PIN, 0);
		}

		if (AUXSwitches[1] > 0)
		{
			digitalWrite(HORN_OUT_PIN, 1);
		}
		else
		{
			digitalWrite(HORN_OUT_PIN, 0);
		}

	}

	// TODO: do something with this chan
	if (bUpdateFlags & (6 << (1 - 1 )))
	{
		steeringServo.writeMicroseconds(SteeringIn);
	}
#endif

	// h-bridges don't get updated so often to insert a dead time between fwd/back switching
	if ((millis() - lasthbridgedelay) > HBRIDGEDELAY)
	{
		motor1_speed = dohbridge(Motor1ThrottleIn, MOTOR1_HBRIDGEPIN0, MOTOR1_HBRIDGEPIN1, motor1_speed);
		motor2_speed = dohbridge(Motor2ThrottleIn, MOTOR2_HBRIDGEPIN0, MOTOR2_HBRIDGEPIN1, motor2_speed);
		motor3_speed = dohbridge(Motor3ThrottleIn, MOTOR3_HBRIDGEPIN0, MOTOR3_HBRIDGEPIN1, motor3_speed);

		lasthbridgedelay = millis();
	}

#ifdef DEBUG
	// Read value from potentiometer, the servo output can be routed back into a RC input for testing (no need for a servo tester)
	analogVal = analogRead(ANALOG_IN_PIN);
	analogVal = map(analogVal, 0, 1023, 1000, 2000);
	steeringServo.writeMicroseconds(analogVal);
#endif

#ifdef SEROUTPUT
	if (((millis() - lastdelaytime) > 500)) // && bUpdateFlags)
	{
#ifdef DEBUG
		Serial.print("AnalogMapped2Servo=");
		Serial.print(analogVal);
		Serial.print("; CH1=");
#else
		Serial.print("CH1=");
#endif
		Serial.print(SteeringIn);
		Serial.print("; CH2=");
		Serial.print(Motor1ThrottleIn);
		Serial.print("; CH3=");
		Serial.print(Motor2ThrottleIn);
		Serial.print("; CH4=");
		Serial.print(Motor3ThrottleIn);
#ifndef MODDEDTX
		Serial.print("; CH5=");
		Serial.print(Switch1In);
		Serial.print("; CH6=");
		Serial.println(Switch2In);
#else
		Serial.print("; CH5=");
		Serial.print(AuxIn);
		Serial.print("; CH6=");
		Serial.println(Aux2In); // TODO: To be used for mode switching?
#endif
		lastdelaytime = millis();
	}
#endif

	bUpdateFlags = 0;

}
