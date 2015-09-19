/*
 * Carson Unimog U300 alternative controller firmware
 * to replace the original EM78P540NS024J controller with an "Arduino pro mini"
 *
 * 2015-08-17 by Sönke J. Peters
 */

#include <Arduino.h>

//#define DEBUG
// serial output of values
#define USESERIAL

// if tx has switches modded to use a R2R resistor ladder on chan 5
//#define MODDEDTX

//#define ORIGINAL_SOUND_MODULE // currently unused, TODO: get to know how it works, implement

// Main motor (pwm speed controlled)
#define MOTOR1_HBRIDGEPIN0 6
#define MOTOR1_HBRIDGEPIN1 5
// Winch (fwd/off/rev only)
#define MOTOR2_HBRIDGEPIN0 2
#define MOTOR2_HBRIDGEPIN1 12
// Platform lift motor (fwd/off/rev only)
#define MOTOR3_HBRIDGEPIN0 11
#define MOTOR3_HBRIDGEPIN1 7

// channel in pins from the receiver
#define CHAN1_IN_PIN A6
#define CHAN2_IN_PIN A1
#define CHAN3_IN_PIN A2
#define CHAN4_IN_PIN A3
#define CHAN5_IN_PIN A4
#define CHAN6_IN_PIN A5

#define MAXCHAN 6

// This pin outputs chan6 on a modded tx
#define STEERING_OUT_PIN 8

#define LIGHTS_OUT_PIN 9
#define BLINK_OUT_PIN 13
#define BREAKLIGHT_OUT_PIN 10
#define ANALOG_IN_PIN A7

#ifdef ORIGINAL_SOUND_MODULE
#define STARTUP_OUT_PIN 7
// Note: D13 is also routed to LED on the Arduino
#define BACKUPBEEPER_OUT_PIN A0
#define HORN_OUT_PIN 3
#else // ORIGINAL_SOUND_MODULE
#define SPEAKER_OUT_PIN 3

#include "soundplay.h"

#include "sounds/sound_crankup.h"
#include "sounds/sound_diesel.h"
#include "sounds/sound_hupe.h"
#include "sounds/sound_feuerwehr_btc.h"
#endif // ORIGINAL_SOUND_MODULE

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

// #define  __STDC_LIMIT_MACROS
#include <stdint.h>

#include <Servo.h>

// See https://github.com/GreyGnome/EnableInterrupt
// prevent clashes when attachInterrupt() is used as is in RCArduinoFastLib.cpp
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>

uint8_t  AUXSwitches[NUMAUXSW] = {0};

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

inline uint8_t dohbridge_pwm(uint16_t ThrottleIn, uint8_t hbridgepin0, uint8_t hbridgepin1, int8_t motorspeed)
{
	if ((ThrottleIn >= (SERVOMSMID + SERVONEUTR)) && (ThrottleIn <= SERVOMSMAX + 50) && motorspeed >= 0)
	{
		// drive forwards
		digitalWrite(hbridgepin0, 0);
		analogWrite(hbridgepin1, map(ThrottleIn, (SERVOMSMID + SERVONEUTR), SERVOMSMAX, HBRIDGEMIN, UINT8_MAX));
		// motorspeed = 1;
		motorspeed = map(ThrottleIn, (SERVOMSMID + SERVONEUTR), SERVOMSMAX, 1, INT8_MAX);
	} else if ((ThrottleIn <= (SERVOMSMID - SERVONEUTR)) && (ThrottleIn >= SERVOMSMIN -50) && motorspeed <= 0)
	{
		// drive backwards
		digitalWrite(hbridgepin1, 0);
		analogWrite(hbridgepin0, map(ThrottleIn, SERVOMSMIN, (SERVOMSMID - SERVONEUTR), UINT8_MAX, HBRIDGEMIN));
		// motorspeed = -1;
		motorspeed = map(ThrottleIn, SERVOMSMIN, (SERVOMSMID - SERVONEUTR), INT8_MIN +1, -1);
	} else
	{
		// stop
		digitalWrite(hbridgepin0, 0);
		digitalWrite(hbridgepin1, 0);

		motorspeed = 0;
	}

	return motorspeed;
}

inline uint8_t dohbridge_onoffon(uint16_t ThrottleIn, uint8_t hbridgepin0, uint8_t hbridgepin1, int8_t motorspeed)
{
	if ((ThrottleIn >= (SERVOMSMID + SERVONEUTR)) && (ThrottleIn <= SERVOMSMAX + 50) && motorspeed >= 0)
	{
		// drive forwards
		digitalWrite(hbridgepin0, 0);
		digitalWrite(hbridgepin1, 1);
		// motorspeed = 1;
		motorspeed = map(ThrottleIn, (SERVOMSMID + SERVONEUTR), SERVOMSMAX, 1, INT8_MAX);
	} else if ((ThrottleIn <= (SERVOMSMID - SERVONEUTR)) && (ThrottleIn >= SERVOMSMIN -50) && motorspeed <= 0)
	{
		// drive backwards
		digitalWrite(hbridgepin1, 0);
		digitalWrite(hbridgepin0, 1);
		// motorspeed = -1;
		motorspeed = map(ThrottleIn, SERVOMSMIN, (SERVOMSMID - SERVONEUTR), INT8_MIN +1, -1);
	} else
	{
		// stop
		digitalWrite(hbridgepin0, 0);
		digitalWrite(hbridgepin1, 0);

		motorspeed = 0;
	}

	return motorspeed;
}

uint8_t backgroundsoundindex;
void setup()
{
#ifdef USESERIAL
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
#ifdef ORIGINAL_SOUND_MODULE
	pinMode(HORN_OUT_PIN, OUTPUT);
	pinMode(BACKUPBEEPER_OUT_PIN, OUTPUT);
	pinMode(STARTUP_OUT_PIN, OUTPUT);
	// TODO: Check what signal is needed for sound module. A simple ENable doesn't seem to do it...
	digitalWrite(STARTUP_OUT_PIN, 1);
#else // ORIGINAL_SOUND_MODULE
	pinMode(SPEAKER_OUT_PIN, OUTPUT);
#endif // ORIGINAL_SOUND_MODULE
	pinMode(LIGHTS_OUT_PIN, OUTPUT);
	pinMode(BLINK_OUT_PIN, OUTPUT);
	pinMode(BREAKLIGHT_OUT_PIN, OUTPUT);

	pinMode(ANALOG_IN_PIN, INPUT);

	steeringServo.attach(STEERING_OUT_PIN);

	// using the EnableInterrupt library, attach the interrupts
	// used to read the channels
	enableInterrupt(CHAN1_IN_PIN, rx_isr1, CHANGE);
	enableInterrupt(CHAN2_IN_PIN, rx_isr2, CHANGE);
	enableInterrupt(CHAN3_IN_PIN, rx_isr3, CHANGE);
	enableInterrupt(CHAN4_IN_PIN, rx_isr4, CHANGE);
	enableInterrupt(CHAN5_IN_PIN, rx_isr5, CHANGE);
	enableInterrupt(CHAN6_IN_PIN, rx_isr6, CHANGE);

#ifndef ORIGINAL_SOUND_MODULE
	soundplayer_setup();

	backgroundsoundindex = soundplayer_play((uint16_t) &sound_diesel,
				sound_diesel_len, SOUND_FORMAT_PCM, MAXCNTRELOAD, finishplay_repeat, 0);
	soundplayer_play_repeat((uint16_t) &sound_crankup,
			sound_crankup_len, SOUND_FORMAT_PCM, 3);
#endif // ORIGINAL_SOUND_MODULE

	lastdelaytime = millis();
}


void loop()
{
	// create local variables to hold a local copies of the channel inputs
	// these are declared static so that their values will be retained
	// between calls to loop.

	static uint16_t rxdata[MAXCHAN+1] = {0};

// TODO: get rid of these ugly defines
#define	analogVal rxdata[0]
#define SteeringIn rxdata[1]
#define Motor1ThrottleIn rxdata[2]
#define Motor2ThrottleIn rxdata[3]
#define Motor3ThrottleIn rxdata[4]
#ifndef MODDEDTX
#define Switch1In rxdata[5]
#define Switch2In rxdata[6]
#else
#define AuxIn rxdata[5]
#define Aux2In rxdata[6]
#endif

	// determines the motor state to insert a delay between switching fwd/back, backward < 0 < forward
	static int8_t motor1_speed = 0, motor2_speed = 0, motor3_speed = 0;

	// local copy of update flags
	static uint8_t bUpdateFlags;
#ifdef DEBUG
	static uint8_t doprint = 1, doserial = 1;
#else
	static uint8_t doprint = 0, doserial = 0;
#endif

	static uint32_t lasthbridgedelay = -1;

#define SERIALSTATEIDLE 0
#define SERIALSTATECHAN 1
#define SERIALSTATECHANNUM 2
#define SERIALSTATESEP 3
#define SERIALSTATEVAL 4

	static uint8_t serialreceivestate = SERIALSTATEIDLE, serialchan = 1, IncomingByte;
	static uint32_t serialval;


	// check shared update flags to see if any channels have a new signal
	if (bUpdateFlagsShared && (doserial == 0))
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
		RC_VALUE_TO_VAR(6, Aux2In); // TODO: To be used for mode switching?
#endif

		// clear shared copy of updated flags as we have already taken the updates
		// we still have a local copy if we need to use it in bUpdateFlags
		bUpdateFlagsShared = 0;

		SREG = oldSREG;	// restore interrupt state
	}

#ifdef USESERIAL
	if (Serial.available() > 0)
	{
		IncomingByte = Serial.read();

		if (serialreceivestate == SERIALSTATEIDLE)
		{
			if (IncomingByte == 'C' || IncomingByte == 'c')
			{
				serialreceivestate = SERIALSTATECHAN;
			}
			else if (IncomingByte == '$')
			{
				// Force printing values
				lastdelaytime = 0;
				doprint = 1;
				// serialreceivestate = SERIALSTATEIDLE; // just for the coding style
			}
			else if (IncomingByte == '?')
			{
				// Small help text
				Serial.print("\r\n\r\n rccontroller "__DATE__" by Soenke J. Peters\r\n\r\n");
				Serial.print(" ?    this help\r\n");
				Serial.print(" $    print values\r\n");
				Serial.print(" !    stop printing values\r\n");
				Serial.print(" Cx   switch to channel x with x in [0..6]\r\n");
				Serial.print(" 123  send value 123 to current channel\r\n");
				Serial.print(" 1212,1500,1750    bulk update of channels\r\n");
				Serial.print(" #    switch to serial input of values\r\n");
				Serial.print(" +    switch to RC input of values\r\n\r\n");
				// serialreceivestate = SERIALSTATEIDLE; // just for the coding style
			}
			else if (IncomingByte == '!')
			{
				// Stop printing values
				doprint = 0;
				// serialreceivestate = SERIALSTATEIDLE; // just for the coding style
			}
			else if (IncomingByte == '#')
			{
				// Set serial as receiver
				doserial = 1;
				// serialreceivestate = SERIALSTATEIDLE; // just for the coding style
			}
			else if (IncomingByte == '+')
			{
				// Set RC as receiver
				doserial = 0;
				// serialreceivestate = SERIALSTATEIDLE; // just for the coding style
			}

			else if (isdigit(IncomingByte) )
			{
				serialval = IncomingByte - 48;
				serialreceivestate = SERIALSTATEVAL;
			}
			else
			{
				// serialreceivestate = SERIALSTATEIDLE; // just for the coding style
			}
		} else if (serialreceivestate == SERIALSTATECHAN)
		{
			// BUG: Allows "chaAhaan=" and similar things (but allows for fewer states)
			if (IncomingByte == 'H' || IncomingByte == 'h'
					|| IncomingByte == 'A' || IncomingByte == 'a'
					|| IncomingByte == 'N' || IncomingByte == 'n'
					|| IncomingByte == ':' || IncomingByte == '=')
			{
				serialreceivestate = SERIALSTATECHAN;
			}
			else if (isdigit(IncomingByte) && ((IncomingByte - 48) >= 0) && ((IncomingByte - 48) < (MAXCHAN + 1)))
			{
				serialchan = (IncomingByte - 48);
				serialreceivestate = SERIALSTATEIDLE;
			}
			else
			{
				serialreceivestate = SERIALSTATEIDLE;
			}
		} else if (serialreceivestate == SERIALSTATEVAL)
		{
			if (isdigit(IncomingByte) )
			{
				serialval *= 10;
				serialval += IncomingByte - 48;
				serialreceivestate = SERIALSTATEVAL;
			} else if (IncomingByte == 'C' || IncomingByte == 'c')
			{
				if (serialchan > 0)
				{
					bUpdateFlags |= (1 << ( serialchan - 1)) ;
				}
				rxdata[serialchan] = serialval;
				serialval = 0;
				serialreceivestate = SERIALSTATECHAN;
			} else if (IncomingByte == ',' || IncomingByte == ';')
			{
				if (serialchan > 0)
				{
					bUpdateFlags |= (1 << ( serialchan - 1)) ;
				}
				rxdata[serialchan] = serialval;
				serialval = 0;
				if (serialchan <= MAXCHAN)
				{
					// bulk update of channels, go for next channel data
					serialchan++;
					serialreceivestate = SERIALSTATEVAL;
				}
				else
				{
					serialchan = 0;
					serialreceivestate = SERIALSTATEIDLE;
				}
			}
			else
			{
				if (serialchan > 0)
				{
					bUpdateFlags |= (1 << ( serialchan - 1)) ;
				}
				rxdata[serialchan] = serialval;
				serialval = 0;
				serialreceivestate = SERIALSTATEIDLE;
			}
		}
		else
		{
			serialreceivestate = SERIALSTATEIDLE;
		}
	}
#endif

#ifndef MODDEDTX
	if (bUpdateFlags & (1 << (5 - 1 )))
	{
		if (((Switch1In > (SERVOMSMID + 2*SERVONEUTR)) || (Switch1In < (SERVOMSMID - 2*SERVONEUTR))) &&
				Switch1In < SERVOMSMAX + 2*SERVONEUTR && Switch1In > SERVOMSMAX - 2*SERVONEUTR)
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
		if (((Switch2In > (SERVOMSMID + 2*SERVONEUTR)) || (Switch2In < (SERVOMSMID - 2*SERVONEUTR))) &&
				Switch2In < SERVOMSMAX + 2*SERVONEUTR && Switch2In > SERVOMSMAX - 2*SERVONEUTR)
		{
#ifdef ORIGINAL_SOUND_MODULE
			digitalWrite(HORN_OUT_PIN, 1);
#else // ORIGINAL_SOUND_MODULE
			soundplayer_play_ds((uint16_t) &sound_feuerwehr_btc, sound_feuerwehr_btc_len, SOUND_FORMAT_BTC,
					20);
#endif // ORIGINAL_SOUND_MODULE
		}
		else
		{
#ifdef ORIGINAL_SOUND_MODULE
			digitalWrite(HORN_OUT_PIN, 0);
#endif // ORIGINAL_SOUND_MODULE
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
		motor1_speed = dohbridge_pwm(Motor1ThrottleIn, MOTOR1_HBRIDGEPIN0, MOTOR1_HBRIDGEPIN1, motor1_speed); // use pwm
		motor2_speed = dohbridge_onoffon(Motor2ThrottleIn, MOTOR2_HBRIDGEPIN0, MOTOR2_HBRIDGEPIN1, motor2_speed);
		motor3_speed = dohbridge_onoffon(Motor3ThrottleIn, MOTOR3_HBRIDGEPIN0, MOTOR3_HBRIDGEPIN1, motor3_speed);

#ifndef ORIGINAL_SOUND_MODULE
		soundqueue[backgroundsoundindex].speed = map(abs(motor1_speed), INT8_MAX, 0,
				(soundqueue[backgroundsoundindex].format == SOUND_FORMAT_PCM ? MAXCNTRELOAD : MAXCNTRELOAD>>1)- (255 - 170), (soundqueue[backgroundsoundindex].format == SOUND_FORMAT_PCM ? MAXCNTRELOAD : MAXCNTRELOAD>>1));
#endif // ORIGINAL_SOUND_MODULE

		lasthbridgedelay = millis();
	}

#ifdef DEBUG
	// Read value from potentiometer, the servo output can be routed back into a RC input for testing (no need for a servo tester)
	analogVal = analogRead(ANALOG_IN_PIN);
	analogVal = map(analogVal, 0, 1023, 1000, 2000);
	steeringServo.writeMicroseconds(analogVal);
#endif

#ifdef USESERIAL
	if (doprint && ((millis() - lastdelaytime) > 500)) // && bUpdateFlags)
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

	static uint32_t lastblinklooptime = 0;
	if ((millis() - lastblinklooptime) > 500)
	{

		lastblinklooptime = millis();
	}

}
