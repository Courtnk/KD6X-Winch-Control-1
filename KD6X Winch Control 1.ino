/*
    Name:       KD6X Winch Control 1.ino
    Created:	3/1/2019 11:55:12 PM
    Author:     Courtney E. Krehbiel
	
	Moto-Mast Firmware Ver. 1.10				* * * UPDATE DISPLAY ON APPROX. LINE 167 * * * 
*/

/*
  Code by Courtney Krehbiel KD6X to run a remote winch and motor by
  using a motor controller with speed set by using PWM and a filtered output.
  Controls are to allow the user to manipulate speed or sweep function.

  Development started Feb. 13, 2019

  Finally completed on May 24, 2020

 */


// Basic includes
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "Ethernet.h"


// Fonts to use
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>

#include <EEPROM.h>


// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
// On Mega, SPI is #50, #51, #52.  Clk = 52, MISO = 50, MOSI = 51
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

const byte ButtonInterrupt1Pin = 3;	// Interrupt 1 input to handle soft button pushes
const byte ButtonInterrupt0Pin = 2;	// Interrupt 0 input to handle shaft rotation

const byte LimitInput = 8;			// Pin to use to read status of Limit switches

volatile unsigned char Lockout = 0;	// Used to lockout interrupts for a few loops to debounce the pushbuttons
volatile byte AnyButtonPush = HIGH;		// Flag to allow any button push to continue

const int ButtonMap[] = { A2, 5, 6, 7, 11, 12 };  // Array to convert button number to actual pin number

volatile byte ButtonPushed[] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };	// Place to store status of buttons returned by interrupt
																		// Remember that lines are neutral pulled up HIGH
																		// So LOW is active push !!!!

volatile byte ActiveScreen[] = { LOW, LOW, LOW, LOW, LOW, LOW };  // Array to store current screen display and corresponding button assignments
// ActiveScreen [0] = Home
// ActiveScreen [1] = Setup
// ActiveScreen [2] = Setup Ht
// ActiveScreen [3] = Setup Spd
// ActiveScreen [4] = Run
// ActiveScreen [5] = Manual

const int PWMtoSpeedMapping[] = { 0, 45, 66, 70, 73, 76, 79, 82, 86, 89, 92, 95, 99, 102, 105, 108, 112, 115, 118, 121, 124 };  // Mapping determined by instrumentation.  Roughly 11 to 320 RPM range.


	
int SpeedPin = 13;		// Speed PWM filter circuit connected to digital pin 13.  Only works on Mega, not Uno

int mspeed = 0;			// Actual speed for motor
int SetSpeed = 75;		// Set speed in computer.  
int mPWM = 0;
float SlowTrigger = 1.0;	// Distance to target in feet where speed show be slowed down.
unsigned long StartMillis;	// Variable to be used in SoftDelay timer routine
unsigned long RevTime;		// Time stamp for when shaft revolution interrupt occurred.
float RotationPeriod;		// How long in ms did current shaft rotaton take


byte VOn48 = LOW;			// Is the 48 volt supply enabled; red if off, green if on
byte Relay1On = LOW;		// Relay 1 on relay shield.  LOW = 48V off and relay inactive.  HIGH = 48V on and relay active.
byte RunActive = LOW;		// Is the winch active; grey if inactive, yellow if active
byte Limit = HIGH;				// Has a limit switch been tripped; green if not tripped (HIGH), red if tripped (LOW)
byte Stall = LOW;					// An indicator that motor stop was due to a stall.  Turn Limit light yellow.
byte LastLimitSetting = HIGH;	// A place to store the limit setting so we don't needlessly keep running DrawLights in QuickLimitCheck.

volatile byte RunningSlow = LOW;			// Internal indicator that motor speed has been programatically reduced as target height is approached.
volatile int DeadManKeepAlive = 0;			// Indicator that power may have been killed by panic button, and special recovery is needed

int SlowSpeed = 50;				// Planned slow speed

float RunHeight = 0.00f;		// Running height
float ShaftCount;				// Counter to keep track of shaft rotations
float OldShaftCount;			// Previous value of ShaftCount
float SetHeight = 20.00f;		// Height target that's set by setup
byte Direction = HIGH;			// 1 = up, 0 = down
byte LimitDirection = LOW;		// Stores the direction of the limit hit.  Going up or down.
byte LimitMode = LOW;			// Limit mode LOW = Read input; mode HIGH = Write Output;
int NeedMotorTurns = 0;				// Counter to count a few turns of the motor to clear limit condition.
byte GlitchReset = LOW;			// Flag to note that height positioning may be reset
int BlockSpeedSensor = 0;	// Flag to temporarily stop processing of motor turn sensor interrupt during transient periods.  HIGH = blocked sensor.
byte Step1 = HIGH;				// Flags for each of the limit processing steps
byte Step2 = LOW;
byte Step3 = LOW;


float Speed = 0.00f;			// Feet per minute of system.  TBD depending on gearing

long eeAddress = 0;				// Location where height to be stored in EEPROM
								// 4096 bytes of EEPROM on Mega, and each good for about 100,000 writes.

// Variables for serial communication
int SerHeight; 
int SerSpeed;
String SerialInput = "MyLongString";		// String to receive serial input for processing.  Just setting aside defined space.
String StrCommand = "Short";
String StrValue = "Short";



// Setup initial stuff
void setup() {

	Serial.begin(9600);		// Initialize serial for troubleshooting
	
	tft.begin();	// Initialize display required

	// Initialize RunHeight from EEPROM
	randomSeed(analogRead(7));		// Ramdom number seed by reading unused port
	eeAddress = random(10,4000);		// select a random address to avoid overusing one area of EEPROM
	Serial.println(F("Random EEPROM address: "));  // Lets take a look
	Serial.println(eeAddress); 
	/*
	EEPROM.put(0, 0.00);	// Temporary Reset of count when active
	*/
	EEPROM.get(0, ShaftCount);	// Get last specially stored height from location 0.
	EEPROM.put(20, eeAddress);	// As a final recovery provision, store the address being used at location 20.  While possible this could be overwritten, it would be really bad luck. Recovery isn't likely to be necessary. 

	EEPROM.put(eeAddress, ShaftCount);	// Copy it to the randomly chosen location as starting point

	// Poplulate RevTime.  This will still produce a "wild" initial revolution count, but at least there's something there.
	RevTime = millis();

	// Initialize outputs to motor controller
	pinMode(A0, OUTPUT);  // Initialize relay driving pin as output.  Using analog pin as digital output to drive 48V relay
	pinMode(A1, OUTPUT);	//  Initialize relay driving pin as output.  Driving motor control up/down

	// Intialize input from limit switches
	pinMode(8, INPUT);		// Hi-Z input   // LimitInput pin

	// Initialize softbuttons around display
	pinMode(12, INPUT_PULLUP);    // Nominally +Increase sign	Button 5
	pinMode(11, INPUT_PULLUP);    // Nominally -Decrease sign	Button 4
	pinMode(7, INPUT_PULLUP);    // Far right button			Button 3
	pinMode(6, INPUT_PULLUP);    // 2nd from right button		Button 2
	pinMode(5, INPUT_PULLUP);    // 2nd from left button		Button 1
	// pinMode(4, INPUT_PULLUP);    // Far left button			Button 0   Replaced due to conflict with Ethernet shield  See ButtonMap line 46
	pinMode(A2, INPUT_PULLUP);	 // Far left button				Button 0

	tft.setRotation(3);  // Set screen to landscape with connections in upper left



	// Now lets briefly display the software version on the screen.
	tft.fillScreen(ILI9341_BLACK);		// Erase screen 
	tft.setTextColor(ILI9341_WHITE);	// text color
	tft.setFont(&FreeMono9pt7b);		// Switch to font
	tft.setTextSize(0);
	tft.setCursor(10, 20);
	tft.println(F("Moto-Mast Firmware Ver. 1.10"));				//	<========================================================  Update this when finished with editing
	delay(2000);
	tft.fillScreen(ILI9341_NAVY);		// Erase screen 

	DisplayHome();			// Display the home screen
	ActiveScreen[0] = HIGH; // Keep track of active screen
	
	SetMotorSpeed(0,0);		// Get PWM going.  Output should be about 1 volt which is just at motor off threshold. 

	// Set interrupt to handle soft button pushes
	pinMode(ButtonInterrupt1Pin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(ButtonInterrupt1Pin), ButtonPush, RISING);

	// Set interrupt to handle shaft rotation counting
	pinMode(ButtonInterrupt0Pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(ButtonInterrupt0Pin), ShaftRotation, RISING);

	// Do a CheckLimit function to catch case of limit fault upon powerup.  This is likely if the antenna is docked at the bottom.  Need to simulate a "Run Scenario"
	RunActive = HIGH;
	CheckLimit();		// Should only trigger Step 1 if limit is engaged.
	Serial.println(F("Initialization Limit Check Completed"));
	RunActive = LOW;

}


// Loop that runs continuously awaiting interrupts and other actions
void loop() {

	SerialCheck();		// Check to see if any serial port input is coming and process accordingly

	HomeActions();		// Implement Home screen softbutton actions

	ManualActions();	// Implement Manual screen softbutton actions

	Setup1Actions();	// Implement Setup1 screen softbutton actions

	SetupHeightActions(); // Implement Setup Height screen softbutton actions

	SetupSpeedActions();  // Implement Setup Speed screen softbutton actions

	RunActions();			// Implement Run Screen softbutton actions


	if (Lockout > 0) {
		Lockout = --Lockout;	// Decrement the lockout counter
	}

	QuickLimitChk();	// Do a quick check to see if status indicator is correct


	delay(20);			// Assume other stuff will take some time, so don't loop constantly
	if (BlockSpeedSensor) {
		-- BlockSpeedSensor;		// Decrement this in case it was set when 48V turned on
	}
	CheckLimit();		// Check status of limit switch, and act accordingly.  This is only active when motor is running.

	// Now check to see if the motor is running normally, or if it's been stopped for some reason such as a stall or abort button
	if (RunActive) {
		DeadManKeepAlive += 1;		// Increment the keep-alive.  If the motor is running normally, this should periodically be reset to 0 by the Shaft Rotation interrupt.

		if (DeadManKeepAlive >= 350) {		// Uhh oh.  Something went wrong and we need to regroup and kill any remaining motor functions.  Threshhold needs to be low enought to recover in a reasonable time
			// But not so low that a very slowly running motor will trigger this.

			// First, let's save the shaftcount in it's special "stopped" place so the height can be recovered during a reboot.
			EEPROM.put(0, ShaftCount);	// Save turning off height in special location 0 in EEPROM
			OldShaftCount = ShaftCount;

			// Now to clean things up a bit so program doesn't hang in an eternal loop.
			VOn48 = LOW;
			Relay1On = LOW;		// Turn Relay1 State to turn 48V off.  Default is off.
			digitalWrite(A0, Relay1On);
			RunActive = LOW;
			mspeed = 0;
			RunningSlow = LOW;	// Set speed back to high.
			DeadManKeepAlive = 0;  // Reset this back to zero so next restart starts with a full clock
			Stall = HIGH;		// Set the Limit indicator to show a stall condition.  This will get cleared at the next button push.
			DrawLights();

		}
	}

//	Serial.println(F("Main Loop Running"));

	
}

/*
======================================= Interrupt Routines Follow =======================================
*/

void ButtonPush() {		// Process button push interrupt
	byte val;
	int i;

	if (Lockout > 0 && AnyButtonPush) {		// Skip interrupt processing while Locked out for debounce.  Have to create an exception for AnyButtonPush halt state???  ...  && AnyButtonPush
		return;
	}

	// Otherwise proceed with interrupt processing
	// Which button was pushed?
	for (i = 0; i < 6; i++) {
		val = digitalRead(ButtonMap[i]);
		if (!val) {
			ButtonPushed[i] = LOW;		// Remember that active button is low!
		}
	}

	Lockout = 5;		// Setup lockout for a short time.  May need to adjust this later as loop delay increases
	AnyButtonPush = HIGH;	// Reset this once a button has been pushed

	Stall = LOW;		// Any push of a button clears the stall indication
/*
	Serial.println(F("Button Routine Exiting"));
	for (i = 0; i < 6; i++) {
		Serial.println(ButtonPushed[i]);
	}
*/
}

void ShaftRotation() {	// Increment or decrement a counter each time the motor shaft rotates.  Plus check for positioning versus limits and settings.

//	Serial.println(millis());		// Put in to check interrupt processing time.  Currently less than 1 mSec.

	if (BlockSpeedSensor) {
		return;						// 48V was just turned on, and false turns may result.  This will be turned off in Main Loop.
	}
	
	RotationPeriod = millis() - RevTime;	// Period of the shaft revolution is current time, minus time of last timestamp.
	if (RotationPeriod < 100) {				// Shaft will never turn at 600 rpm, so if the rotation period is this short, it's a transient in the counter
		return;
	}
	RevTime = millis();						// Update timestamp for the next rotation
	// Printout some diagnostics if needed
/*	Serial.println(F("Diagnostic Dump: Period, Height, ShaftCount"));
	Serial.println(RotationPeriod);	
	Serial.println(RunHeight);
	Serial.println(ShaftCount);
*/


	OldShaftCount = ShaftCount;
	if (Direction) {				// Going Up
		ShaftCount += 1;
	//	Serial.println(ShaftCount);
		DeadManKeepAlive = 0;		// Still running, so no need to take panic stop actions.

		if (!RunningSlow) {						// Skip this if system is already running slow
			if ((SetHeight - RunHeight) < SlowTrigger) {	// If not running slow, check to see if target height is within a foot
				SetMotorSpeed(SlowSpeed, SetSpeed);		// Slow down the motor
				RunningSlow = HIGH;						// Set the indicator flag for slow speed operation
			}
		}
		if (RunHeight >= (SetHeight-0.084) || RunHeight >= 26.74) {			// Want to stop motor immediately when it gets within 1 shaft rotation of target
			mspeed = 0;														// Mast distance traveled by one shaft revolution is .2565 inches.
			if (RunningSlow) {
				SetMotorSpeed(mspeed, SlowSpeed);
			}
			else {
				SetMotorSpeed(mspeed, SetSpeed);
			}
			RunActive = LOW;
			RunningSlow = LOW;			// Reset system for normal speed since system is now stopped 
			DrawLights();
		}

	}
	else {
		ShaftCount -= 1;			// Going Down
		DeadManKeepAlive = 0;		// Still running, so no need to take panic stop actions.

		if (!RunningSlow) {						// Skip this if system is already running slow
			if ((RunHeight - SetHeight) < SlowTrigger) {				// If not running slow, check to see if target height is within a foot
				SetMotorSpeed(SlowSpeed, SetSpeed);		// Slow down the motor
				RunningSlow = HIGH;						// Set the indicator flag for slow speed operation
			}
		}
		if (RunHeight <= (SetHeight+0.083) || RunHeight <= 10.083) {	// Want to stop motor immediately
			mspeed = 0;
			if (RunningSlow) {
				SetMotorSpeed(mspeed, SlowSpeed);
			}
			else {
				SetMotorSpeed(mspeed, SetSpeed);
			}
			RunActive = LOW;
			RunningSlow = LOW;			// Reset system for normal speed since system is stopped.
			DrawLights();
		}
	}



	EEPROM.put(eeAddress, ShaftCount);		// Temporarily store ShaftCount in randomly chosen EEPROM address.  Do I need this?  How would I use it?
	DisplayCurrentHeight(ShaftCount);		// Hopefully this doesn't bog down the interrupt handling

	if (NeedMotorTurns) {					// If we're recovering from a limit and just need a few revs, this will be >0  ie: 3
		NeedMotorTurns -= 1;				// Decrement the counter
		
	}
//	Serial.println(millis());
/*
	Serial.println(F("SetSpeed: "));  // Lets take a look
	Serial.println(SetSpeed);
	Serial.println(F("mspeed: "));  // Lets take a look
	Serial.println(mspeed);
	Serial.println(F("RunHeight: "));  // Lets take a look
	Serial.println(RunHeight);
	Serial.println(F(" "));
*/
}


/*
======================================= Subroutines Follow =======================================
*/

// Serial Check Subroutine -- Check to see if serial port input is available 
byte SerialCheck() {
	if (Serial.available() > 0) {
		// Get the string
		SerialInput = Serial.readStringUntil('\n');		// Read the incoming serial string until newline char
		ParseSerialString();							// Call a subroutine to parse the string and execute as appropriate
	}
}

byte ParseSerialString() {
	Serial.println(SerialInput);					// Echo string back during test
	StrCommand = SerialInput.substring(0, 3);		// Parse the command string
	Serial.println(StrCommand);

	if (StrCommand == "SD#") {						// SET DIRECTION
		StrValue = SerialInput.substring(3, 6);
		Serial.println(StrValue);					// Can remove this after dev
		if (StrValue == "000") {
			Direction = 0;
			Serial.println("Direction Down");
		}
		else if (StrValue == "111") {
			Direction = 1;
			Serial.println("Direction Up");
		}
		else {
			return;									// Abort if setting out of bounds
		}

		DisplayUpdate();
	};

	if (StrCommand == "SS#") {						// SET SPEED
		StrValue = SerialInput.substring(3, 6);
		Serial.println(StrValue);					// Can remove this after dev
		SerSpeed = StrValue.toInt();
		if (SerSpeed < 0 || SerSpeed > 99) {
			return;									// Abort if setting out of bounds
		}
		SetSpeed = SerSpeed;
		Serial.println((String)"SetSpd = " + SetSpeed);
		DisplayUpdate();
	};

	if (StrCommand == "SH#") {						// SET HEIGHT
		StrValue = SerialInput.substring(3, 6);
		Serial.println(StrValue);					// Can remove this after dev
		SerHeight= StrValue.toInt();
		if (SerHeight < 10 || SerHeight> 27) {
			return;									// Abort if setting out of bounds
		}
		SetHeight = SerHeight;
		Serial.println((String)"SetHt = " + SetHeight);
		DisplayUpdate();
	};

};


/* Programming Actions for Screens */

// Programming actions for Home Screen
byte HomeActions() {
	// Actions for Home Screen Softbuttons

	if ((!ButtonPushed[0]) & ActiveScreen[0]) {		// Button 0 selects Setup1 screen
		ResetActiveScreen();
		ActiveScreen[1] = HIGH;
		DisplaySetup1();
		ResetButtons();
	}

	if ((!ButtonPushed[1]) & ActiveScreen[0]) {		// Button 1 selects Run screen
		ResetActiveScreen();
		ActiveScreen[4] = HIGH;
		DisplayRun();
		ResetButtons();
	}

	if ((!ButtonPushed[3]) & ActiveScreen[0]) {		// Button 3 selects the Manual Screen
		ResetActiveScreen();
		ActiveScreen[5] = HIGH;
		DisplayManual();
		ResetButtons();
	}

}

// Programming actions for Manual Screen
byte ManualActions() {
	// Actions for Manual Screen Softbuttons

	if ((!ButtonPushed[0]) & ActiveScreen[5]) {		// Switch to Home Screen
		if (!RunActive) {		// Can't switch screens if motor is running
			ResetActiveScreen();
			ActiveScreen[0] = HIGH;
			mspeed = 0;		// Set speed to zero if leaving this screen
			RunActive = LOW;
			//  Direction = LOW;		// Not sure we want to change direction here
			DisplayHome();
			ResetButtons();
		}
	}


	if ((!ButtonPushed[5]) & ActiveScreen[5]) {		// Increase programmed motor speed
		if (!RunActive) {
			if (SetSpeed <= 95)
			{
				SetSpeed += 5;
			}
			else
			{
				SetSpeed = 100;
			}
			DisplayMotorSpeed(SetSpeed, 35, 135);
		}
		ResetButtons();
	}


	if ((!ButtonPushed[4]) & ActiveScreen[5]) {		// Decrease programmed motor speed
		if (!RunActive) {
			if (SetSpeed >= 5)
			{
				SetSpeed -= 5;
			}
			else
			{
				SetSpeed = 0;
			}
			DisplayMotorSpeed(SetSpeed, 35, 135);
		}
		ResetButtons();
	}

	if ((!ButtonPushed[1]) & ActiveScreen[5]) {		// Flip 48V status
		if (!RunActive) {							// Can't turn off 48V if motor is running
			VOn48 = !VOn48;
			DrawLights();
			Relay1On = !Relay1On;		// Flip Relay1 State to turn 48V on or off.  Default is off.
			if (Relay1On) {
				BlockSpeedSensor = 2;		// Transients on the line to the motor box register false "turns" on the interrupt.  Need to block the speed sensor interrupt temporarily.
			}
			digitalWrite(A0, Relay1On);
		}
		ResetButtons();
	}

	if ((!ButtonPushed[2]) & ActiveScreen[5]) {		// Flip RunActive status
		if (VOn48) {								// Only become active if 48V is on
			noInterrupts();		// Briefly disable interrupts to prevent screen artifacts
			RunActive = !RunActive;
			DrawLights();
			ResetButtons();
			interrupts();
			if (RunActive) {
				mspeed = SetSpeed;
				RunningSlow = LOW;						// Assume we want full speed when first activated; of course with usual startup ramp
				SetMotorSpeed(mspeed, SetSpeed);
			}

		
			else {										// We want to stop the motor.  Need to check whether it's running full speed, or at slow speed to apply correct ramp.
				mspeed = 0;
				if (RunningSlow) {
					SetMotorSpeed(mspeed, SlowSpeed);
				}
				else {
					SetMotorSpeed(mspeed, SetSpeed);
				}
			}
		}
	}


	if ((!ButtonPushed[3]) & ActiveScreen[5]) {		// Flip Up/Down status
		if (!RunActive) {							// Can't change direction while motor is running
			Direction = !Direction;					// Default direction is LOW which is Down
			digitalWrite(A1, Direction);			// Set direction on motor controller
			DisplayUpDown(Direction);

		}
		ResetButtons();
	}
}

// Programming actions for Setup1 Screen
byte Setup1Actions() {
	// Actions for Setup1 Screen Softbuttons

	if ((!ButtonPushed[0]) & ActiveScreen[1]) {		// Switch to Home Screen
		ResetActiveScreen();
		ActiveScreen[0] = HIGH;
		DisplayHome();
		ResetButtons();
	}
	
	if ((!ButtonPushed[1]) & ActiveScreen[1]) {		// Setup height screen
		ResetActiveScreen();	// Setting height goes here
		ActiveScreen[2] = HIGH;
		DisplaySetupHeight();
		ResetButtons();
	}

	if ((!ButtonPushed[2]) & ActiveScreen[1]) {		// Setup Max Speed screen
		ResetActiveScreen();	// Setting max speed goes here
		ActiveScreen[3] = HIGH;
		DisplaySetupSpeed();		// Setup max speed goes here
		ResetButtons();
	}
}

// Programming actions for SetupHeight Screen
byte SetupHeightActions() {
	// Actions for Setupheight Screen Softbuttons

	if ((!ButtonPushed[0]) & ActiveScreen[2]) {		// Switch to Home Screen
		NullGlitchReset();		// If any button other than 3 is pressed, be sure to anul the glitch reset flag
		ResetActiveScreen();
		ActiveScreen[0] = HIGH;
		DisplayHome();
		ResetButtons();
	}

	if ((!ButtonPushed[1]) & ActiveScreen[2]) {		// Back up to previous screen
		// Code to backup to previous screen
		NullGlitchReset();
		ResetActiveScreen();
		ActiveScreen[1] = HIGH;
		DisplaySetup1();
		ResetButtons();
	}

	if ((!ButtonPushed[2]) & ActiveScreen[2]) {		// Go to RUN screen
		// Code to switch to RUN screen 
			
		ResetActiveScreen();
		ActiveScreen[4] = HIGH;
		DisplayRun();
	
		NullGlitchReset();
		ResetButtons();
	}

	if ((!ButtonPushed[3]) & ActiveScreen[2]) {		// Reset ShaftCount -- REQUIRES 2 SUCCESSIVE PUSHES OF BUTTON
	// Code to reset ShaftCount to 0 in case of glitch
		if (!GlitchReset) {		// LOW state meaning this is first push
			tft.setTextColor(ILI9341_ORANGE);  // Set a note that 2nd push required to reset Current Height to zero
			tft.setFont(&FreeMono9pt7b);
			tft.setTextSize(0);
			tft.setCursor(1, 190);
			tft.println(F("Press ResCHt to zero Cur.Ht."));
			GlitchReset = HIGH;
		}
		else {		// This is second push, so reset ShaftCount to 0;
			NullGlitchReset();		// Reset the indicator byte and erase any possible messages on screen
			ShaftCount = 0.00;
			EEPROM.put(0, ShaftCount);	// Save turning off height in special location 0 in EEPROM
			tft.setTextColor(ILI9341_WHITE);	// Send an assurance message
			tft.setCursor(1, 190);   
			tft.println(F("Current Height reset"));
			delay(1000);
			tft.setTextColor(ILI9341_NAVY); 
			tft.setCursor(1, 190);
			tft.println(F("Current Height reset"));		// Erase by printing with background
			}
		ResetButtons();
	}

	if ((!ButtonPushed[5]) & ActiveScreen[2]) {		// Increase height setting
		NullGlitchReset();
		if (SetHeight <= 26)	{
			SetHeight += 1;
		}
		DisplaySetHeight(SetHeight);

		if (SetHeight > RunHeight) {				// Set direction on motor controller intelligently to up if new height is is greater than current height
			Direction = HIGH;
		}
		else if (SetHeight < RunHeight) {			// Same for down direction
			Direction = LOW;
		}
		digitalWrite(A1, Direction);				// Make sure direction is written to hardware	

		ResetButtons();
	}

	if ((!ButtonPushed[4]) & ActiveScreen[2]) {		// Decrease height setting
		NullGlitchReset();
		if (SetHeight >= 11)	{
			SetHeight -= 1;
		}
		DisplaySetHeight(SetHeight);

		if (SetHeight > RunHeight) {				// Set direction on motor controller intelligently to up if new height is is greater than current height
			Direction = HIGH;
		}
		else if (SetHeight < RunHeight) {			// Same for down direction
			Direction = LOW;
		}
		digitalWrite(A1, Direction);				// Make sure direction is written to hardware	

		ResetButtons();
	}
}

// Programming actions for SetupSpeed Screen
byte SetupSpeedActions() {
	// Actions for SetupSpeed Screen Softbuttons

	if ((!ButtonPushed[0]) & ActiveScreen[3]) {		// Switch to Home Screen
		ResetActiveScreen();
		ActiveScreen[0] = HIGH;
		DisplayHome();
		ResetButtons();
	}

	if ((!ButtonPushed[1]) & ActiveScreen[3]) {		// Back up to previous screen
		// Code to backup to previous screen
		ResetActiveScreen();
		ActiveScreen[1] = HIGH;
		DisplaySetup1();
		ResetButtons();
	}

	if ((!ButtonPushed[2]) & ActiveScreen[3]) {		// Go to RUN screen
		// Code to switch to RUN screen 

		ResetActiveScreen();
		ActiveScreen[4] = HIGH;
		DisplayRun();

		ResetButtons();
	}

		if ((!ButtonPushed[5]) & ActiveScreen[3]) {		// Increase speed setting
		if (SetSpeed < 95) {
			SetSpeed += 5;
		}
		else if (SetSpeed >= 95) {
			SetSpeed = 99;
		}
		DisplaySetSpeed(SetSpeed);
		ResetButtons();
	}

	if ((!ButtonPushed[4]) & ActiveScreen[3]) {		// Decrease speed setting
		if (SetSpeed == 99) {
			SetSpeed = 95;
		}
		else if (SetSpeed >= 5) {
			SetSpeed -= 5;
		}
		DisplaySetSpeed(SetSpeed);
		ResetButtons();
	}
}

// Programming actions for Run Screen
byte RunActions() {
	// Actions for Run Screen Softbuttons

	if ((!ButtonPushed[0]) & ActiveScreen[4]) {		// Switch to Home Screen
		ResetActiveScreen();
		ActiveScreen[0] = HIGH;
		DisplayHome();
		ResetButtons();
	}

	if ((!ButtonPushed[1]) & ActiveScreen[4]) {		// Button 1 selects Setup1 screen
		ResetActiveScreen();
		ActiveScreen[1] = HIGH;
		DisplaySetup1();
		ResetButtons();
	}

	if ((!ButtonPushed[5]) & ActiveScreen[4]) {		// Button 5 is emergency halt
		ResetActiveScreen();
		ActiveScreen[4] = HIGH;
		EmergencyHalt();		// Run emergency halt routine.  Return here when any button pushed.
		DisplayRun();
		ResetButtons();
	}

	if ((!ButtonPushed[2]) & ActiveScreen[4]) {		// Button 2 is Auto-Up
		ResetActiveScreen();
		ResetButtons();			// Move this before any scenario to we can check for the HALT button
		ActiveScreen[4] = HIGH;	// Probably didn't need to change the active screen since it stays the same. Keeping this in place to memorialize an epic bug in the code.
		Direction = HIGH;		// Set the global variables automatically for full height
		SetHeight = 25;			// Leave speed setting in place.  Assume we will mostly want to use the same speed most times.
		RunScenario();
		DisplayRun();
	}

	if ((!ButtonPushed[3]) & ActiveScreen[4]) {		// Button 2 is Auto-Down
		ResetActiveScreen();
		ResetButtons();			// Move this before any scenario to we can check for the HALT button
		ActiveScreen[4] = HIGH;	// Probably didn't need to change the active screen since it stays the same.
		Direction = LOW;		// Set the global variables automatically for fully docked position
		SetHeight = 10;			// Leave speed setting in place.  Assume we will mostly want to use the same speed most times.
		RunScenario();
		DisplayRun();
	}

	if ((!ButtonPushed[4]) & ActiveScreen[4]) {		// Button 4 is Run using current settings
		ResetActiveScreen();
		ResetButtons();			// Move this before any scenario to we can check for the HALT button
		ActiveScreen[4] = HIGH;	// Probably didn't need to change the active screen since it stays the same.
		// No change of height, speed, or direction.  Gonna go with what we have.
		RunScenario();
		DisplayRun();
	}


}


/* Screen Displays*/

// Display the Home Screen
byte DisplayHome() {

	int Indent2 = 40;
	int BotLine = 216;
	int StatusTop = 75;

	tft.fillScreen(ILI9341_NAVY);  // Clear old screen

	DrawLights();		// Draw indicator lights at top right on screen
	DrawButtons();		// Draw soft button outlines on display

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setTextSize(0);

	tft.setCursor(25, StatusTop);
	tft.println(F("Status:"));		// Large font just for header

	tft.setCursor(10, 20);
	tft.println(F("Home"));
	tft.setFont(&FreeMono9pt7b);		// Switch to smaller font for all else

	tft.setCursor(Indent2, StatusTop+25);		// Print home status elements
	CalcRunHeight();
	sprintfloat("Current Ht: ", RunHeight);		// Call special function that will parse float and print to screen

	DisplayStatusHeight(SetHeight);

	/*	Removing this bit of "intelligence" since it sort of mucks up remote operation via serial.

	if (SetHeight > RunHeight) {				// Set direction on motor controller intelligently to up if height is is greater than current height
		Direction = HIGH;
	}
	else if (SetHeight < RunHeight) {			// Same for down direction
		Direction = LOW;
	}
	digitalWrite(A1, Direction);				// Make sure direction is written to hardware	
	*/

	DisplayStatusUpDown(Direction);
	
	DisplayStatusSpeed(SetSpeed);

	tft.setTextColor(ILI9341_YELLOW);  // set color

	tft.setCursor(11, BotLine);		// Now display softbutton labels at bottom
	tft.println(F("SETUP"));
	tft.setCursor(100, BotLine);
	tft.println(F("RUN"));
	tft.setCursor(242, BotLine);
	tft.println(F("MANUAL"));

}

// Display the Manual Screen
byte DisplayManual() {

	int Indent2 = 40;
	int BotLine = 216;
	int StatusTop = 75;

	tft.fillScreen(ILI9341_NAVY);  // Clear old screen

	DrawLights();		// Draw indicator lights at top right on screen
	DrawButtons();		// Draw soft button outlines on display

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setTextSize(0);

	tft.setCursor(25, StatusTop);
	tft.println(F("Status:"));		// Large font just for header

	tft.setCursor(10, 20);
	tft.println(F("Manual"));
	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else

	tft.setCursor(Indent2, StatusTop + 25);		// Print home status elements
	CalcRunHeight();
	sprintfloat("Current Ht: ", RunHeight);		// Call special function that will parse float and print to screen

	tft.setCursor(Indent2, StatusTop + 50);
	if (!Direction) {
		tft.println(F("Direction:   Down"));
	}
	else {
		tft.println(F("Direction:   Up"));
	}

	tft.setCursor(Indent2, StatusTop + 75);
	SprintAnInt("Speed 0-100:", SetSpeed);

	tft.setTextColor(ILI9341_YELLOW);  // set color

	tft.setCursor(16, BotLine);		// Now display softbutton labels at bottom
	tft.println(F("HOME"));
	tft.setCursor(100, BotLine);
	tft.println(F("48V"));
	tft.setCursor(165, BotLine);
	tft.println(F("RUN/ST"));
	tft.setCursor(248, BotLine);
	tft.println(F("UP/DN"));

	tft.setCursor(265, 134);	// Display softbutton side labels
	tft.println(F("SPEED"));
	tft.setTextSize(2);		// Switch to larger font size for + and -
	tft.setCursor(282, 97);	
	tft.println(F("+"));
	tft.setCursor(282, 177);
	tft.println(F("-"));

}

// Display the Setup 1 Screen
byte DisplaySetup1() {

	int Indent2 = 40;
	int BotLine = 216;
	int StatusTop = 75;

	tft.fillScreen(ILI9341_NAVY);  // Clear old screen

	DrawLights();		// Draw indicator lights at top right on screen
	DrawButtons();		// Draw soft button outlines on display

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setTextSize(0);

	tft.setCursor(25, StatusTop + 20);
	tft.println(F("Change Setting:"));		// Large font just for header

	tft.setCursor(10, 20);
	tft.println(F("Setup"));
	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else

	tft.setCursor(Indent2, StatusTop + 45);		// Print home status elements
	sprintfloat("Set Height: ", SetHeight);		// Call special function that will parse float and print to screen

	tft.setCursor(Indent2, StatusTop + 70);
	SprintAnInt("Set Speed:  ", SetSpeed);
	
	tft.setTextColor(ILI9341_YELLOW);  // set color

	tft.setCursor(16, BotLine);		// Now display softbutton labels at bottom
	tft.println(F("HOME"));
	tft.setCursor(86, BotLine);
	tft.println(F("HEIGHT"));
	tft.setCursor(169, BotLine);
	tft.println(F("SPEED"));
}

// Display the Setup Height Screen
byte DisplaySetupHeight() {

	int Indent2 = 40;
	int BotLine = 216;
	int StatusTop = 75;

	tft.fillScreen(ILI9341_NAVY);  // Clear old screen

	DrawLights();		// Draw indicator lights at top right on screen
	DrawButtons();		// Draw soft button outlines on display

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setTextSize(0);

	tft.setCursor(20, StatusTop + 20);
	tft.println(F("Edit Max Height:"));		// Large font just for header

	tft.setCursor(70, 165);		// Write Current Height Setting on screen while in big font
	tft.setTextSize(2);
	sprintfloat("", SetHeight);
	tft.setTextSize(0);

	tft.setCursor(10, 20);
	tft.println(F("Set Height"));
	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else

	tft.setCursor(18, StatusTop + 45);
	tft.println(F("(10ft to Max 27ft)"));

	tft.setTextColor(ILI9341_YELLOW);  // set color

	tft.setCursor(16, BotLine);		// Now display softbutton labels at bottom
	tft.println(F("HOME"));
	tft.setCursor(96, BotLine);
	tft.println(F("BACK"));
	tft.setCursor(179, BotLine);
	tft.println(F("RUN"));
	tft.setCursor(244, BotLine);
	tft.println(F("ResCHt"));

	tft.setCursor(252, 134);	// Display softbutton side labels
	tft.println(F("HEIGHT"));
	tft.setTextSize(2);		// Switch to larger font size for + and -
	tft.setCursor(282, 97);
	tft.println(F("+"));
	tft.setCursor(282, 177);
	tft.println(F("-"));
}

// Display the Setup Speed Screen
byte DisplaySetupSpeed() {

	int Indent2 = 40;
	int BotLine = 216;
	int StatusTop = 75;

	tft.fillScreen(ILI9341_NAVY);  // Clear old screen

	DrawLights();		// Draw indicator lights at top right on screen
	DrawButtons();		// Draw soft button outlines on display

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setTextSize(0);

	tft.setCursor(20, StatusTop + 20);
	tft.println(F("Edit Max Speed:"));		// Large font just for header

	tft.setCursor(70, 165);		// Write Current Speed Setting on screen while in big font
	tft.setTextSize(2);
	SprintAnInt("", SetSpeed);
	tft.setTextSize(0);


	tft.setCursor(10, 20);
	tft.println(F("Set Speed"));
	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else

	tft.setCursor(18, StatusTop + 45);
	tft.println(F("(0 to 99%)"));

	tft.setTextColor(ILI9341_YELLOW);  // set color

	tft.setCursor(16, BotLine);		// Now display softbutton labels at bottom
	tft.println(F("HOME"));
	tft.setCursor(96, BotLine);
	tft.println(F("BACK"));
	tft.setCursor(179, BotLine);
	tft.println(F("RUN"));

	tft.setCursor(257, 134);	// Display softbutton side labels
	tft.println(F("SPEED"));
	tft.setTextSize(2);		// Switch to larger font size for + and -
	tft.setCursor(282, 97);
	tft.println(F("+"));
	tft.setCursor(282, 177);
	tft.println(F("-"));
}

// Display the Run Screen
byte DisplayRun() {

	int Indent2 = 40;
	int BotLine = 216;
	int StatusTop = 75;

	tft.fillScreen(ILI9341_NAVY);  // Clear old screen

	DrawLights();		// Draw indicator lights at top right on screen
	DrawButtons();		// Draw soft button outlines on display

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMonoBold12pt7b);

	tft.setCursor(25, StatusTop);
	tft.println(F("RUN Status:"));		// Large font just for header

	tft.setCursor(10, 20);
	tft.println(F("RUN"));
	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else

	tft.setCursor(Indent2, StatusTop + 25);		// Print home status elements
	CalcRunHeight();
	sprintfloat("Current Ht: ", RunHeight);		// Call special function that will parse float and print to screen

	DisplayStatusHeight(SetHeight);

	DisplayStatusUpDown(Direction);
	DisplayStatusSpeed(SetSpeed);

	tft.setTextColor(ILI9341_YELLOW);  // set color

	tft.setCursor(17, BotLine);		// Now display softbutton labels at bottom
	tft.println(F("HOME"));
	tft.setCursor(91, BotLine);
	tft.println(F("SETUP"));
	tft.setCursor(185, BotLine);
	tft.println(F("UP"));
	tft.setCursor(265, BotLine);
	tft.println(F("DN"));
	tft.setCursor(170, BotLine - 17);
	tft.println(F(" ___AUTO___"));

	// Now vertical softbuttons
	tft.setCursor(258, 94);
	tft.println(F("HALT"));
	tft.setCursor(268, 174);
	tft.println(F("RUN"));


}



// Run a scenario driven by the Run Screen
void RunScenario() {	// Uses as inputs the global variables Direction, SetSpeed, SetHeight

	// First turn on 48V
	noInterrupts();  // Turn off interrupts briefly
	if (Step2) {
		CheckLimit();		// Need to activate step 2 if caused by limit hit on previous run.
	}
	VOn48 = HIGH;
	Relay1On = HIGH;		// Flip Relay1 State to turn 48V on or off.  Default is off.
	digitalWrite(A0, Relay1On);
	BlockSpeedSensor = 1;	// Keep false turn indications cause by line transients blocked temporarily
	DrawLights();			// Show 48V has been switched on
	RunActive = HIGH;

	interrupts();

	CheckLimit();			// Just to see if fault might have occurred now that RunActive is high and setting up for motor run.  This could trigger Step 1
							// This takes care of a very unusual circumstance where limit is indicated after the circuit is initialized.
	if (Step2) {
		RunActive = LOW;
		CheckLimit();			// Second pass to activate Step 2 if necessary.  Have to temporarily turn off RunActive for this step to take.
		RunActive = HIGH; 
	}
	
	
	SoftDelay(1000);				// Give 48 volts time to turn on and stabilize
	BlockSpeedSensor = 0;		// Unblock speed sensor.
	noInterrupts();
	DrawLights();					// Update lights to show active running

	digitalWrite(A1, Direction);			// Set direction on motor controller.  HIGH is up, LOW is down

	DisplayStatusHeight(SetHeight);			// Show new settings in case they've been changed prior to running scenario
	DisplayStatusSpeed(SetSpeed);
	DisplayStatusUpDown(Direction);



	RunningSlow = LOW;		// Assume full speed, But...
	if (Direction && (SetHeight - RunHeight) < SlowTrigger) {		// If going up and Set Height minus Current Height is less than trigger distance
		RunningSlow = HIGH;				// If currently less that 1 foot from target height, then select reduced speed of operation
	}
	if (!Direction && (RunHeight - SetHeight) <= SlowTrigger) {	// If going down and Current Height minus Set Height is less than trigger distance
		RunningSlow = HIGH;		// Also slow down the speed
	}

	mspeed = SetSpeed;

	if (RunningSlow) {			// Take care of case where target is real close to actual setting.  (Within trigger distance)
		mspeed = SlowSpeed;
	}

	if (Direction && (RunHeight > (SetHeight - 0.25))) {		// Prevent any running at all if current height is less than one rotation from target
		RunActive = LOW;
	}

	if (!Direction && (RunHeight < (SetHeight + 0.25))) {		// Same for case going down
		RunActive = LOW;
	}
	interrupts();  // Re-enable interrupts prior to running

		// OK ... ready to rumble
	if (RunActive) {
		SetMotorSpeed(mspeed, SetSpeed);
	}
	

	// Now we need to delay while the motor does its thing.  Positioning is being monitored by interrupt routines, so need to leave interrupts active all the time.
	while (RunActive) {		// while the motor is running
		CheckLimit();		// Interrupts may be working, but not cycling through main loop, so need to frequently check to see if limit switch has been hit.
		SoftDelay(100);		// Wait 0.1 seconds and see if the motor is still running

		if (!digitalRead(ButtonMap[5])) {		// Check for Emergency Halt during loop.  Button 5 is emergency halt and will go low.  Not depending on any interrupt processing here.
			ResetActiveScreen();
			ActiveScreen[4] = HIGH;	// Stay on the RUN screen
			
			// First, let's save the shaftcount in it's special "stopped" place so the height can be recovered during a reboot.
			EEPROM.put(0, ShaftCount);	// Save turning off height in special location 0 in EEPROM
			OldShaftCount = ShaftCount;

			EmergencyHalt();		// Run emergency halt routine.  Return here when any button pushed.
			ResetButtons();
		}

		// Now check to see if the motor is running normally, or if it's been stopped for some reason such as a stall or abort button
		if (RunActive) {
			DeadManKeepAlive += 1;		// Increment the keep-alive.  If the motor is running normally, this should periodically be reset to 0 by the Shaft Rotation interrupt.

			if (DeadManKeepAlive >= 50) {		// Uhh oh.  Something went wrong and we need to regroup and kill any remaining motor functions.  Threshhold needs to be low enought to recover in a reasonable time
				// But not so low that a very slowly running motor will trigger this.

				// First, let's save the shaftcount in it's special "stopped" place so the height can be recovered during a reboot.
				EEPROM.put(0, ShaftCount);	// Save turning off height in special location 0 in EEPROM
				OldShaftCount = ShaftCount;

				// Clean up will be done in the regular exit below 
				RunActive = LOW;		// But have to let the regular routine know that it's time to exit.
				Stall = HIGH;			// Set the flag to indicate failure due to stall
			}
		}
	}

	// Only get here once motor has finished and stopped

	// Job done, now to cleanup a bit
	SoftDelay(1000);	// Give things a second to stabilize in off state
	VOn48 = LOW;
	Relay1On = LOW;		// Turn Relay1 State to turn 48V off.  Default is off.
	digitalWrite(A0, Relay1On);
	RunActive = LOW;
	mspeed = 0;
	RunningSlow = LOW;	// Set speed back to high.
	DeadManKeepAlive = 0;  // Reset this back to zero so next restart starts with a full clock
	DrawLights();
	// Serial.println(F("Scenario is exiting."));

}


// Emergency Halt Subroutine
void EmergencyHalt() {
	
	digitalWrite(A0, LOW);	// Turn off 48 Volts instantly
	VOn48 = LOW;		// Have to set the indicators to correctly indicate stopped conditioni
	Relay1On = LOW;
	RunActive = LOW;
	SoftDelay(100);			// Give time for any queued or in-process interrupts to be completed
	noInterrupts();			// Disable interrupts while processing

	tft.fillScreen(ILI9341_NAVY);  // Clear old screen
	DrawLights();		// Draw indicator lights at top right on screen
	DrawButtons();		// Draw soft button outlines on display

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setTextSize(3);

	tft.setCursor(25, 100);
	tft.println(F("HALTED"));		// Large font for emphasis
	tft.setTextSize(0);
	tft.setCursor(40, 180);
	tft.println(F("Press any button"));
	AnyButtonPush = LOW;
	interrupts();
	while (!AnyButtonPush) {
		// Do nothing important and wait for a button to be pushed.  But have to allow interrupts to work!
		SoftDelay(100);
	}
// Then return to normal processing
}

// Calculate RunHeight variable based on ShaftCount setting.  0.2565 inches of travel per shaft revolution.  Winch is 20:1 worm gearing.
// Approx. 116 motor turns resulted in 29 inches of one section being extended.  There are 4 sections for a total of 116 inches of height gained.  There were about 5 1/2 turns of the drum.
// Therefore ShaftCount divided by 12 gives height of extension in feet.

void CalcRunHeight() {
	RunHeight = 10.0 + (ShaftCount/12);
}

// Update displayed screen
void DisplayUpdate() {
	if (ActiveScreen[0]) {
		DisplayHome();
	}
	else if (ActiveScreen[1]) {
		DisplaySetup1();
	}
	else if (ActiveScreen[2]) {
		DisplaySetupHeight();
	}
	else if (ActiveScreen[3]) {
		DisplaySetupSpeed();
	}
	else if (ActiveScreen[4]) {
		DisplayRun();
	}
	else if (ActiveScreen[5]) {
		DisplayManual();
	}
}

// Draw display soft buttons (SB) on the screen
byte DrawButtons() {
	int SBStartX[] = { 14, 93, 172, 251 };
	int SBStartY[] = { 64, 143 };			// Was 55, 134 before tweaking downward
	byte k; 

	for (k = 0; k < 4; ++k) {	//Print each horizontal softbutton
		tft.fillRoundRect(SBStartX[k], 225, 51, 40, 15, ILI9341_IVORY);
	}

	for (k = 0; k < 2; ++k) {
		tft.fillRoundRect(305, SBStartY[k], 40, 51, 15, ILI9341_IVORY);  // Print vertical softbuttons
	}
}

// Draw screen led lights
byte DrawLights() {

	int V48Color = ILI9341_RED;		// Default light colors
	int RunColor = ILI9341_DARKGREY;
	int LimitColor = ILI9341_GREEN; 

	const int YCenter = 12;
	const int YText = 34;
	const int CRadius = 8;

	if (VOn48) {				// Set the colors according to actual status
		V48Color = ILI9341_GREEN;
	}

	if (RunActive == 1) {				
		RunColor = ILI9341_YELLOW;
	}
	if (Stall) {
		LimitColor = ILI9341_YELLOW;  // Turn Limit indicator yellow if there was a motor stall.
	}
	if (!Limit) {				// Trip limit when line goes low
		LimitColor = ILI9341_RED;
	}


	tft.fillCircle(180, YCenter, CRadius, V48Color);  // Draw the colored lights in correct color
	tft.fillCircle(230, YCenter, CRadius, RunColor);
	tft.fillCircle(280, YCenter, CRadius, LimitColor);


	tft.setTextColor(ILI9341_WHITE);		// Setup to print light labels
	tft.setFont(&FreeMono9pt7b);
	tft.setTextSize(0);

	tft.setCursor(164, YText);				// Print light labels
	tft.println(F("48V"));

	tft.setCursor(214, YText);
	tft.println(F("Run"));

	tft.setCursor(255, YText);
	tft.println(F("Limit"));

}

// Quick Limit Check
void QuickLimitChk() {
	if (!LimitMode) {		// Avoid testing the limit circuit when in Step 2 with Brake Overwrite and port D8 is in Output mode.
		Limit = digitalRead(LimitInput);	// Limit fault when a switch is opened will have D8 (LimitInput) in LOW state.  Limit = LOW is red on display.
		if (Limit != LastLimitSetting) {	// Must have had a change in limit switch setting
			DrawLights();					// No immediate action other than correctly updating indicator lights.  The heavy lifting is done when motor is running in CheckLimit routine below.
			LastLimitSetting = Limit;		// Update the limit memory for next loop.
		}
	}
}

// Check Limit switch and take action as appropriate
void CheckLimit() {		// Three Steps which are mutually exclusive and usually only need to be run once
//	Serial.println(F("Limit Check Active"));

	// STEP1
	if (RunActive & (!digitalRead(LimitInput)) & Step1) {	// Only check limits if motor is running.  If D8 is low, a limit switch has been opened
	//	Serial.println(F("Limit hit sensed"));
		Limit = LOW;		// Change Limit state to fault/red
		DrawLights();			// Change display
		LimitDirection = Direction;		// Capture which direction the mast was moving
		if (RunHeight < 11.0) {
			LimitDirection = 0;		// Assume limit direction was down if near the bottom of the range when limit was hit
			if (RunHeight < 10.20) {	// This means lower limit has been reached, but there are some residual counts left due to transients.  So do a reset to zero count.
				RunHeight = 10.0;
				ShaftCount = 0;
			}
		}
		if (RunHeight > 22.0) {
			LimitDirection = 1;		// Assume limit direction was up if near the top of the range when limit was hit
		}
		NeedMotorTurns = 3;			// Assume that three turns of the motor shaft in other direction will clear magnetic trip of limit switch
		Step1 = LOW;
		Step2 = HIGH;				// Now we're ready for the second step

		Serial.println(F("Step1 Complete"));
	}

	// STEP2 - Now further processing of limit recovery
	if (!RunActive & (Direction != LimitDirection) & Step2) {	// Run has been stopped and direction changed
		Limit = HIGH; 
		DrawLights();	// Reset the limit indicators on the display
		pinMode(LimitInput, OUTPUT);	// We're going to use the port as an output to force release of the brake
		digitalWrite(LimitInput, HIGH);
		LimitMode = HIGH;				// Set flag that limit port is in temporary write mode
		Step2 = LOW;					// Don't need to run this again and ready for 3rd step
		Step3 = HIGH;
		Serial.println(F("Step2 Complete"));
	}

	// STEP3 - Now to check and see if it's time to return limit system to read after a few motor turns
	if (LimitMode & RunActive & Step3) {
		if (!NeedMotorTurns) {
			pinMode(LimitInput, INPUT);		// Return limit sensing port to read after a few turns of the motor shaft.
			LimitMode = LOW;				// Reset limit port flag to signify it's in read
			Step3 = LOW;
			Step1 = HIGH;		// Limit has been dealt with so return system to armed condition
			Limit = HIGH;		// Change Limit state to fault/red
			DrawLights();			// Change display
			Serial.println(F("Step3 Complete"));
		}
	}
}

//Reset the active screen array
int ResetActiveScreen() {       // Reset the ActiveScreen array to null
	for (int i = 0; i < 6; i++) {
		ActiveScreen[i] = LOW;
	}
}

// Reset the push-buttons array
int ResetButtons() {			// Reset the buttons
		for (int i = 0; i < 6; i++) {
		ButtonPushed[i] = HIGH;
	}
}

// SoftDelay that doesn't disable interrupts
void SoftDelay(int TimeInMillisec) {
	StartMillis = millis();
	while ((millis() - StartMillis) < TimeInMillisec) {
		// Do nothing for TimeInMillisec
	}
}

// Specific to reset of Current Height counter menu item.  Reset flag to skip counter reset, and erase any reminder by printing same message in background color.
void NullGlitchReset() {
	tft.setTextColor(ILI9341_NAVY);  // Erase 2nd press note that may be present by writing in background color
	tft.setFont(&FreeMono9pt7b);
	tft.setTextSize(0);
	tft.setCursor(1, 190);
	tft.println(F("Press ResCHt to zero Cur.Ht."));
	GlitchReset = LOW;
}

// SPRINTFLOAT - Print String and Floating Point Number at current cursor
byte sprintfloat(char *StringtoPrint, float CurrentHeight) {  // Separate components of Float, and print resulting string at cursor

	double WholeNum, FractionNum, FractWhole;
	int intWhole, intFract;
	char PrnString[16];

	FractionNum = modf(CurrentHeight, &WholeNum);		// Split off whole and fractional parts of float

	intWhole = int(WholeNum);
	FractionNum = FractionNum * 100;					// Move the decimal point 2 places to right on resulting fractional part

	FractionNum = modf(FractionNum, &FractWhole);		// Repeat to get a two digit whole number for fractional part
	intFract = int(FractWhole);

	sprintf(PrnString, "%s %d.%02d", StringtoPrint, intWhole, intFract);  // Print the whole mess to string PrnString
	tft.println(PrnString);

}

// SPRINTANINT - Print String and Integer at current cursor
byte SprintAnInt(char* StringtoPrint, int IntToPrint) {  // Separate components string to print at cursor
	
	char PrnString[16];

	sprintf(PrnString, "%s %d", StringtoPrint, IntToPrint);  // Print the elements to PrnString
	tft.println(PrnString);
}

// Display Motor Speed
byte DisplayMotorSpeed(int ms, int StartX, int StartY) {	

	tft.fillRect(StartX, StartY, 200, 25, ILI9341_NAVY);  // Erase old speed first.  Note that args are (x, y, len x, len y)
	tft.setTextColor(ILI9341_WHITE);
	tft.setFont(&FreeMono9pt7b);
	tft.setTextSize(0);
	tft.setCursor(StartX +5, StartY +15);
	SprintAnInt("Speed 0-100:", ms);
}

// Display Set Height
byte DisplaySetHeight(float sheight) {
	tft.fillRect(65, 130, 180, 40, ILI9341_NAVY);  // Erase old height first.  Note that args are (x, y, len x, len y)
	tft.setTextColor(ILI9341_WHITE);
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setTextSize(2);
	tft.setCursor(70, 165);
	sprintfloat("", sheight);
}

// Display Set Speed
byte DisplaySetSpeed(int sspeed) {
	tft.fillRect(65, 130, 200, 40, ILI9341_NAVY);  // Erase old speed first.  Note that args are (x, y, len x, len y)
	tft.setTextColor(ILI9341_WHITE);  // Text setup
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setCursor(70, 165);		// Write Current Speed Setting on screen while in big font
	tft.setTextSize(2);
	SprintAnInt("", sspeed);
}

// DisplayStatusSpeed  This is primarily used in Home and Run diplays
void DisplayStatusSpeed(int StatusSpeed) {

	int Indent2 = 40;
	int StatusTop = 75;

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMono9pt7b);
	tft.setTextSize(0);

	tft.setCursor(Indent2, StatusTop + 100);
	SprintAnInt("Speed 0-99%:", StatusSpeed);		// Routine will do a conversion of speed to int
}

// DisplayStatusHeight This is primarily used in Home and Run diplays
void DisplayStatusHeight(int StatusHeight) {

	int Indent2 = 40;
	int StatusTop = 75;

	tft.fillRect(155, 110, 150, 25, ILI9341_NAVY);  // Erase old count first.  Note that args are (x, y, len x, len y)

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMono9pt7b);
	tft.setTextSize(0);

	tft.setCursor(Indent2, StatusTop + 50);
	sprintfloat("Set Height: ", StatusHeight);

}

// DisplayStatusUpDown This is primarily used in Home and Run diplays
void DisplayStatusUpDown(int StatusUD) {

	int Indent2 = 40;
	int StatusTop = 75;

	tft.setTextColor(ILI9341_WHITE);  // Preliminary text setup
	tft.setFont(&FreeMono9pt7b);
	tft.setTextSize(0);

	tft.fillRect(35, 135, 200, 25, ILI9341_NAVY);  // Erase old direction first.  Note that args are (x, y, len x, len y)

	tft.setCursor(Indent2, StatusTop + 75);
	if (!Direction) {
		tft.println(F("Direction:   Down"));
	}
	else {
		tft.println(F("Direction:   Up"));
	}
}

// Display Up/Down
byte DisplayUpDown(byte Directn) {
	tft.fillRect(35, 110, 200, 25, ILI9341_NAVY);  // Erase old direction first.  Note that args are (x, y, len x, len y)
	tft.setCursor(40, 125);
	tft.setTextSize(0);
	tft.setFont(&FreeMono9pt7b);
	tft.setTextColor(ILI9341_WHITE);
	if (!Directn) {
		tft.println(F("Direction:   Down"));
	}
	else {
		tft.println(F("Direction:   Up"));
	}
}

// Display Current Height -- Based on global variables ShaftCount and RunHeight
byte DisplayCurrentHeight(float CurHt) {

	tft.fillRect(155, 85, 85, 25, ILI9341_NAVY);  // Erase old count first.  Note that args are (x, y, len x, len y)
	tft.setFont(&FreeMono9pt7b);
	tft.setTextSize(0);
	tft.setTextColor(ILI9341_WHITE);
	tft.setCursor(172, 100);
	CalcRunHeight();			// Calculate Running Height from ShaftCount
	// Serial.println(CurHt);	// Debug display of ShaftCount
	sprintfloat("", RunHeight);
}

// Calculate mPWM from requested speed setting and speed mapping table.  Input is desired speed from 0 to 100 (percent). Routine returns mPWM from table
int CalculatePWM(int RequestedSpeed) {
	int PWMtoReturn;
	if (RequestedSpeed == 99) {
		RequestedSpeed = 100;		// Have to make a small adjustment at top of range for math to work out 100/5 = 20.  99/5 will be 19 in int math. 
	}
	PWMtoReturn = PWMtoSpeedMapping[RequestedSpeed / 5];		// Adding a fudge factor of 5 because motor seems a bit slow
	if (Direction) {				// If it's going up, slow down a bit.  Speed up if going down
		PWMtoReturn -= 5;
	}
	else {
		PWMtoReturn += 8;		// OK, maybe 8.  :-)
		}
	return PWMtoReturn;
}

// Set the speed control PWM output accordingly.  This is what does the work of getting the motor to run.
byte SetMotorSpeed(int ms, int mset) {		// Input for ms and mset is 0 to 100 (percent).  ms is the speed to attain, mset is the speed setting in the computer
	
	// Now to get 256 count equivalent for PWM control of motor.
	// Formula is: mPWM = (1.30 * ms) + 60;
	/* This would be 2.55 to make it 100% on, but the motor control circuit doesn't seem to work above 4.0 volts on the input.
	   Therefore, we're scaling back to about 80% which should yield just short of 4 volts.

	   Actual effective control range on the motor is from about 1.2volts to 3.95 volts.
	*/

//	Serial.print(F("RunningSlow = "));  // Just looking at output for now
//	Serial.println(RunningSlow);


	// Motor controller speed is about 2X faster going up than down due to hardware design.  So let's divide speed going up by 2 to make things even.  
/*
	if (Direction) {		// Up is high, down is low
		mset = mset / 2;
		ms = ms / 2;
	//	Serial.println(mset);
	}
*/

	// Ignoring all this math which didn't work out as well as hoped, we're now using an instrumentation based mapping between PWM and RPM contained in the array PWMtoSpeedMapping.

	int CurrentPWM;		// Local variables needed for some calculations
	int TargetPWM;
	int RampDelay = 50;

	// Write the PWM speed to the pin with a startup or shutdown ramp
	
	if (ms == mset) {		// motor is being turned on and speed should be ramped up
		mPWM = CalculatePWM(mset);
		for (int i = 50; i <= mPWM; i++) {		// Value of 50 is about lowest PWM where motor is still running.  So effectively starts at 0 speed.
			analogWrite(SpeedPin, i);
			delay(RampDelay);					// Adjust this until smooth ramp is obtained.  For some reason, SoftDelay sub doesn't work properly here.  ??
			// Serial.println(i);
			}
		// Motor is now running at assigned speed
	}

	if (ms > 0 && ms < mset) {		// motor is being slowed down.  Most likely because it is approaching targeted height.

		CurrentPWM = CalculatePWM(mset);	// If slowing down, this is the larger PWM number
		TargetPWM = CalculatePWM(ms);

		for (int i = CurrentPWM; i >= TargetPWM; i--) {
			analogWrite(SpeedPin, i);
			delay(RampDelay);
		}			// Motor is now running at reduced speed lower than SetSpeed in computer
	}


	if (ms == 0) {		// motor is being turned off and speed should be ramped down

		CurrentPWM = CalculatePWM(mset);	// Get current speed setting of motor
		for (int i = CurrentPWM; i > 0; i--) {
			analogWrite(SpeedPin, i);
			delay(RampDelay);
		}		// Motor is now stopped

		EEPROM.put(0, ShaftCount);	// Save turning off height in special location 0 in EEPROM
		OldShaftCount = ShaftCount;
	}			
	/*
	Serial.println(F("Set Speed: "));  // Lets take a look
	Serial.println(mset);
	Serial.println(F("Req.Speed: "));  // Lets take a look
	Serial.println(ms);
	Serial.println(F("PWM: ")); 
	Serial.println(mPWM);
	Serial.println(F(" ")); 
*/
}