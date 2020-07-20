/*
    Name:       KD6X Winch Control 1.ino
    Created:	3/1/2019 11:55:12 PM
    Author:     Courtney E. Krehbiel
*/

/*
  Code by Courtney Krehbiel KD6X to run a remote winch and motor by
  using a motor controller with speed set by using PWM and a filtered output.
  Controls are to allow the user to manipulate speed or sweep function.

  Feb. 13, 2019

 */


// Basic includes
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"


// Fonts to use
#include <Fonts/FreeMono9pt7b.h>
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

const int ButtonMap[] = { 4, 5, 6, 7, 11, 12 };  // Array to convert button number to actual pin number

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


	
int SpeedPin = 13;		// Speed PWM filter circuit connected to digital pin 13.  Only works on Mega, not Uno

int mspeed = 0;			// Actual speed for motor
int SetSpeed = 0;		// Set speed in computer.  
int mPWM = 0;

byte VOn48 = LOW;			// Is the 48 volt supply enabled; red if off, green if on
byte Relay1On = LOW;		// Relay 1 on relay shield.  LOW = 48V off and relay inactive.  HIGH = 48V on and relay active.
byte RunActive = LOW;		// Is the winch active; grey if inactive, yellow if active
byte Limit = HIGH;				// Has a limit switch been tripped; green if not tripped (HIGH), red if tripped (LOW)

float RunHeight = 0.00f;		// Running height
float ShaftCount;				// Counter to keep track of shaft rotations
float OldShaftCount;			// Previous value of ShaftCount
float SetHeight = 10.00f;		// Height target that's set by setup
byte Direction = LOW;			// 1 = up, 0 = down
byte LimitDirection;			// Stores the direction of the limit hit.  Going up or down.
byte LimitMode = LOW;			// Limit mode LOW = Read input; mode HIGH = Write Output;
int NeedMotorTurns;				// Counter to count a few turns of the motor to clear limit condition.
byte GlitchReset = LOW;			// Flag to note that height positioning may be reset
byte Step1 = HIGH;				// Flags for each of the limit processing steps
byte Step2 = LOW;
byte Step3 = LOW;

float Speed = 0.00f;			// Feet per minute of system.  TBD depending on gearing

long eeAddress = 0;				// Location where height to be stored in EEPROM
								// 4096 bytes of EEPROM on Mega, and each good for about 100,000 writes.



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
	EEPROM.put(eeAddress, ShaftCount);	// Copy it to the randomly chosen location as starting point

	// Initialize outputs to motor controller
	pinMode(A0, OUTPUT);  // Initialize relay driving pin as output.  Using analog pin as digital output to drive 48V relay
	pinMode(A1, OUTPUT);	//  Initialize relay driving pin as output.  Driving motor control up/down

	// Intialize input from limit switches
	pinMode(8, INPUT);		// Hi-Z input

	// Initialize softbuttons around display
	pinMode(12, INPUT_PULLUP);    // Nominally +Increase sign	Button 5
	pinMode(11, INPUT_PULLUP);    // Nominally -Decrease sign	Button 4
	pinMode(7, INPUT_PULLUP);    // Far right button			Button 3
	pinMode(6, INPUT_PULLUP);    // 2nd from right button		Button 2
	pinMode(5, INPUT_PULLUP);    // 2nd from left button		Button 1
	pinMode(4, INPUT_PULLUP);    // Far left button				Button 0

	tft.setRotation(3);  // Set screen to landscape with connections in upper left

	tft.fillScreen(ILI9341_NAVY);

	DisplayHome();			// Display the home screen
	ActiveScreen[0] = HIGH; // Keep track of active screen
	
	SetMotorSpeed(0,0);		// Get PWM going.  Output should be about 1 volt which is just at motor off threshold. 

	// Set interrupt to handle soft button pushes
	pinMode(ButtonInterrupt1Pin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(ButtonInterrupt1Pin), ButtonPush, RISING);

	// Set interrupt to handle shaft rotation counting
	pinMode(ButtonInterrupt0Pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(ButtonInterrupt0Pin), ShaftRotation, RISING);
}


// Loop that runs continuously awaiting interrupts and other actions
void loop() {

	HomeActions();		// Implement Home screen softbutton actions

	ManualActions();	// Implement Manual screen softbutton actions

	Setup1Actions();	// Implement Setup1 screen softbutton actions

	SetupHeightActions(); // Implement Setup Height screen softbutton actions


	if (Lockout > 0) {
		Lockout = --Lockout;	// Decrement the lockout counter
	}

	delay(30);			// Assume other stuff will take some time, so don't loop constantly

	CheckLimit();		// Check status of limit switch, and act accordingly
	
}

/*
======================================= Interrupt Routines Follow =======================================
*/

void ButtonPush() {		// Process button push interrupt
	byte val;
	int i;

	if (Lockout > 0) {		// Skip interrupt processing while Locked out for debounce
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
}

void ShaftRotation() {	// Increment or decrement a counter each time the motor shaft rotates.

	OldShaftCount = ShaftCount;
	if (Direction) {
		ShaftCount += 1;
	}
	else {
		ShaftCount -= 1; 
	}
	EEPROM.put(eeAddress, ShaftCount);		// Temporarily store ShaftCount in randomly chosen EEPROM address.  Do I need this?  How would I use it?
	DisplayCurrentHeight(ShaftCount);		// Hopefully this doesn't bog down the interrupt handling

	if (NeedMotorTurns) {					// If we're recovering from a limit and just need a few revs, this will be >0  ie: 3
		NeedMotorTurns -= 1;				// Decrement the counter
		
	}

}


/*
======================================= Subroutines Follow =======================================
*/

// Programming actions for Home Screen
byte HomeActions() {
	// Actions for Home Screen Softbuttons

	if ((!ButtonPushed[0]) & ActiveScreen[0]) {		// Button 0 selects Setup1 screen
		ResetActiveScreen();
		ActiveScreen[1] = HIGH;
		DisplaySetup1();
		ResetButtons();
	}

	if ((!ButtonPushed[1]) & ActiveScreen[0]) {		// Button 1
		ResetActiveScreen();
		ActiveScreen[0] = HIGH;
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
		ResetActiveScreen();
		ActiveScreen[0] = HIGH;
		mspeed = 0;		// Set speed to zero if leaving this screen
		RunActive = LOW;
		Direction = LOW;
		SetSpeed = 0;
		DisplayHome();
		ResetButtons();
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
			digitalWrite(A0, Relay1On);
		}
		ResetButtons();
	}

	if ((!ButtonPushed[2]) & ActiveScreen[5]) {		// Flip RunActive status
		if (VOn48) {								// Only become active if 48V is on
			RunActive = !RunActive;
			DrawLights();
			ResetButtons();
			if (RunActive) {
				mspeed = SetSpeed;
				SetMotorSpeed(mspeed, SetSpeed);
			}
			else {
				mspeed = 0;
				SetMotorSpeed(mspeed, SetSpeed);
			}
		}
	}

	if ((!ButtonPushed[3]) & ActiveScreen[5]) {		// Flip Up/Down status
		if (!RunActive) {							// Can't change direction while motor is running
			Direction = !Direction;					// Default direction is LOW which is Down
			digitalWrite(A1, Direction);			// Set direction on motor controller
			tft.fillRect(35, 110, 200, 25, ILI9341_NAVY);  // Erase old direction first.  Note that args are (x, y, len x, len y)
			tft.setCursor(40, 125);
			tft.setTextSize(0);
			tft.setFont(&FreeMono9pt7b);
			tft.setTextColor(ILI9341_WHITE);
			if (!Direction) {
				tft.println(F("Direction:   Down"));
			}
			else {
				tft.println(F("Direction:   Up"));
			}
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
		ResetActiveScreen;	// Setting height goes here
		ActiveScreen[2] = HIGH;
		DisplaySetupHeight();
		ResetButtons();
	}

	if ((!ButtonPushed[2]) & ActiveScreen[1]) {		// Setup Max Speed screen
		ResetActiveScreen;	// Setting max speed goes here
		ActiveScreen[3] = HIGH;
		DisplaySetupSpeed();		// Setup max speed goes here
		ResetButtons();
	}
}

// Programming actions for SetupHeight Screen
byte SetupHeightActions() {
	// Actions for Setup1 Screen Softbuttons

	if ((!ButtonPushed[0]) & ActiveScreen[2]) {		// Switch to Home Screen
		GlitchReset = LOW;		// If any button other than 3 is pressed, be sure to anul the glitch reset flag
		ResetActiveScreen();
		ActiveScreen[0] = HIGH;
		DisplayHome();
		ResetButtons();
	}

	if ((!ButtonPushed[1]) & ActiveScreen[2]) {		// Back up to previous screen
		// Code to backup to previous screen
		GlitchReset = LOW;
		ResetButtons();
	}

	if ((!ButtonPushed[2]) & ActiveScreen[2]) {		// Go to RUN screen
		// Code to switch to RUN screen 
		GlitchReset = LOW;
		ResetButtons();
	}

	if ((!ButtonPushed[3]) & ActiveScreen[2]) {		// Reset ShaftCount -- REQUIRES 2 SUCCESSIVE PUSHES OF BUTTON
	// Code to reset ShaftCount to 0 in case of glitch
		if (!GlitchReset) {		// LOW state meaning this is first push
			GlitchReset = HIGH;
			tft.setTextColor(ILI9341_WHITE);  // Print a note re second push for Current Height zero reset.
			tft.setFont(&FreeMono9pt7b);
			tft.setTextSize(0);
			tft.setCursor(10, 185);
			tft.println(F("Press ResCHt again"));
		}
		else {		// This is second push, so reset ShaftCount to 0;
			GlitchReset = LOW;		// Reset the indicator byte
			ShaftCount = 0.00;
		}
		ResetButtons();
	}

	if ((!ButtonPushed[5]) & ActiveScreen[2]) {		// Increase height setting
		GlitchReset = LOW;
		if (SetHeight <= 26)	{
			SetHeight += 1;
		}
		DisplaySetHeight(SetHeight);
		ResetButtons();
	}

	if ((!ButtonPushed[4]) & ActiveScreen[2]) {		// Decrease height setting
		GlitchReset = LOW;
		if (SetHeight >= 11)	{
			SetHeight -= 1;
		}
		
		DisplaySetHeight(SetHeight);
		ResetButtons();
	}
}


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

	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else
	tft.setCursor(10, 15);
	tft.println(F("Homes"));

	tft.setCursor(Indent2, StatusTop+25);		// Print home status elements
	sprintfloat("Current Ht: ", ShaftCount);		// Call special function that will parse float and print to screen

	tft.setCursor(Indent2, StatusTop+50);
	sprintfloat("Set Height: ", SetHeight);
	
	tft.setCursor(Indent2, StatusTop+75);
	if (!Direction) {
		tft.println(F("Direction:   Down"));
	}
	else {
		tft.println(F("Direction:   Up"));
	}
	
	tft.setCursor(Indent2, StatusTop+100);
	SprintAnInt("Speed 0-100:", Speed);		// Routine will do a conversion of speed to int

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

	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else
	tft.setCursor(10, 15);
	tft.println(F("Manual"));

	tft.setCursor(Indent2, StatusTop + 25);		// Print home status elements
	sprintfloat("Current Ht: ", ShaftCount);		// Call special function that will parse float and print to screen

	tft.setCursor(Indent2, StatusTop + 50);
	if (!Direction) {
		tft.println(F("Direction:   Down"));
	}
	else {
		tft.println(F("Direction:   Up"));
	}

	tft.setCursor(Indent2, StatusTop + 75);
	SprintAnInt("Speed 0-100:", Speed);

	tft.setTextColor(ILI9341_YELLOW);  // set color

	tft.setCursor(16, BotLine);		// Now display softbutton labels at bottom
	tft.println(F("HOME"));
	tft.setCursor(100, BotLine);
	tft.println(F("48V"));
	tft.setCursor(165, BotLine);
	tft.println(F("RUN/ST"));
	tft.setCursor(248, BotLine);
	tft.println(F("UP/DN"));

	tft.setCursor(265, 125);	// Display softbutton side labels
	tft.println(F("SPEED"));
	tft.setTextSize(2);		// Switch to larger font size for + and -
	tft.setCursor(282, 88);	
	tft.println(F("+"));
	tft.setCursor(282, 168);
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

	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else
	tft.setCursor(10, 15);
	tft.println(F("Setup"));

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
		

	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else
	tft.setTextSize(0);
	tft.setCursor(10, 15);
	tft.println(F("Setup Height"));

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

	tft.setCursor(252, 125);	// Display softbutton side labels
	tft.println(F("HEIGHT"));
	tft.setTextSize(2);		// Switch to larger font size for + and -
	tft.setCursor(282, 88);
	tft.println(F("+"));
	tft.setCursor(282, 168);
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

	tft.setCursor(70, 165);		// Write Current Height Setting on screen while in big font
	tft.setTextSize(2);
	SprintAnInt("", SetSpeed);


	tft.setFont(&FreeMono9pt7b);	// Switch to smaller font for all else
	tft.setTextSize(0);
	tft.setCursor(10, 15);
	tft.println(F("Setup Speed"));

	tft.setCursor(18, StatusTop + 45);
	tft.println(F("(0 to 100%)"));

	tft.setTextColor(ILI9341_YELLOW);  // set color

	tft.setCursor(16, BotLine);		// Now display softbutton labels at bottom
	tft.println(F("HOME"));
	tft.setCursor(96, BotLine);
	tft.println(F("BACK"));
	tft.setCursor(179, BotLine);
	tft.println(F("RUN"));

	tft.setCursor(252, 125);	// Display softbutton side labels
	tft.println(F("HEIGHT"));
	tft.setTextSize(2);		// Switch to larger font size for + and -
	tft.setCursor(282, 88);
	tft.println(F("+"));
	tft.setCursor(282, 168);
	tft.println(F("-"));
}

// Draw display soft buttons (SB) on the screen
byte DrawButtons() {
	int SBStartX[] = { 14, 93, 172, 251 };
	int SBStartY[] = { 55, 134 };
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

// Check Limit switch and take action as appropriate
void CheckLimit() {		// Three Steps which are mutually exclusive and usually only need to be run once

	// STEP1
	if (RunActive & (!digitalRead(LimitInput)) & Step1) {	// Only check for limits if actively running.  If D8 is low, a limit switch has been opened
		Limit = LOW;		// Change Limit state to fault/red
		DrawLights();			// Change display
		LimitDirection = Direction;		// Capture which direction the mast was moving
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

	sprintf(PrnString, "%s %d.%d", StringtoPrint, intWhole, intFract);  // Print the whole mess to string PrnString
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
	tft.fillRect(65, 130, 200, 40, ILI9341_NAVY);  // Erase old speed first.  Note that args are (x, y, len x, len y)
	tft.setTextColor(ILI9341_WHITE);
	tft.setFont(&FreeMonoBold12pt7b);
	tft.setTextSize(2);
	tft.setCursor(70, 165);
	sprintfloat("", sheight);
}

// Display Current Height -- Raw count for now
byte DisplayCurrentHeight(float CurHt) {

	tft.fillRect(155, 85, 200, 25, ILI9341_NAVY);  // Erase old count first.  Note that args are (x, y, len x, len y)
	tft.setFont(&FreeMono9pt7b);
	tft.setTextSize(0);
	tft.setCursor(162, 100);
	tft.setTextColor(ILI9341_WHITE);
	tft.setCursor(162, 100);
	sprintfloat("", CurHt);
}

// Set the speed control PWM output accordingly
byte SetMotorSpeed(int ms, int mset) {		// Input for ms and mset is 0 to 100 (percent).  ms is the speed to attain, mset is the speed setting in the computer
	
	// Now to get 256 count equivalent for PWM control of motor.
	// Formula is: mPWM = (1.30 * ms) + 60;
	/* This would be 2.55 to make it 100% on, but the motor control circuit doesn't seem to work above 4.0 volts on the input.
	   Therefore, we're scaling back to about 80% which should yield just short of 4 volts.

	   Actual effective control range on the motor is from about 1.2volts to 3.95 volts.
	*/

	//	Serial.print(F("Pulse Width Mod = "));  // Just looking at output for now
	//	Serial.println(mPWM);

	// Motor controller speed is about 2X faster going up than down due to hardware design.  So let's divide speed going up by 2 to make things even.
	if (Direction) {		// Up is high, down is low
		mset = mset / 2;
		ms = ms / 2;
	//	Serial.println(mset);
	}

		// Write the PWM speed to the pin with a startup or shutdown ramp
	if (ms == mset) {		// motor is being turned on and speed should be ramped up
		for (int i = 0; i <= mset; i++) {
			mPWM = (1.30 * i) + 60;
			analogWrite(SpeedPin, mPWM);
			if (Direction) {
				delay(60);		// Have to slow down ramp because it's been divided by 2 for up direction
			}
			else {
				delay(30);
			}
		}
		EEPROM.put(0, ShaftCount);	// Save turning off height in special location 0 in EEPROM
		OldShaftCount = ShaftCount;
	}

	if (ms == 0) {		// motor is being turned off and speed should be ramped down
		for (int i = mset; i > 0; i--) {
			mPWM = (1.30 * i) + 60;
			analogWrite(SpeedPin, mPWM);
			if (Direction) {
				delay(60);		// Have to slow down ramp because it's been divided by 2 for up direction
			}
			else {
				delay(30);
			}
		}
		EEPROM.put(0, ShaftCount);	// Save turning off height in special location 0 in EEPROM
		OldShaftCount = ShaftCount;
	}

}