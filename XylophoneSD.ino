// ***************************************************************************************
// XylophoneSD.ino - Program for version 2.1 of the Build Your Music Robotic Xylophone
//    The Arduino Uno is connected to a 20x4 I2C Character LCD, KY-040 Rotary Encoder,
//    and data logger shield with SD card.  MIDI files are loaded onto the SD Card, and
//    the Arduino reads the list of MIDI files, and displays them on the LCD.  The user
//    turns the rotary encoder to select a song from the list, and clicks the button to
//    start playing a song.  This program is loaded into the "Arduino Master" which is
//    connected via Serial to muiltiple "Arudino Slaves" with each Slave controlling up
//    to 8 solenoids.
//
//    Author:          John Miller
//    Revision:        2.1.0
//    Date:            10/8/2018
//    Project Source:  https://github.com/jpsrmiller/build-your-music
//    Project Website: https://buildmusic.net 
// 
// The program may be freely copied, modified, and used for any Arduino projects
//
// --------------------------------------------------------------------------------------
// |                                    User Interface                                  |
// --------------------------------------------------------------------------------------
// The program uses a KY-040 Rotary Encoder and I2C Character LCD as the User Interface.
// See https://buildmusic.net/tutorials/user-interface for details
//
// The KY-040 Rotary Encoder is connected to the following Arduino Uno pins
//      o CLK --> Pin 2
//      o DT  --> Pin 3
//      o SW  --> Pin 4
//      o +   --> Pin 5
//      o GND --> Pin 6
// 
// The LCD is connected to the following Arduino Uno pins
//      o GND --> GND
//      o VCC --> 5V
//      o SDA --> Pin A4 (SDA)
//      o SCL --> Pin A5 (SCL)
//
// The Rotary Encoder uses Arduino pins for the GND and +5V so that the remaining
//   5V and GND pins on the Arduino Uno can be used for other peripheral devices
// This works because the the Rotary Encoder draws less than the 40 mA
//   maximum current allowed on the Arduino Uno I/O pins  
//
// --------------------------------------------------------------------------------------
// |                 Master/Slave Communications and Xylophone Notes                    |
// --------------------------------------------------------------------------------------
// This is the program for the "Master Arduino" which controls multiple "Slave Arduinos"
// via Serial.  Each "Slave Arduino" has an L293D Motor Driver Shield which can drive
// up to 8 solenoids.  Each solenoid drives a separate mallet and plays one note on the
// xylophone.  TX (Pin 1) on the Master is connected to RX (Pin 0) on the first slave.
// See https://buildmusic.net/tutorials/motor-driver/ for details.
//
// Each time a note is to be played, a serial message is transmitted in the form:
//
//               <aabb>
//
//  where aa and bb are bytes in Hexadecimal format.
//         aa is the address of the slave
//         bb is a bitmask corresponding to the channel(s) to be energized
//
// For example, to energize channels 2, 4, and 7 on Slave 1, the serial message is:
//
//               <0194>   (94 in Hex = 10010100 in Binary)
//
// The relationship between slave address / channel number and the actual note on the
// xylophone (e.g. C4, C#4, D4, ...) is defined in the constants BYM_NOTE_00 through
// BYM_NOTE_nn, where BYM_NOTE_00 is the lowest xylophone note, BYM_NOTE_01 is the
// second lowest note, and so forth.  Each BYM_NOTE_xx is a byte in hexadecimal format
// where the top nibble is the slave address and the bottom nibble is the channel
// number (0 through 7).  The constants BYM_NOTE_LOWEST and BYM_NOTE_COUNT define
// the lowest xylophone note (as a MIDI note number), and the number of notes.
//
// For example: On a G-to-G 2-octave chromatic xylophone, we would have:
//     BYM_NOTE_LOWEST = 55   (55 is the MIDI note value for G3)
//     BYM_NOTE_COUNT = 25    (a 2-octave xylophone has 25 notes)
// In this case, the highest note would be G5 (MIDI note number 79)
//
// Then the note mapping might be defined similar to:
//   BYM_NOTE_00 = 0x10   (G3 is Channel 0 on Slave 1)
//   BYM_NOTE_01 = 0x36   (G#3 is Channel 6 on Slave 3)
//   BYM_NOTE_02 = 0x11   (A3 is Channel 1 on Slave 1)
//   BYM_NOTE_03 = 0x37   (A#3 is Channel 7 on Slave 3)
//   and so forth, for all 25 notes.
// The actual note mapping will be dependent upon the physical wiring between the
// solenoids and the motor driver shields.
//
// See https://buildmusic.net/software/master-program for more details.
//
// --------------------------------------------------------------------------------------
// |                   Data Logger Shield, SD Card, and MIDI Files                      |
// --------------------------------------------------------------------------------------
// A Data Logger Shield with an SD Card is mounted on top of the Arduino Uno.  MIDI
//      files are copied onto the SD Card from a computer.  Only 8-character file names
//      are supported. 
//
// The program reads the list of MIDI files from the SD card and displays on the LCD.  
//
// MIDI files may be copied to either the Root directory or a sub-directory. The active
//      directory may be selected at runtime via the configuration menu.
//
// Optionally, an INI file may be added into the same directory as the MIDI file.  The
//      INI file needs to have the same base file name as the MIDI file, but with a
//      different extension.  For example, FURELISE.MID would have an INI file FURELISE.INI
// 
// The structure of the INI file is as follows:
//
//  Line 1 : Song Title (required)
//  Line 2 : Additional Song Info 1 (optional)
//  Line 3 : Additional Song Info 2 (optional)
//  Line 4 : Song Length in Seconds (optional)
//  Line 5 : Transpose in # of Half Steps (optional)
//  Line 6 : Target High Note - Attempt to match the highest note on the
//           xylophone to this note, during MIDI playback (optional)
//
//  If INI file does not exist, the program uses the MIDI file name as the dispaly name
//
// Optionally, Playlist files can also be saved onto the SD Card.  Playlist files have
//    have a .TXT extention.  Each Playlist file contains a list of MIDI files to
//    show together as a "Playlist".  The active playlist is selected at runtime via
//    the configuration menu.
//
// See https://buildmusic.net/software/sd-card-config for more details.
// 
// --------------------------------------------------------------------------------------
// |                           External Libraries Required                              |
// --------------------------------------------------------------------------------------
// The following libraries are not included with the standard Arduino IDE build and must
// be downloaded and installed for the program to compile.
//    o OneButton - Used to detect button click and long press
//          Source: https://github.com/mathertel/OneButton
//    o LiquidCrystal_I2C - Needed for the I2C Character LCD
//          Source: https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
//    o SdFat - SD Card File System functions.  Needed by MD_MIDIFile
//          Source: https://github.com/greiman/SdFat
//    o MD_MIDIFile - Standard Midi File processing library.  
//          Source: https://github.com/MajicDesigns/MD_MIDIFile
//          Note: A few small changes need to be made to this library from the version 
//             on GitHub.  See https://buildmusic.net/software
//            
// ***********************************************************************************



#include <Wire.h>
#include <OneButton.h>
#include <LiquidCrystal_I2C.h>
#include <SdFat.h>
#include <MD_MIDIFile.h>
#include <eeprom.h>
#include <avr/pgmspace.h>

// Most I2C LCD's have an I2C Address of either 0x27 or 0x3F
// If the LCD doesn't work with one address, try the other
#define BYM_LCD_I2C_ADDRESS 0x27
//#define BYM_LCD_I2C_ADDRESS     0x3F

// Number of Rows and Columns in the LCD.  Most LCD's are 16x2 or 20x4
#define BYM_LCD_ROW_COUNT       4
#define BYM_LCD_COL_COUNT       20

// Define the IO Pins Used for the Rotary Encoder
#define BYM_PIN_ROTARY_CLK        2	  // Used for generating interrupts using CLK signal
#define BYM_PIN_ROTARY_DAT        3	  // Used for reading DT signal
#define BYM_PIN_ROTARY_SW         4	  // Used for the Rotary push button switch
#define BYM_PIN_ROTARY_5V         5   // Set to HIGH to be the 5V pin for the Rotary Encoder
#define BYM_PIN_ROTARY_GND        6   // Set to LOW to be the GND pin for the Rotary Encoder

// Define the Serial Baud Rate.  This must match the baud rate in the Slave Arduinos
#define BYM_SERIAL_BAUD_RATE		57600

// Lowest note and number of notes in the xylophone.  See MIDI specification for details.
// Common Lowest Note values are 55 (G3), 57 (A3), and 60 (C4)
// A 2-octave chromatic xylophone has 25 notes.
#define BYM_NOTE_LOWEST           55
#define BYM_NOTE_COUNT            25
#define BYM_NOTE_HIGHEST		  (BYM_NOTE_LOWEST + BYM_NOTE_COUNT - 1)

// Assign the mapping of notes to Motor Board and Output
// Top Nibble is the Motor Board number (0, 1, 2, or 3)
// Bottom Nibble is the Pin number (0 to 7) where 0=M1A, 1=M1B, 2=M2A, 3=M2B, ...
#define BYM_NOTE_00               0x10  // G3
#define BYM_NOTE_01               0x36  // G#3
#define BYM_NOTE_02               0x11  // A3
#define BYM_NOTE_03               0x37  // A#3
#define BYM_NOTE_04               0x12  // B3
#define BYM_NOTE_05               0x13  // C4
#define BYM_NOTE_06               0x34  // C#4
#define BYM_NOTE_07               0x15  // D4
#define BYM_NOTE_08               0x32  // D#4
#define BYM_NOTE_09               0x14  // E4
#define BYM_NOTE_10               0x17  // F4
#define BYM_NOTE_11               0x30  // F#4
#define BYM_NOTE_12               0x16  // G4
#define BYM_NOTE_13               0x24  // G#4
#define BYM_NOTE_14               0x00  // A4
#define BYM_NOTE_15               0x25  // A#4
#define BYM_NOTE_16               0x01  // B4
#define BYM_NOTE_17               0x02  // C5
#define BYM_NOTE_18               0x22  // C#5
#define BYM_NOTE_19               0x05  // D5
#define BYM_NOTE_20               0x21  // D#5
#define BYM_NOTE_21               0x04  // E5
#define BYM_NOTE_22               0x07  // F5
#define BYM_NOTE_23               0x20  // F#5
#define BYM_NOTE_24               0x06  // G5
#define BYM_NOTE_25               0x00
#define BYM_NOTE_26               0x00
#define BYM_NOTE_27               0x00
#define BYM_NOTE_28               0x00
#define BYM_NOTE_29               0x00
#define BYM_NOTE_30               0x00
#define BYM_NOTE_31               0x00

// Define Strings used to Show the Menu.
// Store the Strings in PROGMEM to save on RAM 
const char lcdString01_Back[] PROGMEM = "Back";
const char lcdString02_SolenoidNoteTest[] PROGMEM = "Solenoid/Note Test";
const char lcdString03_ChromaticScale[] PROGMEM = "Chromatic Scale";
const char lcdString04_AboutLine1[] PROGMEM = "Build Your Music";
const char lcdString05_AboutLine2[] PROGMEM = "by John Miller";
const char lcdString06_AboutLine3[] PROGMEM = "www.buildmusic.net";
const char lcdString07_AboutLine4[] PROGMEM = "Revision:  2.1.0";
const char lcdString08_AboutLine5[] PROGMEM = "Date: 10/8/2018";
const char lcdString11_Single[] PROGMEM = "Single";
const char lcdString12_Sequential[] PROGMEM = "Sequential";
const char lcdString13_Shuffle[] PROGMEM = "Shuffle";
const char lcdString21_Folder[] PROGMEM = "Folder: ";
const char lcdString22_Playlist[] PROGMEM = "Playlist: ";
const char lcdString23_Mode[] PROGMEM = "Mode: ";
const char lcdString24_HardwareTest[] PROGMEM = "Hardware Test";
const char lcdString25_About[] PROGMEM = "About";
const char lcdString26_Restart[] PROGMEM = "Restart";
const char lcdString31_FolderSelect[] PROGMEM = "Folder Select";
const char lcdString32_PlayMode[] PROGMEM = "Play Mode";
const char lcdString33_PlaylistSelect[] PROGMEM = "Playlist Select";
const char lcdString34_None[] PROGMEM = "[none]";

// SD chip select pin for SPI comms.
// Arduino Ethernet shield, pin 4.
// Default SD chip select is the SPI SS pin (10).
// Other hardware will be different as documented for that hardware.
#define  SD_SELECT  10

// Time to show the Splash Screen in milliseconds
#define SPLASH_SCREEN_DELAY_MS		5000	

// Maximum Idle Time before going to Song Select Menu
#define MENU_MAX_IDLE_TIME			15 

// Maximum silent time in Midi file before ending song.  This is needed
// for cases where the End of Track event is significantly later than
// the last note in the MIDI file.
#define SONG_PLAY_MAX_IDLE_TIME		5   

// Set to "1" to output the names of the notes (e.g. C4, C#4, D4, ...) via
// serial, instead of the command to the Slave Arduino with motor driver.
// This can be used with a PC to test that songs are playing correctly
// without needing the whole xylophone.
// Typically, this should be set to "0"
#define OUTPUT_NOTE_NAMES	0

// Define the lengths of strings for storing the selected directory
// and MIDI file.
#define FULL_PATH_FILE_LENGTH		24
#define DIRECTORY_LENGTH			16

// Define EEPROM addresses for saving user selections.
#define PLAY_MODE_ADDRESS			0
#define DIRECTORY_ADDRESS			16
#define PLAYLIST_ADDRESS			32
#define SELECTED_SONGS_ADDRESS		256

// Define the types of Play Modes
#define PLAY_MODE_SINGLE			0
#define PLAY_MODE_SEQUENTIAL		1
#define PLAY_MODE_SHUFFLE			2

#define SPACE 32

// Used to check for elapsed time during MIDI play
unsigned long lastMidiNoteTime;
unsigned long midiPlayStartTime;

// Use the same variable for LastNavigateTime and LastMidiNoteTime in order to save RAM.
// These two functions will never be called at the same time.
#define menuLastNavigateTime lastMidiNoteTime

// Use the same char array when searching for a directory of a given name.  Saves RAM.
#define subDirectoryName midiFileFullPath
#define subDirectory midiFile

// Use the same File object to save on RAM.
#define playlistFile midiDirectory

// Stores the name of a note (e.g. "C4", "C#4", "D4", ...)
uint8_t noteName[3];

// File, MIDI File, and SD card objects
File midiDirectory;
File midiFile;
SdFat	SD;
MD_MIDIFile SMF;
File textFile;

//# of Half-steps to transpose the MIDI file prior to playing.  Defined in INI file
int8_t songTranspose; 
//# Target highest note for MIDI File.  Defined in INI file
uint8_t songTargetHighNote;

uint8_t playMode;              // Current Play Mode (Single, Sequence, Shuffle)
uint8_t selectedSongCount;     // # of songs to play in sequence
uint8_t selectedSongIndex;     // index of current song in sequence
uint8_t selectedSongFileIndex; // File index of current song in sequence
uint8_t currentSong;           // Index of song currently selected in LCD
uint8_t songCount;             // # of songs shown on LCD list
uint8_t lcdSongPlayUpdating;   // Flag set while writing to LCD during song play
uint16_t oldElapsedPlayTime;   // Determined if elapsed play time needs updating
uint16_t elapsedPlayTime;      // Time in seconds since start of song play
uint16_t songLength;           // Total length in seconds of current song
uint8_t songPlayTimeLcdRow;    // Row in the LCD for showing elapsed play time
 
uint8_t menuCurrent;           // Current menu shown on LCD
uint8_t menuChanged;           // True if menu has changed (refreshes menu)
uint8_t menuLength;            // Number of items in current menu
uint8_t menuSelectedIndex;     // Currently selected menu item
uint8_t menuLcdFirstRow;       // Item in menu currently shown on first LCD row
uint8_t menuHasTitle;          // True if first LCD row is to be a menu title
uint8_t menuNoSelection;       // True if selection option not available on menu
uint8_t lcdMenuLineCharCount;  // Counts # of Characters written to LCD Row
uint8_t playlistFound;         // True if a valid playlist file has been set

char iniFileFullPath[FULL_PATH_FILE_LENGTH];       // Full path of current INI File
char currentDirectory[DIRECTORY_LENGTH];           // Currently-selected directory
char midiFileFullPath[FULL_PATH_FILE_LENGTH];      // Full path of current MIDI file
char playlistFileFullPath[FULL_PATH_FILE_LENGTH];  // Full path of current playlist file
char rootDirectory[] = "/";                        // Root directory on SD Card

// Use these defines to use some of the above char arrays for other purposes
// Gives more clarity to code and saves on RAM.
#define playlistFileName &(playlistFileFullPath[strlen(currentDirectory)])
#define midiFileName &(midiFileFullPath[strlen(currentDirectory)])
#define MIDI_FILENAME_BUFFER_SIZE FULL_PATH_FILE_LENGTH - strlen(currentDirectory)

// Define the extentions of MIDI file and Playlist file
char midiFileExt[] = ".mid";
char playlistFileExt[] = ".txt";
#define FILE_EXT_STRING_LENGTH	4

uint8_t sdInitFail;   // True if failue in SD Card initialization
uint8_t noMidiFiles;  // True if no MIDI files found in directory
uint8_t stopMidi;     // Used to Stop the MIDI play

// Reset function.  Configuration menu allows option to do a software reset.
void(*resetFunc) (void) = 0;//declare reset function at address 0

// Defines the different menus or other display screens shown on LCD.
#define LM_MENU_SONG_SELECT				0x10
#define LM_MENU_SONG_PLAYING			0x11
#define LM_MENU_SETUP					0x20
#define LM_MENU_DIRECTORY_SELECT		0x21
#define LM_MENU_PLAYLIST_SELECT			0x22
#define LM_MENU_PLAYMODE_SELECT			0x23
#define LM_MENU_ABOUT					0x24
#define LM_MENU_HARDWARE_TEST			0x30
#define LM_MENU_NOTE_TEST				0x31

// OneButton class handles Debounce and detects button press
OneButton btnRot(BYM_PIN_ROTARY_SW, HIGH);		  // Rotary Select button

// Intitialize I2C LCD class with # of Rows and # of Columns in LCD.  (Typically 20x4)
LiquidCrystal_I2C lcd(BYM_LCD_I2C_ADDRESS, BYM_LCD_COL_COUNT, BYM_LCD_ROW_COUNT);


volatile byte aFlag = 0; // lets us know when we're expecting a rising edge on pinA 
						 // to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // lets us know when we're expecting a rising edge on pinB 
						 // to signal that the encoder has arrived at a detent 
						 // (opposite direction to when aFlag is set)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt 
						   // pins before checking to see if we have moved a whole detent
						   
uint8_t rotaryDisabled;    // Disables the Rotary Encoder interrupts while the LCD is being updated
uint8_t rotaryTurned;      // True if Rotary Encoder has be turned at least once
uint8_t rotaryDetentCount; // Counts # of Rotary Detents since last LCD update


// ****************************************************************************
// initializeRotaryEncoder() - Initialize the pins and interrupt functions
//                             for the Rotary Encoder
// ****************************************************************************
void initializeRotaryEncoder()
{
	// Set the Directions of the I/O Pins
	pinMode(BYM_PIN_ROTARY_CLK, INPUT_PULLUP);
	pinMode(BYM_PIN_ROTARY_DAT, INPUT_PULLUP);
	pinMode(BYM_PIN_ROTARY_SW, INPUT_PULLUP);
	pinMode(BYM_PIN_ROTARY_GND, OUTPUT);
	pinMode(BYM_PIN_ROTARY_5V, OUTPUT);

	// Set the 5V and GND pins for the Rotary Encoder
	digitalWrite(BYM_PIN_ROTARY_GND, LOW);
	digitalWrite(BYM_PIN_ROTARY_5V, HIGH);

	// set an interrupt on PinA and PinB, looking for a rising edge signal and 
	// executing the "PinA" and "PinB" Interrupt Service Routines
	attachInterrupt(0, PinA, RISING);
	attachInterrupt(1, PinB, RISING);

	// Define the functions for Rotary Encoder Click and Long Press
	btnRot.attachClick(&rotaryClick);
	btnRot.attachLongPressStart(&rotaryLongPress);
	btnRot.setPressTicks(2000);

	rotaryDisabled = 0;
	rotaryTurned = 0;
	rotaryDetentCount = 0;
}

 // ****************************************************************************
// PinA() - Called by the Interrupt pin when the Rotary Encoder Turned
//    Routine taken from:  
//    https://exploreembedded.com/wiki/Interactive_Menus_for_your_project_with_a_Display_and_an_Encoder
// ****************************************************************************
void PinA() {

	if (rotaryDisabled) return;
	rotaryTurned=1;
	
	cli(); //stop interrupts happening before we read pin values
		   // read all eight pin values then strip away all but pinA and pinB's values
	reading = PIND & 0xC;

	//check that both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
	if (reading == B00001100 && aFlag) {
		rotaryUp();
		rotaryDetentCount++;
		bFlag = 0; //reset flags for the next turn
		aFlag = 0; //reset flags for the next turn
	}
	//signal that we're expecting pinB to signal the transition to detent from free rotation
	else if (reading == B00000100) bFlag = 1;
	sei(); //restart interrupts
}

// ****************************************************************************
// PinB() - Called by the Interrupt pin when the Rotary Encoder Turned
//    Routine taken from:  
//    https://exploreembedded.com/wiki/Interactive_Menus_for_your_project_with_a_Display_and_an_Encoder
// ****************************************************************************
void PinB() {

	if (rotaryDisabled) return;
	rotaryTurned=1;

	cli(); //stop interrupts happening before we read pin values
		   //read all eight pin values then strip away all but pinA and pinB's values
	reading = PIND & 0xC;
	//check that both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge 
	if (reading == B00001100 && bFlag) {
		rotaryDown();
		rotaryDetentCount++;
		bFlag = 0; //reset flags for the next turn
		aFlag = 0; //reset flags for the next turn
	}
	//signal that we're expecting pinA to signal the transition to detent from free rotation
	else if (reading == B00001000) aFlag = 1;
	sei(); //restart interrupts
}



// ****************************************************************************
// rotaryUp() - Rotary Encoder is turned 1 detent to the Right (clockwise)
// **************************************************************************** 
void rotaryUp()
{	
	if (menuSelectedIndex > 0) {
		menuSelectedIndex--;
		menuLastNavigateTime = millis();
		menuChanged = 1;
		if (menuSelectedIndex < menuLcdFirstRow) menuLcdFirstRow = menuSelectedIndex;
	}	
}

// **********************************************************************************
// rotaryDown() - Rotary Encoder is turned 1 detent to the Left (counter-clockwise)
// **********************************************************************************
void rotaryDown()
{
	if (menuSelectedIndex < menuLength - 1) {
		menuSelectedIndex++;
		menuChanged = 1;
		menuLastNavigateTime = millis();
		if (menuSelectedIndex > menuLcdFirstRow + BYM_LCD_ROW_COUNT - 1 - menuHasTitle) 
			menuLcdFirstRow = menuSelectedIndex - (BYM_LCD_ROW_COUNT - 1 - menuHasTitle);
	}
}

// ****************************************************************************
// rotaryClick() - Rotary Encoder Select Switch is pressed
// ****************************************************************************
void rotaryClick()
{
	menuLastNavigateTime = millis();

	switch (menuCurrent)
	{
	case LM_MENU_SETUP: rotaryClickSetup(); break;
	case LM_MENU_HARDWARE_TEST:	rotaryClickHardwareTest(); break;
	case LM_MENU_NOTE_TEST:	rotaryClickNoteTest(); break;
	case LM_MENU_DIRECTORY_SELECT: rotaryClickDirectorySelect(); break;
	case LM_MENU_ABOUT:	rotaryClickAbout();	break;
	case LM_MENU_PLAYMODE_SELECT: rotaryClickPlayModeSelect(); break;
	case LM_MENU_SONG_SELECT: rotaryClickSongSelect(); break; 
	case LM_MENU_SONG_PLAYING: rotaryClickSongPlaying(); break;
	case LM_MENU_PLAYLIST_SELECT: rotaryClickPlaylistSelect(); break;
	}
}

// ****************************************************************************
// rotaryClickSetup() - Rotary Encoder clicked from the Setup menu
// ****************************************************************************
void rotaryClickSetup()
{
	// If "Back" clicked, return to the previous menu.
	if (rotaryClickBack(LM_MENU_SONG_SELECT, currentSong)) return;

	switch (menuSelectedIndex)
	{
	case 0: newMenu(LM_MENU_DIRECTORY_SELECT, findDirectorySelectedIndex()); break; // Directory Select Menu
	case 1: newMenu(LM_MENU_PLAYLIST_SELECT, findPlaylistSelectedIndex()); break; // Playlist Menu
	case 2: newMenu(LM_MENU_PLAYMODE_SELECT, playMode); break; // Play Mode select Menu
	case 3: newMenu(LM_MENU_HARDWARE_TEST); break; // Hardware Test Menu
	case 4:	newMenu(LM_MENU_ABOUT); break;  // About Menu
	case 5:	resetFunc(); break; // Reset the program
	}
}

// ****************************************************************************
// rotaryClickHardwareTest() - Rotary Encoder clicked from the Hardware Test menu
// ****************************************************************************
void rotaryClickHardwareTest()
{
	// If "Back" clicked, return to the previous menu.
	if (rotaryClickBack(LM_MENU_SETUP, 3)) return;

	switch (menuSelectedIndex)
	{
	case 0:  // Solenoid/Note Test
		updateLcdNoteTest();
		newMenu(LM_MENU_NOTE_TEST);
		break;
	case 1: // Chromatic Scale
		playChromaticScale();
		break;
	}
}

// ****************************************************************************
// rotaryClickNoteTest() - Rotary Encoder clicked while in Solenoid/Note Test
// ****************************************************************************
void rotaryClickNoteTest()
{
	if (menuSelectedIndex == BYM_NOTE_COUNT)
		// "Back" clicked.  Return to previous menu.
		newMenu(LM_MENU_HARDWARE_TEST);
	else
		// Test the selected note.
		playNoteTest();
}

// ****************************************************************************
// rotaryClickDirectorySelect() - Rotary Encoder button clicked from the
//    Directory Select button.  Save the new directory and return to
//    the previous menu.
// ****************************************************************************
void rotaryClickDirectorySelect()
{
	directorySelected();
	newMenu(LM_MENU_SETUP, 0);
}

// ****************************************************************************
// directorySelected() - Called after selecting a new directory
// ****************************************************************************
void directorySelected()
{
	saveCurrentDirectory(); // Save the new directory
	validatePlaylist();     // Check if selected playlist file still valid
}

// ****************************************************************************
// rotaryClickPlayModeSelect() - Rotary Encoder clicked from the About menu
// ****************************************************************************
void rotaryClickAbout()
{
	// Only the "Back" selection does anything.  Return to previous menu.
	rotaryClickBack(LM_MENU_SETUP, 4);
}

// ****************************************************************************
// rotaryClickPlayModeSelect() - Rotary Encoder clicked from the Play Mode menu
// ****************************************************************************
void rotaryClickPlayModeSelect()
{
	// If "Back" was clicked, return to previous menu
	if (rotaryClickBack(LM_MENU_SETUP, 2)) return;
	
	// Save the selected play mode
	playMode = menuSelectedIndex;
	savePlayMode();

	// Return to the Setup menu with a default selection of "Play Mode"
	newMenu(LM_MENU_SETUP, 2);
}

// ****************************************************************************
// rotaryClickSongSelect() - Rotary Encoder clicked from the Song Select menu 
// ****************************************************************************
void rotaryClickSongSelect()
{
	// Initialize selected song(s) depending upon play mode
	songPlayStart();
	
	// Start MIDI Play for the current song.
	if (startMidiPlay())
	{
		newMenu(LM_MENU_SONG_PLAYING);
	}
	else
	{
		// If there is an error when starting MIDI play then refresh the
		// current menu.
		menuChanged = 1;
	}
}

// ****************************************************************************
// rotaryClickPlaylistSelect() - Rotary Encoder was clicked from the 
//     Playlist Select menu.
// ****************************************************************************
void rotaryClickPlaylistSelect()
{
	playlistSelected();
	// Return to Setup menu, with a default selection of "Select Playlist"
	newMenu(LM_MENU_SETUP, 1);
}

// ****************************************************************************
// playlistSelected() - Called after a new playlist is selected.
// ****************************************************************************
void playlistSelected()
{
	// menuSelectedIndex=0 means the [none] playlist was selected
	if (menuSelectedIndex == 0) playlistFound = 0; else playlistFound = 1;
	savePlaylist();
}

// ****************************************************************************
// rotaryClickSongPlaying() - If the Rotary Encoder was clicked while a song
//    is playing, then stop the MIDI play.
// ****************************************************************************
void rotaryClickSongPlaying()
{
	// stopMidi=2 means to stop the MIDI play and do not play any
	// additional songs that may have been selected.
	stopMidi = 2;
}

// ****************************************************************************
// rotaryClickBack() - Checks to see if Rotary Encoder was clicked on the
//   'Back' menu item (the last item in the list).  If yes, show the 
//   'menuIfBack' menu with a default selection of 'newMenuSelectedIndex'
// ****************************************************************************
uint8_t rotaryClickBack(uint8_t menuIfBack, uint8_t newMenuSelectedIndex)
{
	if (menuSelectedIndex == menuLength - 1)
	{
		newMenu(menuIfBack, newMenuSelectedIndex);
		return 1;
	}
	return 0;
}

// ****************************************************************************
// rotaryLongPress() - Rotary Encoder Select Switch is Held Down (Long Press)
// ****************************************************************************
void rotaryLongPress()
{
	if (menuCurrent == LM_MENU_SONG_SELECT || menuCurrent == LM_MENU_HARDWARE_TEST)
	{
		newMenu(LM_MENU_SETUP);
		return;
	}
}

// ****************************************************************************
// idleTimeSec() - Calculate the idle time in seconds since any user actions
//    were made (e.g. rotary encoder turned, or button pressed).
// ****************************************************************************
uint16_t idleTimeSec()
{
	return (millis() - menuLastNavigateTime) / 1000;
}

// ****************************************************************************
// initializeLcd() - Initialize the LCD
// ****************************************************************************
void initializeLcd()
{
	lcd.begin();
	lcd.backlight();
	lcd.clear();
}

// ****************************************************************************
// updateLcd() - Update the LCD is anything has changed.  The function called
//   to update the menu depends upon which menu is currently selected.
// ****************************************************************************
void updateLcd()
{
	if (!menuChanged) return;

	switch (menuCurrent)
	{
	case LM_MENU_NOTE_TEST: updateLcdNoteTestOnlyNote(); break;
	case LM_MENU_SONG_SELECT: listMidiFiles(); break;
	case LM_MENU_DIRECTORY_SELECT: listDirectories(); break;
	case LM_MENU_PLAYLIST_SELECT: listPlaylists(); break;
	default: updateLcdMenu(); break;
	}
	menuChanged = 0;
}

// ****************************************************************************
// updateLcdMenu() - Refresh the entire LCD menu.
// ****************************************************************************
void updateLcdMenu()
{
	printLcdMenuTitle();
	for (uint8_t i = 0; i < BYM_LCD_ROW_COUNT - menuHasTitle; i++)
		updateLcdMenuLine(i + menuHasTitle, i+menuLcdFirstRow);
}

// ****************************************************************************
// updateLcdMenuLine() - Prints one menu line to the LCD
//    lcdLineIndex = Row of the LCD to print to
//    menuIndex = The line number of the menu to print
// ****************************************************************************
void updateLcdMenuLine(uint8_t lcdLineIndex, uint8_t menuIndex)
{
	lcd.setCursor(0, lcdLineIndex);
	if (menuNoSelection)
	{
		lcdMenuLineCharCount = 0;
	}
	else
	{
		printLcdMenuSelectChar(menuIndex);
		lcdMenuLineCharCount = 1;
	}

	switch (menuCurrent)
	{
	case LM_MENU_SETUP: printLcdMenuLineSetup(menuIndex); break;
	case LM_MENU_HARDWARE_TEST: printLcdMenuLineHardwareTest(menuIndex); break;
	case LM_MENU_ABOUT: printLcdMenuLineAbout(menuIndex); break;
	case LM_MENU_PLAYMODE_SELECT: printLcdMenuLinePlayMode(menuIndex); break;
	}
	
	if (menuIndex == menuLength - 1) printLcdMenuLineString(lcdString01_Back);
	if (lcdLineIndex + menuLcdFirstRow - menuHasTitle >= menuLength) printLcdMenuLineBlank(lcdLineIndex);

	printLcdMenuLinePadEndSpaces();
}

// ****************************************************************************
// printLcdMenuLinePadEndSpaces() - Called during LCD menu print functions.
//    Pads the end of the LCD row with Spaces.
// ****************************************************************************
void printLcdMenuLinePadEndSpaces()
{
	for (uint8_t i = 0; i<BYM_LCD_COL_COUNT - lcdMenuLineCharCount; i++) lcd.write(SPACE);
}

// ****************************************************************************
// printLcdMenuLineHardwareTest() - Prints the text for the Hardware Test menu
// ****************************************************************************
void printLcdMenuLineHardwareTest(uint8_t menuIndex)
{
	switch (menuIndex)
	{
	case 0: printLcdMenuLineString(lcdString02_SolenoidNoteTest); break;
	case 1: printLcdMenuLineString(lcdString03_ChromaticScale); break;
	}
}

// ****************************************************************************
// printLcdMenuLineAbout() - Prints the text for the About menu to the LCD
// ****************************************************************************
void printLcdMenuLineAbout(uint8_t menuIndex)
{
	switch (menuIndex)
	{
	case 0: printLcdMenuLineString(lcdString04_AboutLine1); break;
	case 1: printLcdMenuLineString(lcdString05_AboutLine2); break;
	case 2: printLcdMenuLineString(lcdString06_AboutLine3); break;
	case 3: printLcdMenuLineString(lcdString07_AboutLine4); break;
	case 4: printLcdMenuLineString(lcdString08_AboutLine5); break;
	}
}

// ****************************************************************************
// printLcdMenuLinePlayMode() - Prints the text for the Play Mode to the LCD
// ****************************************************************************
void printLcdMenuLinePlayMode(uint8_t menuIndex)
{
	switch (menuIndex)
	{
	case PLAY_MODE_SINGLE: printLcdMenuLineString(lcdString11_Single); break;
	case PLAY_MODE_SEQUENTIAL: printLcdMenuLineString(lcdString12_Sequential); break;
	case PLAY_MODE_SHUFFLE: printLcdMenuLineString(lcdString13_Shuffle); break;
	}
}

// ****************************************************************************
// printLcdMenuLinePlaylistName() - Prints the name of the currently selected
//     playlist to the LCD.
// ****************************************************************************
void printLcdMenuLinePlaylistName()
{
	if (playlistFound)
		printLcdMenuLineString(playlistFileName, strlen(playlistFileName) - FILE_EXT_STRING_LENGTH);
	else
		printLcdMenuLineString(lcdString34_None);
}

// ****************************************************************************
// printLcdMenuLineSteup() - Prints to LCD the (menuIndex)th item of the Setup 
//      menu
// ****************************************************************************
void printLcdMenuLineSetup(uint8_t menuIndex)
{
	switch (menuIndex)
	{
	case 0:	// Directory Select
		printLcdMenuLineString(lcdString21_Folder);
		if(strlen(currentDirectory)>=3)
			printLcdMenuLineString(&currentDirectory[1],strlen(currentDirectory)-2);
		else
			printLcdMenuLineString(currentDirectory); 
		break;
	case 1: // Playlist Select
		printLcdMenuLineString(lcdString22_Playlist);
		printLcdMenuLinePlaylistName();
		break;
	case 2:  // Play Mode Select
		printLcdMenuLineString(lcdString23_Mode); 
		printLcdMenuLinePlayMode(playMode);
		break;
	case 3: printLcdMenuLineString(lcdString24_HardwareTest); break; //Hardware Test
	case 4: printLcdMenuLineString(lcdString25_About); break;    // About
	case 5: printLcdMenuLineString(lcdString26_Restart); break;  // Reset
	}
}

// ****************************************************************************
// printLcdMenuLineString(const char* s) - Prints a PROGMEM string to LCD
// ****************************************************************************
void printLcdMenuLineString(const char* s)
{
	uint8_t i, n;
	char c;

	n = strlen_P(s);
	if (n > BYM_LCD_COL_COUNT - lcdMenuLineCharCount) n = BYM_LCD_COL_COUNT - lcdMenuLineCharCount;
	lcdMenuLineCharCount += n;

	for (i = 0; i < n; i++) {
		c = pgm_read_byte(s + i);
		lcd.write(c);
	}
}

// ****************************************************************************
// printLcdMenuLineString(char* s, uint8_t n) - Prints a character array 
//     (string) of length n to LCD
// ****************************************************************************
void printLcdMenuLineString(char* s, uint8_t n)
{
	if (n > BYM_LCD_COL_COUNT - lcdMenuLineCharCount) n = BYM_LCD_COL_COUNT - lcdMenuLineCharCount;
	lcdMenuLineCharCount += n;
	for (uint8_t i = 0; i < n; i++) lcd.write(s[i]);
}

// ****************************************************************************
// printLcdMenuLineString(char* s) - Prints a character array (string) to LCD
// ****************************************************************************
void printLcdMenuLineString(char* s)
{
	printLcdMenuLineString(s, strlen(s));
}

// ****************************************************************************
// printLcdMenuTitle() - Prints the Menu Title on the first row of the LCD
//    for menus that are defined to have a title in updateMenuProperties()
// ****************************************************************************
void printLcdMenuTitle()
{
	if (!menuHasTitle) return;

	lcd.setCursor(0, 0);
	lcdMenuLineCharCount = 0;
	
	if (menuCurrent == LM_MENU_HARDWARE_TEST) printLcdMenuLineString(lcdString24_HardwareTest);
	if (menuCurrent == LM_MENU_DIRECTORY_SELECT) printLcdMenuLineString(lcdString31_FolderSelect);
	if (menuCurrent == LM_MENU_PLAYMODE_SELECT) printLcdMenuLineString(lcdString32_PlayMode);	
	if (menuCurrent == LM_MENU_PLAYLIST_SELECT) printLcdMenuLineString(lcdString33_PlaylistSelect);
	
	printLcdMenuLinePadEndSpaces();
}

// ****************************************************************************
// updateLcdNoteTest() - This is called at the beginning of the Solenoid/Note
//     Test Mode to update the LCD with relevant info.
// ****************************************************************************
void updateLcdNoteTest()
{

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("Solenoid/Note Test"));
	lcd.setCursor(0, 1);
	lcd.print(F("Range: "));
	printLcdNoteName(BYM_NOTE_LOWEST);
	lcd.print(F(" to "));
	printLcdNoteName(BYM_NOTE_HIGHEST);
	lcd.setCursor(0, 2);
	lcd.print(F("Note: "));
	updateLcdNoteTestOnlyNote();
}

// ****************************************************************************
// updateLcdNoteTestOnlyNote() - This is called while in Solenoid/Note Test
//     mode to update the name of the selected note, after the rotary encoder
//     is turned.
// ****************************************************************************
void updateLcdNoteTestOnlyNote()
{
	lcd.setCursor(6, 2);
	if (menuSelectedIndex<BYM_NOTE_COUNT)
		printLcdNoteName(BYM_NOTE_LOWEST + menuSelectedIndex);
	else
		lcd.print(F("Back"));
	lcd.print(F("    "));
}

// ****************************************************************************
// printLcdNoteName() - Prints the note name (e.g. C5, C#5, D5, etc.) 
//     corresponding to 'noteNumber' to the LCD.
// ****************************************************************************
void printLcdNoteName(uint8_t noteNumber)
{
	getMidiNoteName(noteNumber);
	for (uint8_t i = 0; i < 3; i++) lcd.write(noteName[i]);
}

// ****************************************************************************
// printLcdMenuSelectChar() - Prints the selection character (*) if menuIndex
//    is the currently selected menu item.  Otherwise, prints SPACE
// ****************************************************************************
void printLcdMenuSelectChar(uint8_t menuIndex)
{
	if (menuIndex == menuSelectedIndex) lcd.print(F("*")); else	lcd.print(F(" "));
}

// ****************************************************************************
// printLcdMenuLine() - Prints one line to the LCD.  This is called once for
//   each row on the LCD while updating the menu.
// ****************************************************************************
void printLcdMenuLine(uint8_t lcdRow, char* text, uint8_t textLength, uint8_t itemIndex)
{
	uint8_t i;

	// Set cursor to the beginning of the LCD Row.
	lcd.setCursor(0, lcdRow);

	// Print the Selection character (*) if this is the currently-selected
	// menu item.
	printLcdMenuSelectChar(itemIndex);

	// Write the characters contains in 'text' to the LCD.
	// Pad any extra extra characters at the end of the row with SPACE
	for (i = 0; i < BYM_LCD_COL_COUNT - 1; i++)
	{
		if (i < textLength) lcd.write(text[i]); else lcd.write(SPACE);
	}
	lcdMenuLineCharCount = BYM_LCD_COL_COUNT;
}

// ****************************************************************************
// printLcdMenuLineBlank() - Prints a blank row to the LCD.
// ****************************************************************************
void printLcdMenuLineBlank(uint8_t lcdRow)
{
	printLcdMenuLine(lcdRow, NULL, 0, 255);
}

// ****************************************************************************
// updateMenuProperties() - Sets properties of the menu (# of Items in menu
//    and whether or not the menu has a title), based on which menu is
//    the current menu.
// ****************************************************************************
void updateMenuProperties()
{
	menuNoSelection = 0;

	switch (menuCurrent)
	{
	case LM_MENU_SETUP:
		menuLength = 7;
		menuHasTitle = 0;
		break;
	case LM_MENU_HARDWARE_TEST:
		menuLength = 3;
		menuHasTitle = 1;
		break;
	case LM_MENU_NOTE_TEST:
		menuLength = BYM_NOTE_COUNT + 1;
		menuHasTitle = 0;
		break;
	case LM_MENU_SONG_SELECT:
		menuHasTitle = 0;
		menuLength = countMidiFiles();
		songCount = menuLength;
		break;
	case LM_MENU_DIRECTORY_SELECT:
		menuHasTitle = 1;
		menuLength = countDirectories();
		break;
	case LM_MENU_ABOUT:
		menuLength = 6;
		menuHasTitle = 0;
		break;
	case LM_MENU_PLAYMODE_SELECT:
		menuLength = 4;
		menuHasTitle = 1;
		break;
	case LM_MENU_PLAYLIST_SELECT:
		menuHasTitle = 1;
		menuLength = countPlaylistFiles();
		break;
	}
}

// ****************************************************************************
// newMenu() - Shows a new menu on the LCD, with a default selection of
//    the first item.
// ****************************************************************************
void newMenu(uint8_t newMenuIndex)
{
	newMenu(newMenuIndex, 0);
}

// ****************************************************************************
// newMenu() - Shows a new menu on the LCD.
//      newMenuIndex = Index of the new menu (see LM_MENU_ defines at beginning
//                     of file)
//      initialMenuSelectedIndex = Default selection in new menu
// ****************************************************************************
void newMenu(uint8_t newMenuIndex, uint8_t initialMenuSelectedIndex)
{
	menuLastNavigateTime = millis();
	menuCurrent = newMenuIndex;
	menuChanged = 1;
	menuSelectedIndex = initialMenuSelectedIndex;
	menuLcdFirstRow = 0;
	updateMenuProperties();
	if (menuSelectedIndex >= menuLength) menuSelectedIndex = 0;
	if (menuSelectedIndex >= (BYM_LCD_ROW_COUNT - menuHasTitle))
		menuLcdFirstRow = menuSelectedIndex - (BYM_LCD_ROW_COUNT - menuHasTitle - 1);
	
}

// ****************************************************************************
// countMidiFiles() - Counts the number of lines in the current Playlist file.
//   Each line should contain the name of a MIDI file, although this function
//   doesn't check for that.  The function only checks the total # of line.
// ****************************************************************************
uint8_t countLinesInPlaylist()
{
	uint8_t count;
	uint8_t endOfFile;
	int nextByte;

	if (!playlistFound) return 0;

	playlistFile = SD.open(playlistFileFullPath);
	endOfFile = 0;
	count = 0;
	while (!endOfFile)
	{
		nextByte = playlistFile.read();
		if (nextByte == '\n' && count<250) count++;
		if (nextByte < 0) endOfFile = 1;
	}

	playlistFile.close();
	return count;
}

// ****************************************************************************
// countMidiFiles() - Counts the number of MIDI files available for selection.
//   This is either the number of lines (MIDI Files) defined in the Playlist
//   file, or the number of MIDI files in 'currentDirectory'
// ****************************************************************************
uint8_t countMidiFiles()
{
	uint8_t count;

	if (playlistFound) return countLinesInPlaylist();

	midiDirectory = SD.open(currentDirectory);

	count = 0;
	midiFile = midiDirectory.openNextFile();
	while(midiFile && count<250)
	{	
		midiFile = midiDirectory.openNextFile();
		if (isMidiFile(&midiFile)) count++;
	} 

	midiDirectory.close();
	return count;
}

// ****************************************************************************
// countPlaylistFiles() - Counts the total number of Playlist Files (*.txt)
//    within 'currentDirectory'
// ****************************************************************************
uint8_t countPlaylistFiles()
{
	uint8_t count;

	midiDirectory = SD.open(currentDirectory);

	count = 1;  // Start with 1 for the 'none' playlist
	midiFile = midiDirectory.openNextFile();
	while (midiFile && count<250)
	{
		midiFile = midiDirectory.openNextFile();
		if (isPlaylistFile(&midiFile)) count++;
	}

	midiDirectory.close();
	return count;
}

// ****************************************************************************
// countDirectories() - Counts the total number of directories avaiable, which
//    is the Root directory, plus all sub-directories within the Root directory
// ****************************************************************************
uint8_t countDirectories()
{
	uint8_t count;

	midiDirectory = SD.open(rootDirectory);

	count = 1; // Start with 1 for the Root Directory
	subDirectory = midiDirectory.openNextFile();
	while (subDirectory && count<250)
	{
		if (subDirectory.isDirectory()) count++;
		subDirectory = midiDirectory.openNextFile();
	}

	midiDirectory.close();
	return count;
}

// ****************************************************************************
// findDirectorySelectedIndex() - Finds the index within the file system of
//   the Directory defined in the char array 'currentDirectory'.  
// ****************************************************************************
uint8_t findDirectorySelectedIndex()
{
	uint8_t count;

	// If 'currentDirectory' is the root directory, then return the
	// index of the root directory.
	if (strlen(currentDirectory) == strlen(rootDirectory)) return 0;

	// Open the root directory
	midiDirectory = SD.open(rootDirectory);

	// Loop through all sub-directories in the root directory until we
	// find the one matching 'currentDirectory'
	count = 1;
	subDirectory = midiDirectory.openNextFile();
	while (subDirectory)
	{
		if (subDirectory.isDirectory())
		{
			subDirectory.getFilename(subDirectoryName);
			if (strstr(currentDirectory, subDirectoryName))
			{
				midiDirectory.close();
				return count;
			}
			count++;
		}
		subDirectory = midiDirectory.openNextFile();
	}

	// If 'currentDirectory' was not found, then return the index of 
	// the root directory.
	midiDirectory.close();
	return 0;
}

// ****************************************************************************
// findPlaylistSelectedIndex() - Finds the index within the file system of
//   the playlist file defined in the char array 'playlistFileFullPath'.  
//   This function has not been implemented yet. 
// ****************************************************************************
uint8_t findPlaylistSelectedIndex()
{
	return 0;
}

// ****************************************************************************
// isMidiFile() - Checks whether or not the filename in File object 'pFile'
//     ends with the extention '.MID'
// ****************************************************************************
uint8_t isMidiFile(File* pFile)
{
	return isFileOfType(pFile, midiFileExt);
}

// ****************************************************************************
// isPlaylistFile() - Checks whether or not the filename in File object 'pFile'
//     ends with the extention '.TXT'
// ****************************************************************************
uint8_t isPlaylistFile(File* pFile)
{
	return isFileOfType(pFile, playlistFileExt);
}

// ****************************************************************************
// isFileOfType() - Checks whether or not the filename in File object 'pFile'
//     ends with the extention 'fileExt'
// ****************************************************************************
uint8_t isFileOfType(File* pFile, char* fileExt)
{
	
	if (!pFile->isFile()) return 0;
	uint8_t fileLength = strlen(pFile->name());
	if (fileLength <= FILE_EXT_STRING_LENGTH) return 0;
	if (strstr(strlwr(pFile->name() + (fileLength - FILE_EXT_STRING_LENGTH)), fileExt)) return 1;
	return 0;
}

// ****************************************************************************
// openNextFileOfType() - Finds the next file object in current directory
//   which is has the extention specified in 'fileExt'
// ****************************************************************************
void openNextFileOfType(char* fileExt)
{
	while (1)
	{
		midiFile = midiDirectory.openNextFile();
		if (!midiFile) return;
		if (isFileOfType(&midiFile, fileExt)) return;
	}
}

// ****************************************************************************
// openNextMidiFile() - Finds the next file object in current directory
//   which is a MIDI file (extention *.MID)
// ****************************************************************************
void openNextMidiFile()
{
	openNextFileOfType(midiFileExt);
}

// ****************************************************************************
// openNextPlaylistFile() - Finds the next file object in current directory
//   which is a Playlist file (extention *.TXT)
// ****************************************************************************
void openNextPlaylistFile()
{
	openNextFileOfType(playlistFileExt);
}

// ****************************************************************************
// openNextDirectory() - Finds the next file object in the current directory
//   which is a directory.  Order of file objects determined by file system.
// ****************************************************************************
void openNextDirectory()
{
	while (1)
	{
		subDirectory = midiDirectory.openNextFile();
		if (!subDirectory) return;
		if (subDirectory.isDirectory()) return;
	}
}

// ****************************************************************************
// openMidiFileByIndex() - Finds the (i)th MIDI File in the current directory.
//    Order of MIDI files is determined by the File System.
// ****************************************************************************
void openMidiFileByIndex(uint8_t midiIndex)
{
	uint8_t i;
	midiDirectory = SD.open(currentDirectory);
	for (i = 0; i < midiIndex; i++) openNextMidiFile();
	midiDirectory.close();
}

// ****************************************************************************
// listMidiFilesPlaylist() - Shows on the LCD a list of MIDI Files from
//    the currently-selected Playlist File.
// ****************************************************************************
void listMidiFilesPlaylist()
{
	uint8_t i;

	// Opens the playlist file
	playlistFile = SD.open(playlistFileFullPath);

	// Get the Midi files prior to menuLcdFirstRow - do nothing with these
	for (i = 0; i < menuLcdFirstRow; i++) textFileReadInt(&playlistFile);

	// List the next 4 Midi Files on the LCD
	for (i = 0; i < BYM_LCD_ROW_COUNT; i++)
	{
		if (menuLcdFirstRow + i < menuLength)  // If enough menu items for this row
		{	
			// Construct MIDI filename as [currentDirectory] + [Next Line from Playlist File]
			strcpy(midiFileFullPath, currentDirectory);
			textFileReadString(&playlistFile, midiFileName, MIDI_FILENAME_BUFFER_SIZE);
			
			// Get INI File corresponding to the midiFile
			getIniFileName(0);

			// If this is the currently-selected menu item, then save the index
			// of the selected song.  This is used to start playing the song when
			// the cutton is clicked.
			if (menuLcdFirstRow + i == menuSelectedIndex) {
				selectedSongFileIndex = menuSelectedIndex;
				currentSong = menuSelectedIndex;
			}
					
			if (!SD.exists(iniFileFullPath))
			{
				// If INI file does not exist, then print the MIDI Filename as the song title.
				printLcdMenuLine(i, midiFileName, strlen(midiFileName), menuLcdFirstRow + i);
			}
			else
			{
				// Otherwise, print the song title from the INI file
				printLcdSongTitle(i);
			}
		}
		else
		{
			// If not enough menu items to fill all rows on the LCD, then 
			// fill in the remaining rows with Blanks
			printLcdMenuLineBlank(i);
		}
	}

	playlistFile.close();
	menuChanged = 0;
}

// ****************************************************************************
// listMidiFiles() - Shows on the LCD a list of MIDI Files in the Current
//   Directory.  If a valid Playlist File has been selected, then show
//   a list of MIDI files from the Playlist file.  If no valid playlist, then
//   list all MIDI files in the directory.
// ****************************************************************************
void listMidiFiles()
{
	uint8_t i;

	// If a valid playlist was found, then call a separate function for
	// showing MIDI files from the Playlist file.
	if (playlistFound)
	{
		listMidiFilesPlaylist();
		return;
	}

	// Open the current directory
	midiDirectory = SD.open(currentDirectory);

	// Get the Midi files prior to menuLcdFirstRow - do nothing with these
	for (i = 0; i < menuLcdFirstRow; i++) openNextMidiFile();

	// List the next 4 Midi Files on the LCD
	for (i = 0; i < BYM_LCD_ROW_COUNT; i++)
	{
		if (menuLcdFirstRow + i < menuLength) // If enough menu items for this row
		{
			// Open the next MIDI file.
			openNextMidiFile();
			
			// If an INI file exists for the current MIDI file, then print the song
			// title as defined in the INI file.  If the INI file is not found, then
			// print the name of the MIDI file (e.g. MYSONG.MID)
			getIniFileName(1);
			if (!SD.exists(iniFileFullPath))
			{
				// Print name of MIDI file as the song title
				printLcdMenuLine(i, midiFile.name(), strlen(midiFile.name()), menuLcdFirstRow + i);
			}
			else
			{
				// Print the song title from the INI file.
				printLcdSongTitle(i);
			}

			// While writing the name of the currently-selected MIDI file, save the 
			// name and selected index of that MIDI file.  This is used to start
			// playing the song if the button is clicked.
			if (menuLcdFirstRow + i == menuSelectedIndex) {
				strcpy(midiFileFullPath, currentDirectory);
				midiFile.getFilename(&(midiFileFullPath[strlen(currentDirectory)]));
				selectedSongFileIndex = menuSelectedIndex;
				currentSong = menuSelectedIndex;
			}
		}
		else
		{
			// If not enough menu items to fill all rows on the LCD, then 
			// fill in the remaining rows with Blanks
			printLcdMenuLineBlank(i);
		}
	}

	midiDirectory.close();
	menuChanged = 0;
}

// ****************************************************************************
// listDirectories() - Shows on the LCD a list of all Root-Level 
//    Sub-Directories on the SD Card.  The first item on the list is "\"
//    which is the Root directory of the SD card.
// ****************************************************************************
void listDirectories()
{
	uint8_t i,n;
	uint8_t useRootDirectory=0;

	// Show the text "Directory Select" as the first line on the LCD.
	printLcdMenuTitle();

	// Open the Root Directory
	midiDirectory = SD.open(rootDirectory); 

	// menuLcdFirstRow is the index of the menu item to show on the first LCD Row.
	// If menuLcdFirstRow=0, then show the first menu item on the first LCD row.
	// If menuLcdFirstRow=1, then show the second menu item on the first LCD row, etc.
	if(menuLcdFirstRow==0) 
		// The first directory selection is "\"
		useRootDirectory = 1;
	else
		// Get the Directories prior to menuLcdFirstRow - do nothing with these
		for (i = 0; i < (menuLcdFirstRow-1); i++) openNextDirectory();

	// List the next 3 or 4 Directories on the LCD
	// (depending upon if the menu title is on the first row)
	for (i = 0; i < BYM_LCD_ROW_COUNT-menuHasTitle; i++)
	{
		if (menuLcdFirstRow + i < menuLength) // If enough menu items for this row
		{		
			if (useRootDirectory)
			{
				// Show the Root Directory "\"
				printLcdMenuLine(i+menuHasTitle, rootDirectory, strlen(rootDirectory), menuLcdFirstRow + i);
				strcpy(currentDirectory, rootDirectory);
			}
			else
			{
				// Open the next directory and write to the LCD.
				openNextDirectory();
				printLcdMenuLine(i+menuHasTitle, subDirectory.name(), strlen(subDirectory.name()), menuLcdFirstRow + i);
				
				// While writing the directory name for the currently-selected menu item,
				// save the directory name to the char array 'currentDirectory'
				if (menuLcdFirstRow + i == menuSelectedIndex) {
					currentDirectory[0] = '/';
					subDirectory.getFilename(&(currentDirectory[1]));
					n = strlen(currentDirectory);
					currentDirectory[n] = '/';
					currentDirectory[n + 1] = 0;
				}
			}

			useRootDirectory = 0;
		}
		else
		{
			// If not enough menu items to fill all rows on the LCD, then 
			// fill in the remaining rows with Blanks
			printLcdMenuLineBlank(i+menuHasTitle);
		}
	}

	midiDirectory.close();
	menuChanged = 0;
}

// ****************************************************************************
// listPlaylists() - Shows a list of Playlist Files (*.txt) on the LCD.
//    The first item on the list is "[none]", which means that the Playlist
//    is all MIDI files in the current directory.
// ****************************************************************************
void listPlaylists()
{
	uint8_t i, n;
	uint8_t showNonePlaylist=0;

	// Show the text "Playlist Select" as the first line on the LCD.
	printLcdMenuTitle();  

	// Open the current directory
	midiDirectory = SD.open(currentDirectory); 

	// menuLcdFirstRow is the index of the menu item to show on the first LCD Row.
	// If menuLcdFirstRow=0, then show the first menu item on the first LCD row.
	// If menuLcdFirstRow=1, then show the second menu item on the first LCD row, etc.
	if (menuLcdFirstRow==0)
		// The first playlist selection is "[none]"
		showNonePlaylist = 1;
	else
		// Get the Playlist files prior to menuLcdFirstRow - do nothing with these
		for (i = 0; i < (menuLcdFirstRow-1); i++) openNextPlaylistFile();

	// List the next 3 or 4 Playlist Files on the LCD
	// (depending upon if the menu title is on the first row)
	for (i = 0; i < BYM_LCD_ROW_COUNT-menuHasTitle; i++)
	{
		if (menuLcdFirstRow + i < menuLength) // If enough menu items for this row
		{
			if (showNonePlaylist)
			{
				// Show the menu item "[none]"
				printLcdMenuLine(i + menuHasTitle, NULL, 0, menuLcdFirstRow + i);
				lcd.setCursor(1, i + menuHasTitle);
				lcdMenuLineCharCount = 1;
				printLcdMenuLineString(lcdString34_None);
			}
			else
			{
				// Open the next Playlist File (file with *.txt extention) and
				// write that base file name (without the .TXT) to the LCD.
				openNextPlaylistFile();
				printLcdMenuLine(i + menuHasTitle, midiFile.name(), 
					strlen(midiFile.name()) - FILE_EXT_STRING_LENGTH, menuLcdFirstRow + i);

				// While writing the Playlist file currently selected in the menu
				// save the name of this file to the char array "playlistFileNmae"
				if (menuLcdFirstRow + i == menuSelectedIndex) {
					strcpy(playlistFileFullPath, currentDirectory);
					midiFile.getFilename(playlistFileName);
				}
			}
			showNonePlaylist = 0;
		}
		else
		{
			// If not enough menu items to fill all rows on the LCD, then 
			// fill in the remaining rows with Blanks
			printLcdMenuLineBlank(i+menuHasTitle);
		}
	}
	midiDirectory.close();

}

// ****************************************************************************
// loadPlayMode() - Loads the Play Mode from EEPROM.  
// ****************************************************************************
void loadPlayMode()
{
	playMode = EEPROM.read(PLAY_MODE_ADDRESS);
	if (playMode > PLAY_MODE_SHUFFLE) playMode = 0;
}

// ****************************************************************************
// savePlayMode() - Saves the Currently Selected Play Mode to EEPROM
// ****************************************************************************
void savePlayMode()
{
	EEPROM.write(PLAY_MODE_ADDRESS,playMode);
}

// ****************************************************************************
// loadCurrentDirectory() - Loads the Selected Directory from EEPROM
// ****************************************************************************
void loadCurrentDirectory()
{
	for (uint8_t i = 0; i < DIRECTORY_LENGTH; i++) 
		currentDirectory[i] = EEPROM.read(DIRECTORY_ADDRESS + i);
}

// ****************************************************************************
// saveCurrentDirectory() - Saves the Currently Selected Directory to EEPROM
// ****************************************************************************
void saveCurrentDirectory()
{
	for (uint8_t i = 0; i < DIRECTORY_LENGTH; i++) 
		EEPROM.write(DIRECTORY_ADDRESS + i, currentDirectory[i]);
}

// ****************************************************************************
// savePlaylist() - Saves Playlist Filename to EEPROM.  If no valid playlist
//   file has been set, then save an array of 0's to this EEPROM location.
// ****************************************************************************
void savePlaylist()
{
	uint8_t i;

	if (playlistFound)
		for (i = 0; i < FULL_PATH_FILE_LENGTH; i++) EEPROM.write(PLAYLIST_ADDRESS + i, playlistFileFullPath[i]);
	else
		for (i = 0; i < FULL_PATH_FILE_LENGTH; i++) EEPROM.write(PLAYLIST_ADDRESS + i, 0);
}

// ****************************************************************************
// loadPlaylist() - Load the Playlist Filename (char array) from EEPROM, and
//   check whether or not it is a valid Playlist file.
// ****************************************************************************
void loadPlaylist()
{
	for (uint8_t i = 0; i < FULL_PATH_FILE_LENGTH; i++) 
		playlistFileFullPath[i] = EEPROM.read(PLAYLIST_ADDRESS + i);
	validatePlaylist();
}

// ****************************************************************************
// validatePlaylist() - Validates whether or not the character arrary
//    "playlistFileFullPath" contains the name of a valid Playlist File.
//    Sets the global variable playlistFound=1 if the Playlist file is valid
// ****************************************************************************
void validatePlaylist()
{
	uint8_t i, n;

	// If playlist file full path does not start with the currently selected directory, 
	// then consider this to be a playlist not found
	n = strlen(currentDirectory);
	for (i = 0; i < n; i++)
	{
		if (currentDirectory[i] != playlistFileFullPath[i])
		{
			playlistFound = 0;
			return;
		}
	}

	// If the currently selected director is the Root directory, check to see 
	// if the playlist path has a sub-directory.  (E.g. if it contains the '\'
	// character).  If yes, it is not a valid playlist file.
	if (n == 1) // Length of currentDirectory can only be 1 if it is "\"
	{
		for (i = 1; i < strlen(playlistFileFullPath); i++)
		{
			// If the ith character is '\'
			if (playlistFileFullPath[i] == currentDirectory[0])
			{
				playlistFound = 0;
				return;
			}
		}
	}

	// If code gets to this point, then playlistFileFullPath is within
	// currentDirectory.  In that case, check to see if the file exists
	// on the SD card.
	if (!SD.exists(playlistFileFullPath))
	{
		playlistFound = 0;
		return;
	}
	playlistFound = 1;
}

// ****************************************************************************
// getIniFileName() - Gets the name of the 'ini' file from the current MIDI
//   file selected.  E.g. if midiFileFullPath = "\SONGS\MYSONG.MID" then
//   this function sets iniFileFullPath = "\SONGS\MYSONG.INI".
//   If copyFromMidiFileObject = True, then the filename is copied from
//   a currently-open File object.
// ****************************************************************************
uint8_t getIniFileName(uint8_t copyFromMidiFileObject)
{
	
	if (copyFromMidiFileObject)
	{
		//Copy MIDI File name from the "File" object "midiFile"
		strcpy(iniFileFullPath, currentDirectory);
		midiFile.getFilename(&(iniFileFullPath[strlen(currentDirectory)]));
	}
	else
	{
		strcpy(iniFileFullPath, midiFileFullPath);
	}

	return setIniFileExt();
}

// ****************************************************************************
// setIniFileExt() - Sets the extention of the file iniFileFullPath to 'ini'
// ****************************************************************************
uint8_t setIniFileExt()
{
	uint8_t n;
	n = strlen(iniFileFullPath);
	if (n < 5) return 0;
	if (iniFileFullPath[n - 4] != '.') return 0;
	iniFileFullPath[n - 3] = 'i';
	iniFileFullPath[n - 2] = 'n';
	iniFileFullPath[n - 1] = 'i';
	return 1;
}

// ****************************************************************************
// printLcdSongTitle() - Print the song title of the current MIDI/INI file
//  to the LCD at row lcdRow.  This is used to show the list of songs.
// ****************************************************************************
void printLcdSongTitle(uint8_t lcdRow)
{
	lcd.setCursor(0, lcdRow);
	printLcdMenuSelectChar(menuLcdFirstRow + lcdRow);

	textFile = SD.open(iniFileFullPath, FILE_READ);
	textFileToLcd(&textFile, lcdRow, 1, 1);
	textFile.close();
}

// ----------------------------------------------------------------------------
// TODO: Make sure that the Lowest Note / Highest Note, etc., are handled
//  correctly for the case where there are multiple xylophones, and
//  multiple versions of the same song.
// ----------------------------------------------------------------------------
//
// ****************************************************************************
// readMidiFileProperties() - Reads an ini file that contains information
//    associated with a MIDI file, such as song title, author, total time, etc.
//    Ini file needs to have the same base filename as the MIDI file.  For
//    example, the MIDI file FURELISE.MID would have an ini file FURELISE.INI
//    The structure of the INI file is as follows:
//
//  Line 1 : Song Title (required)
//  Line 2 : Author or other Song Info (optional)
//  Line 3 : Additional Song Info (optional)
//  Line 4 : Song Length in Seconds (optional)
//  Line 5 : Transpose in # of Half Steps (optional)
//  Line 6 : Target High Note - Attempt to match the highest note on the
//           xylophone to this note, during MIDI playback (optional)
//
//  Lines 1-3 are strings, while lines 4-7 are integers.  Any integer values
//  that are not specified default to 0.
//  If the INI file does not exist, the program uses the MIDI file name as
//  the song title.
// ****************************************************************************
void readMidiFileProperties()
{	
	uint8_t songInfoRowCount;

	if (!getIniFileName(0)) return;

	if (!SD.exists(iniFileFullPath))
	{
		lcdRowAsterisks(0);
		printLcdMenuLine(1, &midiFileFullPath[strlen(currentDirectory)], strlen(midiFileFullPath) - strlen(currentDirectory), 255);
		lcdRowBlank(2);
		lcdRowAsterisks(3);
		songPlayTimeLcdRow = 2;
		songLength = 0;
		songTranspose = 0;
		songTargetHighNote = BYM_NOTE_HIGHEST;
		return;
	}

	textFile = SD.open(iniFileFullPath, FILE_READ);

	songInfoRowCount = textFileCountPrintableRows(&textFile, 3);
	songLength = textFileReadInt(&textFile);
	textFile.rewind();

	switch (songInfoRowCount)
	{
	case 3:
		textFileToLcd(&textFile, 0, 3, 0);
		lcdRowBlank(3);
		songPlayTimeLcdRow = 3;
		break;
	case 2:
		lcdRowAsterisks(0);
		textFileToLcd(&textFile, 1, 3, 0);
		songPlayTimeLcdRow = 3;
		break;
	case 1:
		lcdRowAsterisks(0);
		textFileToLcd(&textFile, 1, 3, 0);
		songPlayTimeLcdRow = 2;
		lcdRowAsterisks(3);
		break;
	}

	
	// Next Row is the Song Length which was read previously
	textFileReadInt(&textFile);   // Line 4

	songTranspose = textFileReadInt(&textFile);       // Line 5
	songTargetHighNote = textFileReadInt(&textFile);  // Line 6
	
	//if (songLowestNote == 0) songLowestNote = BYM_NOTE_LOWEST;
	if (songTargetHighNote == 0) songTargetHighNote = BYM_NOTE_HIGHEST;
	
	// Read one more line (Line 7) from the INI file as an integer.
	// If this number is > songTargetHighNote and Less than or equal to 108
	// (C8, the highest note on a piano), then take this to be the new
	// songTargetHighNote.  This will accomodate earlier version of the INI file
	// which used Line 6 for the Lowest note and Line 7 for the Highest note.
	songInfoRowCount = textFileReadInt(&textFile);  // Line 7
	if (songInfoRowCount > songTargetHighNote && songInfoRowCount <= 108)
		songTargetHighNote = songInfoRowCount;

	textFile.close();
}

// ****************************************************************************
// lcdRowAsterisks() - Print a row of asterisks (*) to the LCD.
// ****************************************************************************
void lcdRowAsterisks(uint8_t lcdRow)
{
	lcd.setCursor(0, lcdRow);
	lcd.print(F("********************"));
}

// ****************************************************************************
// lcdRowBlank() - Print a blank row to the LCD.
// ****************************************************************************
void lcdRowBlank(uint8_t lcdRow)
{
	lcd.setCursor(0, lcdRow);
	lcd.print(F("                    "));
}

// ****************************************************************************
// textFileCountPrintableRows() - This is used to count how many rows in a
//   text file has text that could be displayed on the LCD.  This is used
//   to determine if any LCD rows need to be filled in with extra characters
// ****************************************************************************
uint8_t textFileCountPrintableRows(File *pfile, uint8_t maxRows)
{
	uint8_t i, j;
	uint8_t rowCount;
	int nextByte;
	uint8_t endOfLine;
	uint8_t rowIsNonBlank;

	rowCount = 0;
	for (i = 0; i < maxRows; i++)
	{
		endOfLine = 0;
		rowIsNonBlank = 0;
		for (j = 0; j < BYM_LCD_COL_COUNT - 2; j++)
		{
			if (!endOfLine) nextByte = pfile->read();
			if (nextByte > SPACE && nextByte <= '~') rowIsNonBlank = 1;
			endOfLine = checkEndOfLine(nextByte);
		}

		if (!endOfLine) textFileReadToEndOfLine(pfile);
		if (rowIsNonBlank) rowCount = i + 1;
	}

	// rowCount can never be less than 1 because a song title always 
	// needs to be shown.
	if (rowCount == 0) rowCount = 1;

	return rowCount;
}

// ****************************************************************************
// textFileToLcd() - Copies rowCount lines from a text file onto the LCD.
//    This is used to display song title, author, or other info avaiable
//    in the MIDI file's ini file.
//    File must already be open prior to calling this function.
//    Information is written to the LCD starting at row lcdFirstRow and
//    column lcdFirstCol. Each LCD row is padded with spaces if it is longer 
//    than the file line
// ****************************************************************************
void textFileToLcd(File *pfile, uint8_t lcdFirstRow, uint8_t rowCount, uint8_t lcdFirstCol)
{
	uint8_t i, j;
	int nextByte;
	uint8_t endOfLine;

	for (i = 0; i < rowCount; i++)
	{
		lcd.setCursor(lcdFirstCol, lcdFirstRow + i);
		endOfLine = 0;

		// Write either text from the file, or a blank space onto each LCD character.
		for (j = 0; j < BYM_LCD_COL_COUNT-lcdFirstCol; j++)
		{
			if (!endOfLine) nextByte = pfile->read();
			if (nextByte > SPACE && nextByte <= '~') lcd.write(nextByte); else lcd.write(SPACE);	
			endOfLine = checkEndOfLine(nextByte);
		}

		// Read to end of line, if more is available in the file.
		if (!endOfLine) textFileReadToEndOfLine(pfile);
	}
}

// ****************************************************************************
// checkEndOfLine() - Returns 1 if a character is End of Line, 0 otherwise.
// ****************************************************************************
uint8_t checkEndOfLine(int byteValue)
{
	if (byteValue == '\n') return 1;
	if (byteValue < 0) return 1;
	return 0;
}

// ****************************************************************************
// textFileReadToEndOfLine() - Move's the cursor in a text file to the
//    end of the current line.
// ****************************************************************************
void textFileReadToEndOfLine(File *pfile)
{
	while (1)
	{
		if (checkEndOfLine(pfile->read())) return;
	}
}

// ****************************************************************************
// textFileReadString() - Reads a string from a text file.  File must be open
//   and at the correct position prior to calling this function.
//   Reads only characters between ' ' (0x20) and '~' (0x7E)
//   Fills any extra buffer space with 0's.
// ****************************************************************************
void textFileReadString(File *pfile, char *buffer, uint8_t bufferLength)
{
	int nextByte;
	uint8_t endOfLine;
	uint8_t i;

	endOfLine = 0;
	for (i = 0; i < bufferLength; i++)
	{
		if (!endOfLine) nextByte = pfile->read();
		if (nextByte >= SPACE && nextByte <= '~' && !endOfLine) buffer[i] = nextByte; else buffer[i] = 0;
		endOfLine = checkEndOfLine(nextByte);
	}

	// Read to the end of the line, if there is more text in the line than
	// available space in the buffer.
	if (!endOfLine) textFileReadToEndOfLine(pfile);
}

// ****************************************************************************
// textFileReadInt() - Reads an integer from a text file.  File must be open
//   and at the correct position prior to calling this function.  If an
//   integer is not found at the current line, function returns 0.
// ****************************************************************************
int textFileReadInt(File *pfile)
{
	int nextByte;
	uint8_t negativeSign;
	uint8_t endOfLine;
	uint8_t i;
	int r;
	r = 0;
	negativeSign = 0;
	endOfLine = 0;

	// Only read up to the first 10 characters in the line.
	for (i = 0; i < 10; i++)
	{
		// Read the next byte in the file
		if (!endOfLine) nextByte = pfile->read();
		
		// If byte is numeric, use to calculate the integer
		if (nextByte >= '0' && nextByte <= '9') r = r * 10 + (nextByte - '0');
		
		// Check for negative sign
		if (nextByte == '-') negativeSign = 1;

		// Check for End of Line.  Quit reading if at end of line.
		endOfLine = checkEndOfLine(nextByte);
	}

	// Read to the end of line ifmore characters are remaining.
	if (!endOfLine) textFileReadToEndOfLine(pfile);

	if (negativeSign) r = -r;
	return r;
}

// ****************************************************************************
// midiCallback() - Callback function from the MD_MIDIFile library.  Function
//   is called while the MIDI file is playing every time there is a new
//   MIDI event.  However, only the Note On events are used to play notes
//   on the xylophone.
//   This callback is set up in the setup() function.
// ****************************************************************************
void midiCallback(midi_event *pev)
{
	// Only the Note On (0x90) Events are used.
	if ((pev->data[0] & 0xF0) == 0x90)
	{
		// Check for the second byte (volume) to be > 0.
		// If volume=0, it is the same as a Note Off event.
		if (pev->data[2] > 0)
		{
			playNote(midiNoteToXylophoneNote(pev->data[1]));
			lastMidiNoteTime = millis();
		}
	}
}

// ****************************************************************************
// midiNoteToXylophoneNote() - Converts a note directly from the MIDI File
//   into the note that is played on the xylophone.  This includes transposing
//   all notes up or down a specified number of half-steps, transposing to the
//   defined target highest note in the MIDI file, and octave-
//   transposing any notes that are higher or lower than the xylophone range.
// ****************************************************************************
uint8_t midiNoteToXylophoneNote(uint8_t midiNote)
{
	int8_t tr;
	uint8_t xyloNote;

	tr = songTranspose;
	if (songTargetHighNote > BYM_NOTE_HIGHEST)
		tr = tr - (songTargetHighNote - BYM_NOTE_HIGHEST);
	if (BYM_NOTE_HIGHEST > songTargetHighNote)
		tr = tr + (BYM_NOTE_HIGHEST - songTargetHighNote);
	
	xyloNote = midiNote + tr;
	while (xyloNote < BYM_NOTE_LOWEST) xyloNote += 12;
	while (xyloNote > BYM_NOTE_HIGHEST) xyloNote -= 12;
	return xyloNote;
}

// ****************************************************************************
// playNoteTest() - Plays one note 3 times.  Used in Hardware Test menu.
// ****************************************************************************
void playNoteTest()
{
	for (uint8_t i = 0; i < 3; i++)
	{
		playNote(menuSelectedIndex + BYM_NOTE_LOWEST);
		delay(500);
		menuLastNavigateTime = millis();
	}
}

// ****************************************************************************
// playChromaticScale() - Plays a Chromatic Scale.  Used in Hardware Test menu.
// ****************************************************************************
void playChromaticScale()
{
	for (uint8_t i = 0; i < BYM_NOTE_COUNT; i++)
	{
		playNote(i + BYM_NOTE_LOWEST);
		delay(300);
		menuLastNavigateTime = millis();
	}
}

// ****************************************************************************
// playNote() - Sends a Serial Command to play a single note.
// ****************************************************************************
void playNote(uint8_t noteNum)
{
	uint8_t boardNum, chNum, i;

	// Set OUTPUT_NOTE_NAMES=1 to output the actual name of the note
	// (e.g. C5, C#5, D5, ...) to Serial.  This is used for debugging.
	// For normal use, set OUTPUT_NOTE_NAMES=0.
	if (OUTPUT_NOTE_NAMES)
	{
		getMidiNoteName(noteNum);
		for (i = 0; i < 3; i++) Serial.write(noteName[i]);
		Serial.write(SPACE);
		return;
	}

	// Get the Channel number and Board (Slave Address) number from the 
	// Note Map.  Slave address is the Top Nibble of the byte, and Channel
	// number is the Bottom Nibble.
	chNum = noteMap(noteNum - BYM_NOTE_LOWEST) % 16;
	boardNum = (noteMap(noteNum - BYM_NOTE_LOWEST) - chNum) / 16;

	// Only channel numbers 0 to 7 are valid.  Bit-shift the channel 
	// number because the Slave is expecting a bitmask of channels
	// 0 through 7.
	if (chNum < 8) chNum = 1 << chNum; else chNum = 0;

	// Send the command to the slave via Serial.
	Serial.print(F("<"));
	serialPrintByteHex(boardNum);
	serialPrintByteHex(chNum);
	Serial.print(F(">"));
}

// ****************************************************************************
// noteMap() - Converts a xylophone note number (0 to BYM_NOTE_COUNT-1) to
//    the Motor Driver Slave address and Channel number that has the solenoid
//    corresponding to that note.
//    The values BYM_NOTE_xx are defined at the beginning of the program.
// ****************************************************************************
uint8_t noteMap(uint8_t noteNum)
{
	switch (noteNum)
	{
	case 0: return BYM_NOTE_00;
	case 1: return BYM_NOTE_01;
	case 2: return BYM_NOTE_02;
	case 3: return BYM_NOTE_03;
	case 4: return BYM_NOTE_04;
	case 5: return BYM_NOTE_05;
	case 6: return BYM_NOTE_06;
	case 7: return BYM_NOTE_07;
	case 8: return BYM_NOTE_08;
	case 9: return BYM_NOTE_09;
	case 10: return BYM_NOTE_10;
	case 11: return BYM_NOTE_11;
	case 12: return BYM_NOTE_12;
	case 13: return BYM_NOTE_13;
	case 14: return BYM_NOTE_14;
	case 15: return BYM_NOTE_15;
	case 16: return BYM_NOTE_16;
	case 17: return BYM_NOTE_17;
	case 18: return BYM_NOTE_18;
	case 19: return BYM_NOTE_19;
	case 20: return BYM_NOTE_20;
	case 21: return BYM_NOTE_21;
	case 22: return BYM_NOTE_22;
	case 23: return BYM_NOTE_23;
	case 24: return BYM_NOTE_24;
	case 25: return BYM_NOTE_25;
	case 26: return BYM_NOTE_26;
	case 27: return BYM_NOTE_27;
	case 28: return BYM_NOTE_28;
	case 29: return BYM_NOTE_29;
	case 30: return BYM_NOTE_30;
	case 31: return BYM_NOTE_31;
	default: return 0;
	}
}

// ****************************************************************************
// serialPrintByteHex() - Print a byte in Hexadecimal formal to Serial
// ****************************************************************************
void serialPrintByteHex(uint8_t b)
{
	if (b < 0x10) Serial.print(F("0"));
	Serial.print(b, HEX);
}

// ****************************************************************************
// getXyloNoteName() - Converts a byte with index of the xylophone note.
//   the actual note name.  0 corresponds to a MIDI node of BYM_NOTE_LOWEST.
// ****************************************************************************
void getXyloNoteName(uint8_t xyloNote)
{
	getMidiNoteName(xyloNote + BYM_NOTE_LOWEST);
}

// ****************************************************************************
// getMidiNoteName() - Converts a byte with the MIDI note to a string with
//   the actual note name.  Example: 0x3C-->C4, 0x3D-->C#4, 0x3E-->D4, ...
// ****************************************************************************
void getMidiNoteName(uint8_t midiNote)
{
	uint8_t noteoct = midiNote % 12;
	uint8_t octave = (midiNote - noteoct) / 12 - 1;

	switch (noteoct)
	{
	case 0:
		noteName[0] = ' ';
		noteName[1] = 'C';
		break;
	case 1:
		noteName[0] = 'C';
		noteName[1] = '#';
		break;
	case 2:
		noteName[0] = ' ';
		noteName[1] = 'D';
		break;
	case 3:
		noteName[0] = 'D';
		noteName[1] = '#';
		break;
	case 4:
		noteName[0] = ' ';
		noteName[1] = 'E';
		break;
	case 5:
		noteName[0] = ' ';
		noteName[1] = 'F';
		break;
	case 6:
		noteName[0] = 'F';
		noteName[1] = '#';
		break;
	case 7:
		noteName[0] = ' ';
		noteName[1] = 'G';
		break;
	case 8:
		noteName[0] = 'G';
		noteName[1] = '#';
		break;
	case 9:
		noteName[0] = ' ';
		noteName[1] = 'A';
		break;
	case 10:
		noteName[0] = 'A';
		noteName[1] = '#';
		break;
	case 11:
		noteName[0] = ' ';
		noteName[1] = 'B';
		break;
	}

	noteName[2] = 48 + octave;
}

// ****************************************************************************
// splayScreen() - If SPLASH_SCREEN_DELAY_MS > 0 then show a Splash Screen
//   with program info.  This is the same as info shown in the About menu.
// ****************************************************************************
void splashScreen()
{
	if (SPLASH_SCREEN_DELAY_MS == 0) return;
	newMenu(LM_MENU_ABOUT);
	menuNoSelection = 1;
	updateLcdMenu();
	delay(SPLASH_SCREEN_DELAY_MS);
}

// ****************************************************************************
// setup() - Main program setup function.  Called on Power-up or Reset.
// ****************************************************************************
void setup()
{
	// Set the Serial Baud Rate.  This needs to be the same Serial Baud Rate
	// as is used for the Slave Arduinos.
	Serial.begin(BYM_SERIAL_BAUD_RATE);

	// Initialize the Rotary Encoder and LCD
	initializeRotaryEncoder();
	initializeLcd();

	// Initialize the SD card
	if (!SD.begin(SD_SELECT, SPI_FULL_SPEED))
	{
		lcd.setCursor(0, 0);
		lcd.print(F("SD Init Fail!"));
		delay(2000);
		
		// If SD Initializiation fails (such as if there is no SD card) then
		// show the Hardware Test menu.
		newMenu(LM_MENU_HARDWARE_TEST);
		sdInitFail = 1;
		return;
	}

	// Initialize MIDIFile object
	SMF.begin(&SD);
	SMF.setMidiHandler(midiCallback);

	loadCurrentDirectory(); // Load Selected Directory from EEPROM
	loadPlayMode();         // Load Play Mode from EEPROM
	loadPlaylist();         // Load Selected Playlist from EEPROM

	// Count the number of MIDI files in the current directory.  If no MIDI
	// files are found, then go to the Hardware Test menu.
	menuLength = countMidiFiles();
	if (menuLength == 0)
	{
		noMidiFiles = 1;
		newMenu(LM_MENU_HARDWARE_TEST);
		return;
	}

	// Show a Splash Screen (program info, etc.) on program startup.
	splashScreen();

	// Go to the Song Select menu and show list of Songs
	newMenu(LM_MENU_SONG_SELECT);
	listMidiFiles();
}

// ****************************************************************************
// loop() - Main program loop function
// ****************************************************************************
void loop()
{
	// If in Song Play mode, then call the MIDI Play Loop
	if (menuCurrent == LM_MENU_SONG_PLAYING)
	{
		playMidiLoop();
		if (stopMidi) stopMidiPlay();
		return;
	}

	// Otherwise, call the Menu Selection loop function.
	menuSelectLoop();
}

// ****************************************************************************
// startMidiPlay() - Called to start playing a new MIDI file.  This is called
//    either when the user presses the button to start MIDI Play, or after a
//    song has finished playing and there is another one in the list.
// ****************************************************************************
uint8_t startMidiPlay()
{
	// Read the index of the song to be played from EEPROM
	currentSong = getSelectedSong(selectedSongIndex);
	
	// If there is an active playlist, then select the MIDI file that is the
	// (currentSong)th song in the Playlist file.  Otherwise, select the 
	// file that is the (currentSong)th MIDI file in the directory.
	if(playlistFound)
		selectPlaylistMidiFileByIndex(currentSong);
	else
		selectMidiFileByIndex(currentSong);

	// Read the ini file that contains  info on the MIDI file (Title,
	// Author, Play Length, etc.).  Write this info to the LCD.
	readMidiFileProperties();

	// Update the song playing time to the LCD.
	lcdUpdatePlayTime();
	
	// Set the MIDI Filename
	SMF.setFilename(midiFileFullPath);
	
	// Store last time of a MIDI event.  This is used to stop the MIDI file if
	// there is too long without any new MIDI events.
	lastMidiNoteTime = millis();

	// Open the MIDI File.
	if (SMF.load() == -1)
	{
		// If MIDI File opens successfully, then mover to Song Play mode.
		rotaryDisabled = 1; // Disable the rotary Encoder
		menuCurrent = LM_MENU_SONG_PLAYING;

		// Time that MIDI play started.  Used to calculate elapsed play time.
		midiPlayStartTime = millis(); 
		stopMidi = 0;
		return 1;
	}
	else
	{
		// If there is an Error opening the MIDI file, then show a message on
		// the LCD before going back to song selection mode.
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Could not open MIDI:"));
		lcd.setCursor(0, 1);
		lcd.print(midiFileFullPath);
		delay(3000);  // Pause 3 seconds
		return 0;
	}
}

// ****************************************************************************
// stopMidiPlay() - Called either when the MIDI file reaches the end, or
//    when the user manually stops the play.  If the MIDI file has reached the
//    end, then play the next song in the list, if in Sequence or Shuffle
//    play mode.  If the user manually stops the song play, or if there are no
//    more songs in the list, then return to Song Selection mode.
// ****************************************************************************
void stopMidiPlay()
{
	// Close the MIDI file
	SMF.close();

	// If there are more songs remaining in the selected song list
	// then move to the next song.
	// If stopMidi=2, then user has pressed the button to stop
	// If stopMidi=1, then the MIDI file has reached the end.
	if (selectedSongIndex < selectedSongCount - 1 && stopMidi<2)
	{
		delay(3000); // Pause 3 seconds between songs
		selectedSongIndex++;
		
		// Attempt to start the MIDI play for the next song in the list.
		// If there is an error, then move to the following song and
		// attempt to start the MIDI play.
		while (!startMidiPlay())
		{
			selectedSongIndex++;
			if (selectedSongIndex == selectedSongCount) return;
		}
		return;
	}

	// If in Sequential Mode and end of Song List has been reached, 
	// then show Song Select Menu with the first song currently selected.
	// Otherwise, the Song Select Menu will be shown with a selection
	// of the last song played.
	if (stopMidi == 1 && playMode == PLAY_MODE_SEQUENTIAL) currentSong = 0;

	// Enabled the Rotary Encoder to navigate the menu
	rotaryDisabled = 0;

	// Go to the Song Selection menu, with a default selection of currentSong
	newMenu(LM_MENU_SONG_SELECT, currentSong);
}

// ****************************************************************************
// playMidiLoop() - Main Loop function while MIDI files is playing.
// ****************************************************************************
void playMidiLoop()
{
	// Called in a Loop function as fast as possible to read through the MIDI
	// file and generate MIDI events in real time.  Events are passed back to
	// the main program via callback.
	SMF.getNextEvent();

	// Check for button press
	btnRot.tick();

	// Update the elapsed play time on the LCD, if it has changed
	lcdUpdatePlayTime();

	// Stop playing the MIDI file if more than SONG_PLAY_MAX_IDLE_TIME seconds
	// have elapsed since the last MIDI event.  This is needed because some
	// MIDI files have an End of Track / End of File marker much after the
	// last actual MIDI events.
	if (idleTimeSec() >= SONG_PLAY_MAX_IDLE_TIME) stopMidi = 1;
	
	// Uncomment this line to play the MIDI file only for 10 seconds.  This is
	// used to test the program, to verify that the Sequence and Shuffle modes
	// work correctly and that songs play in the right order.
	// if (elapsedPlayTime > 10) stopMidi = 1;

	// Stop the MIDI play when the End of File is detected.
	if (SMF.isEOF()) stopMidi = 1;
}

// ****************************************************************************
// lcdUpdatePlayTime() - Write the elapsed song play time, and the total
//   song duration (if available) to the LCD.
// ****************************************************************************
void lcdUpdatePlayTime()
{
	// Do nothing if there is already an update operation active
	if (lcdSongPlayUpdating) return;
	
	// Get elapsed play time in seconds.  If elapsed play time has not changed
	// by at least 1 second, then exit the function.
	elapsedPlayTime = (millis() - midiPlayStartTime) / 1000;
	if (elapsedPlayTime == oldElapsedPlayTime) return;

	// Set lcdSongPlayUpdating so that this function cannot execute a second
	// time before the current execution finishes.
	lcdSongPlayUpdating = 1;

	// If total song length is available, then print both the elapsed play time
	// and the total song length.
	if (songLength > 0)
	{
		lcd.setCursor(4, songPlayTimeLcdRow);
		time2LCD(elapsedPlayTime);
		lcd.print(F(" / "));
		time2LCD(songLength);
	}

	// Otherwise, print just the elapsed play time.
	else
	{
		lcd.setCursor(8, songPlayTimeLcdRow);
		time2LCD(elapsedPlayTime);
	}

	// Store the last elapsedPlayTime for next call of this function
	oldElapsedPlayTime = elapsedPlayTime;
	lcdSongPlayUpdating = 0;
}

// ****************************************************************************
// time2LCD() - Write the total seconds time "tot_time_sec" to the LCD
//    in the format hh:mm:ss or mm:ss
// ****************************************************************************
void time2LCD(long tot_time_sec)
{
	long t;
	uint8_t time_hr, time_min, time_sec;

	// Convert the total time in seconds to hours, minutes, and seconds.
	t = tot_time_sec;
	time_sec = t % 60;
	t = (t - time_sec) / 60;
	time_min = t % 60;
	t = (t - time_min) / 60;
	time_hr = t;

	// Write the "hh" part only if the time is 1 hour or longer
	if (time_hr > 0)
	{
		lcd.print(time_hr);
		lcd.print(F(":"));
		if (time_min < 10) lcd.print(F("0"));
	}

	// Write the "mm:ss" part
	lcd.print(time_min);
	lcd.print(F(":"));
	if (time_sec < 10) lcd.print(F("0"));
	lcd.print(time_sec);
}

// ****************************************************************************
// menuSelectLoop() - Loop function while in Menu Select mode.
//   This function updates the LCD and checks for a button press.
//   The function does not actually update the LCD until either the Rotary 
//   Encoder is no longer turning, or the Rotary Encoder has turned as many
//   detents as there are rows in the LCD.
// ****************************************************************************
void menuSelectLoop()
{
	// Do not detect any rotary detents while processing the menu selection
	rotaryDisabled = 1;

	// Checks to see if the rotary encoder has turned at least 
	// BYM_LCD_ROW_COUNT detents.  If yes, update the LCD regardless of
	// whether there are still more detents.  This ensures that while the
	// rotary encoder is turning in the same direction, every menu line will
	// be shown on the LCD.
	if (rotaryDetentCount >= BYM_LCD_ROW_COUNT) {
		rotaryDetentCount = 0;
		rotaryTurned = 0;
	}

	// Update LCD, check for button press, and check max idle time, only if
	// the rotary encoder is no longer being turned.
	if (!rotaryTurned)
	{		
		updateLcd();
		btnRot.tick();
		menuIdle();
	}
	
	// Start checking for rotary detents again, prior to the "delay"
	rotaryTurned = 0;
	rotaryDisabled = 0;

	// Rotary detents are detected via interrupt functions during the "delay"
	delay(50);	
}

// ****************************************************************************
// menuIdle() - Called continuously while in Menu Selection mode.  After 
//   MENU_MAX_IDLE_TIME seconds have elapsed, go to the Song Select menu.
// ****************************************************************************
void menuIdle()
{
	// No action if MENU_MAX_IDLE_TIME seconds have not yet elapsed
	if (idleTimeSec() < MENU_MAX_IDLE_TIME) return;

	// No action if no MIDI files were found at startup
	if (sdInitFail || noMidiFiles) return;

	// No action if already at the Song Select menu
	if (menuCurrent == LM_MENU_SONG_SELECT) return;

	// If the current menu was either the Playlist Selection or Directory
	// Selection, then accept the currently selected Playlist or Directory
	// before going to the Song Select menu
	if (menuCurrent == LM_MENU_PLAYLIST_SELECT) playlistSelected();
	if (menuCurrent == LM_MENU_DIRECTORY_SELECT) directorySelected();

	// Return to Song Select menu.
	newMenu(LM_MENU_SONG_SELECT, currentSong);
}

// ****************************************************************************
// songPlayStart() - Called at the beginning of Song Play.
//    Determines the selected songs to be played, based on the Play Mode:
//    Single     = Play only the selected song
//    Sequential = Play Songs in order, started with selected song
//    Shuffle    = Play Songs in Random order, but start with selected song
// ****************************************************************************
void songPlayStart()
{
	uint8_t i;

	// Check for the Play Mode Selected
	switch (playMode)
	{
	case PLAY_MODE_SINGLE: //Single - Play only the currently selected song
		selectedSongCount = 1;
		selectedSongIndex = 0;
		setSelectedSong(0, selectedSongFileIndex);
		break;

	case PLAY_MODE_SEQUENTIAL: // Sequential - Play songs in order, starting with the selected song, and ending with the last song on the playlist
		selectedSongCount = songCount - selectedSongFileIndex;
		selectedSongIndex = 0;
		for (i = 0; i < selectedSongCount; i++) setSelectedSong(i, selectedSongFileIndex + i);
		break;

	case PLAY_MODE_SHUFFLE: // Shuffle - Play all songs from playlist in Random order
		selectedSongCount = songCount;
		selectedSongIndex = 0;
		setSelectedSong(0, selectedSongFileIndex);  // First song is the one currently selected
		setSelectedSongRandom(1);
		break;
	}
}

// ****************************************************************************
// selectFileByIndex() - Finds the (index)th MIDI file in the current directory
//    Order is MIDI files is determined by the File System.  Stores the full
//    path of this MIDI file in midiFileFullPath.
// ****************************************************************************
void selectMidiFileByIndex(uint8_t index)
{
	uint8_t i;
	midiDirectory = SD.open(currentDirectory);
	for (i = 0; i <= index; i++) openNextMidiFile();
	strcpy(midiFileFullPath, currentDirectory);
	midiFile.getFilename(&(midiFileFullPath[strlen(currentDirectory)]));
	midiDirectory.close();
}

// ****************************************************************************
// selectPlaylistMidiFileByIndex() - playlistFile is a text file containing a
//   list of MIDI files to display on the LCD.  This function reads the
//   (index)th line from this file and stores the full path name of this file
//   in midiFileFullPath.
// ****************************************************************************
void selectPlaylistMidiFileByIndex(uint8_t index)
{
	uint8_t i;
	playlistFile = SD.open(playlistFileFullPath);

	if (index > 0)
		for (i = 0; i < index; i++) textFileReadInt(&playlistFile);

	strcpy(midiFileFullPath, currentDirectory);
	textFileReadString(&playlistFile, &midiFileFullPath[strlen(currentDirectory)], 
		FULL_PATH_FILE_LENGTH - strlen(currentDirectory));
	playlistFile.close();
}

// ****************************************************************************
// setSelectedSong() - Sets the index of a song selected to be played.
//     List of selected songs is stored in EEPROM.
// ****************************************************************************
void setSelectedSong(uint8_t index, uint8_t playlistIndex)
{
	EEPROM.write(SELECTED_SONGS_ADDRESS + index, playlistIndex);
}

// ****************************************************************************
// getSelectedSong() - Reads the index of a song selected to be played.
//     List of selected songs is stored in EEPROM.
// ****************************************************************************
uint8_t getSelectedSong(uint8_t index)
{
	return EEPROM.read(SELECTED_SONGS_ADDRESS + index);
}

// ****************************************************************************
// setSelectedSongRandom() - Sets the selected songs to be played in
//   random order.  This is used for "Shuffle" mode.
// ****************************************************************************
void setSelectedSongRandom(uint8_t startIndex)
{
	uint8_t i, j, nextIndex, alreadySelected;

	// Initialize the Random Number Generator with the current time
	randomSeed(millis());

	for (i = startIndex; i < songCount; i++)
	{
		do
		{
			nextIndex = random(0, songCount);
			alreadySelected = 0;
			for (j = 0; j < i; j++)
			{
				if (getSelectedSong(j) == nextIndex) alreadySelected = 1;
			}
		} while (alreadySelected);
		setSelectedSong(i, nextIndex);
	}
}


