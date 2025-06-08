/* 
 * Aunnciator:  Software to subscribe to a Particle event and play an associated audio clip.
 *
 *  Uses the Animatronic Mouth circuit board to play a voice clip associated with a Particle
 *  event.  See https://github.com/TeamPracticalProjects/Animatronics for details of the
 *  Mouth circuit board.  Note: the Animatronic Mouth circuit board contains analog
 *  processing circuitry to shape and extract the envelope of the audio clip.  This
 *  analog circuitry is not used for this project and need not be populated on the circuit
 *  board.
 * 
 *  This project was developed to play a voice clip associated with a Hub publication
 *  from the LoRa sensor project; see: https://github.com/TeamPracticalProjects/LoRa.
 *  However, it may be used in conjunction with any publisher of Particle events.  When used
 *  in conjunction with the LoRa sensor project, the published event data is parsed to determine
 *  the proper clip to play (based upon the sensor ID).  This event data parsing and clip 
 *  determination is, obviously, application dependent.
 * 
 *  This project uses the DFRobot miniMP3 player to play recorded MP3 voice clips.  An inexpensive
 *  source for this module is:
 *       https://www.amazon.com/dp/B08V8G1TQZ?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1
 * 
 *  The voice clips must be placed in an /MP3/ folder under the root of a micro SD card (32 GB or less).
 *  The clip to play is designated by the 4 digit number at the beginning of the file name of 
 *  the voice clip in this folder.  In order to ensure a proper mapping of clip numbers to audio 
 *  clip files, the file name of each clip should begin with a 4 digit number (e.g. 0000clipA.mp3, 
 *  0001clipB.mp3, ...). 
 * 
 *  This project uses a Particle Photon 1, which is currently deprecated in favor of Photon 2.
 *  Particle sells a Photon 2 to Photon 1 adator socket, which may be suitable for this project.
 *  However, it should be noted that the DFRobotDFPlayerMini library used has some bugs in it;
 *  specifically, several functions with non-void return values are lacking return statements.
 *  Particle OS versions above 3.0.0 treat these bugs as fatal errors and won't compile the
 *  code.  Particle OS 3.0.0 treats these errors as warnings and compiles just fine.  (The library
 *  functions with these errors in them are not used in this project.) As a consequence of these bugs,
 *  the code cannot be compiled for Photon 2 until either (a) the library bugs are fixed, or (b)
 *  Particle supplies a build option that treats these errors as warnings, or (c) a different
 *  library is adopted (which will likely change the code).
 * 
 *  In addition to the populated circuit board, the following external conponents are used
 *  in this project.
 * 
 *      - Either a small speaker (< 3 watts) or an amplified playback device that connects via
 *          a stereo headphone jack.
 * 
 *      - A RED LED, controlled by Photon pin D5, that indicates that the device is running
 *          and connected to the Particle cloud.
 * 
 *      - a GREEN LED, controlled by Photon pin D6, which may be the backlight on the pushbutton.
 *          It indicates that a clip is currently being played. 
 * 
 *      - a momentary pushbutton, read from Photon pin A3, which can be pressed to replay the
 *          current audio clip. 
 * 
 *  A number of Particle Cloud variables and functions are included for testing purposes.
 *  These may be accessed using the Particle Console or a dedicated app:
 *      - Cloud variables:
 *          * FIRMWARE VERSION:  displays the version of the firmware that is installed on
 *              the Photon device.
 *          * MASTER VOLUME CONTROL: displays the current master volume control setting: 0 - 100%
 *          * CURRENT CLIP NUMBER: displays the number of the clip that is the current clip to be 
 *              played.
 *          *EVENT DATA: displays the data string from the last event that triggered clip playing
 *              (useful for debugging event data parsing).
 * 
 *      - Cloud functions:
 *          * SET MASTER VOLUME LEVEL:  accepts a value 0 - 100 (%) so that a particular
 *              annunciator device's volume can be set to be compatible with the installed
 *              environment.
 *          * PLAY CLIP: plays the clip whose number is the argument to this function.  NOTE:
 *              this is the actual ordinal number of the clip in the /MP3/ folder on the SD
 *              card and not the number of the sensor from the event data that maps to a clip.
 *          * TRIGGER DEVICE NUMBER:  plays the clip associated with the device number. This 
 *              simulates receiving an event from a sensor with the specified device number
 *              set in its jumpers.
 *
 * version 1.0.2 6/8/2025
 *      - expanded ClipsList with 4 new clips for MN MOD announcements
 *  
 * version 1.0.1 5/30/2025
 *      - added cloud function to trigger a clip based upon the device number
 *      - fixed button push when no previous clip has been played
 * 
 *  version 0.9.9 (pre-release); by Bob Glicksman; 3/25/25
 *      - this version is fully working, but needs refinements before release
 *      - (1)  move the clipList and offset into their own header file for easier editing
 *      - (2)  persist vulume setting in EEPROM
 * 
 *  version 1.0.0 (first released version); by Bob Glicksman; 4/12/25
 *      - added:  (1) volume setting persisted in Photon 1 emulated EEPROM.  The volume
 *          is set to this value in setup();  (2) moved the list of clips to play out to
 *          an external header file.
 * 
 *  (c) 2025, Team Practical Projects, Bob Glicksman, Jim Schrempp.  All rights reserved.
 * 
 */

#define VERSION "1.0.2"

// NOTE:  MUST USE PARTICLE OS VERSION 3.0.0 OWING TO BUGS IN MINI MP3 PLAYER LIBRARY.
  //    Specifically, some functions have non-void return value declared but no return statement.
  //    OS versions above 3.0.0 won't treat this as a warning anymore; build will fail.

// Include Particle Device OS APIs
#include "Particle.h"

// Include the list of the clips to play based upon the message index
#include "ClipsList.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
// SYSTEM_THREAD(ENABLED);

// Defined constants for the program
#define RED_LED_PIN D5
#define GREEN_LED_PIN D6
#define STATUS_LED_PIN D7
#define BUSY_PIN D2
#define BUTTON_PIN A3
#define DEFAULT_VOLUME_LEVEL 70 // the volume level is 0% to 100%

const unsigned long BUSY_WAIT = 1000UL;  // Busy pin wait time
const unsigned long DEBOUNCE_TIME = 10UL;  // time for button debouncing

const uint8_t FIRST_CLIP_NUM = 11; // just for testing
const uint8_t LAST_CLIP_NUM = 15;  // just for testing

// Global Variables
  // define main state variable states
enum StateVariable {
    idle,
    triggered,
    clipWaiting,
    clipPlaying,
    clipComplete,
    clipEnd,
    paused
};

  // define enumerated state variable for buttonPressed() function
enum ButtonStates {
    buttonOff,    // the button is not pressed
    pressedTentative,   // button seems to be pressed, need debounce verification
    buttonOn, // button remains pressed but don't indicate true anymore
    releasedTentative  // button seems to be released, need verificaton
};

  // Other globals
int relativeVolumeControl = DEFAULT_VOLUME_LEVEL;  // value between 0 and 100 (%); Preset to 70%
int currentClip = 0;    // number of the last clip played
bool newClip2Play = false;  // set to true to indicate that there is a new clip to play
bool greenLEDFlash = false; // set to true to start the green LED flashing; false to stop it.
String version = VERSION;

String eventDataString = "";    // string to hold event data for debugging

// Include the mp3 player library
#include <DFRobotDFPlayerMini.h>

// Create an instance of the miniMP3 player
DFRobotDFPlayerMini miniMP3Player;

/****************************** CLOUD FUNCTIONS ************************************/
// Cloud function to set the master volume level
int setVolume(String volumeControl) {
    int volume = volumeControl.toInt();
    if (volume < 0) {
        relativeVolumeControl = 0;
    } else if (volume > 100) {
        relativeVolumeControl = 100;
    } else {
        relativeVolumeControl = volume;
    }
    
    // place the volume level into EEPROM
    EEPROM.put(0, relativeVolumeControl);

    return 0;
}   // end of setVolume()

// Cloud function to play a specified clip
int playClip(String clipNumber) {
    int clipNum = clipNumber.toInt();

    // bound the clip number provided to some resonable limits
    if(clipNum < 0) {
        currentClip = FIRST_CLIP_NUM;
    } else if(clipNum > 256) {
        currentClip = LAST_CLIP_NUM;
    } else {
        currentClip = clipNum;
    }
    
    newClip2Play = true;    // play the clip
    return 0;

}   // end of playClip()

// Cloud function to impersonate a particular device number
int triggerDeviceNumber(String deviceNumber) {
    int devNum = deviceNumber.toInt();
    currentClip = ERROR_CLIP_NUM; 

    devNum -= BEGIN_DEV_NUM;    // subtract the base device number to get the index into the clipList
    // bound the device number provided 
    if((devNum >= 0) & (devNum < MAX_NUM_CLIPS)) {
        currentClip = clipList[devNum];
    } 
    // select and play the clip
    newClip2Play = true;    // play the clip
    return devNum;

}

/************************** OTHER FUNCTIONS CALLED BY SETUP OR LOOP ***************************/
// function to flash the green LED rapidly (called from a non-blocking loop())
void flashLED() {
    #define BLINK_TIME 100  // turn on and off every 100 ms
    static unsigned long timeInState = millis();
    static bool onOff = false;

    if(greenLEDFlash == false) {    // do not flash the green LED; turn it off
        digitalWrite(GREEN_LED_PIN, LOW);
    } else {    // flash the green LED
        if((millis() - timeInState) > BLINK_TIME) { // time to toggle the LED state
            if(onOff == false) {
                digitalWrite(GREEN_LED_PIN, HIGH);
                onOff = true;
            } else {
                digitalWrite(GREEN_LED_PIN, LOW);
                onOff = false;
            }
            timeInState = millis();
        }

    }
}   // end of flashLED()

// function to perform non-blocking reading and debouncing of a pushbutton switch
bool buttonPressed() {
    static ButtonStates _buttonState = buttonOff;
    static unsigned long lastTime = millis();

    switch(_buttonState) {

    case buttonOff:
        if(digitalRead(BUTTON_PIN) == HIGH) { // button not pressed
            _buttonState = buttonOff;
            return false;
        }
        else {  // button is pressed, need to debounce and verify
            lastTime = millis();  // set up the timer'
            _buttonState = pressedTentative;
            return false;
        }

    case pressedTentative:   // button seems to be pressed, need debounce verification
        if(digitalRead(BUTTON_PIN) == HIGH) { // button not pressed
            _buttonState = buttonOff;
            return false;
        }
        else {  // button is pressed
            if( (millis() - lastTime < DEBOUNCE_TIME)) { // button not yet debounced
                _buttonState = pressedTentative; 
                return false;
            }
            else {  // button is debounced
                _buttonState = buttonOn;
                return true;  // tell caller that the button has been presed
            }
        }

    case buttonOn: // button remains pressed but don't indicate true anymore
        if(digitalRead(BUTTON_PIN) == LOW)  { // button remains pressed, stay here
            _buttonState = buttonOn;
            return false;
        }
        else {  // button tentatively released, need to verify
            lastTime = millis();  // set timer for debounce
            _buttonState = releasedTentative;
            return false;
        }

    case releasedTentative:  // button seems to be released, need verificaton
        if(digitalRead(BUTTON_PIN) == HIGH) { // button still released
            if( (millis() - lastTime < DEBOUNCE_TIME)) {  // not yet debounced
                _buttonState = releasedTentative;
                return false;
            }
            else {  // debounced and verified released
                _buttonState = buttonOff;
                return false;
            }
        }
        else {  // false reading, button still pressed
            _buttonState = buttonOn;
            return false;
        }

    default:
        if(digitalRead(BUTTON_PIN) == HIGH) { // button not pressed
            _buttonState = buttonOff;
            return false;
        }
        else {  // button is pressed
            _buttonState = pressedTentative;
            return false;
        }
  }

} // end of buttonPressed()

// function to parse the event data to extract the device Number
//  NOTE: this function is SPECIFIC to the LoRa sensor data message format, as of 3/24/25.
//  The function must be modified if any change to the message is made.  Current message format is:
//    message=TESTOK|deviceNum=11|payload=G m: 1 uid: 002D001104674C63000055FE|SNRhub1=10|RSSIHub1=-29
//  The deviceNum is any number between 5 and 12.  Thus, only 2 chars are processed into a number.

// #define DEBUG   // comment out to ignare debugging Serial printing

unsigned int eventDatParse(String evData) {
    unsigned int _indexNum;
    unsigned int _devNum;

    // find the index of "deviceNum ="
#ifdef DEBUG
    Serial.print("Message string is: ");
    Serial.println(evData);
#endif

    _indexNum = evData.indexOf("deviceNum=");
    _indexNum += 10;

#ifdef DEBUG
    Serial.print("Position of device number is:  ");
    Serial.println(_indexNum);
#endif

    evData.remove(0, _indexNum);    // remove the leading part
    evData.remove(2);   // remove all but the first two characters

#ifdef DEBUG
    Serial.print("The edited data is: ");
    Serial.println(evData);
#endif

    _devNum = evData.toInt();

#ifdef DEBUG
    Serial.print("The returned data is: ");
    Serial.println(_devNum);
#endif

    return _devNum;
} // end of eventDatParse()


/*********************** EVENT HANDLER FOR THE EVENT SUBSCRIPTION ***************************/
// event handler for subscription to hub publication event
void particleCallbackEventPublish(const char *event, const char *data) {
    String _eventName = String(event);
    String _eventData = String(data);
    unsigned int _devNumber;

    // get the event data
    eventDataString = "";
    eventDataString += _eventData;

    // parse the event data
    _devNumber = eventDatParse(eventDataString);

    // subtract the device number if ADR jumpers are all in.
    // This is the index into the clipList array of clips to play
    _devNumber -= BEGIN_DEV_NUM;    

    // select and play the clip
    currentClip = clipList[_devNumber];
    newClip2Play = true;    // play the clip

}   // end of particleCallbackEventPublish()


/****************************** SETUP ************************************/
// setup() runs once, when the device is first turned on
void setup() {
    // Photon pin definitions
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(BUSY_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Cloud variables and funtions
    Particle.variable("Firmware Version", version);
    Particle.variable("Master Volume Control", relativeVolumeControl);
    Particle.variable("Current Clip Number", currentClip);
    Particle.variable("Event Data", eventDataString);

    Particle.function("Set Master Volume Level", setVolume); // call to set master volume level
    Particle.function("Play Clip", playClip);
    Particle.function("Trigger Device Number", triggerDeviceNumber); // call to play a clip based on device number

    // subscribe to the published Particle event that triggers a clip playback
    Particle.subscribe("LoRaHubLogging", particleCallbackEventPublish, MY_DEVICES);

    // initialize serial ports and mini MP3 player
    Serial.begin(9600);
    Serial1.begin(9600);
    miniMP3Player.begin(Serial1);
    miniMP3Player.volume(30);       // set the max volume - it may be changed when playing a clip

    // set initial state of green LED
    digitalWrite(GREEN_LED_PIN, LOW);

    // read the last stored volume value from EEPROM
    int vol;
    EEPROM.get(0, vol);

    // clamp the volume levels between 0 and 100 in case EEPROM not initialized
    if ((vol < 0) || (vol > 100)){  // EEPROM not initialized, use default
        vol = DEFAULT_VOLUME_LEVEL;
    } 
    relativeVolumeControl = vol;

    // signal end of setup
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(1000);
    digitalWrite(STATUS_LED_PIN, LOW);

    digitalWrite(RED_LED_PIN, HIGH);    // indicate powered, connected to WiFi and ready

}   // end of setup()

/****************************** LOOP ************************************/
// loop() runs over and over again, as quickly as it can execute.
void loop() {
    static unsigned long busyTime = millis();
    static StateVariable state = idle; // begin in the idle state

    // flash the green LED in non-blocking manner
    flashLED();

    // test the replay button in a non-blocking manner
    if(buttonPressed() == true) {
        /*
        if ((currentClip < 0) | (currentClip >= MAX_NUM_CLIPS)) { // make sure the clip is in range
            currentClip = NO_PREVIOUS_ANNOUNCEMENT; // play this clip if no previous announcement
        } 
        */
        newClip2Play = true;    // set the flag to play the current clip at the correct state
    }

    // state machine to play a clip and flash LED
    switch(state) {
        case idle:  // wait for event or other trigger
            if (digitalRead(BUSY_PIN) == HIGH)  {  // make sure MP3 player is ready
                if(newClip2Play == true) {   // we must play the new clip
                    greenLEDFlash = true; // signal a new value
                    busyTime = millis();    // start the time for the next state
                    state = triggered;
                } else {
                    state = idle;
                } 
                
            } else {    // stay in idle state until MP3 player busy pin is low (asserted)
                state = idle;
            }
            break;

        case triggered: // we must start playing the clip -- light (flash) the green LED
            // keep the green LED lit (flashing) for a bit, and then start a clip playing
            if((millis() - busyTime) < BUSY_WAIT) { // stay in this state until time to play clip
                state = triggered;
            } else {    // it is time to play the clip
                // set the volume
                float vol = 30 * ((float)relativeVolumeControl/100.0);
                miniMP3Player.volume((int)vol);

                // play the designated clip
                miniMP3Player.playMp3Folder(currentClip);
                state = clipWaiting;
            }
            break;

        case clipWaiting:   // wait fop MP3 player busy to assert (low)
            if(digitalRead(BUSY_PIN) == HIGH) {  // waiting for the clip to begin playing
                state = clipWaiting;
            } else {    // clip is now playing
                state = clipPlaying;
            }
            break;

        case clipPlaying:   // clip is playing, wait for busy to unassert (playing complete)
            if(digitalRead(BUSY_PIN) == LOW) {  // clip is still playing
                state = clipPlaying;
            } else {    // clip complete - start time for keeping green LED on (flashing)
                busyTime = millis();
                state = clipComplete;
            }
            break;

        case clipComplete:  // clip has finished, keep flashing the GREEN LED for a while
            if((millis() - busyTime) < BUSY_WAIT) { // keep green LED high (flashing)
                state = clipComplete;
            } else { // turn green LED off (stop flashing)
                greenLEDFlash = false;
                state = clipEnd;
            }
            break;

        case clipEnd:   // flag that no clip is active
            newClip2Play = false; // XXX can't command replay until here -- should this be sooner??
            state = idle;
            break;

        default:    // the next state is idle
            newClip2Play = false;
            state = idle;
            break;

    } // end state machine

}   // end of loop()




