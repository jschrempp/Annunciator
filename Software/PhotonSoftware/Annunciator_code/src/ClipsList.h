/* ClipsList.h
*   This file defines the list of clips to play.
*
*   The index into the clipList[] array is the device ID - base device ID
*       contained in the message from the LoRa sensor.  The message is part of
*       the published event to the Particle cloud that the Annunciator subscribes to.
*
*   Change this file only if the list of MP3 files to play changes. Do not change
*       code in Annunciator_code.ino if all you are changing is the definition of which
*       MP3 clip to play 
*
*   (c) 2025; Team Practicle Projects, Bob Glicksman, Jim Schrempp
*       All rights reserved.
*/
#include "Particle.h"

const uint8_t ERROR_CLIP_NUM = 3; // clip to play if an error occurs
const uint8_t NO_PREVIOUS_ANNOUNCEMENT = 2; // play this clip if no previous announcement when button pressed

const unsigned int BEGIN_DEV_NUM = 5;   // the device number reported if all ADR jumers are in
// xxx need to move this to a file in common with the sensor code
// xxx which currently uses LORA_TRIP_SENSOR_ADDRESS_BASE in RangeTesSensor.ino

// This is the list of MP3 files to play based upon the numerical prefix of the file name
//  in the /MP/ folder on the mini SD card.

// The index into this array is the device number minus BEGIN_DEV_NUM.  The device number
// is the number in the message from the LoRa sensor.  The message is part of the published
// event to the Particle cloud that the Annunciator subscribes to.  The device number is
// the number of the sensor that triggered the event.  The device number is a two-digit
// number, so the first two characters of the message are the device number. 

const uint8_t MAX_NUM_CLIPS = 12; // number of clips in the list

// 0: 21 Reception Desk
// 1: 22 Wood shop
// 2: 23 Cold Shop
// 3: 26 Lasers 3D
// 4: 27 Hot shop
// 5: 28 Electronics
// 6: 1 sensor 6 has no audio
// 7: 24 Frontdoor
// 8: 200 Close in 30 minutes
// 9: 201 Close in 15 minutes
// 10: 202 Close now
// 11: 203 Testin

unsigned int clipList[] = {21, 22, 23, 26, 27, 28, 1, 24, 200, 201, 202, 203}; // clips to play
