/*Nano*/
/*Eigen voeding voor TM1638, en USB-voeding voor Nano niet via de monitor helpt voor minder ruis*/
/*
 * si5351_example.ino - Simple example of using Si5351Arduino library
 *
 * Copyright (C) 2015 - 2016 Jason Milldrum <milldrum@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "si5351.h"
#include "Wire.h"
#include "SN76489.h"
#include "Midi.h"
#include "TM1638lite.h"

/***************************************************************************
***** Directly interface SN76489 IC with the following PIN definitions *****
***** and by calling 8-bit constractor                                 *****
***** The SN76489 pinout considered for this library is as follows:    *****
*****                                                                  *****
*****                        ========                                  *****
*****        D2       --> [ 1]  ()  [16] <-- VCC                       *****
*****        D1       --> [ 2]      [15] <-- D3                        *****
*****        D0       --> [ 3]  7   [14] <-- CLOCK OSC                 *****
*****     READY       <-- [ 4]  6   [13] <-- D4                        *****
*****    NOT WE       --> [ 5]  4   [12] <-- D5                        *****
*****    NOT CE       --> [ 6]  8   [11] <-- D6                        *****
***** AUDIO OUT       <-- [ 7]  9   [10] <-- D7                        *****
*****       GND       --> [ 8]      [ 9] --- N.C.                      *****
*****                        ========                                  *****
***************************************************************************/

//#define FREQUENCY 4286819ULL
#define FREQUENCY 2143409ULL
#define LED 13

#define PIN_NotCE0 8
#define PIN_NotCE1 2
#define PIN_NotCE2 14

#define PIN_NotWE 9
#define PIN_D0 11
#define PIN_D1 12
#define PIN_D2 10
#define PIN_D3 7
#define PIN_D4 6
#define PIN_D5 5
#define PIN_D6 4
#define PIN_D7 3

#define TM_STROBE 15
#define TM_CLOCK 16
#define TM_DATA 17

#define DETUNE1 A7
#define DETUNE2 A6

#define DETUNE_NOISE 16
#define DETUNE_MEAN 8
//impliciet bij een NANO
//Si5351_SDA 18; A4
//Si5351_SCL 19; A5
//MIDI_IN RX 0

//nog over: 1 (TX) 13 (LED) 

SN76489 mySN76489 = SN76489(PIN_NotWE, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6, PIN_D7, FREQUENCY);
byte whichSN[3] = {PIN_NotCE0, PIN_NotCE1, PIN_NotCE2};

Si5351 si5351;
TM1638lite tm(TM_STROBE, TM_CLOCK, TM_DATA);
MIDI_CREATE_DEFAULT_INSTANCE();

#define MIDI_NUMBER 53
#define MIDI_LOW 48
//C#-code to calculate midinote => divider
//freq = Math.Pow(2, (midiNote-69.0)/12.0) * 440.0
//notediv[midiNote - MIDI_LOW] = Math.Round(FREQUENCY/(32.0 * freq), MidpointRounding.AwayFromZero)
uint16_t noteDiv[MIDI_NUMBER] = {
1024,967,912,861,813,767,724,683,645,609,575,542
,512,483,456,431,406,384,362,342,323,304,287,271
,256,242,228,215,203,192,181,171,161,152,144,136
,128,121,114,108,102,96,91,85,81,76,72,68
,64,60,57,54,51};

long detune1, detune2;

#define MAX_POLYPHONY 9
#define MAX_NOTES 2 * MAX_POLYPHONY
byte notes[MAX_POLYPHONY];
byte notesInOrder[MAX_NOTES];
unsigned long times[MAX_POLYPHONY];
char buffer[8];
byte polyphony;
byte highestNoteInOrder;
byte detune_index;
int reading1[DETUNE_MEAN];
int reading2[DETUNE_MEAN];

void displayOn(int input1, int input2, int msg){
  sprintf(buffer, "* %d %d %d", input1, input2, msg);
  tm.displayText(buffer);
  tm.setLED(input1, true);
}

void displayOff(int input1, int input2){
  sprintf(buffer, "- %d %d", input1, input2);
  tm.displayText(buffer);
  tm.setLED(input1, false);
}

void noteOn(byte index, byte msg, byte polyphony){
  if (polyphony > 3){
    digitalWrite(whichSN[index % 3], false);
    mySN76489.setDivider(index / 3, noteDiv[msg - MIDI_LOW]);
    mySN76489.setAttenuation(index / 3, 0x0);
    digitalWrite(whichSN[index % 3], true);
    displayOn(index, index /3, msg);
  } else {
    digitalWrite(whichSN[0], false);
    digitalWrite(whichSN[1], false);
    digitalWrite(whichSN[2], false);
    mySN76489.setDivider(index % 3, noteDiv[msg - MIDI_LOW]);
    mySN76489.setAttenuation(index % 3, 0x0);
    digitalWrite(whichSN[0], true);
    digitalWrite(whichSN[1], true);
    digitalWrite(whichSN[2], true);
    displayOn(index % 3, index /3, msg);
  }
}

void noteOff(byte index, byte polyphony){
  if (polyphony > 3){
    digitalWrite(whichSN[index % 3], false);
    mySN76489.setAttenuation(index / 3, 0xf);
    digitalWrite(whichSN[index % 3], true);
    displayOff(index, index / 3);
} else {
    digitalWrite(whichSN[0], false);
    digitalWrite(whichSN[1], false);
    digitalWrite(whichSN[2], false);
    mySN76489.setAttenuation(index % 3, 0xf);
    digitalWrite(whichSN[0], true);
    digitalWrite(whichSN[1], true);
    digitalWrite(whichSN[2], true);
    displayOff(index % 3, index / 3);
  }
}

void AllOff(){
  tm.displayText("ALL OFF");
  digitalWrite(PIN_NotCE0, false);
  digitalWrite(PIN_NotCE1, false);
  digitalWrite(PIN_NotCE2, false);
  mySN76489.setAttenuation(0, 0xF);
  mySN76489.setAttenuation(1, 0xF);
  mySN76489.setAttenuation(2, 0xF);
  mySN76489.setAttenuation(3, 0xF);
  digitalWrite(PIN_NotCE0, true);
  digitalWrite(PIN_NotCE1, true);
  digitalWrite(PIN_NotCE2, true);
}

void setup()
{
  bool i2c_found;

  // Start serial and initialize the Si5351
  //Serial.begin(115200);
  //Serial.println("");
  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found)
  {
    //Serial.println("Device not found on I2C bus!");
  } else {
    //Serial.println("YES! Device found on I2C bus!");
  }

  // Set CLK0 to output 3.99 MHz
  //si5351.set_freq(399999990ULL, SI5351_CLK0);
  si5351.set_freq(100 * FREQUENCY, SI5351_CLK0);
  si5351.set_freq(101 * FREQUENCY, SI5351_CLK1);
  si5351.set_freq(102 * FREQUENCY, SI5351_CLK2);

  pinMode(PIN_NotCE0, OUTPUT); 
  pinMode(PIN_NotCE1, OUTPUT); 
  pinMode(PIN_NotCE2, OUTPUT); 

  //AllOff();
  pinMode(LED, OUTPUT); 
  pinMode(DETUNE1, INPUT); 
  pinMode(DETUNE2, INPUT); 

  digitalWrite(LED, true);

  tm.setLED(1, true);
  digitalWrite(PIN_NotCE0, false);
  mySN76489.setDivider(0, noteDiv[0]);
  delay(100);

  tm.setLED(2, true);
  digitalWrite(LED, false);
  mySN76489.setDivider(0, noteDiv[11]);
  delay(100);
  tm.setLED(3, true);
  digitalWrite(LED, true);
  mySN76489.setDivider(0, noteDiv[23]);
  delay(100);

  tm.setLED(4, true);
  mySN76489.setAttenuation(0, 0xF);
  //digitalWrite(PIN_NotCE0, true);

  digitalWrite(LED, false);

  MIDI.begin(MIDI_CHANNEL_OMNI);
  polyphony = 9;
    for (byte i = 1; i <=8; i++ ){
    tm.setLED(i, false);
  }
  AllOff();
  detune_index = 0;
  highestNoteInOrder = 0;
}

void loop()
{
    byte type;
    if (MIDI.read()) {                    
      digitalWrite(LED, true);
      type = MIDI.getType();
      switch (type) {
        case midi::PitchBend: {
          long pitchBend = MIDI.getData2() - 64;
          if (pitchBend > 0){
            si5351.set_freq(100 * FREQUENCY + pitchBend * 100 * FREQUENCY / 64, SI5351_CLK0);
          } else {
            si5351.set_freq(100 * FREQUENCY + pitchBend * 1000000, SI5351_CLK0);
          }
          break;
        }
        case midi::NoteOn: 
        case midi::NoteOff:
          byte noteMsg = MIDI.getData1();
          //velocity = MIDI.getData2();
          //channel = MIDI.getChannel();
          if (MIDI_LOW <= noteMsg && noteMsg < MIDI_LOW + MIDI_NUMBER){
            byte effectiveNoteMsg = noteMsg;
            if (type == midi::NoteOn) {
              if (highestNoteInOrder < MAX_NOTES) {
                notesInOrder[highestNoteInOrder++] = noteMsg;
              } else {
                //alles 1 opschuiven
              }
            } else {
              //note ertussenuit halen
            }
            if (type == midi::NoteOn) {
              bool foundEmpty = false;
              for (byte i = 0; i < polyphony; i++){
                if (notes[i] == 0 && !foundEmpty){
                  notes[i] = effectiveNoteMsg;
                  times[i] = millis();
                  noteOn(i, effectiveNoteMsg, polyphony);
                  foundEmpty = true;
                  break;
                }
              }
              if (!foundEmpty){
                byte oldestIndex = 0;
                unsigned long oldestTime = times[0];
                for (byte i = 1; i < polyphony; i++){
                  if (times[i] < oldestTime){
                    oldestIndex = i;
                    oldestTime = times[i];
                  }
                }
                notes[oldestIndex] = effectiveNoteMsg;
                times[oldestIndex] = millis();
                noteOn(oldestIndex, effectiveNoteMsg, polyphony);
              }
            } else {
              for (byte i = 0; i < polyphony; i++){
                if (notes[i] == effectiveNoteMsg){
                  notes[i] = 0;
                  noteOff(i, polyphony);
                  break;
                }
              }
            }
          }
          break;
      }
      digitalWrite(LED, false);
    }
    reading1[detune_index] = analogRead(DETUNE1);
    int reading_sum = 0;
    for (byte i = 0; i < DETUNE_MEAN; i++){
      reading_sum += reading1[i];
    }
    long detuneNew = (reading_sum - 512 * DETUNE_MEAN) * 100 * FREQUENCY / (DETUNE_MEAN * 512);
    if (abs(detuneNew - detune1) > DETUNE_NOISE){
      detune1 = detuneNew;
      si5351.set_freq(100 * FREQUENCY + detune1, SI5351_CLK1);
      sprintf(buffer, "1 %d", reading_sum);
      tm.displayText(buffer);
    }

    reading2[detune_index] = analogRead(DETUNE2);
    reading_sum = 0;
    for (byte i = 0; i < DETUNE_MEAN; i++){
      reading_sum += reading2[i];
    }
    detuneNew = (reading_sum - 512 * DETUNE_MEAN) * 100 * FREQUENCY / (DETUNE_MEAN * 512);
    if (abs(detuneNew - detune2) > DETUNE_NOISE){
      detune2 = detuneNew;
      si5351.set_freq(100 * FREQUENCY + detune2, SI5351_CLK2);
      sprintf(buffer, "2 %d", reading_sum);
      tm.displayText(buffer);
    }

  detune_index++;
  if (detune_index == DETUNE_MEAN){
    detune_index = 0;
  }
    uint8_t buttons = tm.readButtons();
     switch (buttons) {
      case 1 :
        polyphony = 1;
        tm.displayText("MONO");
        break;
      case 2 :
        polyphony = 3;
        tm.displayText("POLY3");
        break;
      case 4 :
        polyphony = 9;
        tm.displayText("POLY9");
        break;
      case 64 :
        tm.sendCommand(ACTIVATE);
        break;
      case 128 :
        AllOff();
        tm.sendCommand(DISPLAY_OFF);
        break;
      default:
        break;
     }
}
