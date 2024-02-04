/* 
 * AXON - MIDI to CV
 * 
 * This program is an adaptation by Arthur LAURENT
 * for the Eurorack module Mental Noise - Axon 
 * https://github.com/atulrnt/mental-noise-axon
 *
 * Rewriten from the orginal version created by Larry McGovern
 * https://github.com/elkayem/midi2cv
 *  
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <MIDI.h>
#include <SPI.h>
#include <EEPROM.h>

/*
 * Note priority is set by pins A0 and A1
 * Last note priority: A0 and A1 high (open)
 * Lowest note priority: A0 low (ground)
 * Highest note priority: A1 low (ground)
 */
#define NOTE_PRIORITY_1 A0
#define NOTE_PRIORITY_2 A1

/*
 * Midi channel is defined in binary 
 * using 4 switches connected to A2-A5
 *
 * 0000 : 1  |  1000 : 9
 * 0001 : 2  |  1001 :10
 * 0010 : 3  |  1010 :11
 * 0011 : 4  |  1011 :12
 * 0100 : 5  |  1100 :13
 * 0101 : 6  |  1101 :14
 * 0110 : 7  |  1110 :15
 * 0111 : 8  |  1111 :16
 */
#define MIDI_CHANNEL_1 A5
#define MIDI_CHANNEL_2 A4
#define MIDI_CHANNEL_3 A3
#define MIDI_CHANNEL_4 A2

// Calibration
#define CALIBRATION_POT A6
#define CALIBRATION_SWITCH A7
#define CALIBRATION_DEFAULT 47.069f
#define CALIBRATION_EEPROM 0

// Output
#define GATE  5
#define CLOCK 4
#define TRIG  3
#define DAC1  9 
#define DAC2  7

// Pulses duration and clock settings
#define TRIGGER_DURATION 20
#define CLOCK_DURATION 20
#define CLOCK_PPQ 24

// 1V/oct calibration range
#define NOTE_MIN_MV 47.001f
#define NOTE_MAX_MV 47.139f

// DAC configuration
#define DAC_PIN_NOTE DAC1
#define DAC_PIN_CONTROL_CHANGE DAC1
#define DAC_PIN_PITCH_BEND DAC2
#define DAC_PIN_VELOCITY DAC2
#define DAC_CHAN_NOTE 0x9000
#define DAC_CHAN_CONTROL_CHANGE 0x1000
#define DAC_CHAN_PITCH_BEND 0x1000
#define DAC_CHAN_VELOCITY 0x9000
#define DAC_GAIN_NOTE 0x0000
#define DAC_GAIN_CONTROL_CHANGE 0x0000
#define DAC_GAIN_PITCH_BEND 0x2000
#define DAC_GAIN_VELOCITY 0x0000

bool isCalibrationActive{false};
float calibrationValue{-1};
bool notes[88]{}; 
int8_t notesOrder[20]{};
int8_t orderIndex{};
unsigned long triggerTimer{};
bool (*noteCommand)(int);

MIDI_CREATE_DEFAULT_INSTANCE();

void setup() 
{
  pinMode(GATE, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DAC1, OUTPUT);
  pinMode(DAC2, OUTPUT);

  digitalWrite(GATE, LOW);
  digitalWrite(TRIG, LOW);
  digitalWrite(CLOCK, LOW);
  digitalWrite(DAC1, HIGH);
  digitalWrite(DAC2, HIGH);

  initCalibration();
  initNotePriority();

  SPI.begin();
  MIDI.begin(getMidiChannel());

  // Set initial pitch bend value (0-127) to mid point
  handlePitchBend(64);
}

int getMidiChannel() 
{
  int channel{1};
  int pins[] = {MIDI_CHANNEL_1, MIDI_CHANNEL_2, MIDI_CHANNEL_3, MIDI_CHANNEL_4};

  for (int i = 0; i < 4; i++) {
    pinMode(pins[i], INPUT_PULLUP);
    channel |= (digitalRead(pins[i]) << i);

    // Release internal pull-up resistor
    pinMode(pins[i], INPUT);
  }

  return channel;
}

void initNotePriority()
{
  pinMode(NOTE_PRIORITY_1, INPUT_PULLUP);
  pinMode(NOTE_PRIORITY_2, INPUT_PULLUP);

  noteCommand = lastNodeCommand;

  if (digitalRead(NOTE_PRIORITY_1)) {
    noteCommand = highestNoteCommand;
  } else if (digitalRead(NOTE_PRIORITY_2)) {
    noteCommand = lowestNoteCommand;
  }

  // Release internal pull-up resistor
  pinMode(NOTE_PRIORITY_1, INPUT);
  pinMode(NOTE_PRIORITY_2, INPUT);
}

void initCalibration()
{
  pinMode(CALIBRATION_SWITCH, INPUT);

  if (digitalRead(CALIBRATION_SWITCH)) {
    isCalibrationActive = true;
    return;
  }

  EEPROM.get(CALIBRATION_EEPROM, calibrationValue);

  // Init the EEPROM with default value if empty
  if (calibrationValue == -1) {
    EEPROM.put(CALIBRATION_EEPROM, CALIBRATION_DEFAULT);
    calibrationValue = CALIBRATION_DEFAULT;
  }
}

void loop()
{
  static unsigned long clockTimer = 0;
  static unsigned int clockCount = 0;

  if ((triggerTimer > 0) && (millis() - triggerTimer > TRIGGER_DURATION)) { 
    // Set trigger low after 20 msec
    digitalWrite(TRIG, LOW);
    triggerTimer = 0;  
  }

  if ((clockTimer > 0) && (millis() - clockTimer > CLOCK_DURATION)) { 
    // Set clock pulse low after 20 msec 
    digitalWrite(CLOCK, LOW);
    clockTimer = 0;
  }
  
  if (!MIDI.read()) {
    continue;
  }

  switch (MIDI.getType()) {
    case midi::NoteOn:
      handleNote(MIDI.getData1() - 21, MIDI.getData2());

    case midi::NoteOff:
      handleNote(MIDI.getData1() - 21, 0);
      break;
      
    case midi::PitchBend:
      handlePitchBend(MIDI.getData2());      
      break;

    case midi::ControlChange:
      handleControlChange(MIDI.getData2());
      break;
      
    case midi::Clock:
      handleClock(clockTimern, clockCount);
      break;
  }
}

void handlePitchBend(int value)
{
  // Value from 0 to 127, mid point = 64
  // Pitch bend output from 0 to 1023 mV.
  // Left shift value by 4 to scale from 0 to 2047.
  // With DAC gain = 1X, this will yield a range from 0 to 1023 mV.
  setVoltage(DAC_PIN_PITCH_BEND, DAC_CHAN_PITCH_BEND, DAC_GAIN_PITCH_BEND, value << 4);
}

void handleControlChange(int value)
{
  // Value from 0 to 127
  // CC range from 0 to 4095 mV.
  // Left shift value by 5 to scale from 0 to 4095, and choose gain = 2X.
  setVoltage(DAC_PIN_CONTROL_CHANGE, DAC_CHAN_CONTROL_CHANGE, DAC_GAIN_CONTROL_CHANGE, value << 5);
}

void handleClock(long& clockTimer, int& clockCount)
{
  static unsigned long clockTimeout = 0;

  // Prevents Clock from starting in between quarter notes after clock is restarted!
  if (millis() > clockTimeout + 300) {
    clockCount = 0;
  }

  clockTimeout = millis();
  
  // Start clock pulse
  if (clockCount == 0) {
    digitalWrite(CLOCK, HIGH);
    clockTimer = millis();    
  }

  clockCount++;

  if (clockCount == CLOCK_PPQ) {
    clockCount = 0;  
  }
}

void handleNote(int note, int velocity)
{
  // Only 88 notes of keyboard are supported
  if ((note < 0) || (note > 87)) {
    return;
  }

  notes[note] = false;

  if (velocity > 0) {
    notes[note] = true;

    // Velocity range from 0 to 4095 mV.
    // Left shift velocity by 5 to scale from 0 to 4095
    setVoltage(DAC_PIN_VELOCITY, DAC_CHAN_VELOCITY, DAC_GAIN_VELOCITY, velocity << 5);
  }

  if (noteCommand == lastNodeCommand && notes[note]) {
    orderIndex = (orderIndex + 1) % 20;
    notesOrder[orderIndex] = note;  
  }

  if (!noteCommand(note)) {
    // All notes are off, turn off gate
    digitalWrite(GATE, LOW);  
  }
}

bool highestNoteCommand(int note)
{
  int highestNote = 0;
  bool noteActive = false;

  for (int i = note; i < 88; i++) {
    if (notes[i]) {
      highestNote = i;
      noteActive = true;
    }
  }

  if (!noteActive) {
    return false;
  }

  outputNote(highestNote);
  return true;
}

bool lowestNoteCommand(int note)
{
  int bottomNote = 0;
  bool noteActive = false;

  for (int i = note; i >= 0; i--) {
    if (notes[i]) {
      bottomNote = i;
      noteActive = true;
    }
  }

  if (!noteActive) {
    return false;
  }

  outputNote(bottomNote);
  return true;
}

bool lastNodeCommand(int note)
{
  int8_t noteIndex;
  
  for (int i = 0; i < 20; i++) {
    noteIndex = notesOrder[ mod(orderIndex - i, 20) ];

    if (notes[noteIndex]) {
      outputNote(noteIndex);
      return true;
    }
  }

  return false;
}

/*
 * Rescale 88 notes to 4096 mV:
 *    noteMsg = 0 -> 0 mV 
 *    noteMsg = 87 -> 4096 mV
 * DAC output will be (4095/87) = 47.069 mV per note, and 564.9655 mV per octive
 * Note that DAC output will need to be amplified by 1.77X for the standard 1V/octave 
 */
void outputNote(int noteNumber)
{
  digitalWrite(GATE, HIGH);
  digitalWrite(TRIG, HIGH);
  triggerTimer = millis();
  
  unsigned int mV = (unsigned int) ((float) noteNumber * getNoteCalibration() + 0.5); 
  setVoltage(DAC_PIN_NOTE, DAC_CHAN_NOTE, DAC_GAIN_NOTE, mV);
}

/*
 * Set DAC voltage output
 * dacpin: chip select pin for DAC.  Note and CC on DAC1, pitch bend and velocity on DAC2
 * channel: 0 (A) or 1 (B).  CC and pitch bend on 0, note and velocity on 2.
 * gain: 0 = 1X, 1 = 2X.
 * mV: integer 0 to 4095.  If gain is 1X, mV is in units of half mV (i.e., 0 to 2048 mV).
 * If gain is 2X, mV is in units of mV
 */
void setVoltage(int dacpin, unsigned int command, unsigned int gain, unsigned int mV)
{
  command |= gain;
  command |= (mV & 0x0FFF);
  
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dacpin, LOW);
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0xFF);
  digitalWrite(dacpin, HIGH);
  SPI.endTransaction();
}

// Get calibration value for the note mV multiplier
float getNoteCalibration()
{
  if (!isCalibrationActive) {
    return calibrationValue;
  }

  static unsigned long lastReadTime = 0;
  static float lastCalibrationValue = calibrationValue;
  static float newCalibrationValue = lastCalibrationValue;
  unsigned long currentTime = millis();

  // Read calibration pot every second to minimize performances impact
  if (currentTime - lastReadTime >= 100) {
    float mappedValue = (analogRead(CALIBRATION_POT) * (NOTE_MAX_MV - NOTE_MIN_MV) / 1023.0) + NOTE_MIN_MV;
    lastReadTime = currentTime;
    newCalibrationValue = (round(mappedValue / 0.001) * 0.001);

    // If the calibration changed, save it to EEPROM
    if (newCalibrationValue != lastCalibrationValue) {
      EEPROM.put(CALIBRATION_EEPROM, newCalibrationValue);
      lastCalibrationValue = newCalibrationValue;
    }
  }

  return lastCalibrationValue;
}

int mod(int a, int b)
{
  int r = a % b;
  return r < 0 ? r + b : r;
}