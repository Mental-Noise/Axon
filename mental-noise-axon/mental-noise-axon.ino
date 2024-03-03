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
#include <stdint.h>

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
 * 0000 : 1  |  0100 : 5  |  1000 : 9  |  1100 :13
 * 0001 : 2  |  0101 : 6  |  1001 :10  |  1101 :14
 * 0010 : 3  |  0110 : 7  |  1010 :11  |  1110 :15
 * 0011 : 4  |  0111 : 8  |  1011 :12  |  1111 :16
 */
#define MIDI_CHANNEL_1 A5
#define MIDI_CHANNEL_2 A4
#define MIDI_CHANNEL_3 A3
#define MIDI_CHANNEL_4 A2

// Calibration
#define CALIBRATION_POT A6
#define CALIBRATION_SWITCH A7
#define CALIBRATION_DEFAULT 47.069f
#define CALIBRATION_MIN 46.590f
#define CALIBRATION_MAX 47.610f
#define CALIBRATION_EEPROM 0

// Switch used to enable or disabled pitch bend addition to note
#define PITCH_BEND_SWITCH 2

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
#define NOTES_HISTORY 20

// DAC configuration
#define DAC_PIN_NOTE DAC1
#define DAC_CHAN_NOTE 0x9000
#define DAC_GAIN_NOTE 0x0000
#define DAC_PIN_CONTROL_CHANGE DAC1
#define DAC_CHAN_CONTROL_CHANGE 0x1000
#define DAC_GAIN_CONTROL_CHANGE 0x0000
#define DAC_PIN_PITCH_BEND DAC2
#define DAC_CHAN_PITCH_BEND 0x1000
#define DAC_GAIN_PITCH_BEND 0x2000
#define DAC_PIN_VELOCITY DAC2
#define DAC_CHAN_VELOCITY 0x9000
#define DAC_GAIN_VELOCITY 0x0000

// Debug
#define DEBUG false
#if DEBUG
  #define ddbegin(x); Serial.begin(x);
  #define dd(x); Serial.println(x);
#else
  #define ddbegin(x);
  #define dd(x);
#endif

// Global variables
int8_t (*noteCommand)(int);
bool isCalibrationActive{false};
float calibrationValue{CALIBRATION_DEFAULT};

volatile bool addPitchBend{false};
volatile bool notes[88] = {}; 
volatile uint8_t notesOrder[NOTES_HISTORY]{};
volatile uint8_t orderIndex{};
volatile int8_t pitchBend{};
volatile uint16_t now{};
volatile uint16_t clockCount{};
volatile uint32_t clockTimer{};
volatile uint32_t clockTimeout{};
volatile uint32_t triggerTimer{};

MIDI_CREATE_DEFAULT_INSTANCE();

void blink(uint8_t times, bool fast = false) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(GATE, HIGH);
    delay(fast ? 50 : 200);
    digitalWrite(GATE, LOW);
    delay(fast ? 50 : 200);
  }
}

int getMidiChannel() {
  uint8_t channel{1};
  uint8_t pins[]{MIDI_CHANNEL_4, MIDI_CHANNEL_3, MIDI_CHANNEL_2, MIDI_CHANNEL_1};

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(pins[i], INPUT_PULLUP);
    
    channel += (1 << i) * !digitalRead(pins[i]);
    
    // Release internal pull-up resistor
    pinMode(pins[i], INPUT);
  }

  return channel;
}

int8_t highestNoteCommand(uint8_t note) {
  for (uint8_t i = 87; i >= note; i--) {
    if (notes[i]) {
      return i;
    }
  }

  return -1;
}

int8_t lowestNoteCommand(uint8_t note) {
  for (uint8_t i = 0; i <= note; i++) {
    if (notes[i]) {
      return i;
    }
  }

  return -1;
}

int8_t lastNodeCommand(uint8_t note) {
  int8_t noteIndex;
  
  for (uint8_t i = 0; i < NOTES_HISTORY; i++) {
    int8_t r = orderIndex - i % NOTES_HISTORY;
    noteIndex = notesOrder[r < 0 ? r + NOTES_HISTORY : r];

    if (notes[noteIndex]) {
      return noteIndex;
    }
  }

  return -1;
}

void initNotePriority() {
  pinMode(NOTE_PRIORITY_1, INPUT_PULLUP);
  pinMode(NOTE_PRIORITY_2, INPUT_PULLUP);

  noteCommand = lastNodeCommand;

  if (!digitalRead(NOTE_PRIORITY_1)) {
    noteCommand = highestNoteCommand;
    dd("Note priority : Highest");
  } else if (!digitalRead(NOTE_PRIORITY_2)) {
    noteCommand = lowestNoteCommand;
    dd("Note priority : Lowest");
  } else {
    dd("Note priority : Latest");
  }

  // Release internal pull-up resistor
  pinMode(NOTE_PRIORITY_1, INPUT);
  pinMode(NOTE_PRIORITY_2, INPUT);
}

void initPitchBendSwitch() {
  pinMode(PITCH_BEND_SWITCH, INPUT_PULLUP);

  addPitchBend = digitalRead(PITCH_BEND_SWITCH);

  // Attach an interrupt to add or remove pitch bend to note when the switch state changes
  attachInterrupt(digitalPinToInterrupt(PITCH_BEND_SWITCH), []{
    addPitchBend = digitalRead(PITCH_BEND_SWITCH) == HIGH;

    if (addPitchBend) {
      dd("Add pitch bend : enabled");
    } else {
      dd("Add pitch bend : disabled");
    }
  }, CHANGE);

  // Set initial pitch bend value (0-127) to mid point
  handlePitchBend(64);
}

void handlePitchBend(int value) {
  pitchBend = value - 64;

  // Value from 0 to 127, mid point = 64
  // Pitch bend output from 0 to 1023 mV.
  // Left shift value by 4 to scale from 0 to 2047.
  // With DAC gain = 1X, this will yield a range from 0 to 1023 mV.
  setVoltage(DAC_PIN_PITCH_BEND, DAC_CHAN_PITCH_BEND, DAC_GAIN_PITCH_BEND, value << 4);
}

void handleControlChange(int value) {
  // Value from 0 to 127
  // CC range from 0 to 4095 mV.
  // Left shift value by 5 to scale from 0 to 4095, and choose gain = 2X.
  setVoltage(DAC_PIN_CONTROL_CHANGE, DAC_CHAN_CONTROL_CHANGE, DAC_GAIN_CONTROL_CHANGE, value << 5);
}

void handleClock() {
  // Prevents Clock from starting in between quarter notes after clock is restarted!
  if (now > clockTimeout + 300) {
    clockCount = 0;
  }

  clockTimeout = now;
  
  // Start clock pulse
  if (clockCount == 0) {
    digitalWrite(CLOCK, HIGH);
    clockTimer = now;    
  }

  clockCount++;

  if (clockCount == CLOCK_PPQ) {
    clockCount = 0;  
  }
}

float getCalibrationPotValue() {
  float mappedValue = (analogRead(CALIBRATION_POT) * (CALIBRATION_MAX - CALIBRATION_MIN) / 1023.0) + CALIBRATION_MIN;
  return (round(mappedValue / 0.001) * 0.001);
}

/*
 * Rescale 88 notes to 4096 mV:
 *    noteMsg = 0 -> 0 mV 
 *    noteMsg = 87 -> 4096 mV
 * DAC output will be (4095/87) = 47.069 mV per note, and 564.9655 mV per octive
 * DAC output will be amplified by the op-amp by 1.77X for the standard 1V/octave
 */
void outputNote(uint8_t noteNumber, bool useCalibrationPot = false) {
  digitalWrite(GATE, HIGH);
  digitalWrite(TRIG, HIGH);

  triggerTimer = now;
  float calibrationMultiplier = useCalibrationPot ? getCalibrationPotValue() : calibrationValue;
  uint16_t mV = (uint16_t) (float) (noteNumber * calibrationMultiplier + 0.5) + (addPitchBend ? pitchBend << 2 : 0);

  dd("Note mV : " + String(mV));
  setVoltage(DAC_PIN_NOTE, DAC_CHAN_NOTE, DAC_GAIN_NOTE, mV);
}

void handleNote(uint8_t note, uint16_t velocity) {
  // Only 88 notes are supported
  if (note < 0 || note > 87) {
    dd("Note " + String(note) + " out of range");
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
    orderIndex = (orderIndex + 1) % NOTES_HISTORY;
    notesOrder[orderIndex] = note;  
  }

  int noteOutput = noteCommand(note);
  dd("Note : " + String(noteOutput));

  if (noteOutput == -1) {
    digitalWrite(GATE, LOW);
    return;
  }

  outputNote(noteOutput);
}

float calibrateVoltage(uint8_t voltage) {
  blink(voltage);
  dd("Adjust to " + String(voltage) + "V");

  while (analogRead(CALIBRATION_SWITCH)) {
    outputNote(12 * voltage, true);
    delay(50);
  }

  return getCalibrationPotValue();
}

void initCalibration() {
  pinMode(CALIBRATION_SWITCH, INPUT);

  if (!analogRead(CALIBRATION_SWITCH)) {
    dd("Calibration : enabled");
    blink(10, true);
    delay(3000);

    if (addPitchBend) {
      dd("Reset calibration to default value : " + String(CALIBRATION_DEFAULT));
      EEPROM.put(CALIBRATION_EEPROM, CALIBRATION_DEFAULT);
      blink(10, true);
      delay(3000);
    }

    float calibrationPoints[4]{};

    calibrationPoints[0] = calibrateVoltage(1);
    calibrationPoints[1] = calibrateVoltage(3);
    calibrationPoints[2] = calibrateVoltage(5);
    calibrationPoints[3] = calibrateVoltage(7);

    float calibrationSum{};

    for (uint8_t i = 0; i < 4; i++) {
      calibrationSum += calibrationPoints[i];
    }

    calibrationValue = calibrationSum / 4;

    // Save new calibration value in EEPROM
    EEPROM.put(CALIBRATION_EEPROM, calibrationValue);
    dd("Calibration done, new value : " + String(calibrationValue));

    blink(10, true);
    return;
  }

  dd("Calibration : disabled");
  EEPROM.get(CALIBRATION_EEPROM, calibrationValue);

  // Init the EEPROM with default value if empty
  if (isnan(calibrationValue) || calibrationValue < CALIBRATION_MIN || calibrationValue > CALIBRATION_MAX) {
    EEPROM.put(CALIBRATION_EEPROM, CALIBRATION_DEFAULT);
    calibrationValue = CALIBRATION_DEFAULT;
  }

  dd("Calibration value : " + String(calibrationValue));
}

/*
 * Set DAC voltage output
 * dacpin: chip select pin for DAC.  Note and CC on DAC1, pitch bend and velocity on DAC2
 * channel: 0 (A) or 1 (B).  CC and pitch bend on 0, note and velocity on 2.
 * gain: 0 = 1X, 1 = 2X.
 * mV: integer 0 to 4095.  If gain is 1X, mV is in units of half mV (i.e., 0 to 2048 mV).
 * If gain is 2X, mV is in units of mV
 */
void setVoltage(uint8_t dacpin, uint16_t command, uint16_t gain, uint16_t mV) {
  command |= gain;
  command |= (mV & 0x0FFF);
  
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dacpin, LOW);
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0xFF);
  digitalWrite(dacpin, HIGH);
  SPI.endTransaction();
}

void setup() {pinMode(GATE, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DAC1, OUTPUT);
  pinMode(DAC2, OUTPUT);

  digitalWrite(GATE, LOW);
  digitalWrite(TRIG, LOW);
  digitalWrite(CLOCK, LOW);
  digitalWrite(DAC1, HIGH);
  digitalWrite(DAC2, HIGH);

  SPI.begin();
  
  // MIDI baudrate
  ddbegin(31250);

  uint8_t midiChannel = getMidiChannel();

  MIDI.begin(midiChannel);
  MIDI.turnThruOff();

  delay(300);
  dd("Debug : enabled");
  dd("MIDI channel : " + String(midiChannel));

  initPitchBendSwitch();
  initCalibration();
  initNotePriority();
}

void loop() {
  now = millis();

  // Set trigger low after TRIGGER_DURATION
  if ((triggerTimer > 0) && (now - triggerTimer > TRIGGER_DURATION)) { 
    digitalWrite(TRIG, LOW);
    triggerTimer = 0;  
  }

  // Set clock pulse low after CLOCK_DURATION
  if ((clockTimer > 0) && (now - clockTimer > CLOCK_DURATION)) { 
    digitalWrite(CLOCK, LOW);
    clockTimer = 0;
  }
  
  if (!MIDI.read()) {
    return;
  }

  uint8_t note = 0;

  switch (MIDI.getType()) {
    case midi::NoteOn:
      note = MIDI.getData1() - 21;

      dd("Note ON : " + String(MIDI.getData1()) + " (" + note + ") - " + String(MIDI.getData2()));
      handleNote(note, MIDI.getData2());
      break;

    case midi::NoteOff:
      note = MIDI.getData1() - 21;

      dd("Note OFF : " + String(MIDI.getData1()) + " (" + String(note) + ") - 0");
      handleNote(note, 0);
      break;
      
    case midi::PitchBend:
      dd("Pitch bend : " + String(MIDI.getData2()));
      handlePitchBend(MIDI.getData2());      
      break;

    case midi::ControlChange:
      dd("Control change : " + String(MIDI.getData2()));
      handleControlChange(MIDI.getData2());
      break;
      
    case midi::Clock:
      dd("Clock : " + String(clockTimer) + " / " + String(clockCount));
      handleClock();
      break;
  }
}
