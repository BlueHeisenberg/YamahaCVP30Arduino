/*
Moura's Keyboard Scanner: turn you broken (or unused) keyboard in a MIDI controller
Copyright (C) 2017 Daniel Moura <oxe@oxesoft.com>

This code is originally hosted at https://github.com/oxesoft/keyboardscanner

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <DIO2.h> // install the library DIO2
#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

//#define DEBUG_SCANS_PER_SECOND
//#define DEBUG_MIDI_MESSAGE

#define KEYS_NUMBER 88

#define KEY_OFF               0
#define KEY_START             1
#define KEY_ON                2
#define KEY_RELEASED          3
#define KEY_SUSTAINED         4
#define KEY_SUSTAINED_RESTART 5

#define MIN_TIME_MS   3
#define MAX_TIME_MS   50
#define MAX_TIME_MS_N (MAX_TIME_MS - MIN_TIME_MS)

#define PEDAL_PIN     99

//find out the pins using a multimeter, starting from the first key
//see the picture key_scheme.png to understand how to map the inputs and outputs

//the following configuration is specific for PSR530
//thanks Leandro Meucchi, from Argentina, by the PDF
//take a look at the scheme detailed in PSR530.pdf and modify the following mapping according to the wiring of your keyboard
#define PIN_MK00  52
#define PIN_MK01  50
#define PIN_MK02  48
#define PIN_MK03  46
#define PIN_MK04  44
#define PIN_MK05  42
#define PIN_MK10  40
#define PIN_MK11  38
#define PIN_MK12  36
#define PIN_MK13  34
#define PIN_MK14  32
#define PIN_MK15  30

#define PIN_B0  23
#define PIN_B1  25
#define PIN_B2  27
#define PIN_B3  29
#define PIN_B4  31
#define PIN_B5  33
#define PIN_B6  35
#define PIN_B7  37
#define PIN_B8  39
#define PIN_B9  41
#define PIN_B10  43
#define PIN_B11  45
#define PIN_B12  47
#define PIN_B13  49
#define PIN_B14  51
#define PIN_B15  53

byte output_pins[] = {
    PIN_B0, //A1
    PIN_B0,
    PIN_B0,
    PIN_B0, //C0
    
    PIN_B0, //A1
    PIN_B0,
    PIN_B0,
    PIN_B0, //C0
 
    PIN_B1, 
    PIN_B1,
    PIN_B1,
    PIN_B1,
    PIN_B1,
    PIN_B1,

    PIN_B1, 
    PIN_B1,
    PIN_B1,
    PIN_B1,
    PIN_B1,
    PIN_B1,
   
    PIN_B2,
    PIN_B2, //C1
    PIN_B2,
    PIN_B2,
    PIN_B2,
    PIN_B2,

    PIN_B2,
    PIN_B2, //C1
    PIN_B2,
    PIN_B2,
    PIN_B2,
    PIN_B2,
    
    PIN_B3,
    PIN_B3,
    PIN_B3,
    PIN_B3, //C2
    PIN_B3,
    PIN_B3,
    
    PIN_B3,
    PIN_B3,
    PIN_B3,
    PIN_B3, //C2
    PIN_B3,
    PIN_B3,

    
    PIN_B4,
    PIN_B4,
    PIN_B4,
    PIN_B4,
    PIN_B4,
    PIN_B4, //C3
    
    PIN_B4,
    PIN_B4,
    PIN_B4,
    PIN_B4,
    PIN_B4,
    PIN_B4, //C3
    
    
    PIN_B5,
    PIN_B5,
    PIN_B5,
    PIN_B5,
    PIN_B5,
    PIN_B5,
    
    PIN_B5,
    PIN_B5,
    PIN_B5,
    PIN_B5,
    PIN_B5,
    PIN_B5,

    
    PIN_B6,
    PIN_B6, //C4
    PIN_B6,
    PIN_B6,
    PIN_B6,
    PIN_B6,
    
    PIN_B6,
    PIN_B6, //C4
    PIN_B6,
    PIN_B6,
    PIN_B6,
    PIN_B6,

    PIN_B7,
    PIN_B7, //C4
    PIN_B7,
    PIN_B7,
    PIN_B7,
    PIN_B7,
    
    PIN_B7,
    PIN_B7, //C4
    PIN_B7,
    PIN_B7,
    PIN_B7,
    PIN_B7,

   
    PIN_B8,
    PIN_B8, //C1
    PIN_B8,
    PIN_B8,
    PIN_B8,
    PIN_B8,

    PIN_B8,
    PIN_B8, //C1
    PIN_B8,
    PIN_B8,
    PIN_B8,
    PIN_B8,
    
    PIN_B9,
    PIN_B9,
    PIN_B9,
    PIN_B9, //C2
    PIN_B9,
    PIN_B9,
    
    PIN_B9,
    PIN_B9,
    PIN_B9,
    PIN_B9, //C2
    PIN_B9,
    PIN_B9,

    
    PIN_B10,
    PIN_B10,
    PIN_B10,
    PIN_B10,
    PIN_B10,
    PIN_B10, //C3
    
    PIN_B10,
    PIN_B10,
    PIN_B10,
    PIN_B10,
    PIN_B10,
    PIN_B10, //C3
    

    PIN_B11,
    PIN_B11,
    PIN_B11,
    PIN_B11,
    PIN_B11,
    PIN_B11,
    
    PIN_B11,
    PIN_B11,
    PIN_B11,
    PIN_B11,
    PIN_B11,
    PIN_B11,

   
     
    PIN_B12,
    PIN_B12, //C4
    PIN_B12,
    PIN_B12,
    PIN_B12,
    PIN_B12,
    
    PIN_B12,
    PIN_B12, //C4
    PIN_B12,
    PIN_B12,
    PIN_B12,
    PIN_B12,

    PIN_B13,
    PIN_B13,
    PIN_B13,
    PIN_B13, //C5
    PIN_B13,
    PIN_B13,
    PIN_B13,
    PIN_B13, //C5
    
    PIN_B13,
    PIN_B13,
    PIN_B13,
    PIN_B13, //C5
    PIN_B13,
    PIN_B13,
    PIN_B13,
    PIN_B13, //C5

    PIN_B14,
    PIN_B14,
    PIN_B14,
    PIN_B14, //C5
    PIN_B14,
    PIN_B14,
    PIN_B14,
    PIN_B14, //C5
    
    PIN_B14,
    PIN_B14,
    PIN_B14,
    PIN_B14, //C5
    PIN_B14,
    PIN_B14,
    PIN_B14,
    PIN_B14, //C5

};

byte caca[]={
  
    PIN_B13,
    PIN_B13,
    PIN_B13,
    PIN_B13, //C5
    
    PIN_B13,
    PIN_B13,
    PIN_B13,
    PIN_B13 //C5
    
};

byte input_pins[] = {
    PIN_MK13,
    PIN_MK03, //C#0
    PIN_MK12,
    PIN_MK02, //F#0
    PIN_MK11,
    PIN_MK01, //G0
    PIN_MK10,
    PIN_MK00, //C1

    PIN_MK15,
    PIN_MK05, //C#1
    PIN_MK14,
    PIN_MK04, //F#1
    PIN_MK13,
    PIN_MK03, //G1
    PIN_MK12,
    PIN_MK02, //C2
    PIN_MK11,
    PIN_MK01, //C#2
    PIN_MK10,
    PIN_MK00, //F#2
    
    PIN_MK15,
    PIN_MK05, //G2
    PIN_MK14,
    PIN_MK04, //C3
    PIN_MK13,
    PIN_MK03, //C#3
    PIN_MK12,
    PIN_MK02, //F#3
    PIN_MK11,
    PIN_MK01, //G3
    PIN_MK10,
    PIN_MK00, //C4
    
    PIN_MK15,
    PIN_MK05, //C#4
    PIN_MK14,
    PIN_MK04, //F#4
    PIN_MK13,
    PIN_MK03, //G4
    PIN_MK12,
    PIN_MK02, //C5
    PIN_MK11,
    PIN_MK01, //C#5
    PIN_MK10,
    PIN_MK00, //F#5
    
    PIN_MK15,
    PIN_MK05, //G5
    PIN_MK14,
    PIN_MK04, //C6
    PIN_MK13,
    PIN_MK03, //C#6
    PIN_MK12,
    PIN_MK02, //F#6
    PIN_MK11,
    PIN_MK01, //G6
    PIN_MK10,
    PIN_MK00,  //C7
    
    PIN_MK15,
    PIN_MK05, //G5
    PIN_MK14,
    PIN_MK04, //C6
    PIN_MK13,
    PIN_MK03, //C#6
    PIN_MK12,
    PIN_MK02, //F#6
    PIN_MK11,
    PIN_MK01, //G6
    PIN_MK10,
    PIN_MK00,  //C7
    
    PIN_MK15,
    PIN_MK05, //G5
    PIN_MK14,
    PIN_MK04, //C6
    PIN_MK13,
    PIN_MK03, //C#6
    PIN_MK12,
    PIN_MK02, //F#6
    PIN_MK11,
    PIN_MK01, //G6
    PIN_MK10,
    PIN_MK00,  //C7

    PIN_MK15,
    PIN_MK05, //C#1
    PIN_MK14,
    PIN_MK04, //F#1
    PIN_MK13,
    PIN_MK03, //G1
    PIN_MK12,
    PIN_MK02, //C2
    PIN_MK11,
    PIN_MK01, //C#2
    PIN_MK10,
    PIN_MK00, //F#2
    
    PIN_MK15,
    PIN_MK05, //G2
    PIN_MK14,
    PIN_MK04, //C3
    PIN_MK13,
    PIN_MK03, //C#3
    PIN_MK12,
    PIN_MK02, //F#3
    PIN_MK11,
    PIN_MK01, //G3
    PIN_MK10,
    PIN_MK00, //C4
    
    
    PIN_MK15,
    PIN_MK05, //C#4
    PIN_MK14,
    PIN_MK04, //F#4
    PIN_MK13,
    PIN_MK03, //G4
    PIN_MK12,
    PIN_MK02, //C5
    PIN_MK11,
    PIN_MK01, //C#5
    PIN_MK10,
    PIN_MK00, //F#5
    
    PIN_MK15,
    PIN_MK05, //G5
    PIN_MK14,
    PIN_MK04, //C6
    PIN_MK13,
    PIN_MK03, //C#6
    PIN_MK12,
    PIN_MK02, //F#6
    PIN_MK11,
    PIN_MK01, //G6
    PIN_MK10,
    PIN_MK00,  //C7
  
    PIN_MK15,
    PIN_MK05, //G5
    PIN_MK14,
    PIN_MK04, //C6
    PIN_MK13,
    PIN_MK03, //C#6
    PIN_MK12,
    PIN_MK02, //F#6
    PIN_MK11,
    PIN_MK01, //G6
    PIN_MK10,
    PIN_MK00,  //C7
    
    PIN_MK15,
    PIN_MK05, //G5
    PIN_MK14,
    PIN_MK04, //C6
    PIN_MK13,
    PIN_MK03, //C#6
    PIN_MK12,
    PIN_MK02, //F#6
    PIN_MK11,
    PIN_MK01, //G6
    PIN_MK10,
    PIN_MK00, //C7

    PIN_MK15,
    PIN_MK05, //G5
    PIN_MK14,
    PIN_MK04, //C6
    PIN_MK13,
    PIN_MK03, //C#6
    PIN_MK12,
    PIN_MK02, //F#6
    PIN_MK11,
    PIN_MK01, //G6
    PIN_MK10,
    PIN_MK00, //C7

    PIN_MK15,
    PIN_MK05, //G5
    PIN_MK14,
    PIN_MK04, //C6
    PIN_MK13,
    PIN_MK03, //C#6
    PIN_MK12,
    PIN_MK02, //F#6
    PIN_MK11,
    PIN_MK01, //G6
    PIN_MK10,
    PIN_MK00, //C7

};

byte caca2[] = {
  
    PIN_MK13,
    PIN_MK03, //C#0
    PIN_MK12,
    PIN_MK02, //F#0
    PIN_MK11,
    PIN_MK01, //G0
    PIN_MK10,
    PIN_MK00 //C1
};

//cheap keyboards often has the black keys softer or harder than the white ones
//uncomment the next line to allow a soft correction
//#define BLACK_KEYS_CORRECTION

#ifdef BLACK_KEYS_CORRECTION
#define MULTIPLIER 192 // 127 is the central value (corresponding to 1.0)
byte black_keys[] = {
    0,1,0,1,0,0,1,0,1,0,1,0,
    0,1,0,1,0,0,1,0,1,0,1,0,
    0,1,0,1,0,0,1,0,1,0,1,0,
    0,1,0,1,0,0,1,0,1,0,1,0,
    0,1,0,1,0,0,1,0,1,0,1,0,
    0
};
#endif

//uncomment the next line to inspect the number of scans per seconds
//#define DEBUG_SCANS_PER_SECOND

/*
426 cyles per second (2,35ms per cycle) using standard digitalWrite/digitalRead
896 cyles per second (1,11ms per cycle) using DIO2 digitalWrite2/digitalRead2
*/

//uncoment the next line to get text midi message at output
//#define DEBUG_MIDI_MESSAGE

byte          keys_state[KEYS_NUMBER];
unsigned long keys_time[KEYS_NUMBER];
boolean       signals[KEYS_NUMBER * 2];
boolean       pedal_enabled;

void setup() {
    //Serial.begin(115200);
    MIDI.begin(MIDI_CHANNEL_OMNI);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    int i;
    for (i = 0; i < KEYS_NUMBER; i++)
    {
        keys_state[i] = KEY_OFF;
        keys_time[i] = 0;
    }
    for (byte pin = 0; pin < sizeof(output_pins); pin++)
    {
        pinMode(output_pins[pin], OUTPUT);
    }
    for (byte pin = 0; pin < sizeof(input_pins); pin++)
    {
        pinMode(input_pins[pin], INPUT_PULLUP);
    }
    //pinMode(PEDAL_PIN, INPUT_PULLUP);
    pedal_enabled = false; //digitalRead(PEDAL_PIN) != HIGH;
}

void send_midi_event(byte status_byte, byte key_index, unsigned long time)
{
    unsigned long t = time;
#ifdef BLACK_KEYS_CORRECTION
    if (black_keys[key_index])
    {
        t = (t * MULTIPLIER) >> 7;
    }
#endif
    if (t > MAX_TIME_MS)
        t = MAX_TIME_MS;
    if (t < MIN_TIME_MS)
        t = MIN_TIME_MS;
    t -= MIN_TIME_MS;
    unsigned long velocity = 127 - (t * 127 / MAX_TIME_MS_N);
    byte vel = (((velocity * velocity) >> 7) * velocity) >> 7;
    byte key = 33 + key_index;
#ifdef DEBUG_MIDI_MESSAGE
    char out[32];
    sprintf(out, "%02X %02d %03d %d", status_byte, key, vel, time);
    Serial.println(out);
#else
    //START PRESS
    if(status_byte == 0x90){
      MIDI.sendNoteOn(key, vel, 1);
    } //STOP PRESS
    else if(status_byte == 0x80){
      MIDI.sendNoteOff(key, vel, 1);
    }
    //Serial.write(status_byte);
    //Serial.write(key);
    //Serial.write(vel);
#endif
}

void loop() {
#ifdef DEBUG_SCANS_PER_SECOND
    static unsigned long cycles = 0;
    static unsigned long start = 0;
    static unsigned long current = 0;
    cycles++;
    current = millis();
    if (current - start >= 1000)
    {
        Serial.println(cycles);
        cycles = 0;
        start = current;
    }
#endif
    byte pedal = LOW;
    if (pedal_enabled)
    {
        pedal = digitalRead2(PEDAL_PIN);
    }
   
    boolean *s = signals;
    for (byte i = 0; i < KEYS_NUMBER * 2; i++)
    {
        byte output_pin = output_pins[i];
        byte input_pin = input_pins[i];
        digitalWrite2(output_pin, LOW);
        *(s++) = !digitalRead2(input_pin);
        digitalWrite2(output_pin, HIGH);
    }

    byte          *state  = keys_state;
    unsigned long *ktime  = keys_time;
    boolean       *signal = signals;
    for (byte key = 0; key < KEYS_NUMBER; key++)
    {
        for (byte state_index = 0; state_index < 2; state_index++)
        {
            switch (*state)
            {
            case KEY_OFF:
                if (state_index == 0 && *signal)
                {
                    *state = KEY_START;
                    *ktime = millis();
                }
                break;
            case KEY_START:
                if (state_index == 0 && !*signal)
                {
                    *state = KEY_OFF;
                    break;
                }
                if (state_index == 1 && *signal)
                {
                    *state = KEY_ON;
                    send_midi_event(0x90, key, millis() - *ktime);
                }
                break;
            case KEY_ON:
                if (state_index == 1 && !*signal)
                {
                    *state = KEY_RELEASED;
                    *ktime = millis();
                }
                break;
            case KEY_RELEASED:
                if (state_index == 0 && !*signal)
                {
                    if (pedal)
                    {
                        *state = KEY_SUSTAINED;
                        break;
                    }
                    *state = KEY_OFF;
                    send_midi_event(0x80, key, millis() - *ktime);
                }
                break;
            case KEY_SUSTAINED:
                if (!pedal)
                {
                    *state = KEY_OFF;
                    send_midi_event(0x80, key, MAX_TIME_MS);
                }
                if (state_index == 0 && *signal)
                {
                    *state = KEY_SUSTAINED_RESTART;
                    *ktime = millis();
                }
                break;
            case KEY_SUSTAINED_RESTART:
                if (state_index == 0 && !*signal)
                {
                    *state = KEY_SUSTAINED;
                    digitalWrite(13, HIGH);
                    break;
                }
                if (state_index == 1 && *signal)
                {
                    *state = KEY_ON;
                    send_midi_event(0x80, key, MAX_TIME_MS);
                    send_midi_event(0x90, key, millis() - *ktime);
                }
                break;
            }
            signal++;
        }
        state++;
        ktime++;
    }
}
