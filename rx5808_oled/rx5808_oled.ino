/*
 * SPI driver based on fs_skyrf_58g-main.c Written by Simon Chambers
 * TVOUT by Myles Metzel
 * Scanner by Johan Hermen
 * Inital 2 Button version by Peter (pete1990)
 * Refactored and GUI reworked by Marko Hoepken
 * Universal version my Marko Hoepken

The MIT License (MIT)

Copyright (c) 2015 Marko Hoepken

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


Adafruit_SSD1306 display(0);

#define spiDataPin 10
#define slaveSelectPin 11
#define spiClockPin 12
#define rssiPin A2
// this two are minimum required
#define buttonSeek 2
#define buttonMode 3


// key debounce delay in ms
// NOTE: good values are in the range of 100-200ms
// shorter values will make it more reactive, but may lead to double trigger
#define KEY_DEBOUNCE 100

#define led 13
// RSSI default raw range
#define RSSI_MIN_VAL 90
#define RSSI_MAX_VAL 300
// 75% threshold, when channel is printed in spectrum
#define RSSI_SEEK_FOUND 75
// 80% under max value for RSSI
#define RSSI_SEEK_TRESHOLD 80
// scan loops for setup run
#define RSSI_SETUP_RUN 10

// milliseconds to wait after tuning a channel
// should be > 20
#define MIN_TUNE_TIME 50

#define STATE_SEEK_FOUND 0
#define STATE_SEEK 1
#define STATE_SCAN 2
#define STATE_MANUAL 3
#define STATE_SAVE 4
#define STATE_RSSI_SETUP 5

#define START_STATE STATE_SEEK
#define MAX_STATE STATE_MANUAL

#define CHANNEL_BAND_SIZE 8
#define CHANNEL_MIN_INDEX 0
#define CHANNEL_MAX_INDEX 31

#define CHANNEL_MAX 31
#define CHANNEL_MIN 0

#define OLED_COLS 128
#define OLED_ROWS 64
#define OLED_X_MAX OLED_COLS-1
#define OLED_Y_MAX OLED_ROWS-1
#define SCANNER_BAR_SIZE 20
#define SCANNER_LIST_X_POS 2
#define SCANNER_LIST_Y_POS 8
#define SCANNER_MARKER_SIZE 1
#define MAX_MENU 3
#define MENU_Y_SIZE 11
#define DISPLAY_GRID 8


#define EEPROM_ADR_STATE 0
#define EEPROM_ADR_TUNE 1
#define EEPROM_ADR_RSSI_MIN_L 2
#define EEPROM_ADR_RSSI_MIN_H 3
#define EEPROM_ADR_RSSI_MAX_L 4
#define EEPROM_ADR_RSSI_MAX_H 5
//#define DEBUG
//#define  RSSI_DEBUG

// Channels to sent to the SPI registers
const uint16_t channelTable[] PROGMEM = {
  // Channel 1 - 8
  0x2A05,    0x299B,    0x2991,    0x2987,    0x291D,    0x2913,    0x2909,    0x289F,    // Band A
  0x2903,    0x290C,    0x2916,    0x291F,    0x2989,    0x2992,    0x299C,    0x2A05,    // Band B
  0x2895,    0x288B,    0x2881,    0x2817,    0x2A0F,    0x2A19,    0x2A83,    0x2A8D,    // Band E
  0x2906,    0x2910,    0x291A,    0x2984,    0x298E,    0x2998,    0x2A02,    0x2A0C  // Band F / Airwave
};

// Channels with their Mhz Values
const uint16_t channelFreqTable[] PROGMEM = {
  // Channel 1 - 8
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880  // Band F / Airwave
};

// do coding as simple hex value to save memory.
const uint8_t channelNames[] PROGMEM = {
  0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8,
  0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8,
  0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8,
  0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8
};

// All Channels of the above List ordered by Mhz
const uint8_t channelList[] PROGMEM = {
  19, 18, 17, 16, 7, 8, 24, 6, 9, 25, 5, 10, 26, 4, 11, 27, 3, 12, 28, 2, 13, 29, 1, 14, 30, 0, 15, 31, 20, 21, 22, 23
};

uint8_t channel = 0;
uint8_t channelIndex = 0;
uint8_t rssi = 0;
uint8_t rssi_scaled = 0;
uint8_t hight = 0;
uint8_t state = START_STATE;
uint8_t state_last_used=START_STATE;
uint8_t last_state= START_STATE+1; // force screen draw
uint8_t writePos = 0;
uint8_t switch_count = 0;
uint8_t man_channel = 0;
uint8_t last_channel_index = 0;
uint8_t force_seek=0;
unsigned long time_of_tune = 0;        // will store last time when tuner was changed
uint8_t last_maker_pos=0;
uint8_t last_active_channel=0;
uint8_t first_channel_marker=1;
uint8_t update_frequency_view=0;
uint8_t seek_found=0;
uint8_t last_dip_channel=255;
uint8_t last_dip_band=255;
uint8_t scan_start=0;
uint8_t first_tune=1;
uint8_t force_menu_redraw=0;
uint16_t rssi_min=0;
uint16_t rssi_max=0;
uint16_t rssi_setup_min=0;
uint16_t rssi_setup_max=0;
uint16_t rssi_seek_found=0;
uint16_t rssi_setup_run=0;


// SETUP ----------------------------------------------------------------------------
void setup()
{
    // IO INIT
    // initialize digital pin 13 LED as an output.
    pinMode(led, OUTPUT); // status pin for TV mode errors

    // minimum control pins
    pinMode(buttonSeek, INPUT);
    digitalWrite(buttonSeek, INPUT_PULLUP);
    pinMode(buttonMode, INPUT);
    digitalWrite(buttonMode, INPUT_PULLUP);

    // RSSI Input pin
    pinMode(rssiPin, INPUT);


#ifdef DEBUG
    Serial.begin(115200);
    Serial.println(F("START:"));
#endif
    // SPI pins for RX control
    pinMode (slaveSelectPin, OUTPUT);
    pinMode (spiDataPin, OUTPUT);
	pinMode (spiClockPin, OUTPUT);
    // tune to first channel


    // init OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C,false);  // initialize with the I2C addr 0x3D (for the 128x64)
    display.setTextColor(WHITE),
    display.setTextSize(1),

    // Setup Done - LED ON
    digitalWrite(led, HIGH);

    // use values only of EEprom is not 255 = unsaved
    uint8_t eeprom_check = EEPROM.read(EEPROM_ADR_STATE);
    if(eeprom_check == 255) // unused
    {
        EEPROM.write(EEPROM_ADR_STATE,START_STATE);
        EEPROM.write(EEPROM_ADR_TUNE,CHANNEL_MIN_INDEX);
        // save 16 bit
        EEPROM.write(EEPROM_ADR_RSSI_MIN_L,lowByte(RSSI_MIN_VAL));
        EEPROM.write(EEPROM_ADR_RSSI_MIN_H,highByte(RSSI_MIN_VAL));
        // save 16 bit
        EEPROM.write(EEPROM_ADR_RSSI_MAX_L,lowByte(RSSI_MAX_VAL));
        EEPROM.write(EEPROM_ADR_RSSI_MAX_H,highByte(RSSI_MAX_VAL));
    }
    // debug reset EEPROM
    //EEPROM.write(EEPROM_ADR_STATE,255);

    // read last setting from eeprom
    state=EEPROM.read(EEPROM_ADR_STATE);
    channelIndex=EEPROM.read(EEPROM_ADR_TUNE);
    rssi_min=((EEPROM.read(EEPROM_ADR_RSSI_MIN_H)<<8) | (EEPROM.read(EEPROM_ADR_RSSI_MIN_L)));
    rssi_max=((EEPROM.read(EEPROM_ADR_RSSI_MAX_H)<<8) | (EEPROM.read(EEPROM_ADR_RSSI_MAX_L)));
    force_menu_redraw=1;
}

// LOOP ----------------------------------------------------------------------------
void loop()
{

    /*******************/
    /*   Mode Select   */
    /*******************/
    state_last_used=state; // save save settings
    if (digitalRead(buttonMode) == LOW) // key pressed ?
    {
        delay(KEY_DEBOUNCE); // debounce
        // on entry wait for release
        while(digitalRead(buttonMode) == LOW)
        {
                // wait for MODE release
        }

        uint8_t menu_id=0;
        // Show Mode Screen
        if(state==STATE_SEEK_FOUND)
        {
            state=STATE_SEEK;
        }
        uint8_t in_menu=1;
        uint8_t in_menu_time_out=10; // 10x 200ms = 2 seconds
        /*
        Enter Mode menu
        Show current mode
        Change mode by MODE key
        Any Mode will refresh screen
        If not MODE changes in 2 seconds, it uses last selected mode
        */
        do
        {
            display.clearDisplay();
            // simple menu
            #ifdef DEBUG
                Serial.println(F("MENU"));
            #endif

            display.print(10, 0,  "MODE SELECTION");
            display.print(10, 5+1*MENU_Y_SIZE, "Auto Search");
            display.print(10, 5+2*MENU_Y_SIZE, "Manual Mode");
            display.print(10, 5+3*MENU_Y_SIZE, "Band Scanner");
            display.print(10, 5+4*MENU_Y_SIZE, "Save Setup");
            // selection by inverted box
            switch (menu_id)
            {
                case 0: // auto search
                    display.drawRect(8,3+1*MENU_Y_SIZE,100,12,  WHITE);
                    state=STATE_SEEK;
                    force_seek=1;
                    seek_found=0;
                    #ifdef DEBUG
                        Serial.println(F("auto search"));
                    #endif
                break;
                case 1: // manual mode
                    display.drawRect(8,3+2*MENU_Y_SIZE,100,12,  WHITE);
                    state=STATE_MANUAL;
                    #ifdef DEBUG
                        Serial.println(F("Manual Mode"));
                    #endif
                break;
                case 2: // Band Scanner
                    display.drawRect(8,3+3*MENU_Y_SIZE,100,12,  WHITE);
                    state=STATE_SCAN;
                    scan_start=1;
                    #ifdef DEBUG
                        Serial.println(F("Band Scanner"));
                    #endif
                break;
                case 3: // Save settings
                    display.drawRect(8,3+4*MENU_Y_SIZE,100,12,  WHITE);
                    state=STATE_SAVE;
                break;
            } // end switch
            display.display();

            while(digitalRead(buttonMode) == LOW)
            {
                // wait for MODE release
                in_menu_time_out=10;
            }
            while(--in_menu_time_out && (digitalRead(buttonMode) == HIGH)) // wait for next mode or time out
            {
                delay(200); // timeout delay
            }
            if(in_menu_time_out==0)
            {
                in_menu=0; // EXIT
                delay(KEY_DEBOUNCE); // debounce
            }
            else // no timeout, must be keypressed
            {
                in_menu_time_out=10;
                delay(KEY_DEBOUNCE); // debounce
                /*********************/
                /*   Menu handler   */
                /*********************/
                if (menu_id < MAX_MENU)
                {
                    menu_id++; // next state
                }
                else
                {
                    menu_id = 0;
                }
            }
        } while(in_menu);
        last_state=255; // force redraw of current screen
        switch_count = 0;
        // clean line?
        display.print(OLED_COLS/2, (OLED_ROWS/2), "             ");
        display.display();

    }
    else // key pressed
    { // reset debounce
        switch_count = 0;
    }

    /***************************************/
    /*   Draw screen if mode has changed   */
    /***************************************/
    if(force_menu_redraw || state != last_state)
    {
        force_menu_redraw=0;
        /************************/
        /*   Main screen draw   */
        /************************/
        // changed state, clear an draw new screen
        display.clearDisplay();
        // simple menu
        switch (state)
        {
            case STATE_SCAN: // Band Scanner
            case STATE_RSSI_SETUP: // RSSI setup

                if(state==STATE_SCAN)
                {
                    display.print(10, 0,  " BAND SCANNER");
                }
                else
                {
                    display.print(10, 0,  "  RSSI SETUP ");

                    display.print(10, SCANNER_LIST_Y_POS, "RSSI Min:     RSSI Max:   ");
                    // prepare new setup
                    rssi_min=0;
                    rssi_max=400; // set to max range
                    rssi_setup_min=400;
                    rssi_setup_max=0;
                    rssi_setup_run=RSSI_SETUP_RUN;
                }

                // trigger new scan from begin
                channel=CHANNEL_MIN;
                writePos=SCANNER_LIST_X_POS; // reset channel list
                channelIndex = pgm_read_byte_near(channelList + channel);
                scan_start=1;
            break;
            case STATE_MANUAL: // manual mode
            case STATE_SEEK: // seek mode

                if (state == STATE_MANUAL)
                {
                    display.print(10, 0,  " MANUAL MODE");
                }
                else if(state == STATE_SEEK)
                {
                    display.print(10, 0,  "AUTO MODE SEEK");
                }
                display.print   (5,1*DISPLAY_GRID,  "BAND:");
                // print 1 2 3 4 5 6 7 8 at the right place
                for( int i=0; i<8; i++) {
                    display.print   (i*16+4,2*DISPLAY_GRID, i+1);
                }
                display.print   (5,3*DISPLAY_GRID,  "FREQ:       GHz");

                display.print(5,0+4*DISPLAY_GRID,  "RSSI");

                first_channel_marker=1;
                update_frequency_view=1;
                force_seek=1;
            break;
            case STATE_SAVE:
                EEPROM.write(EEPROM_ADR_STATE,state_last_used);
                EEPROM.write(EEPROM_ADR_TUNE,channelIndex);

                display.drawRect(0,0,127,95,  WHITE);
                display.drawLine(0,14,127,14,WHITE);
                display.print(10, 3,  "SAVE SETTINGS");
                display.print(10, 5+1*MENU_Y_SIZE, "Mode:");
                switch (state_last_used)
                {
                    case STATE_SCAN: // Band Scanner
                        display.print(50,5+1*MENU_Y_SIZE,  "Scanner");
                    break;
                    case STATE_MANUAL: // manual mode
                        display.print(50,5+1*MENU_Y_SIZE,  "Manual");
                    break;
                    case STATE_SEEK: // seek mode
                        display.print(50,5+1*MENU_Y_SIZE,  "Search");
                    break;
                }
                display.print(10, 5+2*MENU_Y_SIZE, "Band:");
                // print band
                if(channelIndex > 23)
                {
                    display.print(50,5+2*MENU_Y_SIZE,  "F/Airwave");
                }
                else if (channelIndex > 15)
                {
                    display.print(50,5+2*MENU_Y_SIZE,  "E");
                }
                else if (channelIndex > 7)
                {
                    display.print(50,5+2*MENU_Y_SIZE,  "B");
                }
                else
                {
                    display.print(50,5+2*MENU_Y_SIZE,  "A");
                }
                display.print(10, 5+3*MENU_Y_SIZE, "Chan:");
                uint8_t active_channel = channelIndex%CHANNEL_BAND_SIZE+1; // get channel inside band
                display.print(50,5+3*MENU_Y_SIZE,active_channel, DEC);
                display.print(10, 5+4*MENU_Y_SIZE, "FREQ:     GHz");
                display.print(50,5+4*MENU_Y_SIZE, pgm_read_word_near(channelFreqTable + channelIndex));
                uint8_t loop=0;
                display.display();

                delay(1000);

                display.print(10, 1+5*MENU_Y_SIZE,     "Hold MODE to enter RSSI setup");
                display.display();
                delay(2000);
                if (digitalRead(buttonMode) == LOW) // to RSSI setup
                {
                    display.print(10, 1+5*MENU_Y_SIZE, "ENTERING RSSI SETUP ......   " );
                    display.display();
                    uint8_t loop=0;

                    state=STATE_RSSI_SETUP;
                    while(digitalRead(buttonMode) == LOW)
                    {
                        // wait for release
                    }
                    delay(KEY_DEBOUNCE);  // debounce
                }
                else
                {
                    display.print(10, 1+5*MENU_Y_SIZE, "                             ");
                    delay(1000);
                    state=state_last_used; // return to saved function
                }
                    force_menu_redraw=1; // we change the state twice, must force redraw of menu

            // selection by inverted box
            break;
        } // end switch
        last_state=state;
    }
    /*************************************/
    /*   Processing depending of state   */
    /*************************************/

    /*****************************************/
    /*   Processing MANUAL MODE / SEEK MODE  */
    /*****************************************/
    if(state == STATE_MANUAL || state == STATE_SEEK )
    {
        if(state == STATE_MANUAL) // MANUAL MODE
        {
            // handling of keys
            if( digitalRead(buttonSeek) == LOW)        // channel UP
            {
                delay(KEY_DEBOUNCE); // debounce
                channelIndex++;
                if (channelIndex > CHANNEL_MAX_INDEX)
                {
                    channelIndex = CHANNEL_MIN_INDEX;
                }
                update_frequency_view=1;
            }

        }
        // display refresh handler
        if(update_frequency_view) // only updated on changes
        {
            //clear last display of bank
            display.fillRect(50,DISPLAY_GRID,9*10,8,BLACK);
            // show current used channel of bank
            if(channelIndex > 23)
            {
                display.print(50,0+1*DISPLAY_GRID,  "F/Airwave");
            }
            else if (channelIndex > 15)
            {
                display.print(50,0+1*DISPLAY_GRID,  "E");
            }
            else if (channelIndex > 7)
            {
                display.print(50,0+1*DISPLAY_GRID,  "B");
            }
            else
            {
                display.print(50,0+1*DISPLAY_GRID,  "A");
            }
            // show channel inside band
            uint8_t active_channel = channelIndex%CHANNEL_BAND_SIZE; // get channel inside band
            if(!first_channel_marker)
            {
                // clear last marker
                display.drawRect(last_active_channel*16+1 ,-1+2*DISPLAY_GRID,10,9,  BLACK);
            }
            first_channel_marker=0;
            // set new marker
            display.drawRect(active_channel*16+1 ,-1+2*DISPLAY_GRID,10,9,  WHITE); // mark current channel
            display.drawLine(active_channel*16+1 ,-1+2*DISPLAY_GRID,active_channel*16+1+10,-1+2*DISPLAY_GRID,BLACK); // make rect open on top
            last_active_channel=active_channel;
            // show frequency
            display.fillRect(50,0+3*DISPLAY_GRID,10*5,8,BLACK);
            display.print(50,0+3*DISPLAY_GRID, pgm_read_word_near(channelFreqTable + channelIndex));
        }
        // show signal strength
        #define RSSI_BAR_SIZE 90
        rssi_scaled=map(rssi, 1, 100, 1, RSSI_BAR_SIZE);
        // clear last bar
        display.fillRect(32, 4*DISPLAY_GRID, RSSI_BAR_SIZE,7 , BLACK);
        display.drawRect(32, 4*DISPLAY_GRID, RSSI_BAR_SIZE, 7 , WHITE);
        //  draw new bar
        display.fillRect(32, 4*DISPLAY_GRID, rssi_scaled, 7 , WHITE);
        // print bar for spectrum
        channel=channel_from_index(channelIndex); // get 0...31 index depending of current channel
        wait_rssi_ready();
        #define SCANNER_BAR_MINI_SIZE 16
        rssi = readRSSI();
        rssi_scaled=map(rssi, 1, 100, 1, SCANNER_BAR_MINI_SIZE);
        hight = (OLED_Y_MAX - rssi_scaled);
        // clear last bar
        display.fillRect((channel * 4), (OLED_Y_MAX - SCANNER_BAR_MINI_SIZE), 3, SCANNER_BAR_MINI_SIZE , BLACK);
        //  draw new bar
        display.fillRect((channel * 4), hight, 3, rssi_scaled , WHITE);
        // set marker in spectrum to show current scanned channel
        if(channel < CHANNEL_MAX_INDEX)
        {
            // clear last square
            display.fillRect((last_maker_pos * 4)+2, (OLED_Y_MAX + 8),SCANNER_MARKER_SIZE,SCANNER_MARKER_SIZE,  BLACK);
            // draw next
            display.fillRect((channel * 4)+2, (OLED_Y_MAX + 8),SCANNER_MARKER_SIZE,SCANNER_MARKER_SIZE,  WHITE);
            last_maker_pos=channel;
        }
        else
        {
          //  No action on last position to keep frame intact
        }
        display.display();

        // handling for seek mode after screen and RSSI has been fully processed
        if(state == STATE_SEEK) //
        { // SEEK MODE
            if(!seek_found) // search if not found
            {
                if ((!force_seek) && (rssi > RSSI_SEEK_TRESHOLD)) // check for found channel
                {
                    seek_found=1;
                }
                else
                { // seeking itself
                    force_seek=0;
                    // next channel
                    if (channel < CHANNEL_MAX)
                    {
                        channel++;
                    } else {
                        channel=CHANNEL_MIN;
                    }
                    channelIndex = pgm_read_byte_near(channelList + channel);
                }
            }
            else
            { // seek was successful
                display.fillRect(10,0,14*10,8,BLACK);
                display.print(10, 0,  "AUTO MODE LOCK");
                if (digitalRead(buttonSeek) == LOW) // restart seek if key pressed
                {
                    delay(KEY_DEBOUNCE); // debounce
                    force_seek=1;
                    seek_found=0;
                    display.fillRect(10,0,14*10,8,BLACK);
                    display.print(10, 0,  "AUTO MODE SEEK");
                }
                display.display();
            }
        }
    }
    /****************************/
    /*   Processing SCAN MODE   */
    /****************************/
    else if (state == STATE_SCAN || state == STATE_RSSI_SETUP)
    {
        // force tune on new scan start to get right RSSI value
        if(scan_start)
        {
            scan_start=0;
            setChannelModule(channelIndex);
            last_channel_index=channelIndex;
            // keep time of tune to make sure that RSSI is stable when required
            time_of_tune=millis();
        }
        // channel marker
        if(channel < CHANNEL_MAX_INDEX)
        {
            // clear last square
            display.fillRect((last_maker_pos * 4)+2, (OLED_Y_MAX + 8),SCANNER_MARKER_SIZE,SCANNER_MARKER_SIZE,  BLACK);
            // draw next
            display.fillRect((channel * 4)+2, (OLED_Y_MAX + 8),SCANNER_MARKER_SIZE,SCANNER_MARKER_SIZE,  WHITE);
            last_maker_pos=channel;
        }
        else
        {
          //  No action on last position to keep frame intact
        }
        // print bar for spectrum
        wait_rssi_ready();
        // value must be ready
        rssi = readRSSI();
        rssi_scaled=map(rssi, 1, 100, 5, SCANNER_BAR_SIZE);
        hight = (OLED_Y_MAX - rssi_scaled);
        // clear last bar
        display.fillRect((channel * 4), (OLED_Y_MAX - SCANNER_BAR_SIZE), 3, SCANNER_BAR_SIZE , BLACK);
        //  draw new bar
        display.fillRect((channel * 4), hight, 3, rssi_scaled , WHITE);
        // print channelname
        if(state == STATE_SCAN)
        {
            if (rssi > RSSI_SEEK_TRESHOLD)
            {
                display.fillRect(0, SCANNER_LIST_Y_POS, OLED_X_MAX, 20,  BLACK);
                display.print(constrain(writePos,0,OLED_X_MAX-40), SCANNER_LIST_Y_POS, pgm_read_byte_near(channelNames + channelIndex), HEX);
                display.print(constrain(writePos+12,0,OLED_X_MAX-30), SCANNER_LIST_Y_POS, pgm_read_word_near(channelFreqTable + channelIndex));
                writePos += 30;
                // mark bar
                display.fillRect((channel * 4) - 3, hight - 5,8*3,8,BLACK);
                display.print((channel * 4) - 3, hight - 5, pgm_read_byte_near(channelNames + channelIndex), HEX);
                display.display();
            }
        }
        // next channel
        if (channel < CHANNEL_MAX)
        {
            channel++;
        } else {
            channel=CHANNEL_MIN;
            writePos=SCANNER_LIST_X_POS; // reset channel list
            if(state == STATE_RSSI_SETUP)
            {
                if(!rssi_setup_run--)
                {
                    // setup done
                    rssi_min=rssi_setup_min;
                    rssi_max=rssi_setup_max;
                    // save 16 bit
                    EEPROM.write(EEPROM_ADR_RSSI_MIN_L,(rssi_min & 0xff));
                    EEPROM.write(EEPROM_ADR_RSSI_MIN_H,(rssi_min >> 8));
                    // save 16 bit
                    EEPROM.write(EEPROM_ADR_RSSI_MAX_L,(rssi_max & 0xff));
                    EEPROM.write(EEPROM_ADR_RSSI_MAX_H,(rssi_max >> 8));
                    state=EEPROM.read(EEPROM_ADR_STATE);
                }
            }
        }
        // new scan possible by press scan
        if (digitalRead(buttonSeek) == LOW) // force new full new scan
        {
            delay(KEY_DEBOUNCE); // debounce
            last_state=255; // force redraw by fake state change ;-)
            channel=CHANNEL_MIN;
            writePos=SCANNER_LIST_X_POS; // reset channel list
            scan_start=1;
        }
        // update index after channel change
        channelIndex = pgm_read_byte_near(channelList + channel);
    }

    /*****************************/
    /*   General house keeping   */
    /*****************************/
    if(last_channel_index != channelIndex)         // tune channel on demand
    {
        setChannelModule(channelIndex);
        last_channel_index=channelIndex;
        // keep time of tune to make sure that RSSI is stable when required
        time_of_tune=millis();

        if(first_tune)
        {
            first_tune=0;
        }
    }
    //rssi = readRSSI();
    display.display();
}

/*###########################################################################*/
/*******************/
/*   SUB ROUTINES  */
/*******************/


uint8_t channel_from_index(uint8_t channelIndex)
{
    uint8_t loop=0;
    uint8_t channel=0;
    for (loop=0;loop<=CHANNEL_MAX;loop++)
    {
        if(pgm_read_byte_near(channelList + loop) == channelIndex)
        {
            channel=loop;
            break;
        }
    }
    return (channel);
}

void wait_rssi_ready()
{
    // CHECK FOR MINIMUM DELAY
    // check if RSSI is stable after tune by checking the time
    uint16_t tune_time = millis()-time_of_tune;
    // module need >20ms to tune.
    // 30 ms will to a 32 channel scan in 1 second.
    if(tune_time < MIN_TUNE_TIME)
    {
        // wait until tune time is full filled
        delay(MIN_TUNE_TIME-tune_time);
    }
}


uint16_t readRSSI()
{
    //return random(0, 100);
    uint16_t rssi = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        rssi += analogRead(rssiPin);
    }
    rssi=rssi/10; // average
    // special case for RSSI setup
    if(state==STATE_RSSI_SETUP)
    { // RSSI setup
        if(rssi < rssi_setup_min)
        {
            rssi_setup_min=rssi;
            display.print(50, SCANNER_LIST_Y_POS, "   ");
            display.print(50, SCANNER_LIST_Y_POS, rssi_setup_min , DEC);
        }
        if(rssi > rssi_setup_max)
        {
            rssi_setup_max=rssi;
        display.print(110, SCANNER_LIST_Y_POS, "   ");
        display.print(110, SCANNER_LIST_Y_POS, rssi_setup_max , DEC);
        }
        // dump current values
    }
    //display.print(50, SCANNER_LIST_Y_POS-10, rssi_min , DEC);
    //display.print(110, SCANNER_LIST_Y_POS-10, rssi_max , DEC);
    // scale AD RSSI Valaues to 1-100%
    //#define RSSI_DEBUG

    // Filter glitches
    #ifdef RSSI_DEBUG
        display.print(1,40, "RAW:             ");
        display.print(30,40, rssi, DEC);
    #endif
    rssi = constrain(rssi, rssi_min, rssi_max);    //original 90---250
    rssi=rssi-rssi_min; // set zero point (value 0...160)
    rssi = map(rssi, 0, rssi_max-rssi_min , 1, 100);   // scale from 1..100%
    #ifdef RSSI_DEBUG
        display.print(1,50, "SCALED:           ");
        display.print(50,50, rssi, DEC);
    #endif

    return (rssi);
}

void setChannelModule(uint8_t channel)
{
  uint8_t i;
  uint16_t channelData;

  //channelData = pgm_read_word(&channelTable[channel]);
  //channelData = channelTable[channel];
  channelData = pgm_read_word_near(channelTable + channel);

  // bit bash out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  //delay(2);
  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();

  SERIAL_SENDBIT0();

  // remaining zeros
  for (i = 20; i > 0; i--)
    SERIAL_SENDBIT0();

  // Clock the data in
  SERIAL_ENABLE_HIGH();
  //delay(2);
  delayMicroseconds(1);
  SERIAL_ENABLE_LOW();

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--)
  {
    // Is bit high or low?
    if (channelData & 0x1)
    {
      SERIAL_SENDBIT1();
    }
    else
    {
      SERIAL_SENDBIT0();
    }

    // Shift bits along to check the next one
    channelData >>= 1;
  }

  // Remaining D16-D19
  for (i = 4; i > 0; i--)
    SERIAL_SENDBIT0();

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  //delay(2);

  digitalWrite(slaveSelectPin, LOW);
  digitalWrite(spiClockPin, LOW);
  digitalWrite(spiDataPin, LOW);
}


void SERIAL_SENDBIT1()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_SENDBIT0()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, LOW);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_LOW()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_HIGH()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(1);
}
