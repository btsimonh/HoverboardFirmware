/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
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
#ifndef PROCOTOL_H
#define PROCOTOL_H

#include "config.h"

#ifdef INCLUDE_PROTOCOL


#define SPEED_COEFFICIENT   0.5  // higher value == stronger. 0.0 to ~2.0?
#define STEER_COEFFICIENT   0.5  // higher value == stronger. if you do not want any steering, set 

void protocol_init();
/////////////////////////////////////////////////////////////////
// call this to deal with received bytes; normally from main loop
void protocol_run();
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// data set & read by protocol.c
typedef struct tag_PROTOCOL_DATA {
    int sensor_control; // enable sensor control, initialised to 1
    int controlling; // set when we start controlling, to initialise speeds


    int speedB;
    int steerB;
    int speedL;
    int speedR;
    int process_immediate; // whether to accept single keystrokes
    int sensor_stabilise; // not used yet

} PROTOCOL_DATA;

extern PROTOCOL_DATA protocol_data;
/////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
// 'machine' protocol structures and definitions
//
// examples:
// ack - 02 02 41 BD
// nack - 02 02 4E B0
// test - 02 06 54 54 65 73 74 06
/////////////////////////////////////////////////////////////////


typedef struct tag_PROTOCOL_MSG {
    unsigned char SOM; // 0x02
    unsigned char len; // len is len of ALL bytes to follow, including CS
    unsigned char bytes[254];  // variable number of data bytes, with a checksum on the end
    // checksum such that sum of bytes len to CS is zero     
} PROTOCOL_MSG;

// content of 'bytes' above, for single byte commands
typedef struct tag_PROTOCOL_BYTES {
    unsigned char cmd; //
    unsigned char bytes[253];
} PROTOCOL_BYTES;


// content of 'bytes' above, for single byte commands
#define PROTOCOL_CMD_READVAL 'R'
typedef struct tag_PROTOCOL_BYTES_READVALS {
    unsigned char cmd; // 'R'
    unsigned char code; // code of value to read
} PROTOCOL_BYTES_READVALS;

#define PROTOCOL_CMD_WRITEVAL 'W'
typedef struct tag_PROTOCOL_BYTES_WRITEVALS {
    unsigned char cmd; // 'W'
    unsigned char code; // code of value to write
    unsigned char content[252]; // value to write
} PROTOCOL_BYTES_WRITEVALS;

/////////////////////////////////////////////////////////
// command definitions
// ack - no payload
#define PROTOCOL_CMD_ACK 'A'
// nack - no payload
#define PROTOCOL_CMD_NACK 'N'

// a test command - normal payload - 'Test'
#define PROTOCOL_CMD_TEST 'T'

// cause unit to restart - no payload
#define PROTOCOL_CMD_REBOOT 'B'

// response to an unkonwn command - maybe payload
#define PROTOCOL_CMD_UNKNOWN '?'

#define PROTOCOL_SOM 2
//
/////////////////////////////////////////////////////////////////




#endif

#endif