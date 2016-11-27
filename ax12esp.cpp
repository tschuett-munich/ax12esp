/*
  ax12esp.cpp - former arbotiX Library for AX-12 Servos, modified for ESP8266
  Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.
  Modificada el 15/11/09 por Pablo Gindel.
  Modified 08/2016 to 11/2016 for ESP8266 by Thomas Schuett , www.roboshock.de
  -- ax12esp V1.0

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "ax12esp.h"
#include "Arduino.h"
#include <string.h>

#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)   
#include <avr/interrupt.h>
#endif    

byte AX12::ax_rx_buffer[AX12_BUFFER_SIZE];
byte AX12::ax_rx_Pointer;

boolean debugx = false;  // the ESP8266 has a Serial1 for debug output, which can only send data

/******************************************************************************
 * Hardware UART of ESP8266 has a HW buffer, and the controller runs at 80 MHz.
 * Therefore we dont need interrupt driven receive data any more.
 ******************************************************************************/

/*
* helper functions to emulate half-duplex - not available in ESP8266.
* ts: Explizit switching of a controller pin is unreliable, as it is hard to tell,
* when the last bit of the last byte from the HW sendbuffer is out.
* Instead, we use an external circuit with a monoflop, retriggered by Tx edges.
*/
void AX12::setTX () {
}

void AX12::setRX () {
}

void AX12::setNone () {
}

/** Sends a character out the serial port */
byte AX12::writeByte (byte data) {
    Serial.write(data);
    return data;
}

/** initializes serial transmit at baud, 8-N-1 */
void AX12::init (long baud) {
}

/******************************************************************************
 * Initialization
 ******************************************************************************/

byte AX12::autoDetect (byte* list_motors, byte num_motors) { // max possible id is 253
                                                             
    byte counter = 0;      
    byte *data;
    byte retries = 0;
    for (byte i=0; i<254; i++) {
      delay(1);
      if (debugx) { Serial1.print("testing servo "); Serial1.println(i); }
      sendPacket (i, 0, AX_PING, data);
      byte index = readPacket (); // (13); //ts: size of ping response: 6: FF FF ID LENGTH ERROR CHECKSUM
      byte id = ax_rx_buffer [index];
      byte int_error = ax_rx_buffer [index+1];
      if (debugx) { Serial1.print("id "); Serial1.println(id); }
      if (debugx) { Serial1.print("err "); Serial1.println(int_error); }
      if (int_error==0 && id==i) {
        list_motors[counter++] = i;
        if (counter == num_motors) {break;}
      }
      else {
        if (retries < 2) { // ts: 1st try + 2 retries -> 3 tries altogehter
          i--;        // retry with this ID
          retries++;
        }
        else {
          retries = 0; // move on to next ID
        }
      }
    }
    return counter;
}

/** constructors */
AX12::AX12 (long baud, byte motor_id, boolean inv) {
    id = motor_id;
    inverse = inv;
    SRL = RETURN_ALL;
    init (baud);
}

AX12::AX12 (byte motor_id, boolean inv) {
    id = motor_id;
    inverse = inv;
    SRL = RETURN_ALL;
}

AX12::AX12 (long baud, byte motor_id) {
    id = motor_id;
    inverse = false;
    SRL = RETURN_ALL;
    init (baud);
}

AX12::AX12 (byte motor_id) {
    id = motor_id;
    inverse = false;
    SRL = RETURN_ALL;
}

AX12::AX12 () {
    id = BROADCAST_ID;
    inverse = false;
    SRL = RETURN_READ; // ts: orig: RETURN_NONE
}

/******************************************************************************
 * Packet Level
 ******************************************************************************/

/** send instruction packet */
void AX12::sendPacket (byte _id, byte datalength, byte instruction, byte* data) {
    byte checksum = 0;
    setTX();
    writeByte (0xFF);
    writeByte (0xFF);
    checksum += writeByte (_id);
    checksum += writeByte (datalength + 2);
    checksum += writeByte (instruction);
    for (byte f=0; f<datalength; f++) {
      checksum += writeByte (data[f]);
    }
    // checksum = //
    writeByte (~checksum);
    setRX();
}

// =============================================================================
        


byte AX12::readPacket () {  // FF FF ID LENGTH ERROR [PARAMS] CHECKSUM
    unsigned long ulCounter;
    byte timeout, error, status_length, checksum, offset, bcount;
    offset = 0; timeout = 0; bcount = 0;

    boolean inStartPhase = true;
    boolean lastWasFF = false;
    boolean thisIsFF = false;

    int expectedBruttoByteCount = 6; // expectedDataByteCount + 6 // 6 for FF FF ID LENGTH ERROR [] CHECKSUM
     //  size of ping response: 6: FF FF ID LENGTH ERROR CHECKSUM
     // FF FF ID len err param1 param2 param3 ... chksum

    while (bcount < expectedBruttoByteCount) {
        int in = -1;
        int waited = 0;

        while ((in = Serial.read()) == -1) {
            waited++;
            if (waited > 5000) {
                timeout = 1;
                break;
            }
        };

        if (timeout) break;

        if (debugx) { Serial1.print(in); Serial1.print(","); }

        if (inStartPhase) {
            thisIsFF = ((in & 0x0ff) == 0xff);
            if (thisIsFF && lastWasFF) {
                inStartPhase = false; 
                bcount = 2; // even if there were other bytes before this FF FF
            }
            lastWasFF = thisIsFF;
            continue;
        }

        ax_rx_buffer[bcount++] = in & 0x0ff;

        if (bcount == 4) expectedBruttoByteCount = (in & 0x0ff) + 4;
    }
    setNone();

    offset=2;  // now offset points to the 1st byte after leading FF FF
    bcount= bcount - 2;   // bcount is the rest without FF FF


    error = 0; // = timeout (0 or 1)
    status_length = 2 + ax_rx_buffer[offset+1];

    if (bcount != (status_length)) error+=2;   

    checksum = 0;
    for (byte f=0; f<status_length; f++)
        checksum += ax_rx_buffer[offset+f];
    if (checksum != 255) error+=4;

    ax_rx_buffer[offset+1] = error;  // we (re)use the byte where the LENGTH was stored 

    if (debugx) { 
      Serial1.print(" "); Serial1.print(error);
      Serial1.print(" "); Serial1.print(bcount);
      Serial1.print(" "); Serial1.print(status_length);
      Serial1.print(" "); Serial1.print(checksum);
      Serial1.println();
    }

    return offset;   
}

/******************************************************************************
 * Instruction Level
 ******************************************************************************/

/** ping */
int AX12::ping () {
  byte* data;
  sendPacket (id, 0, AX_PING, data);
  return returnData (RETURN_NONE, 0).error;
}

/** reset */
int AX12::reset () {
  byte* data;
  sendPacket (id, 0, RESET, data);
  return returnData (RETURN_ALL, 0).error;
}

/** action */
int AX12::action () {
  byte *data;
  sendPacket (id, 0, ACTION, data);
  return returnData (RETURN_ALL, 0).error;
}

/** read data */
AX12data AX12::readData (byte start, byte length) {
  byte data [2];
  data [0] = start; data [1] = length;
  sendPacket (id, 2, READ_DATA, data);
  return returnData (RETURN_READ, length);
}

/** write data + reg write */
int AX12::writeData (byte start, byte length, byte* values, boolean isReg) {
    byte data [length+1];
    data [0] = start; 
    memcpy (&data[1], values, length);
    if (isReg) {
      sendPacket (id, length+1, REG_WRITE, data);
    } else {
      sendPacket (id, length+1, WRITE_DATA, data);
    }
    int error = returnData (RETURN_ALL).error;
    if (start < 23) {delay (5);}                   // If write to EEPROM, this delay is needed
    return error;
}

int AX12::writePosSpeed (int pos, int speed) {
	byte length = 4;
    byte data [length+1];
    data [0] = GOAL_POSITION; 
	data [1] = lowByte(pos);
	data [2] = highByte(pos);
	data [3] = lowByte(speed);
	data [4] = highByte(speed);

	sendPacket (id, length+1, WRITE_DATA, data);
    int error = returnData (RETURN_ALL).error;
    return error;
}

/** sync write */
void AX12::syncWrite (byte start, byte length, byte targetlength, byte* targets, byte** valuess) {
    byte rowlength = length + 1;
    byte superlength = rowlength*targetlength + 2; 
    byte data [superlength];
    data [0] = start; 
    data [1] = length;
    byte index = 2;
    for (byte f=0; f<targetlength; f++) {
      data [index++] = targets[f];
      memcpy (&data[index], valuess[f], length);
      index += length;
    }
    sendPacket (BROADCAST_ID, superlength, SYNC_WRITE, data);
    setNone();   
}


/******************************************************************************
 * Register Level
 ******************************************************************************/

/** "intelligent" read data */
AX12info AX12::readInfo (byte registr) {
    byte reglength = lengthRead (registr);
    AX12info returninfo;
    returninfo.error = -2;
    if (reglength == 0) {return returninfo;}
    AX12data returndata = readData (registr, reglength);
    returninfo.error = returndata.error;
    returninfo.value = makeInt (returndata.data, reglength);    
    processValue (registr, &returninfo.value);
    return returninfo;
}

/** "intelligent" write data + reg write */
// seteando a "true" el parC!metro adicional se transforma en un reg write
int AX12::writeInfo (byte registr, int value, boolean isReg) {
    byte reglength = lengthWrite (registr);
    if (reglength==0) {return -2;}
    processValue (registr, &value);
    byte values [reglength];
    values [0] = lowByte(value);
    if (reglength > 1) {values[1] = highByte(value);}
    return writeData (registr, reglength, values, isReg);
}

/** "intelligent" sync write */
void AX12::syncInfo (byte registr, byte targetlength, byte* targets, int* values) {
    byte reglength = lengthWrite (registr);
    if (reglength==0) {return;}
    byte valuess [targetlength][reglength];
    byte * pointers [targetlength];
    for (byte f=0; f<targetlength; f++) {
      valuess [f][0] = lowByte(values[f]);
      if (reglength > 1) {valuess[f][1] = highByte(values[f]);}
      pointers[f] = &valuess[f][0];
    }    
    //nota: la sync write no respeta la propiedad "inverse"
    syncWrite (registr, reglength, targetlength, targets, pointers);
}


/******************************************************************************
 * Macro Level
 ******************************************************************************/

void AX12::setEndlessTurnMode (boolean endless) {
    writeInfo (CW_ANGLE_LIMIT, 0);
    if (endless) {
      writeInfo (CCW_ANGLE_LIMIT, 0);
    } else {
      writeInfo (CCW_ANGLE_LIMIT, 1023);
    }
}

void AX12::endlessTurn (int velocidad) {
    boolean direccion = sign2bin (velocidad);
    writeInfo (MOVING_SPEED, abs(velocidad)|((direccion^inverse)<<10));
}

int AX12::presentPSL (int* PSL) {                        // position, speed & load in one packet
    AX12data data = readData (PRESENT_POSITION, 6);
    for (byte f=0; f<3; f++) {
      PSL[f] = makeInt (&data.data[2*f], 2);  
      processValue (PRESENT_POSITION + 2*f, &PSL[f]);
    }
    return data.error;
}

// always write a SRL on startup, so the value in this lib is in sync with the servos
void AX12::setSRL (byte _srl) {
  SRL = _srl;
  writeInfo (STATUS_RETURN_LEVEL, SRL);
}

void AX12::changeID (byte newID) {
  if (newID > 253) {return;}
  writeInfo (ID, newID);
  id = newID;
}
 
int AX12::setPosVel (int pos, int vel) {
    processValue (GOAL_POSITION, &pos);
    byte values [4];
    values [0] = lowByte(pos);
    values[1] = highByte(pos);
    values [2] = lowByte(vel);
    values[3] = highByte(vel);
    return writeData (GOAL_POSITION, 4, values);
}

void AX12::setMultiPosVel (byte targetlength, byte* targets, int* posvalues, int* velvalues) {
    byte valuess [targetlength][4];
    byte * pointers [targetlength];
    for (byte f=0; f<targetlength; f++) {
      valuess [f][0] = lowByte(posvalues[f]);
      valuess[f][1] = highByte(posvalues[f]);
      valuess [f][2] = lowByte(velvalues[f]);
      valuess[f][3] = highByte(velvalues[f]);
      pointers[f] = &valuess[f][0];
    }    
    //nota: la sync write no respeta la propiedad "inverse"
    syncWrite (GOAL_POSITION, 4, targetlength, targets, pointers);  
}


/******************************************************************************
 * Misc.
 ******************************************************************************/

boolean sign2bin (int numero) {         // numero > 0 --> true; numero <= 0 --> false
  return (numero > 0);
}

char bin2sign (boolean var) {           // var = 0 --> sign = -1; var = 1 --> sign = 1
  return 2*var - 1;
}

int makeInt (byte *dir, byte reglength) {          // transforma 2 bytes en un int (segC:n la lC3gica AX12)
  if (reglength > 1) {
    return (dir[1] << 8) | dir[0];
  } else {
    return dir[0];
  }
}

byte lengthRead (byte registr) {
    byte reglength = 0;
    switch (registr) {
      case VERSION: case ID: case BAUD_RATE: case RETURN_DELAY_TIME: 
      case LIMIT_TEMPERATURE: case DOWN_LIMIT_VOLTAGE: case UP_LIMIT_VOLTAGE: 
      case STATUS_RETURN_LEVEL: case ALARM_LED: case ALARM_SHUTDOWN: case 19: case TORQUE_ENABLE: case LED: 
      case CW_COMPLIANCE_MARGIN: case CCW_COMPLIANCE_MARGIN: case CW_COMPLIANCE_SLOPE: case CCW_COMPLIANCE_SLOPE: 
      case PRESENT_VOLTAGE: case PRESENT_TEMPERATURE: case REGISTERED_INSTRUCTION: case MOVING: case LOCK: reglength = 1; break;
      case MODEL_NUMBER: case CW_ANGLE_LIMIT: case CCW_ANGLE_LIMIT: 
      case MAX_TORQUE: case DOWN_CALIBRATION: case UP_CALIBRATION: 
      case GOAL_POSITION: case MOVING_SPEED: case TORQUE_LIMIT: 
      case PRESENT_POSITION: case PRESENT_SPEED: case PRESENT_LOAD: case PUNCH: reglength = 2; break;
    }
    return reglength;
}

byte lengthWrite (byte registr) {
    byte reglength = 0;
    switch (registr) {
      case VERSION: 
      case ID: 
      case BAUD_RATE: 
      case RETURN_DELAY_TIME: 
      case LIMIT_TEMPERATURE: 
      case DOWN_LIMIT_VOLTAGE: 
      case UP_LIMIT_VOLTAGE: 
      case STATUS_RETURN_LEVEL: 
      case ALARM_LED: 
      case ALARM_SHUTDOWN: 
      case TORQUE_ENABLE: 
      case LED: 
      case CW_COMPLIANCE_MARGIN: 
      case CCW_COMPLIANCE_MARGIN: 
      case CW_COMPLIANCE_SLOPE: 
      case CCW_COMPLIANCE_SLOPE: 
      case PRESENT_VOLTAGE: 
      case PRESENT_TEMPERATURE: 
      case REGISTERED_INSTRUCTION: 
      case MOVING: 
      case LOCK: 
            reglength = 1; break;

      case MODEL_NUMBER: 
      case CW_ANGLE_LIMIT: 
      case CCW_ANGLE_LIMIT: 
      case MAX_TORQUE: 
      case GOAL_POSITION: 
      case MOVING_SPEED: 
      case TORQUE_LIMIT: 
      case PRESENT_POSITION: 
      case PRESENT_SPEED: 
      case PRESENT_LOAD: 
      case PUNCH: 
            reglength = 2; break;
    }
    return reglength;
}


AX12data AX12::returnData (byte _srl) {
  return AX12::returnData (_srl, 13 - 6);  // originally 13 byte was the max expected byte count
}

AX12data AX12::returnData (byte _srl, byte expectedDataByteCount) {
  AX12data returndata;
  if (SRL >= _srl) {
    byte index = readPacket ();
    byte status_id = ax_rx_buffer [index];
    byte intern_error = ax_rx_buffer [index+1];
    byte status_error = ax_rx_buffer [index+2];
    returndata.error = (intern_error<<7)   // ts: moving +1,+2,+4 to bits 8,9,10
                       | status_error      // uses bits 0-7
                       | ((status_id != id)<<10); // shifts servo-id discrepance to bit 11
    returndata.data = (byte*) &(ax_rx_buffer [index+3]);
  } else {
    setNone();
    returndata.error = -1;
  }
  return returndata;
}

void AX12::processValue (byte registr, int* value) {   // procesa el valor para la salida segun la propiedad "inverse"
  switch (registr) {
    case PRESENT_POSITION: 
    case GOAL_POSITION:
      if (inverse) {*value = 1023 - *value;}
      break;
    case PRESENT_SPEED: 
    case PRESENT_LOAD:
      *value = ((*value)&0x03FF) * bin2sign(((*value)>0x03FF)^inverse); 
      break;
  } 
}

