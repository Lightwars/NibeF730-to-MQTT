/**
 * Copyright (c) 2010-2022 Contributors to the openHAB project
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * ----------------------------------------------------------------------------
 *
 * Author: pauli.anttila@gmail.com
 *
 */

#include "NibeGw.h"
#include "Arduino.h"

#if defined(HARDWARE_SERIAL_WITH_PINS)
NibeGw::NibeGw(HardwareSerial* serial, int RS485DirectionPin, int RS485RxPin, int RS485TxPin)
#elif defined(HARDWARE_SERIAL)
NibeGw::NibeGw(HardwareSerial* serial, int RS485DirectionPin)
#else
NibeGw::NibeGw(Serial_* serial, int RS485DirectionPin)
#endif
{
#if defined(HARDWARE_SERIAL_WITH_PINS)
  this->RS485RxPin = RS485RxPin;
  this->RS485TxPin = RS485TxPin;
#endif
  verbose = 0;
  ackModbus40 = true;
  ackSms40 = false;
  ackRmu40 = false;
  sendAcknowledge = true;
  state = STATE_WAIT_START;
  connectionState = false;
  RS485 = serial;
  directionPin = RS485DirectionPin;
  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, LOW);
  setCallback(NULL, NULL);
}

void NibeGw::connect() {
  if (!connectionState) {
    state = STATE_WAIT_START;

#if defined(HARDWARE_SERIAL_WITH_PINS)
    RS485->begin(9600, SERIAL_8N1, RS485RxPin, RS485TxPin);
#else
    RS485->begin(9600, SERIAL_8N1);
#endif

    connectionState = true;
  }
}

void NibeGw::disconnect() {
  if (connectionState) {
    RS485->end();
    connectionState = false;
  }
}

boolean NibeGw::connected() {
  return connectionState;
}

void NibeGw::setVerboseLevel(byte level) {
  verbose = level;
}

NibeGw& NibeGw::setCallback(void (*callback_msg_received)(const byte* const data, int len), int (*callback_msg_token_received)(eTokenType token, byte* data)) {
  this->callback_msg_received = callback_msg_received;
  this->callback_msg_token_received = callback_msg_token_received;

  return *this;
}

#ifdef ENABLE_NIBE_DEBUG
NibeGw& NibeGw::setDebugCallback(void (*debug)(byte verbose, char* data)) {
  this->debug = debug;
  return *this;
}
#endif

void NibeGw::setAckModbus40Address(boolean val) {
  ackModbus40 = val;
}

void NibeGw::setAckSms40Address(boolean val) {
  ackSms40 = val;
}

void NibeGw::setAckRmu40Address(boolean val) {
  ackRmu40 = val;
}

void NibeGw::setSendAcknowledge(boolean val) {
  sendAcknowledge = val;
}

boolean NibeGw::messageStillOnProgress() {
  if (!connectionState)
    return false;

  if (RS485->available() > 0)
    return true;

  if (state == STATE_CRC_FAILURE || state == STATE_OK_MESSAGE_RECEIVED)
    return true;

  return false;
}

void NibeGw::loop() {
  if (!connectionState)
    return;

  switch (state) {
    case STATE_WAIT_START:
      if (RS485->available() > 0) {
        byte b = RS485->read();

#ifdef ENABLE_NIBE_DEBUG
        if (debug) {
          sprintf(debug_buf, "%02X ", b);
          debug(3, debug_buf);
        }
#endif

        if (b == 0x5C) {
          buffer[0] = b;
          index = 1;
          state = STATE_WAIT_DATA;

#ifdef ENABLE_NIBE_DEBUG
          if (debug)
            debug(4, "\nFrame start found\n");
#endif
        }
      }
      break;

    case STATE_WAIT_DATA:
      if (RS485->available() > 0) {
        byte b = RS485->read();

#ifdef ENABLE_NIBE_DEBUG
        if (debug) {
          sprintf(debug_buf, "%02X", b);
          debug(3, debug_buf);
        }
#endif

        if (index >= MAX_DATA_LEN) {
          // too long message
          state = STATE_WAIT_START;
        } else {
          buffer[index++] = b;
          int msglen = checkNibeMessage(buffer, index);

#ifdef ENABLE_NIBE_DEBUG
          if (debug) {
            sprintf(debug_buf, "\ncheckMsg=%d\n", msglen);
            debug(5, debug_buf);
          }
#endif

          switch (msglen) {
            case 0: break;                              // Ok, but not ready
            case -1: state = STATE_WAIT_START; break;   // Invalid message
            case -2: state = STATE_CRC_FAILURE; break;  // Checksum error
            default: state = STATE_OK_MESSAGE_RECEIVED; break;
          }
        }
      }
      break;

    case STATE_CRC_FAILURE:
#ifdef ENABLE_NIBE_DEBUG
      if (debug)
        debug(1, "CRC failure\n");
#endif
      if (shouldAckNakSend(buffer[2]))
        sendNak();
      state = STATE_WAIT_START;
      break;

    case STATE_OK_MESSAGE_RECEIVED:
      // now clear double 0x5C, which are to indicate a real value of 0x5C
      for (int i = 5; i < index - 1; i++) {
        if (buffer[i] == 0x5C && buffer[i + 1] == 0x5C) {
          memmove(&buffer[i], &buffer[i + 1], index - 2 - i);
          buffer[4] -= 1;
        }
      }
      index = buffer[4] + 6;

      if (buffer[0] == 0x5C && buffer[1] == 0x00 && buffer[2] == 0x20 && buffer[4] == 0x00 && (buffer[3] == 0x69 || buffer[3] == 0x6B)) {

        eTokenType token = buffer[3] == 0x6B ? WRITE_TOKEN : READ_TOKEN;

#ifdef ENABLE_NIBE_DEBUG
        if (debug)
          debug(1, "Token received\n");
#endif

        int msglen = callback_msg_token_received(token, buffer);
        if (msglen > 0) {
          sendData(buffer, (byte)msglen);
        } else {
#ifdef ENABLE_NIBE_DEBUG
          if (debug)
            debug(2, "No message to send\n");
#endif
          if (shouldAckNakSend(buffer[2]))
            sendAck();
        }
      } else {
#ifdef ENABLE_NIBE_DEBUG
        if (debug) {
          debug(1, "Message received\n");
        }
#endif
        if ((buffer[2] == 0x20) && (buffer[3] == 0xEE)) {
          // Sending Modbus Version v10
          buffer[0] = 0xC0;
          buffer[1] = 0xEE;
          buffer[2] = 0x03;
          buffer[3] = 0x0A;
          buffer[4] = 0x00;
          buffer[5] = 0x02;
          buffer[6] = 0x25;
          sendData(buffer, (byte)7);
        }
        if ((buffer[2] == 0x19) || (buffer[2] == 0x1A) || (buffer[2] == 0x1B) || (buffer[2] == 0x1C)) {  // rmu
          if ((buffer[3] == 0x60) || (buffer[3] == 0x62)) {                                              // send data message
            eTokenType token = buffer[3] == 0x60 ? RMU_WRITE_TOKEN : RMU_TOKEN;
            int msglen = callback_msg_token_received(token, buffer);
            if (msglen > 0) {
              sendData(buffer, (byte)msglen);
            } else {
#ifdef ENABLE_NIBE_DEBUG
              if (debug)
                debug(2, "No message to send\n");
#endif
              if (shouldAckNakSend(buffer[2]))
                sendAck();
            }
          } else if (buffer[3] == 0x63) {
            buffer[0] = 0xC0;
            buffer[1] = 0x60;
            buffer[2] = 0x02;
            buffer[3] = 0x63;
            buffer[4] = 0x00;
            buffer[5] = 0xC1;
            sendData(buffer, (byte)6);
          } else if (buffer[3] == 0xEE) {
            // Sending RMU Version v259
            buffer[0] = 0xC0;
            buffer[1] = 0xEE;
            buffer[2] = 0x03;
            buffer[3] = 0xEE;
            buffer[4] = 0x03;
            buffer[5] = 0x01;
            buffer[6] = 0xC1;
            sendData(buffer, (byte)7);
          } else {
            if (shouldAckNakSend(buffer[2]))
              sendAck();
          }
        }
        if (buffer[2] == 0xA4) {  // eme20
          // for(int bufin = 0; bufin < index; bufin++) {
          //   Serial.printf("%02x", buffer[bufin]);
          // }
          // Serial.println();
          // if (buffer[3] == 0xEE) {
          //   // Sending EME20 Version v???
          //   buffer[0] = 0xC0;
          //   buffer[1] = 0xEE;
          //   buffer[2] = 0x03;
          //   buffer[3] = 0x01;
          //   buffer[4] = 0x00;
          //   buffer[5] = 0x03;
          //   buffer[6] = 0x2F;
          //   sendData(buffer, (byte)7);
          // } else {
            int msglen = callback_msg_token_received(EME20_TOKEN, buffer);
            if (msglen > 0) {
              sendData(buffer, (byte)msglen);
            } else {
              if (shouldAckNakSend(buffer[2]))
                sendAck();
            }
          // }
          // if (buffer[3] == 0x50) {
          //   // Sending EME20 ??? select pv?
          //   buffer[0] = 0xC0;
          //   buffer[1] = 0x50;
          //   buffer[2] = 0x02;
          //   buffer[3] = 0x00;
          //   buffer[4] = 0x00;
          //   buffer[5] = 0x92; // 0x90 ^ 0x02
          //   sendData(buffer, (byte)6);
          // } else if (buffer[3] == 0x60) {
          //   // Sending EME20 ???
          //   buffer[0] = 0xC0;
          //   buffer[1] = 0x60;
          //   buffer[2] = 0x11; // length should be at least 17 or 18
          //   buffer[3] = 0x00; // voltage string 1 low byte /10
          //   buffer[4] = 0x00; // voltage string 1 high byte /10
          //   buffer[5] = 0x00; // voltage string 2 low byte /10
          //   buffer[6] = 0x00; // voltage string 2 high byte /10
          //   buffer[7] = 0x00; // low byte power
          //   buffer[8] = 0x00; // high byte power
          //                     // low byte temperature /10
          //                     // high byte temperature /10
          //                     // fehler code niedrig low byte
          //                     // fehler code niedrig high byte
          //                     // fehler code hoch low byte
          //                     // fehler code hoch high byte
          //                     // 1 byte energy
          //                     // 2 byte energy
          //                     // 3 byte energy
          //                     // 4 byte energy
          //                     // com percentage -> 1 byte 
          //   buffer[9] = 0xA2; // 0x90 ^ 0x02
          //   sendData(buffer, (byte)6);
          // } else if (buffer[3] == 0x70) {
          //   // Sending EME20 ???
          //   buffer[0] = 0xC0; // {2:{112,2,0,0,2,0,10,0,0,0}
          //   buffer[1] = 0x70;
          //   buffer[2] = 0x02;
          //   buffer[3] = 0x00;
          //   buffer[4] = 0x00;
          //   buffer[5] = 0xB2;
          //   sendData(buffer, (byte)6);
          // } else {
          //   if (shouldAckNakSend(buffer[2]))
          //     sendAck();
          // }
        } else {
          if (shouldAckNakSend(buffer[2]))
            sendAck();

          callback_msg_received(buffer, index);
        }
      }
      state = STATE_WAIT_START;
      break;
  }
}

/*
   Return:
    >0 if valid message received (return message len)
     0 if ok, but message not ready
    -1 if invalid message
    -2 if checksum fails
*/
int NibeGw::checkNibeMessage(const byte* const data, byte len) {
  if (len <= 0)
    return 0;

  if (len >= 1) {
    if (data[0] != 0x5C)
      return -1;

    if (len >= 2) {
      if (data[1] != 0x00)
        return -1;
    }

    if (len >= 6) {
      int datalen = data[4];

      if (len < datalen + 6)
        return 0;

      byte checksum = 0;

      // calculate XOR checksum
      for (int i = 2; i < (datalen + 5); i++)
        checksum ^= data[i];

      byte msg_checksum = data[datalen + 5];

#ifdef ENABLE_NIBE_DEBUG
      if (debug) {
        sprintf(debug_buf, "\nchecksum=%02X, msg_checksum=%02X\n", checksum, msg_checksum);
        debug(4, debug_buf);
      }
#endif

      if (checksum != msg_checksum) {
        // if checksum is 0x5C (start character),
        // heat pump seems to send 0xC5 checksum
        if (checksum != 0x5C && msg_checksum != 0xC5)
          return -2;
      }

      return datalen + 6;
    }
  }

  return 0;
}

void NibeGw::sendData(const byte* const data, byte len) {
#ifdef ENABLE_NIBE_DEBUG
  if (debug) {
    debug(1, "Send message to heat pump: ");
    for (int i = 0; i < len; i++) {
      sprintf(debug_buf, "%02X", data[i]);
      debug(1, debug_buf);
    }
    debug(1, "\n");
  }
#endif

  digitalWrite(directionPin, HIGH);
  delay(1);
  RS485->write(data, len);
  RS485->flush();
  delay(1);
  digitalWrite(directionPin, LOW);
}

void NibeGw::sendAck() {
#ifdef ENABLE_NIBE_DEBUG
  if (debug)
    debug(1, "Send ACK\n");
#endif

  digitalWrite(directionPin, HIGH);
  delay(1);
  RS485->write(0x06);
  RS485->flush();
  delay(1);
  digitalWrite(directionPin, LOW);
}

void NibeGw::sendNak() {
#ifdef ENABLE_NIBE_DEBUG
  if (debug)
    debug(1, "Send NACK\n");
#endif

  digitalWrite(directionPin, HIGH);
  delay(1);
  RS485->write(0x15);
  RS485->flush();
  delay(1);
  digitalWrite(directionPin, LOW);
}

boolean NibeGw::shouldAckNakSend(byte address) {
  if (sendAcknowledge) {
    if (address == MODBUS40 && ackModbus40)
      return true;
    else if (address == RMU40 && ackRmu40)
      return true;
    else if (address == SMS40 && ackSms40)
      return true;
    else if (address == EME20)
      return true;
  }

  return false;
}

void NibeGw::testloop() {
  uint16_t index;
  for(index = 0; index < 10000; index++) {
    sendAck();
  }
}