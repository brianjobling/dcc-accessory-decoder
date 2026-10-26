

//packet assembly - packets are variable length (3 bytes for functions/accessories,
//4 bytes for 128-step speed), each byte followed by a 0 bit, packet ends with a 1 bit
static byte pktBytes[6];
static byte pktByteCount = 0;
static byte curByte = 0;
static byte bitInByte = 0;
static byte pendingHalf = 255; //first half of the bit being assembled, 255 = none

//last-seen state for the watched loco, so we only print when something changes
//(DCC-EX rebroadcasts everything continuously)
static int lastSpeed128 = -1;
static int last28Speed = -1;
static int lastFG1 = -1;   //F0-F4
static int lastFG2a = -1;  //F5-F8
static int lastFG2b = -1;  //F9-F12
static int lastF13_20 = -1;
static int lastF21_28 = -1;
static int lastOtherInstr = -1;

/*The ISR buffers the time of every signal transition (half-bits). Here we drain
that buffer, pair the half-bits into bits, and feed them through preamble
detection and byte assembly. Working in half-bits makes the decoder polarity
independent, so the old RISING/FALLING sync phase is no longer needed.*/
void processDCC() {
  static byte ISRTail = 0;
  byte thisByte;
  unsigned int interruptTime;

  while (ISRTail != ISRHead) { //drain every interval the ISR has buffered
    interruptTime = ISRIntervals[ISRTail++];

    if (interruptTime < 70) dbgIvUnder70++;
    else if (interruptTime < 110) dbgIv70to110++;
    else if (interruptTime < 130) dbgIv110to130++;
    else if (interruptTime < 180) dbgIv130to180++;
    else dbgIvOver180++;

    if (interruptTime > 12000) { //signal gap - start fresh
      pendingHalf = 255;
      Preamble = 0;
      PreambleCounter = 0;
      continue;
    }

    //each transition is HALF a bit: ~58us halves make a "1", longer halves a "0".
    //A bit is two consecutive halves of the same kind; a mismatched pair means
    //we're out of phase, so keep the newest half and try again.
    byte half = (interruptTime < halfbittime) ? 1 : 0;
    if (pendingHalf != half) { //covers both "no pending half" (255) and mismatch
      if (pendingHalf != 255) dbgHalfMismatch++;
      pendingHalf = half;
      continue;
    }
    pendingHalf = 255;
    thisByte = half;
    if (thisByte) dbgBitsOne++; else dbgBitsZero++;

    if (Preamble < 1) { //hasn't finished preamble
      if (thisByte == 1) {
        PreambleCounter++;
      } else {
        PreambleCounter = 0;
      }
      if (PreambleCounter > 9) { //NMRA say a decoder must receive aminimum of 10 "1" bits before accepting a packet
        dbgPreambles++;
        Preamble = 1;//pramble condition met
        PacketStart = 0;
        PreambleCounter = 0;
      }
    } else { //preamble has been completed, wait for packet start
      if (PacketStart < 1) {
        if (thisByte == 0) {
          PacketStart = 1;
          pktByteCount = 0;
          curByte = 0;
          bitInByte = 0;
        }
      } else { //collect bytes until the end-of-packet bit
        if (bitInByte < 8) {
          curByte = (curByte << 1) | thisByte;
          bitInByte++;
        } else { //9th bit: 0 = another byte follows, 1 = end of packet
          pktBytes[pktByteCount] = curByte;
          pktByteCount++;
          curByte = 0;
          bitInByte = 0;
          if (thisByte == 1) { //end of packet
            dbgPktAssembled++;
            processpacket();
            Preamble = 0;
            PreambleCounter = 0;
          } else if (pktByteCount >= 6) { //too long to be real, start again
            Preamble = 0;
            PreambleCounter = 0;
          }
        }
      }
    }
  }
}

void processpacket() {
  if (pktByteCount < 3) return; //too short to be a valid packet

  //last byte is the checksum: XOR of all the bytes before it
  byte checksum = 0;
  for (byte q = 0; q < pktByteCount - 1; q++) {
    checksum ^= pktBytes[q];
  }
  if (checksum != pktBytes[pktByteCount - 1]) { dbgChecksumFail++; return; } //failed checksum, ignore

  dbgGoodPackets++;

#if DEBUG_DCC
  //raw dump of every valid packet. DCC-EX endlessly rebroadcasts the same few
  //packets (idles, plus each loco's speed and function refreshes), so remember
  //the last 8 distinct packets and only print ones we haven't seen recently.
  static byte seenPkts[8][6];
  static byte seenLen[8] = {0};
  static byte seenNext = 0;
  //skip idle packets, and service-mode packets (they get decoded PROG lines instead)
  if (pktBytes[0] != 0xFF && !(pktByteCount == 4 && (pktBytes[0] & 0b11110000) == 0b01110000)) {
    byte isNew = 1;
    for (byte s = 0; s < 8; s++) {
      if (seenLen[s] != pktByteCount) continue;
      byte match = 1;
      for (byte q = 0; q < pktByteCount; q++) {
        if (pktBytes[q] != seenPkts[s][q]) { match = 0; break; }
      }
      if (match) { isNew = 0; break; }
    }
    if (isNew) {
      Serial.print(millis());
      Serial.print("ms RAW");
      for (byte q = 0; q < pktByteCount; q++) {
        Serial.print(" ");
        if (pktBytes[q] < 0x10) Serial.print("0");
        Serial.print(pktBytes[q], HEX);
        seenPkts[seenNext][q] = pktBytes[q];
      }
      seenLen[seenNext] = pktByteCount;
      seenNext = (seenNext + 1) & 7;
      Serial.println("");
    }
  }
#endif

  byte AddressByte = pktBytes[0];
  if (AddressByte == 0 || AddressByte == 0b11111111 || AddressByte == 0b11111110) return; //broadcast/idle/reserved

  //service-mode (programming track) packets: 4 bytes, first byte 0111CCVV.
  //Checked before the loco branch because 0x70-0x7F would otherwise look like
  //short loco addresses 112-127.
  if (pktByteCount == 4 && (AddressByte & 0b11110000) == 0b01110000) {
    ServiceModePacket();
    return;
  }

  if (bitRead(AddressByte, 7) == 0) { //short loco address 1-127
    if (learnMode) {
      learnCheck(AddressByte);
    }
    if (AddressByte == watchLocoAddr) {
      LocoPacket();
    }
  } else if (bitRead(AddressByte, 6) == 0) { //accessory decoder 10xxxxxx
    if (pktByteCount == 3) { //basic packet
      AccDecoder(pktBytes[0], pktBytes[1], pktBytes[2]);
    } else {
      Serial.println("Ext pckt format not supported yet");
    }
  }
}

/*Decode direct-mode programming packets (NMRA S-9.2.3): 0111CCVV VVVVVVVV DDDDDDDD.
CC: 01 = verify byte, 11 = write byte, 10 = single-bit manipulation.
The CV number is the 10-bit value (VV VVVVVVVV) plus 1.

We can't hear the loco's acknowledgement (that's a current pulse only the
command station can sense), but the packets tell the whole story: on a CV
READ the station hunts bit-by-bit and then confirms with a "verify byte"
packet - so the value in the final verify IS the value it read. On a write,
the value is right there in the packet.*/
void ServiceModePacket() {
  static byte lastSvc[4] = {255, 255, 255, 255};
  if (pktBytes[0] == lastSvc[0] && pktBytes[1] == lastSvc[1] &&
      pktBytes[2] == lastSvc[2] && pktBytes[3] == lastSvc[3]) {
    return; //programming packets are repeated many times - print each once
  }
  for (byte q = 0; q < 4; q++) lastSvc[q] = pktBytes[q];

  int cv = (((pktBytes[0] & 0b00000011) << 8) | pktBytes[1]) + 1;
  byte cc = (pktBytes[0] >> 2) & 0b00000011;
  byte d = pktBytes[2];

  Serial.print(millis());
  Serial.print("ms PROG ");
  if (cc == 0b01) {
    Serial.print("verify CV");
    Serial.print(cv);
    Serial.print(" == ");
    Serial.print(d);
    Serial.println("  <-- on a read, this is the value the station found");
    if (cv == 1) { //CV1 = the loco's short address
      Serial.print("          ==========> LOCO ADDRESS IS ");
      Serial.print(d);
      Serial.println(" <==========");
    }
  } else if (cc == 0b11) {
    Serial.print("write CV");
    Serial.print(cv);
    Serial.print(" = ");
    Serial.println(d);
    if (cv == 1 && d >= 1 && d <= 127) {
      //an address is being programmed (DCC-EX <W n>) - adopt it, exactly like
      //a real decoder on the programming track would
      learnMode = 0;
      setWatchLoco(d);
    }
  } else if (cc == 0b10) { //bit manipulation: data byte is 111KDBBB
    Serial.print(bitRead(d, 4) ? "write CV" : "verify CV");
    Serial.print(cv);
    Serial.print(" bit");
    Serial.print(d & 0b00000111);
    Serial.print(bitRead(d, 4) ? " = " : " == ");
    Serial.println(bitRead(d, 3));
  } else {
    Serial.print("unknown svc pkt CV");
    Serial.println(cv);
  }
}

/*learn mode: watch the F0-F4 state of every short-address loco; the first one
whose state CHANGES (the user toggling F0 on their throttle) becomes the
watched loco. Just "first packet seen" wouldn't work because DCC-EX endlessly
rebroadcasts every loco in the session.*/
void learnCheck(byte addr) {
  byte instr = pktBytes[1];
  if ((instr >> 5) != 0b100) return; //only function group one packets
  byte fstate = instr & 0b00011111;  //F0-F4 bits
  if (lastFG1ByAddr[addr] == 255) {
    lastFG1ByAddr[addr] = fstate; //first sighting - remember as baseline
  } else if (lastFG1ByAddr[addr] != fstate) {
    learnMode = 0;
    setWatchLoco(addr);
  }
}

//forget everything about the previously watched loco
void resetLocoState() {
  lastSpeed128 = -1;
  last28Speed = -1;
  lastFG1 = -1;
  lastFG2a = -1;
  lastFG2b = -1;
  lastF13_20 = -1;
  lastF21_28 = -1;
  lastOtherInstr = -1;
}

//decode and print any packet addressed to the watched loco
void LocoPacket() {
  byte instr = pktBytes[1];

  if (instr == 0b00111111 && pktByteCount >= 4) { //128 speed step: next byte is D SSSSSSS
    if ((int)pktBytes[2] != lastSpeed128) {
      lastSpeed128 = pktBytes[2];
      byte spd = pktBytes[2] & 0b01111111;
      curSpeed = (spd <= 1) ? 0 : spd - 1;  //0=stop, 1=estop, else speed 1-126
      curDirFwd = bitRead(pktBytes[2], 7);  //direction bit drives the headlight colour
      locoPrefix();
      Serial.print(" SPEED ");
      Serial.print(bitRead(pktBytes[2], 7) ? "FWD " : "REV ");
      if (spd == 0) {
        Serial.println("STOP");
      } else if (spd == 1) {
        Serial.println("ESTOP");
      } else {
        Serial.print(spd - 1);
        Serial.println("/126");
      }
    }
  } else if ((instr >> 6) == 0b01) { //28 speed step: 01 D SSSSS
    if ((int)instr != last28Speed) {
      last28Speed = instr;
      locoPrefix();
      Serial.print(" SPEED28 ");
      Serial.print(bitRead(instr, 5) ? "FWD raw=" : "REV raw=");
      Serial.println(instr & 0b00011111);
    }
  } else if ((instr >> 5) == 0b100) { //function group one: 100 F0 F4 F3 F2 F1
    if ((int)instr != lastFG1) {
      lastFG1 = instr;
      locoPrefix();
      Serial.print(" F0=");
      Serial.print(bitRead(instr, 4));
      Serial.print(" F1=");
      Serial.print(bitRead(instr, 0));
      Serial.print(" F2=");
      Serial.print(bitRead(instr, 1));
      Serial.print(" F3=");
      Serial.print(bitRead(instr, 2));
      Serial.print(" F4=");
      Serial.println(bitRead(instr, 3));
    }
    ControlLocoFunction(watchLocoAddr, bitRead(instr, 4), instr & 0b00001111);
  } else if ((instr >> 5) == 0b101) { //function group two
    if (bitRead(instr, 4)) { //F5-F8
      if ((int)instr != lastFG2a) {
        lastFG2a = instr;
        locoPrefix();
        printFunctionRange(5, instr);
      }
    } else { //F9-F12
      if ((int)instr != lastFG2b) {
        lastFG2b = instr;
        locoPrefix();
        printFunctionRange(9, instr);
      }
    }
  } else if (instr == 0b11011110 && pktByteCount >= 4) { //F13-F20 expansion
    if ((int)pktBytes[2] != lastF13_20) {
      lastF13_20 = pktBytes[2];
      locoPrefix();
      Serial.print(" F13-F20 bits=");
      Serial.println(pktBytes[2], BIN);
    }
  } else if (instr == 0b11011111 && pktByteCount >= 4) { //F21-F28 expansion
    if ((int)pktBytes[2] != lastF21_28) {
      lastF21_28 = pktBytes[2];
      locoPrefix();
      Serial.print(" F21-F28 bits=");
      Serial.println(pktBytes[2], BIN);
    }
  } else { //anything else addressed to our loco - show the raw bytes so nothing is missed
    if ((int)instr != lastOtherInstr) {
      lastOtherInstr = instr;
      locoPrefix();
      Serial.print(" instr bytes:");
      for (byte q = 1; q < pktByteCount - 1; q++) {
        Serial.print(" 0x");
        Serial.print(pktBytes[q], HEX);
      }
      Serial.println("");
    }
  }
}

void locoPrefix() {
  Serial.print(millis());
  Serial.print("ms LOCO ");
  Serial.print(watchLocoAddr);
}

void printFunctionRange(byte firstFunc, byte instr) {
  for (byte q = 0; q < 4; q++) {
    Serial.print(" F");
    Serial.print(firstFunc + q);
    Serial.print("=");
    Serial.print(bitRead(instr, q));
  }
  Serial.println("");
}

void AccDecoder(byte AddressByte, byte InstructionByte, byte ErrorByte) {
  byte index;
  byte dir;
  int AccAddr;
  int BoardAddr;

  dir = bitRead(InstructionByte, 0);
  index = bitRead(InstructionByte, 1);
  if (bitRead(InstructionByte, 2) > 0) {
    bitSet(index, 1);
  }

  BoardAddr = AddressByte - 0b10000000;
  //now get the weird address system from instruction byte
  if (bitRead(InstructionByte, 4) < 1) {
    bitSet(BoardAddr, 6);
  }
  if (bitRead(InstructionByte, 5) < 1) {
    bitSet(BoardAddr, 7);
  }
  if (bitRead(InstructionByte, 6) < 1) {
    bitSet(BoardAddr, 8);
  }
  AccAddr = ((BoardAddr - 1) * 4) + index + 1;
  ControlAccDecoder(index, dir, AccAddr, BoardAddr);
}
