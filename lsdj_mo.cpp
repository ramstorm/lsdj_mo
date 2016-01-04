#include <wiringPi.h>
#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

using namespace std;

int BYTE_DELAY_US = 80;
int BIT_DELAY_US = 2;
int BEFORE_READ_DELAY_US = 0;
uint8_t incomingByte = 0;

bool getIncomingSlaveByte();
void midioutDoAction(int m, int v);

int main (int argc, char *argv[]) {
  uint8_t previousByte = 0;
  bool midiValueMode = false;

  wiringPiSetup();
  pinMode (0, OUTPUT); // connect GPIO-0 to GB clock pin
  pinMode (1, INPUT);  // connect GPIO-1 to GB serial out

  if (argc == 4) {
    BYTE_DELAY_US = atoi(argv[1]);
    BIT_DELAY_US = atoi(argv[2]);
    BEFORE_READ_DELAY_US = atoi(argv[3]);
  }

  cout << "LSDJ MIDI out handler for Raspberry Pi" << endl
       << "Connect RPi to GB as follows:" << endl
       << "GPIO-0 - GB clock, GPIO-1 - GB serial out, RPi GND - GB GND" << endl << endl
       << "Using these delay settings (microseconds):" << endl
       << "  byte delay: " << BYTE_DELAY_US << endl
       << "  bit delay: " << BIT_DELAY_US << endl
       << "  before-read delay: " << BEFORE_READ_DELAY_US << endl << endl
       << "To use other delay settings:" << endl
       << "./lsdj_mo <byte_delay> <bit_delay> <before_read_delay>" << endl << endl;

  while(true) {
    if (!getIncomingSlaveByte()) {
      continue;
    }
    if(incomingByte > 0x6f) {
      switch(incomingByte) {
        case 0x7F: //clock tick
          cout << "*  clock tick" << endl;
          //Serial.write(0xF8);
          break;
        case 0x7E: //seq stop
          cout << "*  seq stop" << endl;
          //Serial.write(0xFC);
          //stopAllNotes();
          break;
        case 0x7D: //seq start
          cout << "*  seq start" << endl;
          //Serial.write(0xFA);
          break;
        default:
          previousByte = (incomingByte - 0x70);
          midiValueMode = true;
          break;
      }
    }
    else if (midiValueMode == true) {
      cout << "*  midi value ";
      midiValueMode = false;
      midioutDoAction(previousByte, incomingByte);
    }
  }

  return 0;
}

void midioutDoAction(int m, int v) {
  if(m < 4) {
    cout << "[note] (m: " << m << ", v: " << v << ")" << endl;
    //if(v) {
    //  checkStopNote(m);
    //  playNote(m,v);
    //}
    //else if (midiOutLastNote[m]>=0) {
    //  stopNote(m);
    //}
  }
  else if (m < 8) {
    cout << "[CC] (m: " << m << ", v: " << v << ")" << endl;
    //m -= 4;
    //playCC(m,v);
  }
  else if(m < 0x0C) {
    cout << "[PC] (m: " << m << ", v: " << v << ")" << endl;
    //m -= 8;
    //playPC(m,v);
  }

  // Temp debugging
  cout << "[other] (m: " << m << ", v: " << v << ")" << endl;
}

bool getIncomingSlaveByte() {
  uint8_t bit = 0;
  incomingByte = 0;

  usleep(BYTE_DELAY_US);
  digitalWrite(0, LOW);
  usleep(BYTE_DELAY_US);
  digitalWrite(0,  HIGH);

  usleep(BEFORE_READ_DELAY_US);
  bit = digitalRead(1);
  if (bit == 1) {
    for(int i = 0; i < 8; i++) {
      digitalWrite(0, LOW);
      usleep(BIT_DELAY_US);
      digitalWrite(0, HIGH);

      usleep(BEFORE_READ_DELAY_US);
      bit = digitalRead(1);

      incomingByte = (incomingByte << 1) + bit;
    }
    return true;
  }
  return false;
}

