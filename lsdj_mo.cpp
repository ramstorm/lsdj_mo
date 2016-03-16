#include <iostream>
#include <cstdlib>
#include <wiringPi.h>
#include <stdint.h>
#include "RtMidi.h"

using namespace std;

int BYTE_DELAY_US = 80;
int BIT_DELAY_US = 2;
int BEFORE_READ_DELAY_US = 0;
RtMidiOut *midiout = 0;
uint32_t incomingByte = 0;
unsigned char ccNumbers[7] = {1,2,3,7,10,11,12};
uint32_t lastNote[4] = {0,0,0,0};
vector<unsigned char> m1;
vector<unsigned char> m2;
vector<unsigned char> m3;

bool chooseMidiPort(RtMidiOut *rtmidi);
bool getIncomingSlaveByte();
void midioutDoAction(uint32_t m, uint32_t v);
void stopNote(uint32_t m);
void stopAllNotes();
void printMsg(std::vector<unsigned char> *message);

int main(int argc, char *argv[]) {
  bool midiValueMode = false;
  uint32_t previousByte = 0;

  m1.push_back(0);
  m2.push_back(0);
  m2.push_back(0);
  m3.push_back(0);
  m3.push_back(0);
  m3.push_back(0);

  wiringPiSetup();
  pinMode (2, OUTPUT); // connect GPIO-2 to GB clock pin
  pinMode (0, INPUT);  // connect GPIO-0 to GB serial out

  if (argc == 4) {
    BYTE_DELAY_US = atoi(argv[1]);
    BIT_DELAY_US = atoi(argv[2]);
    BEFORE_READ_DELAY_US = atoi(argv[3]);
  }

  cout << "LSDJ MIDI out handler for Raspberry Pi" << endl
       << "RPi - GB pin connections:" << endl
       << "GPIO-2 - GB clock, GPIO-0 - GB serial out, RPi GND - GB GND" << endl << endl
       << "Delay settings (microseconds):" << endl
       << "  byte delay: " << BYTE_DELAY_US << endl
       << "  bit delay: " << BIT_DELAY_US << endl
       << "  before-read delay: " << BEFORE_READ_DELAY_US << endl << endl
       << "To use other delay settings:" << endl
       << "./lsdj_mo <byte_delay> <bit_delay> <before_read_delay>" << endl << endl;

  // Setup MIDI out port
  try {
    midiout = new RtMidiOut();
  }
  catch (RtMidiError &error) {
    error.printMessage();
    exit(EXIT_FAILURE);
  }
  try {
    if (chooseMidiPort(midiout) == false) goto cleanup;
  }
  catch (RtMidiError &error) {
    error.printMessage();
    goto cleanup;
  }

  while(true) {
    if (!getIncomingSlaveByte()) {
      continue;
    }
    if(incomingByte > 0x6f) {
      switch(incomingByte) {
        case 0x7F: //clock tick
          //cout << "*  clock tick" << endl;
          //Serial.write(0xF8);
          break;
        case 0x7E: //seq stop
          cout << "*  seq stop" << endl;
          m1[0] = 0xFC;
          printMsg(&m1);
          midiout->sendMessage(&m1);
          stopAllNotes();
          break;
        case 0x7D: //seq start
          cout << "*  seq start" << endl;
          m1[0] = 0xFA;
          printMsg(&m1);
          midiout->sendMessage(&m1);
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

  // Clean up
  cleanup:
    delete midiout;

  return 0;
}

void printMsg(std::vector<unsigned char> *message) {
  unsigned int nBytes = message->size();
  for (unsigned int i=0; i<nBytes; i++)
    std::cout << "Byte " << i << " = " << (int)message->at(i) << " ";
  std::cout << endl;
}

void midioutDoAction(uint32_t m, uint32_t v) {
  if(m < 4) {
    cout << "[note] (m: " << m << ", v: " << v << ")" << endl;
    //if(v) {
    //  checkStopNote(m);
    //  playNote(m,v);
    //}
    //else if (midiOutLastNote[m]>=0) {
    //  stopNote(m);
    //}

    if (v == 0) {
      stopNote(m);
      lastNote[m] = 0;
      return;
    }
    if (lastNote[m] > 0) {
      stopNote(m);
    }
    cout << "new note" << endl;
    m3[0] = 0x90 + m;
    m3[1] = v;
    m3[2] = 100;
    printMsg(&m3);
    midiout->sendMessage(&m3);

    lastNote[m] = v;
    cout << "note end" << endl;
  }
  else if (m < 8) {
    cout << "[CC] (m: " << m << ", v: " << v << ")" << endl;
    cout << "debugCC: " << ((v >> 4) & 0x07) << endl; 
    m3[0] = 0xB0 + m - 4;
    m3[1] = ccNumbers[(v >> 4) & 0x07];
    m3[2] = (v & 0x0F)*8;
    printMsg(&m3);
    midiout->sendMessage(&m3);
    cout << "cc end" << endl;
  }
  else if(m < 0x0C) {
    cout << "[PC] (m: " << m << ", v: " << v << ")" << endl;
    m2[0] = 0xC0 + m - 8;
    m2[1] = v;
    printMsg(&m2);
    midiout->sendMessage(&m2);
    cout << "pc end" << endl;
  }

  // Temp debugging
  //cout << "[other] (m: " << m << ", v: " << v << ")" << endl;
}

void stopNote(uint32_t m) {
  cout << "note off" << endl;
  m3[0] = 0x80 + m;
  m3[1] = lastNote[m];
  m3[2] = 100;
  printMsg(&m3);
  midiout->sendMessage(&m3);
  cout << "note off end" << endl;
}

void stopAllNotes() {
  cout << "stopallnotes start" << endl;
  for (uint32_t m=0; m<4; m++) {
    if (lastNote[m] > 0) {
      stopNote(m);
    }
    m3[0] = 0xB0 + m;
    m3[1] = 123;
    m3[2] = 0x7F;
    printMsg(&m3);
    midiout->sendMessage(&m3);
  }
  cout << "stopallnotes end" << endl;
}

bool getIncomingSlaveByte() {
  uint32_t bit = 0;
  incomingByte = 0;

  delayMicroseconds(BYTE_DELAY_US);
  digitalWrite(2, LOW);
  delayMicroseconds(BYTE_DELAY_US);
  digitalWrite(2,  HIGH);

  delayMicroseconds(BEFORE_READ_DELAY_US);
  bit = digitalRead(0);
  if (bit == 1) {
    for(int i = 0; i < 7; i++) {
      digitalWrite(2, LOW);
      delayMicroseconds(BIT_DELAY_US);
      digitalWrite(2, HIGH);

      delayMicroseconds(BEFORE_READ_DELAY_US);
      bit = digitalRead(0);

      incomingByte = (incomingByte << 1) + bit;
    }
    return true;
  }
  return false;
}

bool chooseMidiPort( RtMidiOut *rtmidi )
{
  std::cout << "\nWould you like to open a virtual output port? [y/N] ";

  std::string keyHit;
  std::getline( std::cin, keyHit );
  if ( keyHit == "y" ) {
    rtmidi->openVirtualPort();
    return true;
  }

  std::string portName;
  uint32_t i = 0, nPorts = rtmidi->getPortCount();
  if ( nPorts == 0 ) {
    std::cout << "No output ports available!" << std::endl;
    return false;
  }

  if ( nPorts == 1 ) {
    std::cout << "\nOpening " << rtmidi->getPortName() << std::endl;
  }
  else {
    for ( i=0; i<nPorts; i++ ) {
      portName = rtmidi->getPortName(i);
      std::cout << "  Output port #" << i << ": " << portName << '\n';
    }

    do {
      std::cout << "\nChoose a port number: ";
      std::cin >> i;
    } while ( i >= nPorts );
  }

  std::cout << "\n";
  rtmidi->openPort( i );

  return true;
}

