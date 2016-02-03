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

bool chooseMidiPort( RtMidiOut *rtmidi );
bool getIncomingSlaveByte();
void midioutDoAction(uint32_t m, uint32_t v);
void stopNote(uint32_t m);
void stopAllNotes();

int main(int argc, char *argv[]) {
  bool midiValueMode = false;
  uint32_t previousByte = 0;

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
      vector<unsigned char> message(3);
      switch(incomingByte) {
        case 0x7F: //clock tick
          //cout << "*  clock tick" << endl;
          //Serial.write(0xF8);
          break;
        case 0x7E: //seq stop
          cout << "*  seq stop" << endl;
          message.push_back(0xFC);
          midiout->sendMessage(&message);
          stopAllNotes();
          break;
        case 0x7D: //seq start
          cout << "*  seq start" << endl;
          message.push_back(0xFA);
          midiout->sendMessage(&message);
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

void midioutDoAction(uint32_t m, uint32_t v) {
  vector<unsigned char> message(3);

  if(m < 4) {
    cout << "[note] (m: " << m << ", v: " << v << ")" << endl;
    //if(v) {
    //  checkStopNote(m);
    //  playNote(m,v);
    //}
    //else if (midiOutLastNote[m]>=0) {
    //  stopNote(m);
    //}


    if (lastNote[m] > 0) {
      stopNote(m);
    }
    cout << "new note" << endl;
    message.push_back(0x90 + m);
    message.push_back(v);
    message.push_back(100);
    midiout->sendMessage(&message);

    lastNote[m] = v;
    cout << "note end" << endl;
  }
  else if (m < 8) {
    cout << "[CC] (m: " << m << ", v: " << v << ")" << endl;
    cout << "debugCC: " << ((v >> 4) & 0x07) << endl; 
    message.push_back(0xB0 + m - 4);
    message.push_back(ccNumbers[(v >> 4) & 0x07]);
    message.push_back((v & 0x0F)*8);
    midiout->sendMessage(&message);
    cout << "cc end" << endl;
  }
  else if(m < 0x0C) {
    cout << "[PC] (m: " << m << ", v: " << v << ")" << endl;
    message.push_back(0xC0 + m - 8);
    message.push_back(v);
    midiout->sendMessage(&message);
    cout << "pc end" << endl;
  }

  // Temp debugging
  //cout << "[other] (m: " << m << ", v: " << v << ")" << endl;
}

void stopNote(uint32_t m) {
  cout << "note off" << endl;
  vector<unsigned char> message(3);
  message.push_back(0x80 + m);
  message.push_back(lastNote[m]);
  message.push_back(100);
  midiout->sendMessage(&message);
  cout << "note off end" << endl;
}

void stopAllNotes() {
  cout << "stopallnotes start" << endl;
  for (uint32_t m=0; m<4; m++) {
    if (lastNote[m] > 0) {
      stopNote(m);
    }
    vector<unsigned char> message(3);
    message.push_back(0xB0 + m);
    message.push_back(123);
    message.push_back(0x7F);
    midiout->sendMessage(&message);
  }
  cout << "stopallnotes end" << endl;
}

bool getIncomingSlaveByte() {
  uint32_t bit = 0;
  incomingByte = 0;

  delayMicroseconds(BYTE_DELAY_US);
  digitalWrite(0, LOW);
  delayMicroseconds(BYTE_DELAY_US);
  digitalWrite(0,  HIGH);

  delayMicroseconds(BEFORE_READ_DELAY_US);
  bit = digitalRead(1);
  if (bit == 1) {
    for(int i = 0; i < 7; i++) {
      digitalWrite(0, LOW);
      delayMicroseconds(BIT_DELAY_US);
      digitalWrite(0, HIGH);

      delayMicroseconds(BEFORE_READ_DELAY_US);
      bit = digitalRead(1);

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

