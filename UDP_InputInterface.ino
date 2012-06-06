/*
 UDP Input Interface
 This sketch detects input changes and sends them out
 
 Caleb Kemere - May 2012
 public domain
 */


#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <UDP_IO.h>


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {  
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(10,4,1,3);
unsigned int localPort = 24601;      // local port to listen on

byte InputState;
byte OldInputState;

int InputChangedFlag = 0;

char MyID = '1';

EthernetUDP Udp;

void setup() {

  noInterrupts();
  
  // Set up port/pin change interrupts
  // We want A0-A5 and D6 and D7 - that corresponds to Port C and the high 2 bits of Port D
  // These correspond to PCINT8-13 (PCIE1, PCMSK1, PCINT1) and PCINT22 and 23 (PCIE2, PCMSK2, PCINT2)
  // 
  PCICR |= (1<<PCIE1);
//  PCICR |= (1<<PCIE1) | (1<<PCIE2);
  PCMSK1 |= (1<< PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13);
//  PCMSK2 |= (1<< PCINT22) | (1 << PCINT23);
  MCUCR = 0x01; // trigger on any change
  
  interrupts();

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  PortCMask = B00111111;
  pinMode(7, INPUT);
  pinMode(6, INPUT);
  PortDMask = B11000000;

  pinMode(9, OUTPUT);
  
  PORTC = PortCMask; // set pull up resistors!!!
  PORTD = PortDMask; // set pull up resistors!!!
  
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);

  Serial.begin(9600);
  
  reportIdentity(UDP_INPUT_SYSTEM);
}

ISR(PCINT1_vect) {  
  OldInputState = InputState;
  InputState = (PINC & PortCMask) | (PIND & PortDMask);
  if (OldInputState != InputState)
    InputChangedFlag = 1;
}


void loop() {
  // the next two variables are set when a packet is received
  byte remoteIp[4];        // holds received packet's originating IP
  unsigned int remotePort; // holds received packet's originating port
  
  // buffers for receiving and sending data
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

  char Command;
  
  int packetSize = Udp.parsePacket(); 
  if(packetSize)
  {
    packetSize = packetSize - 8;      // subtract the 8 byte header
    // read the packet into packetBufffer and get the senders IP addr and port number
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
    Command = packetBuffer[0];
 
    if (Command == 'P') {
      int replyLen = 8;
      uint8_t ReplyBuffer[8];       // a string to send back
      reportState((char *)ReplyBuffer,InputState,&replyLen);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer,replyLen);
      Udp.endPacket();
    }
    else {
      uint8_t ErrorBuffer[8] = "D0 E  0";
      ErrorBuffer[1] = MyID;
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ErrorBuffer,8);
      Udp.endPacket();
    } 
  }
  
  if (InputChangedFlag == 1) {
      int replyLen = 8;
      uint8_t ReplyBuffer[8];       // a string to send back
      reportState((char *)ReplyBuffer,InputState,&replyLen);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer,replyLen);
      Udp.endPacket();
    InputChangedFlag = 0;
  }
}



