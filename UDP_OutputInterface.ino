/*
 UDPSetOutput
 This sketch receives a UDP message controlling 14 bits of digital output on the Arduino.
 
 Modified from  UDPSendReceive.pde
 
 Caleb Kemere - April 2012
 public domain
 */


#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <UDP_IO.h>


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {  
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };
IPAddress ip(10,4,1,4);

unsigned int localPort = 24601;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

byte OutputState;
long int OutputCounter[] = {0, 0,   0, 0, 0, 0, 0, 0};

EthernetUDP Udp;

char MyID = '2';

void setup() {

  noInterrupts();  
  TCCR2A = 0;    /* Configure timer2 in normal mode (pure counting, no PWM etc.) */  
  TCCR2B = 0;  
  
  /* Now configure the prescaler to CPU clock divided by 128 */  
  TCCR2B |= (1<<CS22)  | (1<<CS20); // Set bits  
  
  /* (CPU frequency) / (prescaler value) = 125000 Hz = 8us. 
   * (desired period) / 8us = 125. 
   * MAX(uint8) + 1 - 125 = 131; 
   */
  TCNT2 = 131;
  TIMSK2 |= (1<<TOIE2);
  interrupts();
  
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  PortCMask = B00111111;
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  PortDMask = B11000000;
  
  pinMode(9,OUTPUT);

  // start the Ethernet and UDP:
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);

  Serial.begin(9600);
  
  reportIdentity(UDP_OUTPUT_SYSTEM);  
}

/* 
 * Install the Interrupt Service Routine (ISR) for Timer2 overflow. 
 * This is normally done by writing the address of the ISR in the 
 * interrupt vector table but conveniently done by using ISR()  */  
ISR(TIMER2_OVF_vect) {  
  int i;
  /* Reload the timer */  
  TCNT2 = 131;
  for (i = 0; i < 8; i++) {           // Go through each output pin
    if (OutputCounter[i] != 0) {         // If the pin counter is > 0
      OutputState |= (1 << i);           //   keep it high.
      PORTB |= 2;
      if (OutputCounter[i] > 0) {        //   If appropriate,
        OutputCounter[i]--;              //     decrement the counter.
      }
    }
    else { // turn off bit
      OutputState &= ~((byte)(1 << i));  // Else make pin 0;
      PORTB &= (~2);
    }
  }
  PORTC |= (OutputState & PortCMask);
  PORTC &= (OutputState | ~(PortCMask));
  PORTD |= (OutputState) & PortDMask;
  PORTD &= (OutputState | ~(PortDMask));
}  

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket(); 
  if(packetSize)
  {
    //packetSize = packetSize - 8;      // subtract the 8 byte header
    // read the packet into packetBufffer
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
    char Command = packetBuffer[0];
    // Check for command string
    if ((Command == 'S') && (packetSize > 1)) {
      byte newOutputState = packetBuffer[1];
      Serial.print("Raising pin(s):");
      for (int i = 0; i < 8; i++) {
        if ( ((newOutputState >> i) & 1) != 0) {
          OutputCounter[i] = 1000;
          Serial.println(i);
        }
        else {
          OutputCounter[i] = 0;
        }
      }
      uint8_t ReplyBuffer[8];       // a string to send back
      int replyLen = 8;
      reportState((char *)ReplyBuffer,newOutputState,&replyLen);
      ReplyBuffer[0] = 'S';
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer,8);
      Udp.endPacket();
    }
    else if (Command == 'P') {
      int replyLen = 8;
      uint8_t ReplyBuffer[8];       // a string to send back
      reportState((char *)ReplyBuffer,OutputState,&replyLen);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer,replyLen);
      Udp.endPacket();
    }
    else {
      Serial.print("Not a change command:");
      Serial.print(packetSize,HEX);
      Serial.println(packetBuffer);
      char replyBuffer[8];
      int replyLen = 8;
      reportState(replyBuffer,OutputState,&replyLen);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write((uint8_t *)replyBuffer,replyLen);
      Udp.endPacket();
    }
    
  }
}



