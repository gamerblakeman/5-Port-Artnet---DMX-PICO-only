/*
  Art-Net DMX Controller
  Created by Rory McMahon
  Conceptinetics DMX Shield
  Arduino Mega 2560
  Ethernet Shield
*/

#include <Arduino.h>
#include <DmxOutput.h>
#include "defines.h"
/*#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
*/
#define short_get_high_byte(x) ((HIGH_BYTE & x) >> 8)
#define short_get_low_byte(x)  (LOW_BYTE & x)
#define bytes_to_short(h,l) ( ((h << 8) & 0xff00) | (l & 0x00FF) );
// Declare 8 instances of the DMX output
DmxOutput dmxOutputs[5];
#define UNIVERSE_LENGTH 512
uint8_t universe[UNIVERSE_LENGTH + 1];


//byte mac[] = {0x90, 0xA2, 0xDA, 0x0D, 0x4C, 0x8C} ; //the mac adress in HEX of ethernet shield or uno shield board
//byte ip[] = {192, 168, 0, 20}; // the IP adress of your device, that should be in same universe of the network you are using

// the next two variables are set when a packet is received
byte remoteIp[4];        // holds received packet's originating IP
unsigned int remotePort; // holds received packet's originating port

//customisation: Artnet SubnetID + UniverseID
//edit this with SubnetID + UniverseID you want to receive 
byte SubnetID = {0};
byte UniverseID = {0};
short select_universe= ((SubnetID*16)+UniverseID);

//customisation: edit this if you want for example read and copy only 4 or 6 channels from channel 12 or 48 or whatever.
const int number_of_channels=100; //512 for 512 channels
const int start_address=0; // 0 if you want to read from channel 1

#define RXEN_PIN                5

//buffers
const int MAX_BUFFER_UDP=768;
char packetBuffer[MAX_BUFFER_UDP]; //buffer to store incoming data
byte buffer_channel_arduino[number_of_channels]; //buffer to store filetered DMX data

// art net parameters
unsigned int localPort = 6454;      // artnet UDP port is by default 6454
const int art_net_header_size=17;
const int max_packet_size=576;
char ArtNetHead[8]="Art-Net";
char OpHbyteReceive=0;
char OpLbyteReceive=0;
//short is_artnet_version_1=0;
//short is_artnet_version_2=0;
//short seq_artnet=0;
//short artnet_physical=0;
short incoming_universe=0;
boolean is_opcode_is_dmx=0;
boolean is_opcode_is_artpoll=0;
boolean match_artnet=1;
short Opcode=0;
EthernetUDP Udp;
int portAu = 0;
int portBu = 1;
int portCu = 2;
int portDu = 3;
int portEu = 4;

#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 11, en = 10, d4 = 9, d5 = 8, d6 = 7, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
void univ(){
  lcd.clear();
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0,0);
  lcd.print("A: B: C: D: E:");
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(portAu+1);
  lcd.setCursor(3, 1);
  // print the number of seconds since reset:
  lcd.print(portBu+1);
  lcd.setCursor(6, 1);
  // print the number of seconds since reset:
  lcd.print(portCu+1);
  lcd.setCursor(9, 1);
  // print the number of seconds since reset:
  lcd.print(portDu+1);
  lcd.setCursor(12, 1);
  // print the number of seconds since reset:
  lcd.print(portEu+1);
}
void ipdisp(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Awating Sever IP");
  //String text = "ip" + Ethernet.localIP();
  lcd.setCursor(0,1);
  lcd.print(Ethernet.localIP());
  Serial.println(Ethernet.localIP());
 // lcd.print(text);
}
void setup() {
  //delay(4000);
     Serial.begin(9600);
     //while (!Serial);
     Serial.print("hello");
    
    //setup ethernet and udp socket
    lcd.begin(16, 2);
  #if USE_ETHERNET_PORTENTA_H7
  ET_LOGWARN(F("======== USE_PORTENTA_H7_ETHERNET ========"));
#elif USE_NATIVE_ETHERNET
  ET_LOGWARN(F("======== USE_NATIVE_ETHERNET ========"));
#elif USE_ETHERNET_GENERIC
  ET_LOGWARN(F("=========== USE_ETHERNET_GENERIC ==========="));  
#elif USE_ETHERNET_ESP8266
  ET_LOGWARN(F("=========== USE_ETHERNET_ESP8266 ==========="));
#elif USE_ETHERNET_ENC
  ET_LOGWARN(F("=========== USE_ETHERNET_ENC ==========="));  
#else
  ET_LOGWARN(F("========================="));
#endif

#if !(USE_NATIVE_ETHERNET || USE_ETHERNET_PORTENTA_H7)

#if (USING_SPI2)
  #if defined(CUR_PIN_MISO)
    ET_LOGWARN(F("Default SPI pinout:"));
    ET_LOGWARN1(F("MOSI:"), CUR_PIN_MOSI);
    ET_LOGWARN1(F("MISO:"), CUR_PIN_MISO);
    ET_LOGWARN1(F("SCK:"),  CUR_PIN_SCK);
    ET_LOGWARN1(F("SS:"),   CUR_PIN_SS);
    ET_LOGWARN(F("========================="));
  #endif
#else
  ET_LOGWARN(F("Default SPI pinout:"));
  ET_LOGWARN1(F("MOSI:"), MOSI);
  ET_LOGWARN1(F("MISO:"), MISO);
  ET_LOGWARN1(F("SCK:"),  SCK);
  ET_LOGWARN1(F("SS:"),   SS);
  ET_LOGWARN(F("========================="));
#endif

#if defined(ESP8266)
  // For ESP8266, change for other boards if necessary
  #ifndef USE_THIS_SS_PIN
    #define USE_THIS_SS_PIN   D2    // For ESP8266
  #endif

  ET_LOGWARN1(F("ESP8266 setCsPin:"), USE_THIS_SS_PIN);

  #if ( USE_ETHERNET_GENERIC || USE_ETHERNET_ENC )
    // For ESP8266
    // Pin                D0(GPIO16)    D1(GPIO5)    D2(GPIO4)    D3(GPIO0)    D4(GPIO2)    D8
    // EthernetGeneric    X                 X            X            X            X        0
    // Ethernet_ESP8266   0                 0            0            0            0        0
    // D2 is safe to used for Ethernet, Ethernet2, Ethernet3, EthernetLarge libs
    // Must use library patch for Ethernet, EthernetLarge libraries
    Ethernet.init (USE_THIS_SS_PIN);

  #elif USE_CUSTOM_ETHERNET
  
    // You have to add initialization for your Custom Ethernet here
    // This is just an example to setCSPin to USE_THIS_SS_PIN, and can be not correct and enough
    Ethernet.init(USE_THIS_SS_PIN);
  
  #endif  //( USE_ETHERNET_GENERIC || USE_ETHERNET_ENC )

#elif defined(ESP32)

  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

  #ifndef USE_THIS_SS_PIN
    #define USE_THIS_SS_PIN   5   //22    // For ESP32
  #endif

  ET_LOGWARN1(F("ESP32 setCsPin:"), USE_THIS_SS_PIN);

  // For other boards, to change if necessary
  #if ( USE_ETHERNET_GENERIC || USE_ETHERNET_ENC )
    // Must use library patch for Ethernet, EthernetLarge libraries
    // ESP32 => GPIO2,4,5,13,15,21,22 OK with Ethernet, Ethernet2, EthernetLarge
    // ESP32 => GPIO2,4,5,15,21,22 OK with Ethernet3
  
    //Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (USE_THIS_SS_PIN);
  
  #elif USE_CUSTOM_ETHERNET
  
    // You have to add initialization for your Custom Ethernet here
    // This is just an example to setCSPin to USE_THIS_SS_PIN, and can be not correct and enough
    Ethernet.init(USE_THIS_SS_PIN); 
  
  #endif  //( USE_ETHERNET_GENERIC || USE_ETHERNET_ENC )

#elif ETHERNET_USE_RPIPICO

  pinMode(USE_THIS_SS_PIN, OUTPUT);
  digitalWrite(USE_THIS_SS_PIN, HIGH);
  
  // ETHERNET_USE_RPIPICO, use default SS = 5 or 17
  #ifndef USE_THIS_SS_PIN
    #if defined(ARDUINO_ARCH_MBED)
      #define USE_THIS_SS_PIN   5     // For Arduino Mbed core
    #else  
      #define USE_THIS_SS_PIN   17    // For E.Philhower core
    #endif
  #endif

  ET_LOGWARN1(F("RPIPICO setCsPin:"), USE_THIS_SS_PIN);

  // For other boards, to change if necessary
  #if ( USE_ETHERNET_GENERIC || USE_ETHERNET_ENC )
    // Must use library patch for Ethernet, EthernetLarge libraries
    // For RPI Pico using Arduino Mbed RP2040 core
    // SCK: GPIO2,  MOSI: GPIO3, MISO: GPIO4, SS/CS: GPIO5
    // For RPI Pico using E. Philhower RP2040 core
    // SCK: GPIO18,  MOSI: GPIO19, MISO: GPIO16, SS/CS: GPIO17
    // Default pin 5/17 to SS/CS
  
    //Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (USE_THIS_SS_PIN);
     
  #endif    //( USE_ETHERNET_GENERIC || USE_ETHERNET_ENC )

#else   //defined(ESP8266)
  // unknown board, do nothing, use default SS = 10
  #ifndef USE_THIS_SS_PIN
    #define USE_THIS_SS_PIN   10    // For other boards
  #endif

  #if defined(BOARD_NAME)
    ET_LOGWARN3(F("Board :"), BOARD_NAME, F(", setCsPin:"), USE_THIS_SS_PIN);
  #else
    ET_LOGWARN1(F("Unknown board setCsPin:"), USE_THIS_SS_PIN);
  #endif

  // For other boards, to change if necessary
  #if ( USE_ETHERNET_GENERIC || USE_ETHERNET_ENC || USE_NATIVE_ETHERNET )
    // Must use library patch for Ethernet, Ethernet2, EthernetLarge libraries
  
    Ethernet.init (USE_THIS_SS_PIN);
  
  #elif USE_CUSTOM_ETHERNET
  
    // You have to add initialization for your Custom Ethernet here
    // This is just an example to setCSPin to USE_THIS_SS_PIN, and can be not correct and enough
    Ethernet.init(USE_THIS_SS_PIN);
    
  #endif  //( USE_ETHERNET_GENERIC || USE_ETHERNET_ENC )

#endif    // defined(ESP8266)

#endif    // #if !(USE_NATIVE_ETHERNET)

  // start the ethernet connection and the server:
  // Use DHCP dynamic IP and random mac
  uint16_t index = millis() % NUMBER_OF_MAC;
  // Use Static IP
  //Ethernet.begin(mac[index], ip);
  Ethernet.begin(mac[index]);

#if !(USE_NATIVE_ETHERNET || USE_ETHERNET_PORTENTA_H7)
  ET_LOGWARN(F("========================="));
  
  #if defined( ESP32 )
    // Just info to know how to connect correctly
    // To change for other SPI
    ET_LOGWARN(F("Currently Used SPI pinout:"));
    ET_LOGWARN1(F("MOSI:"), PIN_MOSI);
    ET_LOGWARN1(F("MISO:"), PIN_MISO);
    ET_LOGWARN1(F("SCK:"),  PIN_SCK);
    ET_LOGWARN1(F("SS:"),   PIN_SS);
  #else
    #if defined(CUR_PIN_MISO)
      ET_LOGWARN(F("Currently Used SPI pinout:"));
      ET_LOGWARN1(F("MOSI:"), CUR_PIN_MOSI);
      ET_LOGWARN1(F("MISO:"), CUR_PIN_MISO);
      ET_LOGWARN1(F("SCK:"),  CUR_PIN_SCK);
      ET_LOGWARN1(F("SS:"),   CUR_PIN_SS);
    #else
      ET_LOGWARN(F("Currently Used SPI pinout:"));
      ET_LOGWARN1(F("MOSI:"), MOSI);
      ET_LOGWARN1(F("MISO:"), MISO);
      ET_LOGWARN1(F("SCK:"),  SCK);
      ET_LOGWARN1(F("SS:"),   SS);
    #endif
  #endif
  
  ET_LOGWARN(F("========================="));

#elif (USE_ETHERNET_PORTENTA_H7)
  if (Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("No Ethernet found. Stay here forever");
    
    while (true) 
    {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  
  if (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Not connected Ethernet cable");
  }
#endif

  Serial.print(F("Using mac index = "));
  Serial.println(index);

  Serial.print(F("Connected! IP address: "));
  Serial.println(Ethernet.localIP());

  Serial.println(F("\nStarting connection to server..."));
  // if you get a connection, report back via serial:

    
    /*&if (Ethernet.begin(mac) == 0) {
      Serial.println("Failed to configure Ethernet using DHCP");
      // no point in carrying on, so do nothing forevermore:
      lcd.print("Network Conecti-");
      lcd.setCursor(0,1);
      lcd.print("on issue");
      ipdisp();
      for(;;)
        ;
    }*/
    Serial.println(Ethernet.localIP());
    //Ethernet.begin(mac,ip);
    Udp.begin(localPort);

    //pinMode(RXEN_PIN, OUTPUT);
    for (int i = 0; i < 4; i++)
    {
        dmxOutputs[i].begin(i, pio0);
    }
    for (int i = 4; i < 5; i++)
    {
        dmxOutputs[i].begin(i, pio1);
    }
    Serial.print("Setup Compleat");
    lcd.print("hello");
    // ipdisp();
    //univ();
    digitalWrite(RXEN_PIN, HIGH);
}

void loop() {
  
  int packetSize = Udp.parsePacket();
  
  //FIXME: test/debug check
  if(packetSize>art_net_header_size && packetSize<=max_packet_size) {
    // Identify the IP/Port of the device sending us artnet packets
    IPAddress remote = Udp.remoteIP();
    remotePort = Udp.remotePort();
    Udp.read(packetBuffer,MAX_BUFFER_UDP);
    
    //read header
    match_artnet=1;
    for (int i=0;i<7;i++) {
      //if not corresponding, this is not an artnet packet, so we stop reading
      if(char(packetBuffer[i])!=ArtNetHead[i]) {
        match_artnet=0;break;
      }
    }
    //if its an artnet header
    if(match_artnet==1) { 
      //artnet protocole revision, not really needed
      //is_artnet_version_1=packetBuffer[10]; 
      //is_artnet_version_2=packetBuffer[11];*/

      //sequence of data, to avoid lost packets on routeurs
      //seq_artnet=packetBuffer[12];*/
        
      //physical port of  dmx N°
      //artnet_physical=packetBuffer[13];*/
      //operator code enables to know wich type of message Art-Net it is
      Opcode=bytes_to_short(packetBuffer[9],packetBuffer[8]);
      //if opcode is DMX type
      if(Opcode==0x5000) {
        is_opcode_is_dmx=1;is_opcode_is_artpoll=0;
      }
      //if opcode is artpoll 
      else if(Opcode==0x2000) {
        is_opcode_is_artpoll=1;is_opcode_is_dmx=0;
        //( we should normally reply to it, giving ip adress of the device)
      }
      
      //if its DMX data we will read it now
      if(is_opcode_is_dmx=1) {
        univ();
        Serial.println(incoming_universe);
        //digitalWrite(RXEN_PIN, HIGH);
        //read incoming universe
        incoming_universe= bytes_to_short(packetBuffer[15],packetBuffer[14])
        for(int i=start_address;i< UNIVERSE_LENGTH;i++) {
            //buffer_channel_arduino[i-start_address]= byte(packetBuffer[i+art_net_header_size+1]);
            //Serial.println(byte(packetBuffer[i+art_net_header_size+1]));
            universe[i] = uint8_t(byte(packetBuffer[i+art_net_header_size]));
        }
        //if it is selected universe DMX will be read
        if(incoming_universe==portAu){
          Serial.print(universe[1]);
          dmxOutputs[0].write(universe, UNIVERSE_LENGTH + 1);
        }
        if(incoming_universe==portBu){
          dmxOutputs[1].write(universe, UNIVERSE_LENGTH + 1);
        }
        if(incoming_universe==portCu){
          dmxOutputs[2].write(universe, UNIVERSE_LENGTH + 1);
        }
        if(incoming_universe==portDu){
          dmxOutputs[3].write(universe, UNIVERSE_LENGTH + 1);
        }
        if(incoming_universe==portEu){
          dmxOutputs[4].write(universe, UNIVERSE_LENGTH + 1);
        }
        for (int i = 0; i <5; i++)
    {
        while (dmxOutputs[i].busy())
        {
            // Wait patiently until all outputs are done transmitting
        }
    }

        //digitalWrite(RXEN_PIN, LOW);
      }
    }//end of sniffin
  }
}
