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
 *  Author: pauli.anttila@gmail.com
 *
 *
 *  2.11.2013   v1.00   Initial version.
 *  3.11.2013   v1.01
 *  27.6.2014   v1.02   Fixed compile error and added Ethernet initialization delay.
 *  29.6.2015   v2.00   Bidirectional support.
 *  18.2.2017   v3.00   Redesigned.
 *  14.3.2021   v3.01   Fix Prodino build + fixed UDP issue + debug improvements
 *  3.7.2022    v4.00   Send messages to IP address received from the UDP messages
 *  13.7.2022   v4.01   Fixed target IP address issue
 *  29.7.2022   v5.00   New configuration model and PRODINo ESP32 Ethernet v1 support with OTA update
 */

#define VERSION   "5.00"

// ######### INCLUDES #######################

#include "Config.h"

#if defined(PRODINO_BOARD)
  #include "KmpDinoEthernet.h"
  #include "KMPCommon.h"
  #include "Ethernet/utility/w5100.h"
#elif defined(PRODINO_BOARD_ESP32)
  #include <esp_task_wdt.h>
  #include "KMPProDinoESP32.h"
  #include "KMPCommon.h"
#elif defined(TRANSPORT_ETH_ENC28J60)
 #include <UIPEthernet.h>
#elif defined(ESP32_BOARD)
 #include <esp_task_wdt.h>
#else
 #include <SPI.h>
 #include <Ethernet.h>
 #include <EthernetUdp.h>
#endif
 
#if !defined(PRODINO_BOARD_ESP32) && !defined(ESP32_BOARD)
  #include <avr/wdt.h>
#endif

#include "NibeGw.h"
#include "Debug.h"

// ######### VARIABLES #######################

#if !defined(ESP32_BOARD)
boolean ethInitialized = false;

IPAddress targetIp;
EthernetUDP udp;
EthernetUDP udp4writeCmnds;
#endif

#if defined(PRODINO_BOARD_ESP32)
  HardwareSerial RS485_PORT(1);
  NibeGw nibegw(&RS485_PORT, RS485_DIRECTION_PIN, RS485_RX_PIN, RS485_TX_PIN);
#elif defined(ESP32_BOARD)
 NibeGw nibegw(&RS485_PORT, RS485_DIRECTION_PIN, RS485_RX_PIN, RS485_TX_PIN);
#else
  NibeGw nibegw(&RS485_PORT, RS485_DIRECTION_PIN);
#endif

#if defined(PRODINO_BOARD_ESP32) && defined(ENABLE_DYNAMIC_CONFIG)
  boolean dynamicConfigStarted = false;

  class ConfigObserver: public ConfigurationObserver {
  public:
    void onConfigurationChanged(const ConfigurationPropertyChange value) {
      DEBUG_PRINT_VARS(0, "Configuration parameter '%s' changed from '%s' to '%s'\n", 
        String(value.key).c_str(), 
        String(value.oldValue).c_str(),
        String(value.newValue).c_str());
    }
  };

#endif

#if defined(ESP32_BOARD)
 // more functions and variables
 static void initRegisterFuel();
 void onReadRegisterReq(const String& topic, const String& message);
 void onWriteRegisterReq(const String& topic, const String& message);
 void onLoglevelSet(const String& topic, const String& message);
 void onAddRegisterSet(const String& topic, const String& message);
 void writeToMqtt(const byte* const data, int len);

 #define RINGBUFSIZE 10
 static byte readReq[RINGBUFSIZE][6];
 static uint8_t readFill = 0;
 static uint8_t readDrain = 0;
 static byte writeReq[RINGBUFSIZE][10];
 static uint8_t writeFill = 0;
 static uint8_t writeDrain = 0;
 static byte rmuReq[RINGBUFSIZE][8] = {0};
 enum rmuType_e {RMUNONE = 0, RMU1 = 0x19, RMU2, RMU3, RMU4};
 //uint8_t mqttPublishTimer = 5;
 static esp_task_wdt_config_t wdt_config = {
   .timeout_ms = 2 * 1000,
   .trigger_panic = true,
 };
 static int logLevel = 0;
#endif

// ######### SETUP #######################

void setup() {
  #if defined(PRODINO_BOARD_ESP32)
    KMPProDinoESP32.begin(ProDino_ESP32_Ethernet);
    KMPProDinoESP32.setStatusLed(red);
  #endif
  
  #if defined(PRODINO_BOARD_ESP32) && defined(ENABLE_DYNAMIC_CONFIG)
    if (isDynamicConfigModeActivated()) {
      setupDynamicConfigMode();
    } else {
      setupStaticConfigMode();
    }
  #else
    setupStaticConfigMode();
  #endif
    
}

void setupStaticConfigMode() {
  #if defined(PRODINO_BOARD_ESP32) && defined(ENABLE_DYNAMIC_CONFIG)
    // Use temporarily longer wathdog time as possible flash formating might take a while
    esp_task_wdt_init(60, true);
    esp_task_wdt_add(NULL);
    KMPProDinoESP32.setStatusLed(white);
    Bleeper
      .configuration
        .set(&config)
        .done()
      .storage
        .set(new SPIFFSStorage())
        .done()
      .init();
    esp_task_wdt_reset();
    KMPProDinoESP32.setStatusLed(red);
  #endif

  #if defined(PRODINO_BOARD_ESP32)
    Serial.begin(115200, SERIAL_8N1);
  #elif defined(PRODINO_BOARD)
    Serial.begin(115200, SERIAL_8N1);
    DinoInit();
  #elif defined(ESP32_BOARD)
    //pinMode(BLUE_LED, OUTPUT); // blue led
    Serial.begin(115200, SERIAL_8N1);
    otaActive = false;
    ArduinoOTA
      .onStart([]() {
        otaActive = true;
        nibegw.disconnect();
        telnet.stop();
        esp_task_wdt_deinit();
      })
      .onEnd([]() {
        otaActive = false;
        esp_task_wdt_init(&wdt_config);
        esp_task_wdt_add(NULL);
      });
    client.enableOTA();
    //ArduinoOTA.begin();
    telnet.begin(23);
    initRegisterFuel();
  #endif
   
  // Start watchdog
  #if defined(PRODINO_BOARD_ESP32)
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
  #elif defined(ESP32_BOARD)
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
  #else
    wdt_enable(WDTO_2S);
  #endif

  nibegw.setCallback(nibeCallbackMsgReceived, nibeCallbackTokenReceived);
  nibegw.setSendAcknowledge(config.nibe.ack.sendAck);
  nibegw.setAckModbus40Address(config.nibe.ack.modbus40);
  nibegw.setAckSms40Address(config.nibe.ack.sms40);
  nibegw.setAckRmu40Address(config.nibe.ack.rmu40);

  #ifdef ENABLE_NIBE_DEBUG
    nibegw.setDebugCallback(nibeDebugCallback);
    nibegw.setVerboseLevel(config.debug.level);
  #endif
  
  #if !defined(ESP32_BOARD)
  targetIp.fromString(config.nibe.targetIp);
  #endif

  DEBUG_PRINT_VARS(0, "%s version %s Started\n", config.boardName.c_str(), VERSION);
}

#if defined(PRODINO_BOARD_ESP32) && defined(ENABLE_DYNAMIC_CONFIG)

boolean isDynamicConfigModeActivated() {
  if (KMPProDinoESP32.getOptoInState(0)) {
      delay(50);
      if (KMPProDinoESP32.getOptoInState(0)) {
        return true;
      }
  }
  return false; 
}

void setupDynamicConfigMode() {
  KMPProDinoESP32.setStatusLed(white);
  Bleeper
    .verbose(115200)
    .configuration
      .set(&config)
      .addObserver(new ConfigObserver(), {})
      .done()
    .configurationInterface
      .addDefaultWebServer()
      .done()
    .connection
      .setSingleConnectionFromPriorityList({
          new AP()
      })
      .done()
    .storage
      .set(new SPIFFSStorage())
      .done()
    .init(); 

    ElegantOTA.begin(&otaServer);
    otaServer.begin();

    dynamicConfigStarted = true;
    KMPProDinoESP32.setStatusLed(blue);
}
#endif

// ######### MAIN LOOP #######################

void loop() {
  #if defined(PRODINO_BOARD_ESP32) && defined(ENABLE_DYNAMIC_CONFIG)
    if (dynamicConfigStarted) {
      loopDynamicConfigMode();
    } else {
      loopNormalMode();
    }
  #else
    #if defined(ESP32_BOARD)
    if(otaActive) {
      loopOtaMode();
    } else {
    #endif
    loopNormalMode();
    #if defined(ESP32_BOARD)
    }
    #endif
  #endif
}

void loopNormalMode() {
  #if defined(PRODINO_BOARD_ESP32)
    esp_task_wdt_reset();
  #elif defined(ESP32_BOARD)
    esp_task_wdt_reset();
  #else
    wdt_reset();
  #endif
  
  long now = millis() / 1000;

  if (!nibegw.connected()) {
    nibegw.connect();
  } else {
    do {
      nibegw.loop();
      //nibegw.testloop();
      #ifdef TRANSPORT_ETH_ENC28J60
        Ethernet.maintain();
      #endif
    } while (nibegw.messageStillOnProgress());
  }

  #if !defined(ESP32_BOARD)
    if (!ethInitialized && now >= config.eth.initDelay) {
      initializeEthernet();
      #ifdef ENABLE_DEBUG
        telnet.begin();
      #endif
    }
  #else
    client.loop();
    if(!client.isConnected()) {
      //digitalWrite(BLUE_LED, LOW); // turn blue led off
    } else {
      telnet.loop();
      // send serial input to telnet as output
      if (Serial.available()) {
        telnet.print(Serial.read());
      }
    }
    //ArduinoOTA.handle();
  #endif
  #if defined(ENABLE_DEBUG) && defined(ENABLE_REMOTE_DEBUG)
    if (ethInitialized) {
        handleTelnet();
    }
  #endif
}

#if defined(PRODINO_BOARD_ESP32) && defined(ENABLE_DYNAMIC_CONFIG)
void loopDynamicConfigMode() {
  Bleeper.handle();
  otaServer.handleClient();
}
#endif

#if defined(ESP32_BOARD)
void loopOtaMode() {
  //esp_task_wdt_reset();
  client.loop();
  //ArduinoOTA.handle();
}
#endif

// ######### FUNCTIONS #######################

#if !defined(ESP32_BOARD)
void initializeEthernet() {
  DEBUG_PRINT_MSG(1, "Initializing Ethernet\n");

  uint8_t   mac[6];
  sscanf(config.eth.mac.c_str(), "%x:%x:%x:%x:%x:%x", mac, mac+1, mac+2, mac+3, mac+4, mac+5);
  
  IPAddress ip;
  IPAddress dns;
  IPAddress gw;
  IPAddress mask;
  
  ip.fromString(config.eth.ip);
  dns.fromString(config.eth.dns);
  gw.fromString(config.eth.gateway);
  mask.fromString(config.eth.mask);
  
  Ethernet.begin(mac, ip, dns, gw, mask);

  #if defined(PRODINO_BOARD_ESP32)
    Ethernet.setRetransmissionCount(1);
    Ethernet.setRetransmissionTimeout(50);
  #elif defined(PRODINO_BOARD)
    W5100.setRetransmissionCount(1);
    W5100.setRetransmissionTime(50);
  #endif
  
  ethInitialized = true;
  udp.begin(config.nibe.readCmdsPort); 
  udp4writeCmnds.begin(config.nibe.writeCmdsPort);

  printInfo();
  
  #if defined(PRODINO_BOARD_ESP32)
    KMPProDinoESP32.offStatusLed();
  #endif
}
#endif

void nibeCallbackMsgReceived(const byte* const data, int len) {
  #if defined(PRODINO_BOARD_ESP32)
    KMPProDinoESP32.setStatusLed(green);
  #endif
  #if !defined(ESP32_BOARD)
    if (ethInitialized) {
      sendUdpPacket(data, len);
    }
  #endif

  #if defined(PRODINO_BOARD_ESP32)
    KMPProDinoESP32.offStatusLed();
  #endif

  #if defined(ESP32_BOARD)
    //if(client.isConnected()) {
      // if(((data[3]==0x68) && (data[4]==0x50))) {
      //   mqttPublishTimer++;
      //   if(mqttPublishTimer > 4) {
      //     mqttPublishTimer = 0; // just publish roughly every 10 seconds
      //   } else {
      //     return;
      //   }
      // }
      if(data[3] != 0x6D) {
        writeToMqtt(data, len);
      }
    //}
  #endif
}

int nibeCallbackTokenReceived(eTokenType token, byte* data) {
  int len = 0;
  int8_t rmuFound = -1;
  #if !defined(ESP32_BOARD)
  if (ethInitialized) {
  #else
  if(client.isConnected()) {
  #endif
    if (token == READ_TOKEN) {
      DEBUG_PRINT_MSG(3, "Read token received from nibe\n");
    #if !defined(ESP32_BOARD)
      int packetSize = udp.parsePacket();
      if (packetSize) {
        #if defined(PRODINO_BOARD_ESP32)
          KMPProDinoESP32.setStatusLed(white);
        #endif
        targetIp = udp.remoteIP();
        len = udp.read(data, packetSize);
        DEBUG_PRINT_VARS(2, "Send read command to nibe, len=%d, ", len);
        DEBUG_PRINT_MSG(1, " data in: ");
        DEBUG_PRINT_ARRAY(1, data, len)
        DEBUG_PRINT_MSG(1, "\n");
        
        #if defined(TRANSPORT_ETH_ENC28J60)
          udp4readCmnds.flush();
          udp4readCmnds.stop();
          udp4readCmnds.begin(config.nibe.readCmdsPort);
        #endif
      }
    #else
      if(readDrain != readFill) {
        memcpy(data,readReq[readDrain],6);
        len = 6;
        readDrain++;
        if(readDrain == RINGBUFSIZE) {
          readDrain = 0;
        }
        if(logLevel) {
          char data_s[32] = {0};
          sprintf(data_s, "read %x %x", data[3], data[4]);
          client.publish("nibeF730/log", data_s);
        }
      }
    #endif
    } else if (token == WRITE_TOKEN) {
      DEBUG_PRINT_MSG(3, "Write token received from nibe\n");
    #if !defined(ESP32_BOARD)
      int packetSize = udp4writeCmnds.parsePacket();
      if (packetSize) {
        #if defined(PRODINO_BOARD_ESP32)
          KMPProDinoESP32.setStatusLed(orange);
        #endif
        targetIp = udp.remoteIP();
        len = udp4writeCmnds.read(data, packetSize);
        DEBUG_PRINT_VARS(2, "Send write command to nibe, len=%d, ", len);
        DEBUG_PRINT_MSG(1, " data in: ");
        DEBUG_PRINT_ARRAY(1, data, len)
        DEBUG_PRINT_MSG(1, "\n");
        
        #if defined(TRANSPORT_ETH_ENC28J60)
          udp4writeCmnds.flush();
          udp4writeCmnds.stop();
          udp4writeCmnds.begin(config.nibe.writeCmdsPort);
        #endif
      }
    #else
      if(writeDrain != writeFill) {
        memcpy(data,writeReq[writeDrain],10);
        len = 10;
        writeDrain++;
        if(writeDrain == RINGBUFSIZE) {
          writeDrain = 0;
        }
        if(logLevel) {
          char data_s[32] = {0};
          sprintf(data_s, "write %x %x: %x %x %x %x", data[3], data[4], data[5], data[6], data[7], data[8]);
          client.publish("nibeF730/log", data_s);
        }
        //mqttPublishTimer = 4;
      }
    #endif
    } else if (token == RMU_TOKEN) {
      DEBUG_PRINT_MSG(3, "RMU token received from nibe\n");
    #if defined(ESP32_BOARD)
      for(int rmuNo = 0; rmuNo < 4; rmuNo++) {
        if(rmuReq[rmuNo][7] == data[2]) {
          rmuFound = rmuNo;
          break;
        }
      }
      if(rmuFound != -1) {
        memcpy(data,rmuReq[rmuFound],7);
        len = 7;
        if(logLevel) {
          char data_s[32] = {0};
          sprintf(data_s, "rmu %x %x", data[4], data[5]);
          client.publish("nibeF730/log", data_s);
        }
      }
    #endif
    }

    #if defined(PRODINO_BOARD_ESP32)
      KMPProDinoESP32.offStatusLed();
    #endif
  }
  return len;
}

void nibeDebugCallback(byte level, char* data) {
  DEBUG_PRINT_MSG(level, data);
}
#if !defined(ESP32_BOARD)
void sendUdpPacket(const byte* const data, int len) {
  #ifdef ENABLE_DEBUG
    DEBUG_PRINT_VARS(2, "Sending UDP packet to %s:%d, len=%d", IPtoString(targetIp).c_str(), config.nibe.targetPort, len);
    DEBUG_PRINT_MSG(1, " data out: ");
    DEBUG_PRINT_ARRAY(1, data, len)
    DEBUG_PRINT_MSG(1, "\n");
  #endif

  #if defined(PRODINO_BOARD_ESP32)
    EthernetLinkStatus linkStatus = Ethernet.linkStatus();
    if (linkStatus != LinkON) {
      DEBUG_PRINT_VARS(0, "Ethernet link is down, link status = %d\n", linkStatus);
      return;
    }
  #endif
    
  udp.beginPacket(targetIp, config.nibe.targetPort);
  
  udp.write(data, len);
  int retval = udp.endPacket();
  if (retval) {
    DEBUG_PRINT_MSG(3, "UDP packet sent succeed\n");
  } else {
    DEBUG_PRINT_MSG(1, "UDP packet sent failed\n");
  }
}
#endif

String IPtoString(const IPAddress& address) {
  return String() + address[0] + "." + address[1] + "." + address[2] + "." + address[3];
}

void printInfo() {
  #ifdef ENABLE_DEBUG
  DEBUG_PRINT_VARS(0, "%s version %s\nUsing configuration:\n", config.boardName.c_str(), VERSION);
  DEBUG_PRINT_VARS(0, "MAC=%s\n", config.eth.mac.c_str());
  DEBUG_PRINT_VARS(0, "IP=%s\n", config.eth.ip.c_str());
  DEBUG_PRINT_VARS(0, "DNS=%s\n", config.eth.dns.c_str());
  DEBUG_PRINT_VARS(0, "MASK=%s\n", config.eth.mask.c_str());
  DEBUG_PRINT_VARS(0, "GATEWAY=%s\n", config.eth.gateway.c_str());
  DEBUG_PRINT_VARS(0, "ETH_INIT_DELAY=%d\n", config.eth.initDelay);
  #if !defined(ESP32_BOARD)
  DEBUG_PRINT_VARS(0, "TARGET_IP=%s\n", IPtoString(targetIp).c_str());
  #endif
  DEBUG_PRINT_VARS(0, "TARGET_PORT=%d\n", config.nibe.targetPort);
  DEBUG_PRINT_VARS(0, "INCOMING_PORT_READCMDS=%d\n", config.nibe.readCmdsPort);
  DEBUG_PRINT_VARS(0, "INCOMING_PORT_WRITECMDS=%d\n", config.nibe.writeCmdsPort);
  DEBUG_PRINT_VARS(0, "SEND_ACK=%s\n", config.nibe.ack.sendAck ? "true" : "false");
  if (config.nibe.ack.sendAck) {
    DEBUG_PRINT_VARS(0, "ACK_MODBUS40=%s\n", config.nibe.ack.modbus40 ? "true" : "false");
    DEBUG_PRINT_VARS(0, "ACK_SMS40=%s\n", config.nibe.ack.sms40 ? "true" : "false");
    DEBUG_PRINT_VARS(0, "ACK_RMU40=%s\n", config.nibe.ack.rmu40 ? "true" : "false");
  }
  #endif
  DEBUG_PRINT_VARS(0, "VERBOSE_LEVEL=%d\n", config.debug.verboseLevel);
  
  #if defined(ENABLE_DEBUG) && defined(ENABLE_REMOTE_DEBUG)
    DEBUG_PRINT_MSG(0, "REMOTE_DEBUG_ENABLED=true\n");
  #else
    DEBUG_PRINT_MSG(0, "REMOTE_DEBUG_ENABLED=false\n");
  #endif
  
  #if defined(PRODINO_BOARD_ESP32) && defined(ENABLE_DYNAMIC_CONFIG)
    DEBUG_PRINT_MSG(0, "DYNAMIC_CONFIG_ENABLED=true\n");
  #else
    DEBUG_PRINT_MSG(0, "DYNAMIC_CONFIG_ENABLED=false\n");
  #endif
}

#if defined(ENABLE_DEBUG) && defined(ENABLE_REMOTE_DEBUG)
void handleTelnet() {
  EthernetClient client = telnet.available();
  
  if (client) {
    char c = client.read();

    switch (c) {

      case '?':
      case 'h':
        client.println(config.boardName.c_str());
        client.println("Commands:");
        client.println(" E -> exit");
        client.println(" i -> info");
        #ifdef ENABLE_DEBUG
        client.println(" 1 -> set verbose level to 1");
        client.println(" 2 -> set verbose level to 2");
        client.println(" 3 -> set verbose level to 3");
        client.println(" 4 -> set verbose level to 4");
        client.println(" 5 -> set verbose level to 5");
        #endif
        break;
        
      case 'i':
        printInfo();
        break;

      case 'E':
        client.println("Connection closed");
        client.flush();
        client.stop();
        break;

      #ifdef ENABLE_DEBUG
      case '1':
      case '2':
      case '3':
      case '4':
      case '5': 
        client.print("Setting verbose level to ");
        client.println(c);
        config.debug.verboseLevel = c - 0x30;
        break;
      #endif

      case '\n':
      case '\r':
        break;

      default:
        client.print("Unknown command ");
        client.println(c);
    }
  }
}
#endif

#if defined(ESP32_BOARD)
enum dataSize_e {U8, U16, U32, S8, S16, S32};
typedef enum dataSize_e dataSize_t;

typedef struct nibeF730Registers {
  uint16_t regAddress;
  uint8_t factor;
  dataSize_t size;
} nibeF730Registers_t;

#define NUMBER_OF_REGS 255

static nibeF730Registers_t NIBEF730REGISTER[NUMBER_OF_REGS] = {
  {45001, 1, S16}, //"Alarm"}, //R 0:= Keine Alarme, 64:= Niedrige Ablufttemperatur, 83:= Auftauen fehlgeschlagen, 164:= Niedrige Ablufttemperatur, 180:= Einfrierschutz, 181:= Fehler bei der regelmäßigen Erhöhung, 182:= Netzschalter aktiviert, 183:= Auftauen, 184:= Alarm filtern, 251:= Komm.-Modbus
  {32260, 1, U8}, //"Inverter State"}, //R
  {40004, 10,S16}, //"BT1-Outdoor-Temperature"}, //R
  {40067, 10,S16}, //"BT1-Average"}, //R
  {40185, 10,S16}, //"BT1-Average-1h"}, //R
  {42100, 10,S16}, //"BT1-Average-24h"}, //R
  {40033, 10,S16}, //"BT50-Room-Temperature"}, //R
  {40195, 10,S16}, //"BT50-Room-Temp-S1-Average"}, //R
  {47398, 10,S16}, //"Room-sensor-setpoint-S1"}, // R/W
  {40008, 10,S16}, //"BT2-Supply-temp-S1"}, //R
  {40012, 10,S16}, //"EB100-EP14-BT3-Return-temp"}, //R
  {43009, 10,S16}, //"Calc.-Supply-S1"}, //R
  {40940, 10,S32}, //"Degree-Minutes"}, // R/W
  {43005, 10,S16}, //"Degree-Minutes-16bit"}, // R/W
  {47050, 1, S8}, //"Periodic-HotWater"}, // R/W
  {48743, 1, S8}, //"Hot-water-high-power-mode"}, // R/W
  {47260, 1, U8}, //"Fan-Mode"}, // R/W 0:= Normal, 1:= Hastighet 1, 2:= Hastighet 2, 3:= Hastighet 3, 4:= Hastighet 4
  {47134, 1, U8}, //Period-HW
  {40013, 10,S16}, // //R
  {40014, 10,S16}, // //R
  {40017, 10,S16}, // //R
  {40018, 10,S16}, // //R
  {40019, 10,S16}, // //R
  {40020, 10,S16}, // //R
  {40022, 10,S16}, // //R
  {40025, 10,S16}, // //R
  {40026, 10,S16}, // //R
  {40032, 10,S16}, // //R
  {40050, 10,S16}, // //R
  {40051,100,S16}, // //R
  {40079, 10,U32}, // EB100-BE3 Current //R
  {40081, 10,U32}, // EB100-BE2 Current //R
  {40083, 10,U32}, // EB100-BE1 Current //R
  {40879, 1, S8}, // //R/W 
  {40880, 1, S8}, // //R/W 
  {40888, 1, U8}, // //R/W
  {41846, 10,U32}, // HP consumed energy due to ventilation //R
  {41848, 10,U32}, // HP consumed energy due to hot water //R
  {41850, 10,U32}, // HP consumed energy due to heating //R
  {43084, 100,S16}, // Int. el.add. Power //R
  {43141, 1, U16}, // compr. in power //R
  {43096, 10,S16}, // //R
  {43108, 1, U8}, // //R
  {43375, 1,S16}, // compr. in power mean //R
  {45171, 1, U8}, // Reset alarm by setting value 1 //R/W
  {47007, 1, S8}, // //R
  {47011, 1, S8}, // Heat Offset S1 //R/W
  {47015, 10,S16}, // //R
  {47019, 10,S16}, // //R
  {47026, 1, S8}, // //R
  {47027, 1, S8}, // //R
  {47028, 1, S8}, // //R
  {47041, 1, S8}, // Hot water comfort mode //R/W
  {47101, 1, U8}, // Compressor frequency regulator P //R/W
  {47137, 1, U8}, // Operational mode //R/W
  {47138, 1, U8}, // Operational mode heat medium pump //R/W
  {47212,100,S16}, // Max int add. power //R/W
  {47370, 1, U8}, // Allow Additive Heating //R/W
  {47371, 1, U8}, // Allow Heating //R/W
  {47387, 1, U8}, // HW production //R/W
  {48043, 1, U8}, // Holiday - Activated //R/W
  {48132, 1, S8}, // Temporary Lux //R/W
  {48973, 1, U8}, // Reduced ventilation //R/W
  // RMU40
  {10060, 10,S16}, // //R
  {10160, 10,S16}, // //R
};

typedef struct nibeRegs_s {
  uint8_t factor;
  dataSize_t size;
} nibeRegs_t;

static uint8_t regFuel = 41;

static void initRegisterFuel()
{
  regFuel = 0;
  for (unsigned regIndex = 0; regIndex < NUMBER_OF_REGS; regIndex++) {
    regFuel++;
    if (NIBEF730REGISTER[regIndex].regAddress == 10160) { // last entry
      break;  // end loop here
    }
  }
}

static byte calcCrc(byte* data, uint8_t datalen)
{
  byte calc_checksum = 0;
      
  // calculate XOR checksum
  for(int i = 2; i < (datalen + 5); i++) {
    calc_checksum ^= data[i];
  }
  return calc_checksum;
}

static byte calcCrc2(byte* data, uint8_t datalen)
{
  byte calc_checksum = 0;
      
  // calculate XOR checksum
  for(int i = 0; i < (datalen + 3); i++) {
    calc_checksum ^= data[i];
  }
  return calc_checksum;
}

void onConnectionEstablished()
{
  //digitalWrite(BLUE_LED, HIGH); // turn blue led on
  client.subscribe("nibeF730/read/#", onReadRegisterReq);
  client.subscribe("nibeF730/write/#", onWriteRegisterReq);
  client.subscribe("nibeF730/loglevel/set", onLoglevelSet);
  client.subscribe("nibeF730/add", onAddRegisterSet);
  onLoglevelSet("", String(0));
  Serial.println("connected");
}

void onReadRegisterReq(const String& topic, const String& message)
{
  String regAddress_s = topic.substring(14);
  uint16_t regAddress = regAddress_s.toInt();
  //unsigned regIndex;

  // for(regIndex = 0; regIndex < regFuel; regIndex++) {
  //   if(NIBEF730REGISTER[regIndex].regAddress == regAddress) {
  //     break; // end loop here
  //   }
  // }
  // if(regIndex == regFuel) {
  //   return;
  // }
  readReq[readFill][0] = 0xc0; // frame start
  readReq[readFill][1] = 0x69; // read request
  readReq[readFill][2] = 0x2; // data length
  readReq[readFill][3] = regAddress & 0xff; // register address low byte
  readReq[readFill][4] = (regAddress >> 8) & 0xff; // register address high byte
  readReq[readFill][5] = calcCrc2(readReq[readFill], 2); // crc checksum
  readFill++;
  if(readFill == RINGBUFSIZE) {
    readFill = 0;
  }
}

void onWriteRegisterReq(const String& topic, const String& message)
{
  String regAddress_s = topic.substring(15);
  uint16_t regAddress = regAddress_s.toInt();
  uint32_t data;
  unsigned regIndex;
  uint8_t rmuNo = 0;
  
  for(regIndex = 0; regIndex < regFuel; regIndex++) {
    if(NIBEF730REGISTER[regIndex].regAddress == regAddress) {
      break; // end loop here
    }
  }
  if(regIndex == regFuel) {
    if(logLevel) {
      char data_s[32] = {0};
      sprintf(data_s, "reg %d: %.2f", regAddress, message.toFloat());
      client.publish("nibeF730/logerror", data_s);
    }
    return;
  }

  if((regAddress == 10060) || (regAddress == 10160)) {
      if(regAddress == 10160) {
        rmuNo = 1;
      }
    rmuReq[rmuNo][0] = 0xc0; // frame start
    rmuReq[rmuNo][1] = 0x60; // rmu write
    rmuReq[rmuNo][2] = 0x3; // data length
    rmuReq[rmuNo][3] = 0x60; // rmu register
    if(NIBEF730REGISTER[regIndex].factor == 1) {
      data = message.toInt();
    } else {
      data = (uint32_t)(message.toFloat() * NIBEF730REGISTER[regIndex].factor);
    }
    if(data >= 32768) {
      data = data - 7;
    } else {
      data = data + 7;
    }
    rmuReq[rmuNo][4] = data & 0xff; // value
    rmuReq[rmuNo][5] = (data >> 8) & 0xff; // value
    rmuReq[rmuNo][6] = calcCrc2(rmuReq[rmuNo], 3); // crc checksum
    switch(regAddress) {
      case 10060:
        rmuReq[rmuNo][7] = RMU1;
        break;
      case 10160:
        rmuReq[rmuNo][7] = RMU2;
        break;
      default:
        rmuReq[rmuNo][7] = RMUNONE;
    }
    return;
  }
  writeReq[writeFill][0] = 0xc0; // frame start
  writeReq[writeFill][1] = 0x6b; // write request
  writeReq[writeFill][2] = 0x6; // data length
  writeReq[writeFill][3] = regAddress & 0xff; // register address low byte
  writeReq[writeFill][4] = (regAddress >> 8) & 0xff; // register address high byte
  if(NIBEF730REGISTER[regIndex].factor == 1) {
    data = message.toInt();
  } else {
    data = (uint32_t)(message.toFloat() * NIBEF730REGISTER[regIndex].factor);
  }
  writeReq[writeFill][5] = data & 0xff; // value
  writeReq[writeFill][6] = (data >> 8) & 0xff; // value
  writeReq[writeFill][7] = (data >> 16) & 0xff; // value
  writeReq[writeFill][8] = (data >> 24) & 0xff; // value
  writeReq[writeFill][9] = calcCrc2(writeReq[writeFill], 6/*writeReq[writeFill][2]*/); // crc checksum
  writeFill++;
  if(writeFill == RINGBUFSIZE) {
    writeFill = 0;
  }
}

void onLoglevelSet(const String& topic, const String& message)
{
  logLevel = message.toInt();
  client.publish("nibeF730/loglevel", String(logLevel));
}

/* message: {"register":"12345","factor":"10","size":"U16"} */
void onAddRegisterSet(const String& topic, const String& message)
{
  String subString = message.substring(13);
  uint16_t subInt = subString.toInt();

  if (regFuel < NUMBER_OF_REGS) {
    NIBEF730REGISTER[regFuel].regAddress = subInt;
    subInt = message.indexOf("factor") + 3;
    subString = message.substring(subInt);
    NIBEF730REGISTER[regFuel].factor = subString.toInt();
    subInt = message.indexOf("size") + 3;
    subString = message.substring(subInt);
    NIBEF730REGISTER[regFuel].size = (dataSize_t)subString.toInt();
    regFuel++;
  } else if(logLevel) {
      char data_s[32] = {0};
      sprintf(data_s, "add %d not possible at %d", subInt, regFuel);
      client.publish("nibeF730/logerror", data_s);
  }
}

/*
 * Data Format
 *  +------------+------------+-----------+-----------+-----------+-----------+
 *  | RegAddr LB | RegAddr HB | Value1 LB | Value1 HB | Value2 LB | Value2 HB |
 *  +------------+------------+-----------+-----------+-----------+-----------+
 *  U8, U16, S8, S16 -> Value1 only; U32, S32 -> Value1 is HW and Value2 LW
 */
void writeToMqtt(const byte* const data, int len)
{
  uint16_t regAddress;
  int value = 0;
  uint32_t tempU32;
  
  for(unsigned dataIndex = 5; dataIndex < (len - 4); dataIndex++) {
    char topic[20] = {0};
    char data_s[32] = {0};
    regAddress = data[dataIndex+1]<<8 | data[dataIndex];
    if(regAddress == 0xffff) {
      return;
    }
    if((data[3] == 0x6A) && logLevel) {
      char data_s[len*5] = {0};
      for(unsigned index = 0; index < len; index++) {
        sprintf(data_s, "%s x%x", data_s, data[index]);
      }
      client.publish("nibeF730/logrespond", data_s);
    }
    for(unsigned index = 0; index < regFuel; index++) {
      if(NIBEF730REGISTER[index].regAddress == regAddress) {
        sprintf(topic,"nibeF730/%d", regAddress);
        switch(NIBEF730REGISTER[index].size) {
          case U8:
          case U16:
            value = data[dataIndex+3] << 8 | data[dataIndex+2];
            dataIndex += 3;
            break;
          case U32:
            value = data[dataIndex+3] << 24 | data[dataIndex+2] << 16 | data[dataIndex+5] << 8 | data[dataIndex+4];
            dataIndex += 5;
            break;
          case S8:
            value = data[dataIndex+3] << 8 | data[dataIndex+2];
            if (value > 128 && value < 32768) {
              value -= 256;
            } else if (value >= 32768) {
              value -= 65536;
            }
            dataIndex += 3;
            break;
          case S16:
            value = data[dataIndex+3] << 8 | data[dataIndex+2];
            if (value >= 32768) {
              value = value - 65536;
            }
            dataIndex += 3;
            break;
          case S32:
            tempU32 = data[dataIndex+3] << 24 | data[dataIndex+2] << 16 | data[dataIndex+5] << 8 | data[dataIndex+4];
            if (tempU32 >= 2147483647) {
              value = (tempU32 - 4294967294);
            } else {
              value = tempU32;
            }
            dataIndex += 5;
            break;
          default:
            dataIndex += 3;
        }
        switch(NIBEF730REGISTER[index].factor) {
          case 1:
            sprintf(data_s, "%d", value);
            break;
          case 10:
            sprintf(data_s, "%.1f", (float)value/10);
            break;
          case 100:
            sprintf(data_s, "%.2f", (float)value/100);
            break;
        }
        if(client.isConnected()) {
          // Publish a message to "nibeF730/register data"
          client.publish(topic, data_s); // You can activate the retain flag by setting the third parameter to true
        //client.loop();
        }
        if ((regAddress == 45001) && (value == 251)) {  // if modbus communication alarm, reset alarm
          writeReq[writeFill][0] = 192;
          writeReq[writeFill][1] = 107;
          writeReq[writeFill][2] = 6;
          writeReq[writeFill][3] = 115;
          writeReq[writeFill][4] = 176;
          writeReq[writeFill][5] = 1;
          writeReq[writeFill][6] = 0;
          writeReq[writeFill][7] = 0;
          writeReq[writeFill][8] = 0;
          writeReq[writeFill][9] = 111;
          writeFill++;
          if (writeFill == RINGBUFSIZE) {
            writeFill = 0;
          }
        }
        break;
      }
      if(index == regFuel-1) {
        dataIndex += 3;
        if(logLevel) {
          char data_s[32] = {0};
          sprintf(data_s, "from nibe reg %d not found %d", regAddress, data[3]);
          client.publish("nibeF730/logerror", data_s);
        }
      }
      // if(regAddress == 40004) {
      //   sprintf(topic,"nibeF730/raw/%d", regAddress);
      //   sprintf(data_s, "%d %d", data[dataIndex+3], data[dataIndex+2]);
      //   client.publish(topic, data_s); // You can activate the retain flag by setting the third parameter to true
      // } else if(regAddress == 40185) {
      //   sprintf(topic,"nibeF730/raw/%d", regAddress);
      //   sprintf(data_s, "%d %d", data[dataIndex+3], data[dataIndex+2]);
      //   client.publish(topic, data_s); // You can activate the retain flag by setting the third parameter to true
      // }
    }
  }
}
#endif
