/*
 * Controllino MEGA Automation integrate in HomeAssistant
 * 
 * 
 * v 0.3
 */

#include <Controllino.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// jalousie Kitchen
#define jalup_1 CONTROLLINO_D1
#define jaldown_1 CONTROLLINO_R2
#define jalswitchup_1 CONTROLLINO_A1
#define jalswitchdown_1 CONTROLLINO_A0

// jalousie moving door
#define jalup_2 CONTROLLINO_D2
#define jaldown_2 CONTROLLINO_R3
#define jalswitchup_2 CONTROLLINO_A2
#define jalswitchdown_2 CONTROLLINO_A3

// jalousie living room window 1
#define jalup_3 CONTROLLINO_D3
#define jaldown_3 CONTROLLINO_R4
#define jalswitchup_3 CONTROLLINO_A4
#define jalswitchdown_3 CONTROLLINO_A5

// jalousie living room window 2
#define jalup_4 CONTROLLINO_D4
#define jaldown_4 CONTROLLINO_R5
#define jalswitchup_4 CONTROLLINO_A4
#define jalswitchdown_4 CONTROLLINO_A5

// define Light and Switches Ports
// light_ = output (relais)
// lswitch_ = input (switch)

// Kitchen
#define light_1 CONTROLLINO_R0 // Kitchen
#define lswitch_1 CONTROLLINO_A10 // Kitchen

#define light_2 CONTROLLINO_D0 // Kitchen Workplace (Cooking plate)
#define lswitch_2 0 // Kitchen Workplace (Cooking plate)

#define light_3 CONTROLLINO_R1 // Kitchen Workplace
#define lswitch_3 CONTROLLINO_A9 // Kitchen Workplace

// dinning table
#define light_4 CONTROLLINO_R12 // dinning table
#define lswitch_4 CONTROLLINO_A11 // dinning table

// floor
#define light_5 CONTROLLINO_R13 // floor
#define lswitch_5 CONTROLLINO_A12 // floor

#define light_6 CONTROLLINO_R14 // floor (behind)
#define lswitch_6 CONTROLLINO_A12 // floor (behind) (A12 ??)

#define light_7 CONTROLLINO_R10 // Entry
#define lswitch_7 CONTROLLINO_A14 // Entry

#define light_8 CONTROLLINO_R13 // floor
#define lswitch_8 CONTROLLINO_A13 // floor

#define light_9 CONTROLLINO_R14 // floor (behind)
#define lswitch_9 CONTROLLINO_A13 // floor (behind)

#define light_10 CONTROLLINO_R10 // Entry
#define lswitch_10 CONTROLLINO_A15 // Entry (switch door)

#define light_11 CONTROLLINO_R11 // storage
#define lswitch_11 CONTROLLINO_A7 // storage

#define light_12 CONTROLLINO_R12 // dinning table
#define lswitch_12 CONTROLLINO_A8 // dinning table


// MQTT settings
#define mqttserver "192.168.2.242"
#define mqtt_user "mosquitto"
#define mqtt_password "platinum-2018"
#define mqtt_clientid "bsn-conneubau1"

// Update these with values suitable for your network.
byte mac[]    = { 0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };


// lights
uint8_t lightsindex = 12;
uint8_t lights[] = {light_1, light_2, light_3, light_4, light_5, light_6, light_7, light_8, light_9, light_10, light_11, light_12};
uint8_t lightswitches[] = {lswitch_1, lswitch_2, lswitch_3, lswitch_4, lswitch_5, lswitch_6, lswitch_7, lswitch_8, lswitch_9, lswitch_10, lswitch_11, lswitch_12};
bool lightstate[] = {false, false, false, false, false, false, false, false, false, false, false, false};
unsigned long lighttimercurrentstates[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long lighttimers[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// jalousie
uint8_t jalousieindex = 4;
uint8_t jalswitchesup[] = {jalswitchup_1, jalswitchup_2, jalswitchup_3, jalswitchup_4};
uint8_t jalswitchesdown[] = {jalswitchdown_1, jalswitchdown_2, jalswitchdown_3, jalswitchdown_4};
uint8_t jalups[] = {jalup_1, jalup_2, jalup_3, jalup_4};
uint8_t jaldowns[] = {jaldown_1, jaldown_2, jaldown_3, jaldown_4};
unsigned long jaltimers[] = {20000UL, 40000UL, 40000UL, 40000UL};
unsigned long jaltimercurrentstates[] = {0, 0, 0, 0};
bool jalpositiondown[] = {false, false, false, false};


const uint8_t progversion = 3;
const byte address = 2;

bool eepromupdate, laststateeepromupdate, mqttcommandreceived, ethernet_connected = false;
unsigned long currmillis, eepromtask, jalousietask, switchtask, serialtask, lastsend, checkmqtt, ethtask, alive = 0;

uint8_t serialbuffersize = 0;
char serialbuffer[100];

struct memorystruct{
  bool meminit;
  bool debug;
  bool enable_mqtt;
  bool automaticdrive;
  uint8_t jaltimer1;
  uint8_t jaltimer2;
  uint8_t jaltimer3;
  uint8_t jaltimer4;
  uint8_t timestamp_down_h;
  uint8_t timestamp_down_m;
  uint8_t timestamp_up_h;
  uint8_t timestamp_up_m;
  uint8_t timerlight1;
  uint8_t timerlight2;
  uint8_t timerlight3;
  uint8_t timerlight4;
  uint8_t timerlight5;
  uint8_t timerlight6;
  uint8_t timerlight7;
  uint8_t timerlight8;
  uint8_t timerlight9;
  uint8_t timerlight10;
  uint8_t timerlight11;
  uint8_t timerlight12;
  uint8_t eepromversion;
  byte i2caddress;
};

memorystruct memory = {
  false,
  false,
  false,
  false,
  30,
  40,
  40,
  40,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  progversion,
  address,
};


char* msgtype;
StaticJsonDocument<256> doc;
/*
EthernetClient ethClient;
PubSubClient client(ethClient);
*/

void(* resetFunc) (void) = 0;


void loadeeprom(int idx = 0, bool setdefault = false){
  EEPROM.get(idx, memory);
  if(setdefault){
    memory.debug = false;
    memory.eepromversion = progversion;
    memory.i2caddress = address;
    eepromupdate = true;
  }
}

void saveeeprom(int idx = 0){
  memory.meminit = true;
  EEPROM.put(idx, memory);
  eepromupdate = false;
}


void PrintHelp(){
  Serial.println(F("Controllino Automation Help"));
  Serial.println();
  Serial.println(F("commands:"));
  Serial.println();
  Serial.println(F("help,1                                                                  show this help page"));
  Serial.println(F("switch,<id>,<1=on, 2=off>                                               switch the light on or off"));
  Serial.println(F("jal,<id>,<1=up, 2=down, 3 or 0 = stop>                                  jalousie drive up or down or stop"));
  Serial.println(F("jaltimer,<id>,<time in seconds between 0 and 255>                        set time in seconds to stop the jalousie"));
  Serial.println(F("timerlight,<id>,<time in minutes between 0 and 121 (0 = deactivated)>    set specified light to automatic off light"));
  Serial.println();
  Serial.println(F("debug,<1=on, 0=off>                                                     show verbose informations"));
  Serial.println(F("reboot,1                                                                reboot"));
  Serial.println(F("showsettings,1                                                          show settings"));
  Serial.println(F("factory,1                                                               reset settings to factory "));
  Serial.println(F("i2caddress,<address between 1 and 255>                                  set / change I2C Address"));
  Serial.println(F("mqtt,<1=on, 0=off>                                                      switch MQTT Communication on or off"));
}


void ShowSettings(){
  Serial.println(F("-----------Settings---------------"));
  Serial.print(F("eeprom version = "));
  Serial.println((uint8_t)memory.eepromversion);
  Serial.print(F("size = "));
  Serial.println(sizeof(memory));
  Serial.print(F("Program version = "));
  Serial.println(progversion);
  Serial.print(F("Debug mode = "));
  Serial.println(memory.debug);
  Serial.print(F("i2c address = "));
  Serial.println(memory.i2caddress);
  Serial.print(F("MQTT "));
  if(memory.enable_mqtt){
    Serial.println(F("enabled"));
  } else {
    Serial.println(F("disabled"));
  }
  Serial.println(F("----------------------------------"));
  Serial.println(F("index = Switch INPUT = light OUTPUT = current state = timer (enabled / time in minutes)"));
  for(uint8_t i=0; i<lightsindex; i++){
    if(!lightswitches[i]){
      continue;
    }
    Serial.print(i);
    Serial.print(F(" = "));
    Serial.print(lightswitches[i]);
    Serial.print(F(" = "));
    Serial.print(lights[i]);
    Serial.print(F(" = "));
    Serial.print(digitalRead(lights[i]));
    Serial.print(F(" = "));
    Serial.println(lighttimers[i] / 60000UL);
  }
  Serial.println(F("----------------------------------"));
  Serial.print(F("Jalousie Automatic drive is "));
  if(memory.automaticdrive){
    Serial.println(F("on"));
    Serial.print(F("Automatic down at "));
    Serial.print(memory.timestamp_down_h);
    Serial.print(F(":"));
    Serial.println(memory.timestamp_down_m);
    Serial.print(F("Automatic up at "));
    Serial.print(memory.timestamp_up_h);
    Serial.print(F(":"));
    Serial.println(memory.timestamp_up_m);
  } else {
    Serial.println(F("off"));
  }
  Serial.println(F("----------------------------------"));
  Serial.println(F("INPUT Jalousie number = UP = DOWN"));
  for(uint8_t i=0; i<jalousieindex; i++){
    if(!jalswitchesdown[i] || !jalswitchesup[i]){
      continue;
    }
    Serial.print(F("Jalousie "));
    Serial.print(i);
    Serial.print(F(" = "));
    Serial.print(jalswitchesup[i]);
    Serial.print(F(" = "));
    Serial.println(jalswitchesdown[i]);
  }
  Serial.println(F("----------------------------------"));
  Serial.println(F("OUTPUT Jalousie number = UP = DOWN"));
  for(uint8_t i=0; i<jalousieindex; i++){
    Serial.print(F("Jalousie "));
    Serial.print(i);
    Serial.print(F(" = "));
    Serial.print(jalups[i]);
    Serial.print(F(" = "));
    Serial.println(jaldowns[i]);
  }
  Serial.println(F("----------------------------------"));
  Serial.println(F("Jalousie Timer Settings"));
  Serial.println(F("Jalousie Number = Activetime in Seconds"));
  for(uint8_t i=0; i<jalousieindex; i++){
    Serial.print(F("Jalousie "));
    Serial.print(i);
    Serial.print(F(" = "));
    Serial.println(jaltimers[i] / 1000);
  }
  Serial.println(F("----------------------------------"));
}


void clsbuffer(){
  memset(serialbuffer, 0, sizeof(serialbuffer));
  serialbuffersize = 0;
}


void setup_io(uint8_t port, bool active = false, bool output = false){
  uint8_t on = LOW;
  if(active)on = HIGH;

  byte iotype = INPUT;
  if(output)iotype = OUTPUT;

  pinMode(port, iotype);
  digitalWrite(port, on);
}

void init_jaltimers(){
  if(memory.debug)Serial.println(F("update Jalousie Timer Array"));
  if(memory.jaltimer1 > 254 || memory.jaltimer2 > 254 || memory.jaltimer3 > 254 || memory.jaltimer4 > 254)eepromupdate = true;
  if(memory.jaltimer1 > 254)memory.jaltimer1 = 20;
  if(memory.jaltimer2 > 254)memory.jaltimer2 = 40;
  if(memory.jaltimer3 > 254)memory.jaltimer3 = 40;
  if(memory.jaltimer4 > 254)memory.jaltimer4 = 40;
  jaltimers[0] = (unsigned long)(memory.jaltimer1 * 1000UL);
  jaltimers[1] = (unsigned long)(memory.jaltimer2 * 1000UL);
  jaltimers[2] = (unsigned long)(memory.jaltimer3 * 1000UL);
  jaltimers[3] = (unsigned long)(memory.jaltimer4 * 1000UL);
  if(memory.debug)Serial.println(F("Finish"));
}

void init_input(){
  if(memory.debug)Serial.print(F("init switch..."));
  for(uint8_t i=0; i<jalousieindex; i++){
    setup_io(jalswitchesdown[i]);
    }
  for(uint8_t i=0;i<jalousieindex; i++){
    setup_io(jalswitchesup[i]);
  }
  if(memory.debug)Serial.println(F("finish"));
}

void init_relais(){
  if(memory.debug)Serial.print(F("init relais..."));
  for(uint8_t i=0;i<lightsindex; i++){
    setup_io(lights[i], false, true);
  }
  if(memory.debug)Serial.println(F("finish"));
}

void init_digitaloutput(){
  if(memory.debug)Serial.print(F("init external relais..."));
  for(uint8_t i=0; i < jalousieindex; i++){
    setup_io(jaldowns[i], false, true);
  }
  for(uint8_t i=0; i< jalousieindex; i++){
    setup_io(jalups[i], false, true);
  }
  if(memory.debug)Serial.println(F("finish"));
}

void init_lighttimercurrentstates(){
  for(uint8_t i=0; i< lightsindex; i++){
    lighttimercurrentstates[i] = 0;
  }
}


void init_lighttimers(){
  if(memory.timerlight1 > 254 || memory.timerlight2 > 254 || memory.timerlight3 > 254 || 
  memory.timerlight4 > 254 || memory.timerlight5 > 254 || memory.timerlight6 > 254 || 
  memory.timerlight7 > 254 || memory.timerlight8 > 254 || memory.timerlight9 > 254 || 
  memory.timerlight10 > 254 || memory.timerlight11 > 254 || memory.timerlight12)eepromupdate = true;

  if(memory.timerlight1 > 254)memory.timerlight1 = 0;
  if(memory.timerlight2 > 254)memory.timerlight2 = 0;
  if(memory.timerlight3 > 254)memory.timerlight3 = 0;
  if(memory.timerlight4 > 254)memory.timerlight4 = 0;
  if(memory.timerlight5 > 254)memory.timerlight5 = 0;
  if(memory.timerlight6 > 254)memory.timerlight6 = 0;
  if(memory.timerlight7 > 254)memory.timerlight7 = 0;
  if(memory.timerlight8 > 254)memory.timerlight8 = 0;
  if(memory.timerlight9 > 254)memory.timerlight9 = 0;
  if(memory.timerlight10 > 254)memory.timerlight10 = 0;
  if(memory.timerlight11 > 254)memory.timerlight11 = 0;
  if(memory.timerlight12 > 254)memory.timerlight12 = 0;
  
  lighttimers[0] = (unsigned long)(memory.timerlight1 * 60000UL);
  lighttimers[1] = (unsigned long)(memory.timerlight2 * 60000UL);
  lighttimers[2] = (unsigned long)(memory.timerlight3 * 60000UL);
  lighttimers[3] = (unsigned long)(memory.timerlight4 * 60000UL);
  lighttimers[4] = (unsigned long)(memory.timerlight5 * 60000UL);
  lighttimers[5] = (unsigned long)(memory.timerlight6 * 60000UL);
  lighttimers[6] = (unsigned long)(memory.timerlight7 * 60000UL);
  lighttimers[7] = (unsigned long)(memory.timerlight8 * 60000UL);
  lighttimers[8] = (unsigned long)(memory.timerlight9 * 60000UL);
  lighttimers[9] = (unsigned long)(memory.timerlight10 * 60000UL);
  lighttimers[10] = (unsigned long)(memory.timerlight11 * 60000UL);
  lighttimers[11] = (unsigned long)(memory.timerlight12 * 60000UL);
}


void init_12c(){
  if(memory.debug)Serial.print(F("Run setupwire on address "));
  if(memory.debug)Serial.println(memory.i2caddress);
  Wire.begin(memory.i2caddress);
//  Wire.onReceive(receiveEvent);
//  Wire.onRequest(requestEvent);
}


void switch_io(uint8_t port, bool on = false){
  uint8_t state = LOW;
  if(on)state = HIGH;
  digitalWrite(port, state);
}



bool is_lighton(uint8_t index){
  return digitalRead(lights[index]);
}


// switch over Serial and Web event
void switch_tasks_auto(uint8_t index, bool on = false){
  if((index > lightsindex) || lightswitches[index] == 0)return;
  bool lighton = is_lighton(index);
  if(lighton != on){
    switch_io(lights[index], !lighton);
    if(lighttimers[index])lighttimercurrentstates[index] = millis();
  }
}

bool switchlaststate[] = {false, false, false, false, false, false, false, false, false, false, false, false};
void switch_tasks(bool load_prev_state = false){
  for(uint8_t i=0; i<lightsindex; i++){
    if(lightswitches[i] == 0){
      continue;
    }

    bool lighton = is_lighton(i);

    if(load_prev_state){
      switch_io(lights[i], lightstate[i]);
    }
    else if(digitalRead(lightswitches[i]) && !switchlaststate[i]){
      switch_io(lights[i], !lighton);
      switchlaststate[i] = true;
      if(lighttimers[i] && !lighton)lighttimercurrentstates[i] = millis();
      if(lighttimers[i] && lighton)lighttimercurrentstates[i] = 0;
    }
    else if(!digitalRead(lightswitches[i]) && switchlaststate[i]){
      switchlaststate[i] = false;
    }

    if(lighttimers[i] && lighttimercurrentstates[i] > 0 && is_lighton(i) && (unsigned long)(millis() - lighttimercurrentstates[i]) >= lighttimers[i]){
      switch_io(lights[i], false);
      lighttimercurrentstates[i] = 0;
    }
  }
}


bool is_jalmoving(uint8_t index){
  return (digitalRead(jaldowns[index]) || digitalRead(jalups[index]));
}

// control over Serial and Webevent
void jalousie_tasks_auto(uint8_t index, bool up = false, bool down = false){
  if((index > jalousieindex))return;

  bool jalmoving = is_jalmoving(index);
    
  if(up){
    if(jalmoving && digitalRead(jaldowns[index])){
      switch_io(jaldowns[index], false);
      return;
    }
    switch_io(jalups[index], true);
    jaltimercurrentstates[index] = millis();
    jalpositiondown[index] = false;
  }
  else if(down){
    if(jalmoving && digitalRead(jalups[index])){
      switch_io(jalups[index], false);
      return;
    }
    switch_io(jaldowns[index], true);
    jaltimercurrentstates[index] = millis();
    jalpositiondown[index] = true;
  }
  else {
    switch_io(jaldowns[index], false);
    switch_io(jalups[index], false);
    jaltimercurrentstates[index] = 0;
  }
}

bool jalswitchlaststate[] = {false, false, false, false};
unsigned long jalswitchedduration []= {0, 0, 0, 0};
void jalousie_tasks(){
  for(uint8_t i=0; i<jalousieindex; i++){
    bool jalmoving = false;
    if(jalswitchesup[i] == 0 || jalswitchesdown[i] == 0){
      continue;
    }

    jalmoving = is_jalmoving(i);

    // detect if button is short for setup or long for drive pressed
    if(jalmoving && !digitalRead(jalswitchesup[i]) && digitalRead(jalups[i]) && ((unsigned long)(millis() - jalswitchedduration[i]) < 2000UL)){
      switch_io(jalups[i], false);
    }
    else if(jalmoving && !digitalRead(jalswitchesdown[i]) && digitalRead(jaldowns[i]) && ((unsigned long)(millis() - jalswitchedduration[i]) < 2000UL)){
      switch_io(jaldowns[i], false);
    }

    if(!digitalRead(jalswitchesup[i]) && !digitalRead(jalswitchesdown[i])){
      jalswitchlaststate[i] = false;
    }

    if(digitalRead(jalswitchesup[i]) && !digitalRead(jalups[i]) && !jalswitchlaststate[i]){
      jalswitchlaststate[i] = true;

      if(jalmoving && digitalRead(jaldowns[i])){
        switch_io(jaldowns[i], false);
        continue;
      }

      switch_io(jalups[i], true);
      jaltimercurrentstates[i] = millis();
      jalswitchedduration[i] = millis();
      jalpositiondown[i] = false;
    }
    else if(digitalRead(jalswitchesdown[i]) && !digitalRead(jaldowns[i]) && !jalswitchlaststate[i]){
      jalswitchlaststate[i] = true;

      if(jalmoving && digitalRead(jalups[i])){
        switch_io(jalups[i], false);
        continue;
      }
      switch_io(jaldowns[i], true);
      jaltimercurrentstates[i] = millis();
      jalswitchedduration[i] = millis();
      jalpositiondown[i] = true;
    }
    else if(jalmoving && (unsigned long)(millis() - jaltimercurrentstates[i]) >= jaltimers[i]){
      switch_io(jalups[i], false);
      switch_io(jaldowns[i], false);
      jaltimercurrentstates[i] = 0;
      jalswitchedduration[i] = 0;
      jalswitchlaststate[i] = false;
    }
  }

}

void SerialEvent(){
  bool tosend = false;
  while(Serial.available() > 0){
    tosend = true;
    char c = (char)Serial.read();
    serialbuffer[serialbuffersize++] = c;
    if(c == '\n' || c == '\r')break;
  }
  yield();
  if(tosend)ProcessSerialEvent(serialbuffersize);
}


void setup() {  
  loadeeprom(0);
  if((memory.eepromversion > 254) || (memory.eepromversion < progversion)){
    loadeeprom(0, true);
  }
  delay(10);
  Serial.begin(115200);
  delay(1000);
  if(memory.debug && !Serial)memory.debug = false;
  if(memory.debug)Serial.println(F("start setup"));
  if(memory.debug)Serial.println(F("init RTC Unit"));
  //Controllino_RTC_init(0);
  //Controllino_SetTimeDate(20,3,9,23,22,9,5); // (Day of the month, Day of the week, Month, Year, Hour, Minute, Second)
  init_input();
  init_relais();
  init_digitaloutput();
  init_lighttimercurrentstates();
  init_lighttimers();
  init_jaltimers();
  //init_12c();
  if(memory.debug)Serial.println(F("setup watchdog to 8s"));
  wdt_enable(WDTO_8S);
  if(memory.debug)Serial.println(F("setup finish"));
  wdt_reset();
  if(memory.debug)PrintHelp();
}

void doc_collect_switch_states(){
    for(uint8_t i=0;i<lightsindex; i++){
      doc["light"] = i;
      doc["on"] = digitalRead(lights[i]);
    }
  }

void doc_collect_jalousie_states(){
  for(uint8_t i=0;i<jalousieindex; i++){
    doc["jalousie"] = i;
    doc["position"] = "up";
    if(jalpositiondown[i])doc["position"] = "down";
    doc["moving"] = bool(digitalRead(jaldowns[i]) || digitalRead(jalups[i]));
  }
}

/*
bool send_mqtt_docs(char msgtype = "alive"){
  char mqttpath[32] = "neubau/states/";
  doc.clear();

  doc["v"] = 1;
  doc["rom"] = memory.eepromversion;
  if(strcmp(msgtype, "light") == 0){
    doc_collect_switch_states();
    strcat(mqttpath, "lights");
  }
  else if(strcmp(msgtype, "jalousie") == 0){
    doc_collect_jalousie_states();
    strcat(mqttpath, "jalousie");
  }
  else if(strcmp(msgtype, "alive") == 0){
    doc["alive"] = random(1,99);
    strcat(mqttpath, "alive");
  }
  else if(strcmp(msgtype, "state") == 0){
    doc_collect_switch_states();
    doc_collect_jalousie_states();
    strcat(mqttpath, "all");
  }
  else {
    return false;
  }

  if(memory.debug){
    Serial.print(F("Message = "));
    serializeJson(doc, Serial);
    Serial.println();
  }

  char msgbuffer[256];
  size_t msgbufferlen = serializeJson(doc, msgbuffer);
  return client.publish(mqttpath, msgbuffer, msgbufferlen);
}
*/

void loop() {
  currmillis = millis();
  if((unsigned long)(currmillis - eepromtask) >= 5000){
    if(eepromupdate){
      if(memory.debug)Serial.println(F("save settings to eeprom"));
      saveeeprom();
    }
    eepromtask = millis();
  }
  // jalousie task
  if((unsigned long)(currmillis - jalousietask) >= 250){
    jalousie_tasks();
    jalousietask = millis();
  }

  // switch tasks
  if((unsigned long)(currmillis - switchtask) >= 150){
    switch_tasks();
    switchtask = millis();
  }

  if((unsigned long)(currmillis - serialtask) >= 10){
    SerialEvent();
    serialtask = millis();
  }
/*
  // check every seconds the connection to mqtt server
  if(memory.enable_mqtt && (unsigned long)(currmillis - ethtask) >= 1000){
    if((unsigned long)(currmillis - checkmqtt) >= 5000){
      if(!client.connected()){
        mqtt_reconnect();
      }
      checkmqtt = millis();
    }
    if(client.connected()){
      client.loop();

      if(client.connected() && (unsigned long)(currmillis - alive) >= 30000UL){
        send_mqtt_docs("alive");
        alive = millis();
      }
    }
    ethtask = millis();
  }
  */
  wdt_reset();
  yield();
}


void ProcessSerialEvent(uint8_t index) {
  char* cmd = strtok(serialbuffer, ',');
  char* sep = strchr(serialbuffer, ',');
  *sep = 0;
  ++sep;

  if(memory.debug){
    Serial.print(F("Command = "));
    Serial.println(cmd);
    Serial.print(F("seperator = "));
    Serial.println(sep); 
  }

  if(strcmp(cmd,"switch") == 0){
    uint8_t id = atoi(strtok(sep, ','));
    char* on = strchr(sep, ',');
    *on = 0;
    ++on;
    
    bool state = false;
    if(atoi(on)>0)state = true;
    switch_tasks_auto(id, state);
    if(memory.debug){
      Serial.print(F("id = "));
      Serial.print(id);
      Serial.print(F(" switch on = "));
      Serial.println(state);
    }
  }

  else if(strcmp(cmd, "jal") == 0){
    uint8_t id = atoi(strtok(sep, ','));
    char* direction = strchr(sep, ',');
    *direction;
    ++direction;

    // 1 = up, 2 = down, 0 or all other = stop
    bool up = false;
    bool down = false;
    Serial.println(atoi(direction));
    if((uint8_t)atoi(direction) == 1){
      up = true;
    }
    else if((uint8_t)atoi(direction) == 2){
      down = true;
    }

    jalousie_tasks_auto(id, up, down);
    if(memory.debug){
      Serial.print(F("jalousie "));
      Serial.println(id);
      Serial.print(F("up = "));
      Serial.println(up);
      Serial.print(F("down = "));
      Serial.println(down);
    }
  }

  else if(strcmp(cmd, "jaltimer") == 0){
    uint8_t id = atoi(strtok(sep, ','));
    char* tseconds = strchr(sep, ',');
    *tseconds;
    ++tseconds;
    if(atoi(sep) >= 0 && atoi(sep) < 255){
      switch(uint8_t(id)){
        case 0:
          memory.jaltimer1 = (uint8_t)atoi(tseconds);
          break;
        case 1:
          memory.jaltimer2 = (uint8_t)atoi(tseconds);
          break;
        case 2:
          memory.jaltimer3 = (uint8_t)atoi(tseconds);
          break;
        case 3:
          memory.jaltimer4 = (uint8_t)atoi(tseconds);
          break;
        default:
          break;
      }
      jaltimers[id] = (unsigned long)(((uint8_t)atoi(tseconds)) * 1000UL);
      eepromupdate = true;
    }
  }

  else if(strcmp(cmd, "timerlight") == 0){
    uint8_t id = atoi(strtok(sep, ','));
    char* minutest = strchr(sep, ',');
    *minutest;
    ++minutest;
    if(atoi(minutest) >= 0 && atoi(minutest) < 121){
      switch(uint8_t(atoi(id))){
        case 0:
          memory.timerlight1 = uint8_t(atoi(minutest));
          break;
        case 1:
          memory.timerlight2 = uint8_t(atoi(minutest));
          break;
        case 2:
          memory.timerlight3 = uint8_t(atoi(minutest));
          break;
        case 3:
          memory.timerlight4 = uint8_t(atoi(minutest));
          break;
        case 4:
          memory.timerlight5 = uint8_t(atoi(minutest));
          break;
        case 5:
          memory.timerlight6 = uint8_t(atoi(minutest));
          break;
        case 6:
          memory.timerlight7 = uint8_t(atoi(minutest));
          break;
        case 7:
          memory.timerlight8 = uint8_t(atoi(minutest));
          break;
        case 8:
          memory.timerlight9 = uint8_t(atoi(minutest));
          break;
        case 9:
          memory.timerlight10 = uint8_t(atoi(minutest));
          break;
        case 10:
          memory.timerlight11 = uint8_t(atoi(minutest));
          break;
        case 11:
          memory.timerlight12 = uint8_t(atoi(minutest));
          break;
        default:
          break;
      }
      lighttimers[id] = (unsigned long)((uint8_t(atoi(minutest))) * 60000UL);
      eepromupdate = true;
    }
  }

  else if(strcmp(cmd, "mqtt") == 0){
    memory.enable_mqtt = false;
    if(atoi(sep) > 0)memory.enable_mqtt = true;
    eepromupdate = true;
  }

  else if(strcmp(cmd, "showsettings") == 0){
    ShowSettings();
  }
  
  else if(strcmp(cmd, "debug") == 0){
    if(atoi(sep)>0){
      memory.debug = true;
    } else {
      memory.debug = false;
    }
    eepromupdate = true;
  }
  else if(strcmp(cmd, "factory") == 0){
    if(atoi(sep)>0){
      loadeeprom(0, true);
    }
  }
  else if(strcmp(cmd, "reboot") == 0){
    if(atoi(sep)>0){
      resetFunc();
    }
  }
  else if(strcmp(cmd, "i2caddress") == 0){
    if(atoi(sep)>0 && atoi(sep)<250){
      int chartoInt = atoi(sep);
      memory.i2caddress = (byte)chartoInt;
      eepromupdate = true;
    }
  }
  else if(strcmp(cmd, "help") == 0){
    PrintHelp();
  }
  clsbuffer(); //cleanup buffer
}


/*
// mqtt callback
void callback(char* topic, byte* payload, unsigned int msglength) {
  doc.clear();
  DeserializationError error = deserializeJson(doc, payload);
  if(memory.debug){
    Serial.println("mqtt received");
    Serial.print("topic = ");
    Serial.println(topic);
    Serial.print("payload = ");
    serializeJson(doc, Serial);
  }
  if(error){
    return;
  }
  yield();

  if (strcmp(topic, "neubau/switch") == 0) {
    uint8_t id = doc["switch"];
    bool on = doc["on"];
    switch_tasks_auto(id, on);
    send_mqtt_docs("lights");
  }

  if(strcmp(topic, "neubau/jalousie") == 0){
    uint8_t id = doc["jalousie"];
    bool up, down = false;
    if(strcmp(doc["drive"], "up") == 0)up = true;
    if(strcmp(doc["drive"], "down") == 0)down = true;
    jalousie_tasks_auto(id, up, down);
    send_mqtt_docs("jalousie");
  }
  
  return;
}


void mqtt_reconnect() {
  // Attempt to connect (clientId, username, password)
  if(memory.debug)Serial.print(F("mqtt connect"));
  if ( client.connect(mqtt_clientid, mqtt_user, mqtt_password) ) {
    // json values
    client.subscribe("neubau/jalousie");
    client.subscribe("neubau/switch");
  }
  yield();
}


void setup_mqtt_client() {
  //setup mqtt client
  client.setKeepAlive(60);
  client.setServer(mqttserver, 1883);
  client.setCallback(callback);
  lastsend = 0;
}

/*boardid
void requestEvent(){
  if(memory.debug)Serial.println(F("read states"));
  clsbuffer();
  statestruct i2cstate = {
    digitalRead(CONTROLLINO_D0),
    digitalRead(CONTROLLINO_D1),
    digitalRead(CONTROLLINO_D2),
    digitalRead(CONTROLLINO_D3),
    digitalRead(CONTROLLINO_D4),
    digitalRead(CONTROLLINO_D5),
    analogRead(CONTROLLINO_A0),
    analogRead(CONTROLLINO_A1),
    analogRead(CONTROLLINO_A2),
    analogRead(CONTROLLINO_A3),
    analogRead(CONTROLLINO_A4),
    analogRead(CONTROLLINO_A5),
    memory.eepromversion,
    memory.debug,
  };
  if(memory.debug){
    Serial.print(F("Sending struct with "));
    Serial.print(sizeof i2cstate);
    Serial.println(F("bytes"));
  }
  Wire.write((byte *)&i2cstate, sizeof i2cstate);
}
*/