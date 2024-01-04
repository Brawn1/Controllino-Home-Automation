/*
 * Controllino MEGA Automation integrate in HomeAssistant
 * 
 * 
 * v 0.4
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
#define lswitch_1 CONTROLLINO_A7 // Kitchen

#define light_2 CONTROLLINO_D0 // Kitchen Workplace (Cooking plate)
#define lswitch_2 CONTROLLINO_A8 // Kitchen Workplace (Cooking plate)

#define light_3 CONTROLLINO_R1 // Kitchen Workplace
#define lswitch_3 CONTROLLINO_A9 // Kitchen Workplace

// dinning table
#define light_4 CONTROLLINO_R6 // dinning table
#define lswitch_4 CONTROLLINO_A9 // dinning table

// floor
#define light_5 CONTROLLINO_R7 // floor
#define lswitch_5 CONTROLLINO_A9 // floor

#define light_6 0 // floor (behind)
#define lswitch_6 0 // floor (behind) (A12 ??)

#define light_7 0 // Entry
#define lswitch_7 0 // Entry

#define light_8 0 // floor
#define lswitch_8 0 // floor

#define light_9 0 // floor (behind)
#define lswitch_9 0 // floor (behind)

#define light_10 0 // Entry
#define lswitch_10 0 // Entry (switch door)

#define light_11 0 // storage
#define lswitch_11 0 // storage

#define light_12 0 // dinning table
#define lswitch_12 0 // dinning table


// MQTT settings
IPAddress mqttserver(192, 168, 2, 242);
#define mqtt_user "controllino"
#define mqtt_password "controllino-2023"
#define mqtt_clientid "bsn-mega1"

// Update these with values suitable for your network.
byte mac[]    = { 0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };


// lights
uint8_t lightsindex = 12;
uint8_t lights[] = {light_1, light_2, light_3, light_4, light_5, light_6, light_7, light_8, light_9, light_10, light_11, light_12};
uint8_t lightswitches[] = {lswitch_1, lswitch_2, lswitch_3, lswitch_4, lswitch_5, lswitch_6, lswitch_7, lswitch_8, lswitch_9, lswitch_10, lswitch_11, lswitch_12};
bool lightstate[] = {false, false, false, false, false, false, false, false, false, false, false, false};
unsigned long lighttimercurrentstates[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long lighttimers[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t switchlightgroups[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// jalousie
uint8_t jalousieindex = 4;
uint8_t jalswitchesup[] = {jalswitchup_1, jalswitchup_2, jalswitchup_3, jalswitchup_4};
uint8_t jalswitchesdown[] = {jalswitchdown_1, jalswitchdown_2, jalswitchdown_3, jalswitchdown_4};
uint8_t jalups[] = {jalup_1, jalup_2, jalup_3, jalup_4};
uint8_t jaldowns[] = {jaldown_1, jaldown_2, jaldown_3, jaldown_4};
unsigned long jaltimers[] = {20000UL, 40000UL, 40000UL, 40000UL};
unsigned long jaltimercurrentstates[] = {0, 0, 0, 0};
bool jalpositiondown[] = {false, false, false, false};


const uint8_t progversion = 5;
const byte address = 2;

bool eepromupdate, laststateeepromupdate, mqttcommandreceived, ethernet_connected = false;
unsigned long currmillis, eepromtask, jalousietask, switchtask, serialtask, lastsend, checkmqtt, ethtask, alive = 0;

uint8_t serialbuffersize = 0;
char serialbuffer[100];

uint8_t mqttpathsize = 0;
char mqttpath[64] = "bailey/m1/";

struct memorystruct{
  bool meminit;
  bool debug;
  bool mqtt_enabled;
  bool automaticdrive;
  uint8_t jaltimers[4];
  uint8_t timestamp_down_h;
  uint8_t timestamp_down_m;
  uint8_t timestamp_up_h;
  uint8_t timestamp_up_m;
  uint8_t timerlights[12];
  uint8_t eepromversion;
  byte i2caddress;
};

memorystruct memory = {
  false,
  false,
  false,
  false,
  {20, 40 ,40, 40},
  0,
  0,
  0,
  0,
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  progversion,
  address,
};


char* msgtype;
StaticJsonDocument<256> doc;

EthernetClient ethClient;
PubSubClient client(ethClient);


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


void clsmqttpath(){
  memset(mqttpath, 0, sizeof(mqttpath));
  mqttpathsize = 0;
  strcpy(mqttpath, "bailey/m1/");
}


void PrintHelp(){
  Serial.println(F("Controllino Automation Help"));
  Serial.println();
  Serial.println(F("commands:"));
  Serial.println();
  Serial.println(F("help,1                                                                  show this help page"));
  Serial.println(F("switch,<id>,<1=on, 2=off>                                               switch the light on or off"));
  Serial.println(F("jal,<id>,<1=up, 2=down, 3 or 0 = stop>                                  jalousie drive up or down or stop"));
  Serial.println(F("jaltimer,<id>,<time in seconds between 0 and 255>                       set time in seconds to stop the jalousie"));
  Serial.println(F("timerlight,<id>,<time in minutes between 0 and 121 (0 = deactivated)>   set specified light to automatic off light"));
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
  Serial.print(sizeof(memory));
  Serial.println(F(" bytes"));
  Serial.print(F("Program version = "));
  Serial.println(progversion);
  Serial.print(F("Debug mode = "));
  Serial.println(memory.debug);
  Serial.print(F("i2c address = "));
  Serial.println(memory.i2caddress);
  Serial.println(F("=================================="));
  Serial.print(F("IP :"));
  Serial.println(Ethernet.localIP());
  
  Serial.print(F("MQTT "));
  if(memory.mqtt_enabled){
    Serial.println(F("enabled"));
  } else {
    Serial.println(F("disabled"));
  }

  Serial.println(F("=================================="));
  Serial.println(F("index = Switch = light = current state = timer (enabled / time in minutes) = current countdown in seconds"));
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
    Serial.print(lighttimers[i] / 60000UL);
    Serial.print(F(" = "));
    if(lighttimercurrentstates[i] > 0){
      Serial.println((unsigned long)((millis() - lighttimercurrentstates[i]) / 1000));
    } else {
      Serial.println(0);
    }
  }
  Serial.println(F("=================================="));
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
  Serial.println(F("=================================="));
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
  Serial.println(F("=================================="));
  Serial.println(F("OUTPUT Jalousie number = UP = DOWN"));
  for(uint8_t i=0; i<jalousieindex; i++){
    Serial.print(F("Jalousie "));
    Serial.print(i);
    Serial.print(F(" = "));
    Serial.print(jalups[i]);
    Serial.print(F(" = "));
    Serial.println(jaldowns[i]);
  }
  Serial.println(F("=================================="));
  Serial.println(F("Jalousie Timer Settings"));
  Serial.println(F("Jalousie Number = Activetime in Seconds"));
  for(uint8_t i=0; i<jalousieindex; i++){
    Serial.print(F("Jalousie "));
    Serial.print(i);
    Serial.print(F(" = "));
    Serial.println(jaltimers[i] / 1000);
  }
  Serial.println(F("=================================="));
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
  for(uint8_t i = 0; i<4; i++){
    if(!memory.jaltimers[i] || memory.jaltimers[i] > 254){
      if(i < 1){
        memory.jaltimers[i] = 20;
      } else {
        memory.jaltimers[i] = 40;
      }
      eepromupdate = true;
    }
    jaltimers[i] = (unsigned long)(memory.jaltimers[i] * 1000UL);
  }
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
  for(uint8_t i = 0; i < lightsindex; i++){
    if(memory.timerlights[i] > 254){
      memory.timerlights[i] = 0;
      eepromupdate = true;
    }
    lighttimers[i] = (unsigned long)(memory.timerlights[i] * 60000UL);
  }
}


void switch_io(uint8_t port, bool on = false){
  uint8_t state = LOW;
  if(on)state = HIGH;
  digitalWrite(port, state);
}



bool is_lighton(uint8_t index){
  return digitalRead(lights[index]);
}


void doc_collect_switch_states(){
  for(uint8_t i=0;i<lightsindex; i++){
    doc["lights_on"].add(digitalRead(lights[i])? "true":"false");
  }
}


void doc_collect_jalousie_states(){
  for(uint8_t i=0;i<jalousieindex; i++){
    doc["jalousie_moving"].add((digitalRead(jaldowns[i]) || digitalRead(jalups[i]))? "true":"false");
  }
}


bool send_mqtt_message(char* type = "state", bool cleanup_doc = true, bool reset_mqttpath = true){
  if(!memory.mqtt_enabled || !client.connected())return;

  if(reset_mqttpath){
    clsmqttpath();
    strcat(mqttpath, type);
  }
  
  if(cleanup_doc){
    doc.clear();
  }
  
  doc["v"] = 1;
  if(strcmp(type, "state") == 0){
    doc_collect_switch_states();
    doc_collect_jalousie_states();
  }
  else if(strcmp(type, "config") == 0){
    doc["eeprom_size"] = (uint16_t)sizeof(memory);
    doc_collect_jalousie_states();
    doc_collect_switch_states();
    doc["debug"] = (bool) memory.debug;
    doc["version"] = (uint16_t) memory.eepromversion;
  }
  if(memory.debug){
    Serial.print(F("topic = "));
    Serial.println(mqttpath);
    Serial.print(F("Message = "));
    serializeJson(doc, Serial);
    Serial.println();
  }

  char msgbuffer[256];
  size_t msgbufferlen = serializeJson(doc, msgbuffer);
  return client.publish(mqttpath, msgbuffer, msgbufferlen);
}

char partpath[18];
char indexbuffer[2];

void send_mqtt_light_state(uint8_t index, bool on = false){
  if(!memory.mqtt_enabled || !client.connected())return;
  doc.clear();
  doc["switch"] = index;
  doc["state"] = on?"ON":"OFF";
  if(memory.timerlights[index]){
    doc["timer"] = memory.timerlights[index];
  }  
  strcpy(partpath, "switch/state/");
  itoa(index, indexbuffer, 10);
  strcat(partpath, indexbuffer);
  send_mqtt_message(partpath, false, true);
}


void send_mqtt_jalousie_state(uint8_t index, bool up = false, bool down = false){
  if(!memory.mqtt_enabled || !client.connected())return;
  doc.clear();
  doc["jalousie"] = (uint8_t)index;
  doc["moving"] = false;
  doc["timer"] = (uint8_t)memory.jaltimers[index];
  if(up || down)doc["moving"] = true;
  doc["direction"] = (char*)0;
  if(up)doc["direction"] = "up";
  else if(down)doc["direction"] = "down";

  strcpy(partpath, "jalousie/state/");
  itoa(index, indexbuffer, 10);
  strcat(partpath, indexbuffer);
  send_mqtt_message("jalousie/state", false, true);
}

// switch over Serial and Web event
void switch_tasks_auto(uint8_t index, bool on = false){
  if((index > lightsindex) || lights[index] == 0)return;
  bool lighton = is_lighton(index);
  if(lighton != on){
    switch_io(lights[index], !lighton);
    if(lighttimers[index] && !lighton)lighttimercurrentstates[index] = millis();
  }
  send_mqtt_light_state(index, is_lighton(index));
}

bool switchlaststate[] = {false, false, false, false, false, false, false, false, false, false, false, false};
void switch_tasks(bool load_prev_state = false){
  for(uint8_t i=0; i<lightsindex; i++){
    if(lightswitches[i] == 0 || lights[i] == 0){
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
      else if(lighttimers[i] && lighton)lighttimercurrentstates[i] = 0;
      // send state live to mqtt
      send_mqtt_light_state(i, is_lighton(i));
    }
    else if(!digitalRead(lightswitches[i]) && switchlaststate[i]){
      switchlaststate[i] = false;
    }

    if(lighttimers[i] && lighttimercurrentstates[i] > 0 && is_lighton(i) && ((unsigned long)(millis() - lighttimercurrentstates[i]) >= lighttimers[i])){
      switch_io(lights[i], false);
      lighttimercurrentstates[i] = 0;
      // send live to
      send_mqtt_light_state(i, false);
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
    send_mqtt_jalousie_state(index, true, false);
  }
  else if(down){
    if(jalmoving && digitalRead(jalups[index])){
      switch_io(jalups[index], false);
      return;
    }
    switch_io(jaldowns[index], true);
    jaltimercurrentstates[index] = millis();
    jalpositiondown[index] = true;
    send_mqtt_jalousie_state(index, false, true);
  }
  else {
    switch_io(jaldowns[index], false);
    switch_io(jalups[index], false);
    jaltimercurrentstates[index] = 0;
    send_mqtt_jalousie_state(index, false, false);
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
      send_mqtt_jalousie_state(i, true);
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
      send_mqtt_jalousie_state(i, false, true);
    }
    else if(jalmoving && (unsigned long)(millis() - jaltimercurrentstates[i]) >= jaltimers[i]){
      switch_io(jalups[i], false);
      switch_io(jaldowns[i], false);
      jaltimercurrentstates[i] = 0;
      jalswitchedduration[i] = 0;
      jalswitchlaststate[i] = false;
      send_mqtt_jalousie_state(i);
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
  setup_mqtt_client();
  if(memory.debug)Serial.println(F("setup watchdog to 8s"));
  wdt_enable(WDTO_8S);
  if(memory.debug)Serial.println(F("setup finish"));
  wdt_reset();
  if(memory.debug)PrintHelp();
}


bool send_mqtt_states(){
  return send_mqtt_message("state", true, true);
}

bool send_mqtt_config(){
  return send_mqtt_message("config", true, true);
}


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

  // check every 0.5 seconds to the mqtt
  if(memory.mqtt_enabled && (unsigned long)(currmillis - ethtask) >= 250){
    if((unsigned long)(currmillis - checkmqtt) >= 5000){
      if(!client.connected()){
        mqtt_reconnect();
      }
      checkmqtt = millis();
    }
    if(client.connected()){
      client.loop();
    }
    ethtask = millis();
  }
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
    }  uint8_t timerlights[11];
  }

  else if(strcmp(cmd, "jaltimer") == 0){
    uint8_t id = atoi(strtok(sep, ','));
    char* tseconds = strchr(sep, ',');
    *tseconds;
    ++tseconds;
    if((atoi(sep) >= 0 && atoi(sep) < 255) && id < jalousieindex){
      memory.jaltimers[id] = (uint8_t)atoi(tseconds);
      jaltimers[id] = (unsigned long)(((uint8_t)atoi(tseconds)) * 1000UL);
      eepromupdate = true;
    }
  }

  else if(strcmp(cmd, "timerlight") == 0){
    uint8_t id = atoi(strtok(sep, ','));
    char* minutest = strchr(sep, ',');
    *minutest;
    ++minutest;
    if(((uint8_t(atoi(minutest) >= 0)) && (uint8_t(atoi(minutest) < 121))) && id < lightsindex){
        memory.timerlights[id] = uint8_t(atoi(minutest));
        lighttimers[id] = (unsigned long)((uint8_t(atoi(minutest))) * 60000UL);
        eepromupdate = true;
    }
  }

  else if(strcmp(cmd, "mqtt") == 0){
    memory.mqtt_enabled = false;
    if(atoi(sep) > 0)memory.mqtt_enabled = true;
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

uint8_t getdigit(char* msg){
  int digitPosition = -1;
  uint8_t value = 255;
  for (uint8_t i = 0; i < strlen(msg); i++) {
    if (isdigit(msg[i])) {
      digitPosition = i;
      break;
    }
  }
  if (digitPosition != -1){
    value = atoi(msg + digitPosition);
  }
  return value;
}

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
    Serial.println("");
  }
  if(error){
    return;
  }
  yield();

  if (strcmp(topic, "bailey/m1/switch") == 0) {
    uint8_t id = doc["switch"];
    bool on = false;
    if(strcmp(doc["state"], "ON") == 0)on = true;
    switch_tasks_auto(id, on);
  }

  if(strcmp(topic, "bailey/m1/jalousie") == 0){
    uint8_t id = doc["jalousie"];
    bool up, down = false;
    if(strcmp(doc["state"], "UP") == 0)up = true;
    if(strcmp(doc["state"], "DOWN") == 0)down = true;
    jalousie_tasks_auto(id, up, down);
  }

  if (strcmp(topic, "bailey/m1/get") == 0) {
    if(strcmp(doc["type"], "state") == 0)send_mqtt_states();
    else if(strcmp(doc["type"], "config") == 0)send_mqtt_config();
  }
  
  return;
}


void mqtt_reconnect() {
  // Attempt to connect (clientId, username, password)
  if(memory.debug)Serial.println(F("mqtt connect"));
  if ( client.connect(mqtt_clientid, mqtt_user, mqtt_password) ) {
    // json values
    client.subscribe("bailey/m1/jalousie");
    client.subscribe("bailey/m1/switch");
  }
  yield();
}


void setup_mqtt_client() {
  //setup mqtt client
  client.setKeepAlive(60);
  client.setServer(mqttserver, 1883);
  client.setCallback(callback);
  Ethernet.begin(mac);
  lastsend = 0;
}
