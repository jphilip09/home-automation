#include <Arduino.h>

/*
  Water Level Controller

  Circuit:
    On a Arduino Mega with Ethernet shield
    Now on NodeMCU
    Ultrasound sensors for checking water level in both tanks
    Relay to control motor
    ACS712 20A current sensor  to check current flow for motor
    LCD display 20x4 for displaying details

  Ethernet shield used to publish details to MQTT server

  by Philip Joseph (jphilip09@gmail.com)

  Licensed under the Apache License, Version 2.0 (the "License"); you may
  not use this file except in compliance with the License. You may obtain
  a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  License for the specific language governing permissions and limitations
  under the License.

*/

// Enable MQTT
// #define NW_SUPPORT

// USe Wifi
// #define WIFI_SUPPORT

// Enable LCD
// #define LCD_DISPLAY
// #define LCD_I2C

#include <EEPROM.h>
#include <SPI.h>
#ifdef WIFI_SUPPORT
#include <ESP8266WiFi.h>
#else
#include <Ethernet.h>
#endif
#include <PubSubClient.h>

#ifdef LCD_DISPLAY
#include <LiquidCrystal.h>
#define LCDISPLAY
#endif

#ifdef LCD_I2C
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#define LCDISPLAY
#endif


// Enable or disable debug prints on serial
// #define DEBUG true
// #define INFO true
#define PRINTF_BUF 124 // define the tmp buffer size
#define EEPROM_MAX_ADDR 511 // Restrict the EEPROM address usage

// Current measurement needs to be fixed
// #define CURRENT_ENABLE

// In case DHCP fails, static IP to use
#define FALLBACK_IP 10, 168, 1, 12

// MQTT server
#define MQTT_SERVER 10, 168, 1, 10
#define MQTT_USER "pi"
#define MQTT_PASSWD "raspberry"

// Topic
#define TOPIC "jr-mqtt/wlc"
#define DBG_TOPIC "jr-mqtt/debug"

// Max interval between publish of a topic in minutes
#define PUB_INTERVAL 1

#define topic_cat(sub) TOPIC#sub
#define topic_dbg(sub) DBG_TOPIC#sub

// Overhead Tank specifics
// Topic suffix
#define OHT_TOPIC topic_cat(/oht/level)
#define OHT_DBG topic_dbg(/oht)
// Max water level in cms
#define OHT_HT 140
// Offset of sensor
#define OHT_OFF 26
// High thershold in %
#define OHT_HIGH 90
// Low threshold
#define OHT_LOW 50
// sensor GPIO pins
#define OHT_TRIG 16 //D0
#define OHT_ECHO 5  //D1

// Storage Tank specifics
// Topic suffix
#define ST_TOPIC topic_cat(/s2/level)
#define ST_DBG topic_dbg(/s2)
// Max water level in cms
#define ST_HT 180
// Offset of sensor
#define ST_OFF 50
// High thershold in %
#define ST_HIGH 100
// Low threshold
#define ST_LOW 17
// sensor GPIO pins
#define ST_TRIG 4 //D2
#define ST_ECHO 0 //D3

// Motor 2 specifics
// ON/OFF Topic suffix
#define M2_STATE_TOPIC topic_cat(/motor2/state)
#define M2_DBG topic_dbg(/motor2)
// Manual override topic
#define M2_MANUAL topic_cat(/motor2/switch)

// Set the Automatic or Manual mode
#define M2_MANUAL_MODE topic_cat(/motor2/manual-mode)
#define M2_MANUAL_STATE topic_cat(/motor2/manual-state)
#define M2_MANUAL_MODE_ADDR 0x00

// Maximum time to run at a time in mins
#define M2_MAX_DUR 60
// Relay GPIO pin
#define M2_RELAY 2 //D5
#define MOTOR_ON LOW
#define MOTOR_OFF HIGH
// Current sensor Analog pin
#define M2_CUR A0
// Flow check
#define FLOW_DUR 5 // Check flow every 5 mins
#define MIN_FLOW_EXP 3 // Minimum change in level

// Current sensor details for ACS712 - 20A
#define MVPERAMP 100
#define ACSOFFSET 2500
#define MAX_RANGE 1024
#define INPUT_MV 5000
// Current drawn topic
#define M2_CUR_TOPIC topic_cat(/motor2/current)

// LCD specifics
#define LCD_COLS 20
#define LCD_ROWS 4
#define LCD_RS_GPIO 28
#define LCD_EN_GPIO 29
#define LCD_D4_GPIO 30
#define LCD_D5_GPIO 31
#define LCD_D6_GPIO 32
#define LCD_D7_GPIO 33

#define LCD_I2C_ADDR 0x27
#define LCD_I2C_SDA 14
#define LCD_I2C_SCL 12

#define MOTOR_ROW 1
#define OHT_ROW 2
#define ST_ROW 3

// Reread delay for level sensor
#define READ_DELAY 50

// Check the levels interval in secs
#define CHECK_INT 15

#ifdef LCD_DISPLAY
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS_GPIO, LCD_EN_GPIO, LCD_D4_GPIO, LCD_D5_GPIO, LCD_D6_GPIO, LCD_D7_GPIO);
#endif

#ifdef LCD_I2C
// Initialize LCD DIsplay with I2C interface
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
#endif

#ifdef NW_SUPPORT

#ifdef WIFI_SUPPORT
char ssid[16]     = "";
char password[32] = "";
#define WIFI_DETAILS_ADDR 0x04

WiFiClient espClient;

#else

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {
  0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x41
};

// Fallback IP
IPAddress ip(FALLBACK_IP);

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
EthernetClient ethClient;

#endif

// MQTT server
IPAddress server(MQTT_SERVER);

// Maximum topics to subscribe
#define MAX_SUBS 10

PubSubClient client;
#endif

//
// Reads a string starting from the specified address.
// Returns true if at least one byte (even only the
// string terminator one) is read.
// Returns false if the start address falls outside
// or declare buffer size os zero.
// the allowed range.
// The reading might stop for several reasons:
// - no more space in the provided buffer
// - last eeprom address reached
// - string terminator byte (0x00) encountered.
// The last condition is what should normally occur.
//
boolean eeprom_read_string(int addr, char* buffer, int bufSize) {
  // byte read from eeprom
  byte ch;

  // number of bytes read so far
  int bytesRead;

  // check start address
  if (addr >= EEPROM_MAX_ADDR+1) {
    return false;
  }

  // how can we store bytes in an empty buffer ?
  if (bufSize == 0) {
    return false;
  }

  // is there is room for the string terminator only,
  // no reason to go further
  if (bufSize == 1) {
    buffer[0] = 0;
    return true;
  }

  // initialize byte counter
  bytesRead = 0;

  // read next byte from eeprom
  EEPROM.begin(EEPROM_MAX_ADDR+1);
  ch = EEPROM.read(addr + bytesRead);

  // store it into the user buffer
  buffer[bytesRead] = ch;

  // increment byte counter
  bytesRead++;

  // stop conditions:
  // - the character just read is the string terminator one (0x00)
  // - we have filled the user buffer
  // - we have reached the last eeprom address
  while ( (ch != 0x00) && (bytesRead < bufSize) && ((addr + bytesRead) < 511) ) {
    // if no stop condition is met, read the next byte from eeprom
    ch = EEPROM.read(addr + bytesRead);

    // store it into the user buffer
    buffer[bytesRead] = ch;

    // increment byte counter
    bytesRead++;
  }

  // make sure the user buffer has a string terminator
  // (0x00) as its last byte
  if ((ch != 0x00) && (bytesRead >= 1)) {
    buffer[bytesRead - 1] = 0;
  }

  EEPROM.commit();
  EEPROM.end();
  return true;
}

void update_eeprom(int addr, char val) {
  EEPROM.begin(EEPROM_MAX_ADDR+1);
  EEPROM.write(addr, val);
  EEPROM.commit();
  EEPROM.end();
}

char read_eeprom(int addr) {
  byte ch;
  EEPROM.begin(EEPROM_MAX_ADDR+1);
  ch = EEPROM.read(addr);
  EEPROM.commit();
  EEPROM.end();
  return ch;
}

class Debug {
  bool deb = false;
  bool inf = true;
  char buf[PRINTF_BUF];

  public:
    Debug(bool debug_en, bool info_en) {
      deb = debug_en;
      inf = info_en;
      if (deb) {
        inf = true;
      }
    }

    void error(const char* format, ...){
      va_list args;
      va_start(args, format);
      vsnprintf(buf, (PRINTF_BUF-1), format, args);
      Serial.println(buf);
      va_end(args);
    }

    void debug(const char* format, ...) {
      if (deb) {
        va_list args;
        va_start(args, format);
        vsnprintf(buf, (PRINTF_BUF-1), format, args);
        Serial.println(buf);
        va_end(args);
      }
    }

    void info(const char* format, ...) {
      if (inf) {
        va_list args;
        va_start(args, format);
        vsnprintf(buf, (PRINTF_BUF-1), format, args);
        Serial.println(buf);
        va_end(args);
      }
    }
};

Debug debug = Debug(DEBUG, INFO);


class LcdDisplay {
#ifdef LCDISPLAY
  int rows = LCD_ROWS - 1;
  int cols = LCD_COLS;
  int err_row = 0; // First line on display
  bool err_disp = false;
#endif

  public:
    void display(int row, const char* str) {
#ifdef LCDISPLAY
      int r = row;
      if (row > rows) {
        r = rows;
      }
      lcd.setCursor(0, r);
      lcd.print(str);
#endif
    }

    void error(const char* str) {
#ifdef LCDISPLAY
      err_disp = true;
      lcd.setCursor(0, err_row);
      lcd.print(str);
#endif
    }

    void clear_err() {
#ifdef LCDISPLAY
      err_disp = false;
      lcd.setCursor(0, err_row);
      lcd.print(' ');
#endif
    }
};

LcdDisplay ld;

class LevelSensor {
  int trigger;
  int echo;
  const long tries = 3;
  long max_ht = 0;
  long min_ht = 0;

  public:
    void set(int trig_gpio, int echo_gpio, long min_val, long max_val) {
      trigger = trig_gpio;
      echo = echo_gpio;
      pinMode(trigger, OUTPUT);
      pinMode(echo, INPUT);
      min_ht = min_val;
      max_ht = max_val;
    }

    long check() {
      long t = 0, h = 0;

      // Transmitting pulse
      digitalWrite(trigger, LOW);
      delayMicroseconds(2);
      digitalWrite(trigger, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger, LOW);

      // Waiting for pulse
      t = pulseIn(echo, HIGH);

      // Calculating distance
      h = t / 58;
      debug.debug("Height calculated: %d", h);

      if ((h >= min_ht) && (h <= max_ht)) {
        return h;
      }
      else {
        return -1;
      }
    }

    long check_avg() {
      long acc = 0;
      int count = 0;
      for (int i=0; i<tries; i++) {
        long h = check();
        if ((h >= min_ht) && (h <= max_ht)) {
          acc += h;
          count++;
        }
        delay(READ_DELAY);
      }
      if (count == 0) {
        return -1;
      }
      long avg = acc/count;
      debug.debug("Height avg calculated: %d", avg);
      return avg;
    }
};


class Subscribe {
  // Virtual class to support subscribe callback function
  public:
    virtual void callback(String& topic, String& payload) = 0;
};

class Publish {
#ifdef NW_SUPPORT
  long interval;
  long lastUpdate;
  long diff;
  long lastValue;
  bool lastState;
  char *my_topic;
  char *my_debug;

  static int sub_count;
  static char* sub_topic[MAX_SUBS];
  static Subscribe* sub_instance[MAX_SUBS];

  // buffer to convert value to string
  static const int buf_len = 80;
  static char buf[buf_len];
#endif

  static boolean reconnect(bool force=false) {
#ifdef NW_SUPPORT
    if (force) {
      if (client.connected()) {
        client.disconnect();
      }
    }
    if (!client.connected()) {
#ifdef MQTT_USER
      if (client.connect("jrMqttClient", MQTT_USER, MQTT_PASSWD)) {
#else
      if (client.connect("jrMqttClient")) {
#endif
        debug.info("MQTT connected");
        for (int i=0; i<sub_count; i++) {
          client.subscribe(sub_topic[i]);
          debug.info("Subscribe for %s", sub_topic[i]);
        }
      }
      else {
        debug.error("MQTT connect failed!");
      }
    }

    return client.connected();
#else
    return false;
#endif
  }

  public:
    void set(const char *topic, long max_interval, long max_diff, const char *dbg_top)
    {
#ifdef NW_SUPPORT
      my_topic = (char*)topic;
      my_debug = (char*)dbg_top;
      interval = max_interval * 60 * 1000; //Convert to millisecs
      diff = max_diff;
      lastUpdate = 0;
      lastValue = -1;
      lastState = false;
      reconnect();
#endif
    }

    void set(const char* topic, long max_interval, const char *dbg_top) {
      set(topic, max_interval, 0, dbg_top);
    }

    bool add_subscribe(Subscribe* instance, String topic) {
#ifdef NW_SUPPORT
      if (sub_count >= MAX_SUBS) {
        // Already max topics subscribed
        return false;
      }
      sub_instance[sub_count] = instance;
      sub_topic[sub_count] = (char*)malloc(topic.length()+1);
      strncpy(sub_topic[sub_count], (char*)topic.c_str(), topic.length()+1);
      debug.debug("Subscribe to topic: %s", sub_topic[sub_count]);
      sub_count++;
      reconnect(true);
#endif
      return true;
    }

  void pub_dbg(const char* str) {
#ifdef NW_SUPPORT
    if (reconnect()) {
      client.publish(my_debug, str);
    }
#endif
  }

  void pub_dbg1(const char* format, ...) {
#ifdef NW_SUPPORT
    if (reconnect()) {
      va_list args;
      va_start(args, format);
      vsnprintf(buf, (PRINTF_BUF-1), format, args);
      va_end(args);
      client.publish(my_debug, buf);
    }
#endif
 }

 void updateStr(const char* str) {
#ifdef NW_SUPPORT
   snprintf(buf, buf_len-1, "%s", str);
   pub_dbg(buf);
   client.publish(my_topic, buf);
   debug.info("Published topic %s with value: %s", my_topic, buf);
#endif
 }

  void updateValue(long value) {
#ifdef NW_SUPPORT
    snprintf(buf, buf_len-1, "%ld", value);
    pub_dbg(buf);
    if ((abs(lastValue - value) >= diff) || ((millis() - lastUpdate) >= interval)) {
      if (reconnect()) {
        lastValue = value;
        lastUpdate = millis();
        client.publish(my_topic, buf);
        debug.info("Published topic %s with value: %s (%ld)", my_topic, buf, value);
      }
    }
#endif
  }

    void updateValue_json(long value, int percentage) {
#ifdef NW_SUPPORT
      snprintf(buf, buf_len-1, "{ \"value\": %ld, \"percentage\": %d }", value, percentage);
      pub_dbg(buf);
      if ((abs(lastValue - value) >= diff) || ((millis() - lastUpdate) >= interval)) {
        if (reconnect()) {
          lastValue = value;
          lastUpdate = millis();
          client.publish(my_topic, buf);
          debug.info("Published topic %s with value: %s (%ld)", my_topic, buf, value);
        }
      }
#endif
    }

    void updateState(bool state) {
#ifdef NW_SUPPORT
      if (state) {
        strncpy(buf, "ON", buf_len-1);
      }
      else {
        strncpy(buf, "OFF", buf_len-1);
      }
      pub_dbg(buf);
      if ((state != lastState) || (lastUpdate == 0) || ((millis() - lastUpdate) >= interval)) {
        if (reconnect()) {
          lastState = state;
          lastUpdate = millis();
          client.publish(my_topic, buf);
          debug.debug("Published topic %s with state: %s", my_topic, buf);
        }
      }
#endif
    }

    static Subscribe* get_instance(String& topic) {
      int i = 0;
      const char* top = topic.c_str();
      debug.debug("Check topic: %s", top);
      for (i=0; i<sub_count; i++) {
        debug.debug("Check against %s", sub_topic[i]);
        if (strcmp(top, sub_topic[i]) == 0) {
          return sub_instance[i];
        }
      }
      return NULL;
    }
};

#ifdef NW_SUPPORT
// Define the static members
int Publish::sub_count = 0;
Subscribe* Publish::sub_instance[MAX_SUBS];
char* Publish::sub_topic[MAX_SUBS];
char Publish::buf[Publish::buf_len] = "";
#endif

class Tank {
  LevelSensor ls;
  Publish pub;
  int trigger;
  int echo;
  long max_ht; // Max water height
  long offset; // Distance of sensor from max_ht
  long upper_th; //Upper threshold in percentage
  long lower_th; //Lower threshold
  long check_int = 1; // Check interval in mins
  long high_th = 0; // Caluclated upper thershold
  long low_th = 0; // Lower threshold
  long last_h = 0; // Last measurement

  int disp_row = 0; //Display row
  const char* disp_name; // Name for display

  public:
    void set_display(int row, const char* name) {
      disp_row = row;
      disp_name = name;
    }

    void set(int trig, int ec, int ht, int off, int upper, int lower, const char* topic, const char* dbg_top) {
      ls.set(trig, ec, off, ht+off);
      pub.set(topic, PUB_INTERVAL, 2, dbg_top);

      trigger = trig;
      echo = ec;
      max_ht = ht;
      offset = off;
      upper_th = upper;
      lower_th = lower;
      high_th = max_ht * upper_th / 100;
      low_th = max_ht * lower_th / 100;
    }

    long get_last() {
      return last_h;
    }

    int check() {
      int ret = 0;
      long l = ls.check_avg();
      int tries = 0;
      while (l == -1) {
        delay(100);
        tries++;
        l = ls.check();
        if (tries > 5) {
          break;
        }
      }
      long h = -1;
      int p = -1;
      if (l != -1) {
        h = max_ht - (l - offset);
        last_h = h;
        p = h * 100 / max_ht;
      }

      char str[LCD_COLS+1];
      snprintf(str, LCD_COLS, "%-9s:%3ld (%3d)", disp_name, h, p);
      ld.display(disp_row, str);
      debug.debug("%s", str);
      pub.pub_dbg(str);

      if (h == -1) {
        // Sensor not working
        debug.error("%s Tank sensor not working!", disp_name);
        return -2;
      }

      if (h <= low_th) {
        ret = -1;
      }
      else if (h >= high_th) {
        ret = 1;
      }
      debug.debug("%s Tank check: %d (%ld)", disp_name, ret, h);
      pub.updateValue_json(h, p);
      return ret;
    }
};

class OverheadTank: public Tank {
  public:
    OverheadTank(){
      set_display(OHT_ROW, "Overhead");
    }
};

class StorageTank: public Tank {
  public:
    StorageTank(){
      set_display(ST_ROW, "Sump");
    }
};

class Motor: public Subscribe {
  int relay;
  int curr;
  unsigned long max_duration = 0;
  unsigned long min_interval = 0;
  const long tries = 5;
  bool on = false;
  bool manual = false; // If the motor is in manual control
  unsigned long last_on = 0;
  unsigned long last_off = 0;
  Publish spub;
  Publish cpub;
  Publish mpub;
  const int disp_row = MOTOR_ROW;
  const char *disp_name;
  bool automatic = true; // Disable automatic pumping
  const char* switch_topic = NULL;
  const char* mode_topic = NULL;

  public:
    Motor():
    disp_name("Motor"){
    }

    void lcdisplay() {
      char str[LCD_COLS+1];
      if (on) {
        snprintf(str, LCD_COLS, "%-9s: ON ", disp_name);
      }
      else {
        snprintf(str, LCD_COLS, "%-9s: OFF", disp_name);
      }
      ld.display(disp_row, str);
      spub.pub_dbg(str);

      if (automatic) {
        snprintf(str, LCD_COLS, "%-9s: OFF", "Manual");
      }
      else {
        snprintf(str, LCD_COLS, "%-9s: ON ", "Manual");
      }
      ld.display(0, str);
      spub.pub_dbg(str);
    }

    void set(int relay_gpio, int curr_gpio, long dur, const char* state_topic,
      const char* cur_topic, const char* sub_topic, const char* dbg_top,
      const char* man_topic, const char* man_state, bool manual_mode) {
      spub.set(state_topic, PUB_INTERVAL, dbg_top);
      spub.add_subscribe(this, sub_topic);
      cpub.set(cur_topic, PUB_INTERVAL, 1, dbg_top);
      mpub.set(man_state, PUB_INTERVAL, dbg_top);
      mpub.add_subscribe(this, man_topic);

      relay = relay_gpio;
      curr = curr_gpio;
      max_duration = dur * 60L * 1000L; //Convert to ms
      min_interval = 5 * 60L * 1000L; // 5 min break before switch on
      on = false;
      manual = false;
      last_on = 0;
      last_off = 0;
      switch_topic = sub_topic;
      mode_topic = man_topic;
      pinMode(relay, OUTPUT);
      digitalWrite(relay, HIGH);
      spub.updateState(on);
      automatic = !manual_mode;
      mpub.updateState(manual_mode);
      lcdisplay();
    }

    bool is_auto() {
      return manual;
    }

    bool is_manual() {
      return manual;
    }

    bool is_on() {
      return on;
    }

    unsigned long on_duration() {
      spub.updateState(on);
      if (on) {
        return (millis() - last_on);
      }
      return 0;
    }

    long current() {
#ifdef CURRENT_EN
      long acc = 0;
      for (int i=0; i<tries; i++) {
        long val = analogRead(M2_CUR);
        debug.debug("Anolg read: %ld", val);
        acc += val;
        //delayMicroseconds(10);
      }
      long val = acc/tries;
      debug.debug("Analog read avg: %ld", val);
      long volt = val * INPUT_MV / MAX_RANGE;
      long amps = abs((volt - ACSOFFSET) / MVPERAMP);
      debug.debug("Current drawn: %d", amps);
      cpub.updateValue(amps);
      return amps;
#else
      return -1;
#endif
    }

    void switch_off() {
      if (on) {
        last_off = millis();
        on = false;
        manual = false;
        debug.info("Motor off");
        lcdisplay();
      }
      debug.info("Switching off motor");
      digitalWrite(relay, MOTOR_OFF);
      spub.updateState(on);
      // current();
    }

    bool check() {
      spub.updateState(on);
      if (on) {
#ifdef CURRENT_EN
        long cur = current();
        if (cur < 1) {
        debug.info("No current being drawn");
          // Motor is not on. Could be power outage
        switch_off();
        }
        else {
#endif
        unsigned long dur = millis() - last_on;
        if (dur > max_duration) {
          debug.info("Motor on longer than max duration (%lu, %lu)", dur, max_duration);
          spub.pub_dbg("Motor on longer than max duration");
          switch_off();
        }
      }
      return on;
    }

    void switch_on(bool force=false) {
      if (not automatic and not force) {
        // IF automatic mode is OFF
        return;
      }

      if (on) {
        check();
      }
      else if (not force) {
        if ((last_on == 0) || ((millis() - last_off) >= min_interval)) {
          debug.info("Switching on the motor");
          last_on = millis();
          on = true;
        }
        else {
          unsigned long dur = millis() - last_off;
          debug.debug("Not switching on motor: %ld (%ld)", dur, min_interval);
          return;
        }
      }

      // If manual switch on
      if (force) {
        manual = true;
        debug.debug("Switching on the motor maunally");
        spub.pub_dbg("Switching on the motor maunally");
        last_on = millis();
        on = true;
      }

      if (on) {
        debug.debug("Sending relay ON for motor");
        digitalWrite(relay, MOTOR_ON);
        spub.updateState(on);
        lcdisplay();
      }
    }

    void callback(String& topic, String& payload) {
      static String on_str = String("ON");
      static String off_str = String("OFF");

      debug.debug(("Motor sub " + topic + ": " + payload).c_str());
      if (topic.equals(switch_topic)) {
        if (payload.equals(on_str)) {
          debug.info("Switch on motor manually");
          switch_on(true);
        }
        else if (payload.equals(off_str)) {
          debug.info("Switch off motor manually");
          switch_off();
        }
        else {
          debug.error("Unknown payload %s: %s", topic.c_str(), payload.c_str());
        }
      }
      else if (topic.equals(mode_topic)) {
        if (payload.equals(on_str)) {
          debug.info("Manual motor control");
          automatic = false;
          mpub.updateState(!automatic);
          update_eeprom(M2_MANUAL_MODE_ADDR, 0x01);
          lcdisplay();
          switch_off();
        }
        else if (payload.equals(off_str)) {
          debug.info("Automatic motor control");
          automatic = true;
          mpub.updateState(!automatic);
          update_eeprom(M2_MANUAL_MODE_ADDR, 0x00);
          lcdisplay();
          switch_off();
        }
        else {
          debug.error("Unknown payload %s: %s", topic.c_str(), payload.c_str());
        }
      }
    }
};

class WaterLevelController {
  unsigned long last_check = 0;
  unsigned long interval = CHECK_INT * 1000L;
  unsigned long flow_dur = FLOW_DUR * 60L * 1000L;
  long min_flow = MIN_FLOW_EXP;
  long last_level = 0;
  unsigned long last_flow_chk = 0;
  OverheadTank oht = OverheadTank();
  StorageTank st = StorageTank();
  Motor motor = Motor();

  public:

    void set(bool manual_mode) {
      oht.set(OHT_TRIG, OHT_ECHO, OHT_HT, OHT_OFF, OHT_HIGH, OHT_LOW, OHT_TOPIC, OHT_DBG);
      st.set(ST_TRIG, ST_ECHO, ST_HT, ST_OFF, ST_HIGH, ST_LOW, ST_TOPIC, ST_DBG);
      motor.set(M2_RELAY, M2_CUR, M2_MAX_DUR, M2_STATE_TOPIC, M2_CUR_TOPIC,
        M2_MANUAL, M2_DBG, M2_MANUAL_MODE, M2_MANUAL_STATE, manual_mode);
    }

    void loop() {
      if ((millis() - last_check) >= interval) {
        last_check = millis();
        int stl = st.check();
        int ohtl = oht.check();
        if ((stl == -2) || (ohtl == -2)) {
          // One of the tank sensors is not working
          motor.switch_off();
          last_level = 0;
          return;
        }
        if (stl >= 0) {
          // We have enough water in storage tank
          unsigned long mot_on = motor.on_duration();
          debug.debug("Motor on for %lu (%lu, %ld)", mot_on, flow_dur, last_level);

          if ((motor.is_manual() == false) && (mot_on > flow_dur)) {
            // Check if water is getting pumped
            if ((millis() - last_flow_chk) > flow_dur) {
              last_flow_chk = millis();
              long level = oht.get_last();
              debug.debug("Check flow rate : %ld (%ld, %ld)", level, min_flow, last_level);
              if ((level - last_level) < min_flow) {
                // Motor not working
                debug.info("Motor not pumping. Switching off");
                motor.switch_off();
                last_level = 0;
              }
              else {
                last_level = level;
              }
            }
          }
          if (ohtl < 0) {
            // Water in overhead tank less than lower threshold
            debug.debug("Switch on motor");
            motor.switch_on();
            if (last_level == 0) {
              last_level = oht.get_last();
            }
            // Check if motor is still on
            if (not motor.is_on()) {
              last_level = 0;
            }
          }
          else if (ohtl > 0) {
            // Over the high threshold
            debug.debug("Switch off motor");
            motor.switch_off();
            last_level = 0;
          }
          else if (not motor.is_on()) {
            last_level = 0;
          }
        }
        else {
          // Water in storage tank below threshold
          motor.switch_off();
          last_level = 0;
          debug.error("Water level in storage tank below threshold!");
        }
      }
    }
};


#ifdef NW_SUPPORT
#ifdef WIFI_SUPPORT
#else
long last_eth_check = 0;
long eth_check_interval = 60L * 1000; // 1 min
void eth_maintain() {
  if ((millis() - last_eth_check) >= eth_check_interval) {
    last_eth_check = millis();
    switch (Ethernet.maintain())
    {
      case 1:
        //renewed fail
        debug.error("Error: renewed fail");
        break;

      case 2:
        //renewed success
        debug.debug("Renewed success");

        //print your local IP address:
        if (INFO) {
          printIPAddress();
        }
        break;

      case 3:
        //rebind fail
        debug.error("Error: rebind fail");
        break;

      case 4:
        //rebind success
        debug.debug("Rebind success");

        //print your local IP address:
        if (INFO) {
          printIPAddress();
        }
        break;

      default:
        //nothing happened
        break;

    }
  }
}
#endif
#endif

WaterLevelController wlc;

char message_buff[100];

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  String top = String(topic);
  debug.info("Message arrived:  topic: %s", topic);
  debug.debug("Length: %u", length);

  int i = 0;
  // create character buffer with ending null terminator (string)
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';

  String msgString = String(message_buff);

  debug.debug("Payload: %s", msgString.c_str());

  Subscribe* instance = Publish::get_instance(top);
  if (instance == NULL) {
    debug.error("Did not find instance for topic: %s", topic);
    return;
  }

  instance->callback(top, msgString);
}

#ifdef WIFI_SUPPORT
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  if (eeprom_read_string(WIFI_DETAILS_ADDR, ssid, 16)) {
    if (eeprom_read_string(WIFI_DETAILS_ADDR + strlen(ssid) + 1, password, 32)) {
      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(ssid);
      // Serial.println(password);
      Serial.println();
      Serial.print("MAC: ");
      Serial.println(WiFi.macAddress());

      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);

      while (WiFi.status() != WL_CONNECTED) {
        // WiFi.printDiag(Serial);
        delay(500);
        Serial.print(".");
      }

      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
    else {
      while (true) {
        Serial.println();
        Serial.print("ERROR reading WIFI password!");
        delay(1000);
      }
    }
  }
  else {
    while (true) {
      Serial.println();
      Serial.print("ERROR reading WIFI SSID!");
      delay(1000);
    }
  }
}
#endif

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

#ifdef LCD_DISPLAY
  lcd.begin(LCD_COLS, LCD_ROWS);
#endif

#ifdef LCD_I2C
  Wire.begin(LCD_I2C_SDA, LCD_I2C_SCL);
  // initialize the LCD
  lcd.init(LCD_I2C_SDA, LCD_I2C_SCL);

  lcd.backlight();
#endif

#ifdef NW_SUPPORT
#ifdef WIFI_SUPPORT
  // Read the ssid and password

  setup_wifi();
  client = PubSubClient(server, 1883, callback, espClient);
#else
  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Do static config
    Ethernet.begin(mac, ip);
  }

  // print your local IP address:
  printIPAddress();

  client = PubSubClient(server, 1883, callback, ethClient);
#endif

  //client.setServer(server, 1883);
  client.setCallback(callback);
#endif

  // WaterLevelController init
  byte ch = read_eeprom(M2_MANUAL_MODE_ADDR);
  wlc.set((ch == 0x01));
}



void loop() {

  wlc.loop();

  client.loop();

#ifdef NW_SUPPORT
#ifdef WIFI_SUPPORT
#else
  eth_maintain();
#endif
#endif

}

#ifdef NW_SUPPORT
#ifdef WIFI_SUPPORT
#else
void printIPAddress()
{
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }

  Serial.println();
}
#endif
#endif
