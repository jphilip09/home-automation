/*
  Water Level Controller

  Circuit:
   Ethernet shield attached to pins 10, 11, 12, 13

  by Philip Joseph (jphilip09@gmail.com)
  
*/

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <LiquidCrystal.h>


// Enable or disable debug prints on serial
#define DEBUG true
#define INFO true
#define PRINTF_BUF 80 // define the tmp buffer size

// Topic
#define TOPIC "jr-mqtt/wlc"

#define topic_cat(sub) TOPIC#sub

// Overhead Tank specifics
// Topic suffix
#define OHT_TOPIC topic_cat(/oht/level)
// Max water level in cms
#define OHT_HT 140
// Offset of sensor
#define OHT_OFF 26
// High thershold in %
#define OHT_HIGH 90
// Low threshold
#define OHT_LOW 35
// sensor GPIO pins
#define OHT_TRIG 24
#define OHT_ECHO 25

// Storage Tank 2 specifics
// Topic suffix
#define S2_TOPIC topic_cat(/s2/level)
// Max water level in cms
#define S2_HT 180
// Offset of sensor
#define S2_OFF 30
// High thershold in %
#define S2_HIGH 100
// Low threshold
#define S2_LOW 17
// sensor GPIO pins
#define S2_TRIG 22
#define S2_ECHO 23

// Motor 2 specifics
// ON/OFF Topic suffix
#define M2_STATE_TOPIC topic_cat(/motor2/state)
// Current drawn topic
#define M2_CUR_TOPIC topic_cat(/motor2/current)
// Manual override topic
#define M2_MANUAL topic_cat(/motor2/manual)
// Maximum time to run at a time in mins
#define M2_MAX_DUR 60
// Relay GPIO pin
#define M2_RELAY 26
// Cuurent sensor Analog pin
#define M2_CUR A0
// Flow check
#define FLOW_DUR 5 // Check flow every 5 mins
#define MIN_FLOW_EXP 5 // Minimum change in level
// Current sensor details for ACS712 - 20A
#define MVPERAMP 100
#define ACSOFFSET 2500
#define MAX_RANGE 1024
#define INPUT_MV 5000

// LCD specifics
#define LCD_COLS 20
#define LCD_ROWS 4
#define LCD_RS_GPIO 28
#define LCD_EN_GPIO 29
#define LCD_D4_GPIO 30
#define LCD_D5_GPIO 31
#define LCD_D6_GPIO 32
#define LCD_D7_GPIO 33
#define MOTOR_ROW 1
#define OHT_ROW 2
#define S2_ROW 3

// Check the levels interval
#define CHECK_INT 15

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS_GPIO, LCD_EN_GPIO, LCD_D4_GPIO, LCD_D5_GPIO, LCD_D6_GPIO, LCD_D7_GPIO);

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {
  0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x41
};

// Fallback IP
IPAddress ip(10, 168, 1, 12);

// MQTT server
IPAddress server(10, 168, 1, 10);

// Maximum topics to subscribe
#define MAX_SUBS 10

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
EthernetClient ethClient;
PubSubClient client;

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
  int rows = LCD_ROWS - 1;
  int cols = LCD_COLS;
  int err_row = 0; // First line on display
  bool err_disp = false;

  public:
    void display(int row, const char* str) {
      int r = row;
      if (row > rows) {
        r = rows;
      }
      lcd.setCursor(0, r);
      lcd.print(str);
    }

    void error(const char* str) {
      err_disp = true;
      lcd.setCursor(0, err_row);
      lcd.print(str);
    }

    void clear_err() {
      err_disp = false;
      lcd.setCursor(0, err_row);
      lcd.print(' ');
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
        return h;
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
        delay(100);
      }
      if (count == 0) {
        return -1;
      }
      long avg = acc/count;
      debug.debug("Height avg calculated: %d", avg);
      return avg;
    }
};

class Publish {
  int lastUpdate;
  long lastValue;
  bool lastState;
  char *my_topic;
  int interval;
  int diff;
  
  static int sub_count;
  static char* sub_topic[MAX_SUBS];

  // buffer to convert value to string
  static const int buf_len = 16;
  static char buf[buf_len];

  static boolean reconnect() {
    if (!client.connected()) {
      if (client.connect("jrMqttClient")) {
        debug.info("MQTT connected");
        if (sub_count) {
          for (int i=0; i<sub_count; i++) {
            client.subscribe(sub_topic[i]);
            debug.info("Subscribe for %s", sub_topic[i]);
          }
        }
      }
      else {
        debug.error("MQTT connect failed!");
      }
    }
  
    return client.connected();  
  }
  
  public:
    void set(const char *topic, int max_interval, int max_diff)
    {
      my_topic = topic;
      interval = max_interval * 60 * 1000; //Convert to millisecs
      diff = max_diff;
      lastUpdate = 0;
      lastValue = -1;
      lastState = false;
      reconnect();
    }

    void set(const char* topic, int max_interval) {
      set(topic, max_interval, 0);
    }

    bool add_subscribe(String topic) {
      if (sub_count >= MAX_SUBS) {
        // Already max topics subscribed
        return false;
      }
      sub_topic[sub_count] = topic.c_str();
      sub_count++;
      reconnect();
      return true;
    }

    void updateValue(long value) {
      if ((abs(lastValue - value) >= diff) || ((millis() - lastUpdate) >= interval)) {
        if (reconnect()) {
          lastValue = value;
          snprintf(buf, buf_len-1, "%ld", value);
          lastUpdate = millis();
          client.publish(my_topic, buf);
          debug.info("Published topic %s with value: %s (%ld)", my_topic, buf, value);
        }
      }
    }

    void updateState(bool state) {
      if ((state != lastState) || (lastUpdate == 0) || ((millis() - lastUpdate) >= interval)) {
        if (reconnect()) {
          lastState = state;
          if (state) {
            strncpy(buf, "ON", buf_len-1);
          }
          else {
            strncpy(buf, "OFF", buf_len-1);
          }
          lastUpdate = millis();
          client.publish(my_topic, buf);
          debug.debug("Published topic %s with state: %s", my_topic, buf);
        }
      }
    }
};

// Define the static members
static int Publish::sub_count = 0;
static char* Publish::sub_topic[MAX_SUBS];
static char Publish::buf[Publish::buf_len] = "";

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

    void set(int trig, int ec, int ht, int off, int upper, int lower, const char* topic) {
      ls.set(trig, ec, off, ht+off);
      pub.set(topic, 60, 3);

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
      while (l == -1) {
        delay(500);
        l = ls.check();
      }
      long h = max_ht - (l - offset);
      last_h = h;
      int p = h * 100 / max_ht;
      char str[LCD_COLS+1];
      snprintf(str, LCD_COLS, "%-9s:%3ld (%3d)", disp_name, h, p);
      ld.display(disp_row, str);
      debug.debug("%s", str);

      if (h <= low_th) {
        ret = -1;
      }
      else if (h >= high_th) {
        ret = 1;
      }
      debug.debug("%s Tank check: %d", disp_name, ret);
      pub.updateValue(p);
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
      set_display(S2_ROW, "Sump");
    }
};

class Motor {
  int relay;
  int curr;
  long max_duration;
  long min_interval;
  const long tries = 5;
  bool on;
  long last_on;
  long last_off;
  Publish spub;
  Publish cpub;
  const int disp_row = MOTOR_ROW;
  const char *disp_name;

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
    }
    
    void set(int relay_gpio, int curr_gpio, int dur, const char* state_topic, const char* cur_topic) {
      spub.set(state_topic, 60);
      cpub.set(cur_topic, 60, 1);
      relay = relay_gpio;
      curr = curr_gpio;
      max_duration = dur * 60 * 1000; //Convert to ms
      min_interval = 5 * 60 * 1000; // 5 min break before switch on
      on = false;
      last_on = 0;
      last_off = 0;
      pinMode(relay, OUTPUT);
      digitalWrite(relay, HIGH); 
      lcdisplay();
      spub.updateState(on);
    }

    bool is_on() {
      return on;
    }

    long on_duration() {
      if (on) {
        return (millis() - last_on);
      }
      return -1;
    }
    
    long current() {
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
    }
    
    void switch_off() {
      if (on) {
        last_off = millis();
        on = false;
        debug.info("Motor off");
        lcdisplay();
      }
      debug.info("Switching off motor");
      digitalWrite(relay, HIGH);
      spub.updateState(on);
      // current();
    }

    bool check() {
      if (on) {
        // long cur = current();
        // if (cur < 1) {
        //  debug.info("No current being drawn");
          // Motor is not on. Could be power outage
        //   switch_off();
        // } else
        long dur = millis() - last_on;
        if (dur > max_duration) {
          debug.info("Motor on longer than max duration (%ld, %ld)", dur, max_duration);
          switch_off();
        }
      }
      return on;
    }
    
    void switch_on() {
      if (on) {
        check();
      } 
      else {
        if ((last_on == 0) || ((millis() - last_off) >= min_interval)) {
          debug.info("Switching on the motor");
          last_on = millis();
          on = true;
        }
        else {
          return;
        }
      }
      if (on) {
        debug.debug("Switching on motor");
        digitalWrite(relay, LOW);
        spub.updateState(on);
        lcdisplay();
      }
    }
};

class WaterLevelController {
  long last_check = 0;
  long interval = CHECK_INT * 1000;
  long flow_dur = FLOW_DUR * 60 * 1000;
  long min_flow = MIN_FLOW_EXP;
  long last_level = 0;
  long last_flow_chk = 0;
  OverheadTank oht = OverheadTank();
  StorageTank s2 = StorageTank();
  Motor motor = Motor();
  
  public:

    void set() {
      oht.set(OHT_TRIG, OHT_ECHO, OHT_HT, OHT_OFF, OHT_HIGH, OHT_LOW, OHT_TOPIC);
      s2.set(S2_TRIG, S2_ECHO, S2_HT, S2_OFF, S2_HIGH, S2_LOW, S2_TOPIC);
      motor.set(M2_RELAY, M2_CUR, M2_MAX_DUR, M2_STATE_TOPIC, M2_CUR_TOPIC);
    }
  
    void loop() {
      if ((millis() - last_check) >= interval) {
        last_check = millis();
        if (s2.check() >= 0) {
          // We have enough water in storage tank 2
          int oht_check = oht.check();
          if (motor.on_duration() > flow_dur) {
            if ((millis() - last_flow_chk) > flow_dur) {
              last_flow_chk = millis();
              long level = oht.get_last();
              if ((level - last_level) < min_flow) {
                // Motor not working
                debug.info("Motor not pumping. Switching off");
                motor.switch_off(); 
              }
              last_level = level;
            }
          }
          else if ((last_level == 0) && motor.is_on()) {
            // Update the first time motor is on
            last_level = oht.get_last();
          }
          if (oht_check < 0) {
            debug.debug("Switch on motor");
            // Water in overhead tank less than lower threshold
            motor.switch_on();
          }
          else if (oht_check > 0) {
            // Over the high threshold
            debug.debug("Switch off motor");
            motor.switch_off();
            last_level = 0;
          }
        }
        else {
          // Water in storage tank 2 below threshold
          motor.switch_off();
          last_level = 0;
          debug.error("Water level in storage tank below threshold!");
        }
      }
    }
};

long last_eth_check = 0;
long eth_check_interval = 60 * 1000; // 1 min
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

WaterLevelController wlc;

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  lcd.begin(LCD_COLS, LCD_ROWS);

  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Do static config
    Ethernet.begin(mac, ip);
  }
  
  // print your local IP address:
  printIPAddress();

  client = PubSubClient(server, 1883, ethClient);
  
  //client.setServer(server, 1883);
  client.setCallback(callback);

  // WaterLevelController init
  wlc.set();
}



void loop() {

  wlc.loop();
  
  client.loop();

  eth_maintain();
  
}

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
