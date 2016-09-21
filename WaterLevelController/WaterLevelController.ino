/*
  DHCP-based IP printer

  This sketch uses the DHCP extensions to the Ethernet library
  to get an IP address via DHCP and print the address obtained.
  using an Arduino Wiznet Ethernet shield.

  Circuit:
   Ethernet shield attached to pins 10, 11, 12, 13

  created 12 April 2011
  modified 9 Apr 2012
  by Tom Igoe
  modified 02 Sept 2015
  by Arturo Guadalupi

*/

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>


// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {
  0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x41
};

// Fallback IP
IPAddress ip(10, 168, 1, 12);

// MQTT server
IPAddress server(10, 168, 1, 10);

// Sump sensor
int s2_trig = 22;
int s2_echo = 23;

// Overhead tank sensor
int oh_trig = 24;
int oh_echo = 25;

// Relay control
int relay = 26;

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

long lastReconnectAttempt = 0;

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
EthernetClient ethClient;
PubSubClient client;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  pinMode(oh_trig, OUTPUT);
  pinMode(oh_echo, INPUT); 

  pinMode(s2_trig, OUTPUT);
  pinMode(s2_echo, INPUT); 

  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);

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

}

boolean reconnect() {
  if (!client.connected()) {
    if (client.connect("arduinoClient")) {
      Serial.print("MQTT connected\n");
      
      // ... and resubscribe
      client.subscribe("jr-mqtt/ohtmotor-in");
    }
    else {
      Serial.print("MQTT reconnect failed!\n");
    }
    
    // client.loop();
  }
  
  return client.connected();  
}

long oh_prev = 0;
void publish_overhead(long level) {

  if ((level - oh_prev) > 3) {
    oh_prev = level;
    if (reconnect()) { 
      String l = String(level);
      client.publish("jr-mqtt/oh-tank/level", l.c_str());
    }
  }
}

long s2_prev = 0;
void publish_sump(long level) {

  if ((level - s2_prev) > 3) {
    s2_prev = level;
    if (reconnect()) { 
      String l = String(level);
      client.publish("jr-mqtt/sump-2/level", l.c_str());
    }
  }
}

long get_level(int trig, int echo) {
  
  long t = 0, h = 0;
  
  // Transmitting pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  // Waiting for pulse
  t = pulseIn(echo, HIGH);
  
  // Calculating distance 
  h = t / 58;

 return h;
}


void loop() {

  long s2 = get_level(s2_trig, s2_echo);
  // Sending to computer
  Serial.print("Sump level:");
  Serial.print(s2);
  Serial.print("\n");
  publish_sump(s2);

  long oh = get_level(oh_trig, oh_echo);
  // Sending to computer
  Serial.print("Overhead tank level:");
  Serial.print(oh);
  Serial.print("\n");
  publish_overhead(oh);

  client.loop();
  
  switch (Ethernet.maintain())
  {
    case 1:
      //renewed fail
      Serial.println("Error: renewed fail");
      break;

    case 2:
      //renewed success
      Serial.println("Renewed success");

      //print your local IP address:
      printIPAddress();
      break;

    case 3:
      //rebind fail
      Serial.println("Error: rebind fail");
      break;

    case 4:
      //rebind success
      Serial.println("Rebind success");

      //print your local IP address:
      printIPAddress();
      break;

    default:
      //nothing happened
      break;

  }

  delay(10000);
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
