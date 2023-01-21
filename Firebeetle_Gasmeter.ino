/*******************************************************************************
#                                                                              #
# Using the Firebeetle 2 ESP32-E as battery powered gasmeter-reader            #
# Project: https://github.com/Torxgewinde/Firebeetle-2-ESP32-E                 # 
#                                                                              #
# Firebeetle documentation at:                                                 #
# https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654                #                                                             #
#                                                                              # 
# Copyright (C) 2023 Tom Stöveken                                              #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; version 2 of the License.                      #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
********************************************************************************/

#include <WiFi.h>
#include <MQTT.h>
#include <stdlib.h> 

#include "esp_adc_cal.h"
#include "driver/rtc_io.h"

#define ESSID "My Wifi SSID"
#define PSK   "My Wifi Password"

#define LOW_BATTERY_VOLTAGE 3.20
#define VERY_LOW_BATTERY_VOLTAGE 3.10
#define CRITICALLY_LOW_BATTERY_VOLTAGE 3.00

//char *MQTTServer = "server.lan";
IPAddress MQTTServer = IPAddress(192,168,1,1);
uint16_t MQTTPort = 1883;
String MQTTUsername = "username";
String MQTTPassword = "password";
String MQTTDeviceName = "Gaszaehler";
String MQTTRootTopic = "Keller/Gaszaehler";

enum _state {
  S_STARTUP = 0,
  S_DEBOUNCE_LOW,
  S_DEBOUNCE_HIGH,
  S_LOW,
  S_HIGH
};

enum _message {
  M_COUNTER = 0,
  M_STATUS
};

RTC_NOINIT_ATTR struct {
  uint8_t bssid[6];
  uint8_t channel;

  float BatteryVoltage;      //battery voltage in V
  uint64_t NumberOfRestarts; //number of restarts
  uint64_t ActiveTime;       //time being active in ms

  enum _state state;         //keep track of current state

  uint64_t counter;          //gasmeter-counter
  uint64_t debounceTime;    //debounce time in ms
} cache;

//REED contact is connected to GPIO4 (Pin: D12)
#define REED_GPIO 4
#define REED_DEEPSLEEP_PIN GPIO_NUM_4

//#define DEBOUNCE_TIME 1*1000000ULL // 1000 msec
#define DEBOUNCE_TIME 200*1000ULL //200 msec

//periodically wakeup and report battery status
#define LONG_TIME 2*60*60*1000000ULL

//time to spend on establishing Wifi connection in ms
#define MAX_WIFI_CONNECTION_TIME 6000UL
//time to spend on establishing MQTT connection in ms
#define MAX_MQTT_CONNECTION_TIME 2000

/******************************************************************************
Description.: bring the WiFi up
Input Value.: When "tryCachedValuesFirst" is true the function tries to use
              cached values before attempting a scan + association
              "max_connection_time" in ms is the time to spend on establishing 
              the WiFi connection
Return Value: true if WiFi is up, false if it timed out
******************************************************************************/
bool WiFiUP(bool tryCachedValuesFirst, unsigned long max_connection_time) {
  unsigned long start = millis();

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  
  if(tryCachedValuesFirst && cache.channel > 0) {
    Serial.printf("Cached values as follows:\r\n");
    Serial.printf(" Channel....: %d\r\n", cache.channel);
    Serial.printf(" BSSID......: %x:%x:%x:%x:%x:%x\r\n", cache.bssid[0], \
                                                         cache.bssid[1], \
                                                         cache.bssid[2], \
                                                         cache.bssid[3], \
                                                         cache.bssid[4], \
                                                         cache.bssid[5]);

    WiFi.begin(ESSID, PSK, cache.channel, cache.bssid);

    for (unsigned long i=start; millis()-i < max_connection_time;) {
      delay(10);

      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("WiFi connected with cached values (%lu)\r\n", millis()-i);
        return true;
      } 
    }
  }

  cache.channel = 0;
  for (uint32_t i = 0; i < sizeof(cache.bssid); i++)
    cache.bssid[i] = 0;

  // try it with the slower process
  WiFi.begin(ESSID, PSK);
  
  for (unsigned long i=start; millis()-i < max_connection_time;) {
    delay(10);
  
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi connected (%lu)\r\n", millis()-i);
  
      uint8_t *bssid = WiFi.BSSID();
      for (uint32_t i = 0; i < sizeof(cache.bssid); i++)
        cache.bssid[i] = bssid[i];
      cache.channel = WiFi.channel();
    
      return true;
    }
  }

  Serial.printf("WiFi NOT connected\r\n");
  return false;
}

/******************************************************************************
Description.: reads the battery voltage through the voltage divider at GPIO34
              if the ESP32-E has calibration eFused those will be used.
              In comparison with a regular voltmeter the values of ESP32 and
              multimeter differ only about 0.05V
Input Value.: -
Return Value: battery voltage in volts
******************************************************************************/
float readBattery() {
  uint32_t value = 0;
  int rounds = 11;
  esp_adc_cal_characteristics_t adc_chars;

  //battery voltage divided by 2 can be measured at GPIO34, which equals ADC1_CHANNEL6
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  switch(esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars)) {
    case ESP_ADC_CAL_VAL_EFUSE_TP:
      Serial.println("Characterized using Two Point Value");
      break;
    case ESP_ADC_CAL_VAL_EFUSE_VREF:
      Serial.printf("Characterized using eFuse Vref (%d mV)\r\n", adc_chars.vref);
      break;
    default:
      Serial.printf("Characterized using Default Vref (%d mV)\r\n", 1100);
  }

  //to avoid noise, sample the pin several times and average the result
  for(int i=1; i<=rounds; i++) {
    value += adc1_get_raw(ADC1_CHANNEL_6);
  }
  value /= (uint32_t)rounds;

  //due to the voltage divider (1M+1M) values must be multiplied by 2
  //and convert mV to V
  return esp_adc_cal_raw_to_voltage(value, &adc_chars)*2.0/1000.0;
}

/******************************************************************************
Description.: send MQTT message
Input Value.: msg selects the message to send
Return Value: true if OK, false if errors occured
******************************************************************************/
bool SendMessage(enum _message msg, int timeout) {
  char buf[256] = {0};

  //read RTC
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  //establish connection to MQTT server
  WiFiClient net;
  MQTTClient MQTTClient(256);
  MQTTClient.begin(MQTTServer, MQTTPort, net);
  MQTTClient.setOptions(30, true, timeout);

  if( MQTTClient.connect(MQTTDeviceName.c_str(), MQTTUsername.c_str(), MQTTPassword.c_str())) {
    switch(msg) {
      case M_COUNTER:
        Serial.printf("Sending counter: %d\r\n", cache.counter);

        MQTTClient.publish(MQTTRootTopic+"/counter", String(cache.counter), true, 2);
        snprintf(buf, sizeof(buf)-1, "%.2f", cache.counter/100.0);
        MQTTClient.publish(MQTTRootTopic+"/qubicmeter", buf, false, 2);
        break;
      case M_STATUS:
        Serial.printf("Sending status\r\n");

        MQTTClient.publish(MQTTRootTopic+"/counter", String(cache.counter), true, 2);
        snprintf(buf, sizeof(buf)-1, "%.2f", cache.counter/100.0);
        MQTTClient.publish(MQTTRootTopic+"/qubicmeter", buf, false, 2);
        MQTTClient.publish(MQTTRootTopic+"/BatteryVoltage", String(cache.BatteryVoltage, 3), false, 2);
        snprintf(buf, sizeof(buf)-1, "%ld.%06ld", tv.tv_sec, tv.tv_usec);
        MQTTClient.publish(MQTTRootTopic+"/BatteryRuntime", buf, false, 2);
        snprintf(buf, sizeof(buf)-1, "%llu", cache.NumberOfRestarts);
        MQTTClient.publish(MQTTRootTopic+"/Restarts", buf, false, 2);
        snprintf(buf, sizeof(buf)-1, "%llu", cache.ActiveTime);
        MQTTClient.publish(MQTTRootTopic+"/ActiveTime", buf, false, 2);
        snprintf(buf, sizeof(buf)-1, "%ld", WiFi.RSSI());
        MQTTClient.publish(MQTTRootTopic+"/RSSI", buf, false, 2);
        break;
      default:
        Serial.printf("unkown message, not sending anything\r\n");
    }

    MQTTClient.disconnect();
    return true;
  }

  return false;
}

/******************************************************************************
Description.: get the config from MQTT, if it is retained we use it as start
Input Value.: timeout in ms
Return Value: true if MQTT connection was OK, false if errors occured
              also true even if no config was retrieved as this might be intended
******************************************************************************/
bool GetConfig(unsigned long timeout) {
  char buf[256] = {0};
  bool gotCounter = false;
  bool gotDebounceTime = false;

  //establish connection to MQTT server
  WiFiClient net;
  MQTTClient MQTTClient(256);
  MQTTClient.begin(MQTTServer, MQTTPort, net);
  MQTTClient.setOptions(30, true, 5000);

  //callback as lambda-function, capture gotCounter by reference to signal when done
  MQTTClient.onMessage((MQTTClientCallbackSimpleFunction)([&](String &topic, String &payload) -> void {
    Serial.println("Received MQTT message: " + topic + " - " + payload);

    if ( topic == MQTTRootTopic+"/config/counter") {
      cache.counter = strtoull(payload.c_str(), NULL, 10);
      gotCounter = true;
    }

    if ( topic == MQTTRootTopic+"/config/debounceTime") {
      cache.debounceTime = strtoull(payload.c_str(), NULL, 10);
      gotDebounceTime = true;
    }

    return;
  }));

  if( MQTTClient.connect(MQTTDeviceName.c_str(), MQTTUsername.c_str(), MQTTPassword.c_str()) ) {
    MQTTClient.subscribe(MQTTRootTopic+"/config/counter");
    MQTTClient.subscribe(MQTTRootTopic+"/config/debounceTime");

    for (unsigned long i=millis(); millis()-i < timeout;) {
      MQTTClient.loop();
      if(gotCounter && gotDebounceTime) {
        Serial.println("received counter and DebounceTime, will use it");
        break;
      }
      delay(10);
    }

    MQTTClient.disconnect();
    return true;
  }

  return false;
}

/******************************************************************************
Description.: since this is a battery sensor, everything happens in setup
              and when the tonguing' is done the device enters deep-sleep
Input Value.: -
Return Value: -
******************************************************************************/
void setup() {
  //visual feedback when we are active, turn on onboard LED
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  cache.NumberOfRestarts++;

  Serial.begin(115200);
  Serial.print("===================================================\r\n");
  Serial.printf("FireBeetle active\r\n" \
                " Compiled at: " __DATE__ " - " __TIME__ "\r\n" \
                " ESP-IDF: %s\r\n", esp_get_idf_version());

  //read battery voltage
  cache.BatteryVoltage = readBattery();
  Serial.printf("Voltage: %4.3f V\r\n", cache.BatteryVoltage);

  //a reset is required to wakeup again from below CRITICALLY_LOW_BATTERY_VOLTAGE
  //this is to prevent damaging the empty battery by saving as much power as possible
  if (cache.BatteryVoltage < CRITICALLY_LOW_BATTERY_VOLTAGE) {
    Serial.println("Battery critically low, hibernating...");

    //switch off everything that might consume power
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_CPU, ESP_PD_OPTION_OFF);

    //disable all wakeup sources
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    cache.ActiveTime += millis();
    digitalWrite(2, LOW);
    esp_deep_sleep_start();

    Serial.println("This should never get printed");
    return;
  }

  //if battery is below LOW_BATTERY_VOLTAGE but still above CRITICALLY_LOW_BATTERY_VOLTAGE, 
  //stop doing the regular work
  //when put on charge the device will wakeup after a while and recognize voltage is OK
  //this way the battery can run low, put still wakeup without physical interaction
  if (cache.BatteryVoltage < LOW_BATTERY_VOLTAGE) {
    Serial.println("Battery low, deep sleeping...");

    //sleep ~60 minutes if battery is CRITICALLY_LOW_BATTERY_VOLTAGE to VERY_LOW_BATTERY_VOLTAGE
    //sleep ~10 minutes if battery is VERY_LOW_BATTERY_VOLTAGE to LOW_BATTERY_VOLTAGE
    uint64_t sleeptime = (cache.BatteryVoltage >= VERY_LOW_BATTERY_VOLTAGE) ? \
                           10*60*1000000ULL : 60*60*1000000ULL;
    
    esp_sleep_enable_timer_wakeup(sleeptime);
    cache.ActiveTime += millis();
    digitalWrite(2, LOW);
    esp_deep_sleep_start();
    
    Serial.println("This should never get printed");
    return;
  }

  //configure GPIO of reed-switch
  pinMode(REED_GPIO, INPUT_PULLUP);
  rtc_gpio_pullup_en(REED_DEEPSLEEP_PIN);
  rtc_gpio_pulldown_dis(REED_DEEPSLEEP_PIN);

  //read level of reed-switch
  int level = digitalRead(REED_GPIO);
  Serial.printf("Reed-switch: %s\r\n", (level)?"NOMAGNET (=HIGH)":"MAGNET (=LOW)");

  //check if a reset/power-on occured
  if (esp_reset_reason() == ESP_RST_POWERON || esp_reset_reason() == ESP_RST_SW) {
      Serial.printf("ESP was just switched ON or software reset\r\n");

      cache.state = S_STARTUP;
      cache.ActiveTime = 0;
      cache.NumberOfRestarts = 0;
      cache.counter = 0;
      cache.debounceTime = DEBOUNCE_TIME;

      //set RTC to 0
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 0;
      settimeofday(&tv, NULL);

      //default is to have WiFi off
      if (WiFi.getMode() != WIFI_OFF) {
        Serial.printf("WiFi wasn't off!\r\n");
        WiFi.persistent(true);
        WiFi.mode(WIFI_OFF);
      }

      //try to connect to WiFi for first time, use a very long timeout here
      if ( !WiFiUP(false, 60000) ) {
        //We only like to proceed if WiFi is working at least once, so just restart and try again...
        Serial.printf("WiFi does not connect, will try again right now...\r\n");
        ESP.restart();
      }
      //retrieve the previous gasmeter-counter, will return quickly if it is retained
      GetConfig(10*1000);
      WiFi.disconnect(true, true);

      //transition to new state
      if(level == HIGH) {
        cache.state = S_DEBOUNCE_HIGH;
        Serial.printf("RESET: S_STARTUP -> S_DEBOUNCE_HIGH\r\n");
        esp_sleep_enable_timer_wakeup(cache.debounceTime);
        esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);
      } else {
        cache.state = S_DEBOUNCE_LOW;
        Serial.printf("RESET: S_STARTUP -> S_DEBOUNCE_LOW\r\n");
        esp_sleep_enable_timer_wakeup(cache.debounceTime);
        esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);
      }
  }

  // check if ESP returns from deepsleep
  if (esp_reset_reason() == ESP_RST_DEEPSLEEP) {
    switch(esp_sleep_get_wakeup_cause()) {
      case ESP_SLEEP_WAKEUP_TIMER:
        Serial.printf("ESP woke up due to timer\r\n");

        //This state should not occur, no debounce state
        if(level == HIGH && cache.state == S_LOW) {
          cache.state = S_DEBOUNCE_HIGH;
          Serial.printf("TIMER: S_LOW -> S_DEBOUNCE_HIGH\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);
          break;
        }

        //can occur if LONG_TIME passed and level remains low
        if(level == LOW && cache.state == S_LOW) {
          cache.state = S_LOW;
          Serial.printf("TIMER: S_LOW -> S_LOW\r\n");
          esp_sleep_enable_timer_wakeup(LONG_TIME);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);

          if (WiFiUP(true, MAX_WIFI_CONNECTION_TIME)) {
            SendMessage(M_STATUS, MAX_MQTT_CONNECTION_TIME);
            WiFi.disconnect(true, true);
          }
          break;
        }

        //can occur if LONG_TIME passed and level remains high
        if(level == HIGH && cache.state == S_HIGH) {
          cache.state = S_HIGH;
          Serial.printf("TIMER: S_HIGH -> S_HIGH\r\n");
          esp_sleep_enable_timer_wakeup(LONG_TIME);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);

          if (WiFiUP(true, MAX_WIFI_CONNECTION_TIME)) {
            SendMessage(M_STATUS, MAX_MQTT_CONNECTION_TIME);
            WiFi.disconnect(true, true);
          }
          break;
        }

        //this state should not occur, no debounce state
        if(level == LOW && cache.state == S_HIGH) { 
          cache.state = S_DEBOUNCE_LOW;
          Serial.printf("TIMER: S_HIGH -> S_DEBOUNCE_LOW\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);
          break;
        }

        //level was HIGH for the whole DEBOUNCE_TIME and still is
        if(level == HIGH && cache.state == S_DEBOUNCE_HIGH) { 
          cache.state = S_HIGH;
          Serial.printf("TIMER: S_DEBOUNCE_HIGH -> S_HIGH\r\n");
          esp_sleep_enable_timer_wakeup(LONG_TIME);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);
          break;
        }

        //timer is up, but level changed without triggering EXT0, strange but deal with it
        if(level == LOW && cache.state == S_DEBOUNCE_HIGH) { 
          cache.state = S_DEBOUNCE_LOW;
          Serial.printf("TIMER: S_DEBOUNCE_HIGH -> S_DEBOUNCE_LOW\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);
          break;
        }

        //timer is up, but level changed without triggering EXT0, strange but deal with it
        if(level == HIGH && cache.state == S_DEBOUNCE_LOW) { 
          cache.state = S_DEBOUNCE_HIGH;
          Serial.printf("TIMER: S_DEBOUNCE_LOW -> S_DEBOUNCE_HIGH\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);
          break;
        }

        //level was LOW for the whole debounce time and still is
        if(level == LOW && cache.state == S_DEBOUNCE_LOW) {
          cache.state = S_LOW;
          Serial.printf("TIMER: S_DEBOUNCE_LOW -> S_LOW\r\n");
          esp_sleep_enable_timer_wakeup(LONG_TIME);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);

          cache.counter++;          

          if(cache.counter % 20 == 0) {
            if (WiFiUP(true, MAX_WIFI_CONNECTION_TIME)) {
              SendMessage(M_STATUS, MAX_MQTT_CONNECTION_TIME);
              WiFi.disconnect(true, true);
            }
          } else if (cache.counter % 1 == 0) {
            if (WiFiUP(true, MAX_WIFI_CONNECTION_TIME)) {
              SendMessage(M_COUNTER, MAX_MQTT_CONNECTION_TIME);
              WiFi.disconnect(true, true);
            }
          }
          break;
        }
        break;
    
      case ESP_SLEEP_WAKEUP_EXT0:
        Serial.printf("ESP woke up by EXT0\r\n");

        //level just changed from LOW to HIGH, start debounce time
        if(level == HIGH && cache.state == S_LOW) { 
          cache.state = S_DEBOUNCE_HIGH;
          Serial.printf("EXT0: S_LOW -> S_DEBOUNCE_HIGH\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);
          break;
        }

        //Level was LOW, still is, no reason to change state
        if(level == LOW && cache.state == S_LOW) { 
          cache.state = S_LOW;
          Serial.printf("EXT0: S_LOW -> S_LOW\r\n");
          esp_sleep_enable_timer_wakeup(LONG_TIME);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);
          break;
        }

        //level was HIGH, still is, no reason to change state
        if(level == HIGH && cache.state == S_HIGH) { 
          cache.state = S_HIGH;
          Serial.printf("EXT0: S_HIGH -> S_HIGH\r\n");
          esp_sleep_enable_timer_wakeup(LONG_TIME);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);
          break;
        }     

        //level was HIGH, just changed to LOW, start to debounce, trigger EXT0 if bouncing to HIGH
        if(level == LOW && cache.state == S_HIGH) { 
          cache.state = S_DEBOUNCE_LOW;
          Serial.printf("EXT0: S_HIGH -> S_DEBOUNCE_LOW\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);
          break;
        }

        //level was HIGH, still is, however something triggered EXT0, restart DEBOUNCE_TIME
        if(level == HIGH && cache.state == S_DEBOUNCE_HIGH) {
          cache.state = S_DEBOUNCE_HIGH;
          Serial.printf("EXT0: S_DEBOUNCE_HIGH -> S_DEBOUNCE_HIGH, but restart DEBOUNCE_TIME\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);
          break;
        }

        //level was HIGH, just changed to LOW, might be just bouncing
        if(level == LOW && cache.state == S_DEBOUNCE_HIGH) {
          cache.state = S_DEBOUNCE_LOW;
          Serial.printf("EXT0: S_DEBOUNCE_HIGH -> S_DEBOUNCE_LOW\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);
          break;
        }

        //level was LOW, now it is HIGH, might just be bouncing
        if(level == HIGH && cache.state == S_DEBOUNCE_LOW) { 
          cache.state = S_DEBOUNCE_HIGH;
          Serial.printf("EXT0: S_DEBOUNCE_LOW -> S_DEBOUNCE_HIGH\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, LOW);
          break;
        }

        //level was LOW, still is, however something triggered EXT0, restart DEBOUNCE_TIME
        if(level == LOW && cache.state == S_DEBOUNCE_LOW) { 
          cache.state = S_DEBOUNCE_LOW;
          Serial.printf("EXT0: S_DEBOUNCE_LOW -> S_DEBOUNCE_LOW, but restart DEBOUNCE_TIME\r\n");
          esp_sleep_enable_timer_wakeup(cache.debounceTime);
          esp_sleep_enable_ext0_wakeup(REED_DEEPSLEEP_PIN, HIGH);
          break;
        }
        break;

      default:
        Serial.printf("ESP woke up due to an unknown reason\r\n");
    }
  }

  Serial.printf("counter: %llu, debounceTime: %llu\r\n", cache.counter, cache.debounceTime);
  Serial.printf("=== entering deepsleep after %d ms ===\r\n", millis());
  cache.ActiveTime += millis();
  digitalWrite(2, LOW);
  esp_deep_sleep_start();
  
  Serial.println("This should never get printed");
}

void loop() {
  Serial.println("This should never get printed");
}
