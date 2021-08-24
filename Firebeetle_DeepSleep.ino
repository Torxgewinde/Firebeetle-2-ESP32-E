/*******************************************************************************
#                                                                              #
# Using the Firebeetle 2 ESP32-E as battery powered sensor node                #
#                                                                              #
#                                                                              #
# Copyright (C) 2021 Tom Stöveken                                              #
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

#include "esp_adc_cal.h"

#define ESSID "YOUR WIFI SSID"
#define PSK   "YOUR WIFI PASSWD"

#define LOW_BATTERY_VOLTAGE 3.20
#define VERY_LOW_BATTERY_VOLTAGE 3.10
#define CRITICALLY_LOW_BATTERY_VOLTAGE 3.00

String MQTTServerName = "YOUR MQTT SERVER";
uint16_t MQTTPort = 1883;
String MQTTUsername = "MQTT USERNAME";
String MQTTPassword = "MQTT PASSWORD";
String MQTTDeviceName = "Firebeetle";
String MQTTRootTopic = "test/firebeetle";

RTC_NOINIT_ATTR struct {
  uint8_t bssid[6];
  uint8_t channel;

  uint64_t NumberOfRestarts;
} cache;

/******************************************************************************
Description.: bring the WiFi up
Input Value.: When "tryCachedValuesFirst" is true the function tries to use
              cached values before attempting a scan + association
Return Value: true if WiFi is up, false if it timed out
******************************************************************************/
bool WiFiUP(bool tryCachedValuesFirst) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  
  if(tryCachedValuesFirst && cache.channel > 0) {
    Serial.printf("Cached values as follows:\n");
    Serial.printf(" Channel....: %d\n", cache.channel);
    Serial.printf(" BSSID......: %x:%x:%x:%x:%x:%x\n", cache.bssid[0], \
                                                       cache.bssid[1], \
                                                       cache.bssid[2], \
                                                       cache.bssid[3], \
                                                       cache.bssid[4], \
                                                       cache.bssid[5]);

    WiFi.begin(ESSID, PSK, cache.channel, cache.bssid);

    for (unsigned long i=millis(); millis()-i < 10000;) {
      delay(10);

      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("WiFi connected with cached values (%lu)\n", millis()-i);
        return true;
      } 
    }
  }

  cache.channel = 0;
  for (uint32_t i = 0; i < sizeof(cache.bssid); i++)
    cache.bssid[i] = 0;

  // try it with the slow process
  WiFi.begin(ESSID, PSK);
  
  for (unsigned long i=millis(); millis()-i < 60000;) {
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
  Serial.printf("FireBeetle starting up\r\n" \
                " Compiled at: " __DATE__ " - " __TIME__ "\r\n" \
                " ESP-IDF: %s\r\n", esp_get_idf_version());

  //read battery voltage
  float BatteryVoltage = readBattery();
  Serial.printf("Voltage: %4.3f V\r\n", BatteryVoltage);

  //a reset is required to wakeup again from below CRITICALLY_LOW_BATTERY_VOLTAGE
  //this is to prevent damaging the empty battery by saving as much power as possible
  if (BatteryVoltage < CRITICALLY_LOW_BATTERY_VOLTAGE) {
    Serial.println("Battery critically low, hibernating...");

    //switch off everything that might consume power
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_CPU, ESP_PD_OPTION_OFF);

    //disable all wakeup sources
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    
    digitalWrite(2, LOW);
    esp_deep_sleep_start();

    Serial.println("This should never get printed");
    return;
  }

  //if battery is below LOW_BATTERY_VOLTAGE but still above CRITICALLY_LOW_BATTERY_VOLTAGE, 
  //stop doing the regular work
  //when put on charge the device will wakeup after a while and recognize voltage is OK
  //this way the battery can run low, put still wakeup without physical interaction
  if (BatteryVoltage < LOW_BATTERY_VOLTAGE) {
    Serial.println("Battery low, deep sleeping...");

    //sleep ~60 minutes if battery is CRITICALLY_LOW_BATTERY_VOLTAGE to VERY_LOW_BATTERY_VOLTAGE
    //sleep ~10 minutes if battery is VERY_LOW_BATTERY_VOLTAGE to LOW_BATTERY_VOLTAGE
    uint64_t sleeptime = (BatteryVoltage >= VERY_LOW_BATTERY_VOLTAGE) ? \
                           10*60*1000000ULL : 60*60*1000000ULL;
    
    esp_sleep_enable_timer_wakeup(sleeptime);
    digitalWrite(2, LOW);
    esp_deep_sleep_start();
    
    Serial.println("This should never get printed");
    return;
  }

  //distinguish first start or wakeup
  if (esp_reset_reason() == ESP_RST_POWERON) {
    Serial.printf("ESP was just switched ON\r\n");
    cache.NumberOfRestarts = 0;

    //default is to have WiFi off
    if (WiFi.getMode() != WIFI_OFF) {
      Serial.printf("WiFi wasn't off!\r\n");
      WiFi.persistent(true);
      WiFi.mode(WIFI_OFF);
    }

    //set RTC to 0
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);

    WiFiUP(false);
  } else {
    Serial.printf("ESP woke up (%lu)\n", cache.NumberOfRestarts);

    //read RTC
    struct timeval tv;
    gettimeofday(&tv, NULL);
    Serial.printf("Total Time since RstPowerOn: %lu s\r\n", tv.tv_sec);

    WiFiUP(true);
  }

  //establish connection to MQTT server
  WiFiClient net;
  MQTTClient MQTTClient;
  MQTTClient.setTimeout(5000);
  MQTTClient.begin(MQTTServerName.c_str(), MQTTPort, net);
  if( MQTTClient.connect(MQTTDeviceName.c_str(), MQTTUsername.c_str(), MQTTPassword.c_str())) {
    Serial.println("Publishing MQTT message");
    MQTTClient.publish(MQTTRootTopic+"/BatteryVoltage", String(BatteryVoltage, 3), false, 2);
    MQTTClient.publish(MQTTRootTopic+"/PIR", (digitalRead(4))?"On":"Off", false, 2);
  }

  //bring everything down
  WiFi.disconnect(true, true);

  //define what wakes device up again
  //wakeup in ~12h or if GPIO4 changes state
  esp_sleep_enable_timer_wakeup(12*60*60*1000000ULL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, (digitalRead(4))?0:1);

  Serial.printf("=== Sleep at %d ms ===\r\n", millis());

  //LED off and sleep
  digitalWrite(2, LOW);
  esp_deep_sleep_start();
  
  Serial.println("This should never get printed");
}

void loop() {
  Serial.println("This should never get printed");
}
