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
#include <WiFiClientSecure.h>
#include <MQTT.h>

#include "esp_wifi.h"
#include "esp_pm.h"

#include "esp_adc_cal.h"

#define ESSID "YOUR WIFI SSID"
#define PSK   "YOUR WIFI PASSWD"

String MQTTServerName = "YOUR MQTT SERVER";
uint16_t MQTTPort = 8883;
String MQTTUsername = "MQTT USERNAME";
String MQTTPassword = "MQTT PASSWORD";
String MQTTDeviceName = "Firebeetle";
String MQTTRootTopic = "test/firebeetle";
String MQTTRootCA = "-----BEGIN CERTIFICATE-----\n" \
                    "YOUR ROOT CA CERTIFICATE HERE 6789012345678912345012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567890123456789012345678901234567890123456789012345678901234\n" \
                    "1234567=\n" \
                    "-----END CERTIFICATE-----\n";

//cached WiFi parameters
//DHCP does not take long, so static IPs are commented out
RTC_NOINIT_ATTR struct {
  uint8_t mac [6];
  uint8_t chl;
  //uint32_t ip;
  //uint32_t gw;
  //uint32_t msk;
  //uint32_t dns;
} cfgbuf;

RTC_NOINIT_ATTR uint64_t NumberOfRestarts;

/******************************************************************************
Description.: bring the WiFi up
Input Value.: When "tryCachedValuesFirst" is true the function tries to use
              cached values before attempting a scan + association
Return Value: true if WiFi is up, false if it timed out
******************************************************************************/
bool WiFiUP(bool tryCachedValuesFirst) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("Firebeetle");

  /*wifi_config_t current_conf;
  esp_wifi_get_config(WIFI_IF_STA, &current_conf);
  current_conf.sta.listen_interval = 1;
  esp_wifi_set_config(WIFI_IF_STA, &current_conf);
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);*/
  
  if(tryCachedValuesFirst && cfgbuf.chl > 0) {
    Serial.printf("Cached values as follows:\n");
    //Serial.printf(" Local IP...: %s\n", IPAddress(cfgbuf.ip).toString().c_str());
    //Serial.printf(" Subnet Mask: %s\n", IPAddress(cfgbuf.msk).toString().c_str());
    //Serial.printf(" DNS........: %s\n", IPAddress(cfgbuf.dns).toString().c_str());
    //Serial.printf(" Gateway IP.: %s\n", IPAddress(cfgbuf.gw).toString().c_str());
    Serial.printf(" Channel....: %d\n", cfgbuf.chl);
    Serial.printf(" BSSID......: %x:%x:%x:%x:%x:%x\n", cfgbuf.mac[0], cfgbuf.mac[1], cfgbuf.mac[2], cfgbuf.mac[3], cfgbuf.mac[4], cfgbuf.mac[5]);

    // This prevents DHCP, but it does not save more than perhaps 10ms: 
    // WiFi.config(cfgbuf.ip, cfgbuf.gw, cfgbuf.msk, cfgbuf.dns);
    WiFi.begin(ESSID, PSK, cfgbuf.chl, cfgbuf.mac);

    for (unsigned long i=millis(); millis()-i < 2000;) {
      delay(10);

      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("WiFi connected with cached values (%lu)\n", millis()-i);
        return true;
      } 
    }
  }

  memset(&cfgbuf, 0, sizeof(cfgbuf));
  
  // try it with the slow process
  WiFi.begin(ESSID, PSK);
  
  for (unsigned long i=millis(); millis()-i < 60000;) {
    delay(10);
  
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi connected (%lu)\n", millis()-i);
  
      uint8_t *bssid = WiFi.BSSID();
      for (uint32_t i = 0; i < sizeof(cfgbuf.mac); i++) cfgbuf.mac[i] = bssid[i];
      cfgbuf.chl = WiFi.channel();
      //cfgbuf.ip = WiFi.localIP();
      //cfgbuf.gw = WiFi.gatewayIP();
      //cfgbuf.msk = WiFi.subnetMask();
      //cfgbuf.dns = WiFi.dnsIP();
    
      return true;
    }
  }

  Serial.printf("WiFi NOT connected\n");
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

  // Is this ESP32-E two point calibrated?
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    Serial.println("eFuse Two Point: Supported");
  } else {
    Serial.println("eFuse Two Point: NOT supported");
  }

  //is this ESP32-E one point calibrated?
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    Serial.println("eFuse Vref: Supported");
  } else {
    Serial.println("eFuse Vref: NOT supported");
  }

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
  
  //to avoid noise sample the pin several times and average the result
  for(int i=1; i<=rounds; i++) {
    value += adc1_get_raw(ADC1_CHANNEL_6);
  }
  value /= (uint32_t)rounds;

  //due to the voltage divider (1M+1M) values must be multiplied by 2
  //and convert mV to V
  return esp_adc_cal_raw_to_voltage(value, &adc_chars)*2.0/1000.0;
}

/******************************************************************************
Description.: since this is battery sensor everything happens in setup
              and when the tonguing' is done the device enters deep-sleep
Input Value.: -
Return Value: -
******************************************************************************/
void setup() {
  //visual feedback when we are active, turn on onboard LED
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  
  Serial.begin(115200);
  Serial.print("===================================================\r\n");
  Serial.println("FireBeetle starting up\r\nCompiled at: " __DATE__ " - " __TIME__);

  //read battery voltage
  float BatteryVoltage = readBattery();
  Serial.printf("Voltage: %4.3f V\r\n", BatteryVoltage);
  
  //distinguish first start or wakeup
  if (esp_reset_reason() == ESP_RST_POWERON) {
    Serial.printf("ESP was just switched ON\n");
    NumberOfRestarts = 0;

    //default is to have WiFi off
    if (WiFi.getMode() != WIFI_OFF) {
      Serial.printf("WiFi wasn't off!\n");
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
    NumberOfRestarts++;
    Serial.printf("ESP woke up (%lu)\n", NumberOfRestarts);

    //read RTC
    struct timeval tv;
    gettimeofday(&tv, NULL);
    Serial.printf("Total Time since RstPowerOn: %lu s\n", tv.tv_sec);

    WiFiUP(true);
  }

  //establish connection to MQTT server, use the ROOT-CA to authenticate
  //"net" is instanciated from a class that uses ciphers
  //using TLS is a HUGE strain on battery, without TLS the battery will last longer
  WiFiClientSecure net;
  MQTTClient MQTTClient;
  net.setCACert(MQTTRootCA.c_str());
  MQTTClient.setTimeout(5000);
  MQTTClient.begin(MQTTServerName.c_str(), MQTTPort, net);
  if( MQTTClient.connect(MQTTDeviceName.c_str(), MQTTUsername.c_str(), MQTTPassword.c_str())) {
    MQTTClient.publish(MQTTRootTopic+"/battery", String(BatteryVoltage), false, 1);
  }
  
  //bring everything down
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  Serial.printf("Sleep at %d ms\n", millis());
  esp_sleep_enable_timer_wakeup(10*60*1000*1000); // Sleep 10 minutes (in microseconds)

  //LED off and sleep
  digitalWrite(2, LOW);
  esp_deep_sleep_start();
  
  Serial.printf("This should never get printed\n");
}

void loop() {
  Serial.printf("This should never get printed\n");
}
