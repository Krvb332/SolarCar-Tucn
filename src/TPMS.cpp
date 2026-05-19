#include <Arduino.h>
#include "driver/twai.h"

// Pinii catre modulul CAN
#define TX_PIN GPIO_NUM_5
#define RX_PIN GPIO_NUM_4

// Structura pentru a retine starea fiecarui senzor
struct DateRoata {
  float presiune = 0.0;
  int temp = 0;
  float baterie = 0.0;
  bool scapaAer = false;
  bool tempExtrema = false;
  unsigned long ultimulMesaj = 0; 
};

// Array pentru cele 4 roti
DateRoata senzor[5]; 
const char *const numeRoti[5] = {
  "",
  "FATA STANGA  ",
  "FATA DREAPTA ",
  "SPATE DREAPTA ",
  "SPATE STANGA"
};

unsigned long timerAfisare = 0;
const unsigned long TIMEOUT_SENZOR = 2000; // 2 secunde

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=========================================");
  Serial.println("    SISTEM TPMS MONITORIZARE LIVE        ");
  Serial.println("   Detectie automata deconectare: ACTIV  ");
  Serial.println("=========================================");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
    Serial.println("Sistem CAN pornit corect.\n");
  } else {
    Serial.println("Eroare Critica la pornirea CAN!");
  }
}

void loop() {
  twai_message_t message;
  
  // 1. CITIRE DATE DE PE CAN
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (message.identifier == 0x18FEF433) {
      int idRoata = message.data[0]; 
      
      if(idRoata >= 1 && idRoata <= 4) {
          int raw_pressure = (message.data[1] << 8) | message.data[2]; 
          
          senzor[idRoata].presiune = (raw_pressure * 0.0101) + 0.35; 
          senzor[idRoata].temp = message.data[3]; 
          senzor[idRoata].baterie = message.data[4] * 0.1237; 
          senzor[idRoata].scapaAer = bitRead(message.data[5], 0);  
          senzor[idRoata].tempExtrema = bitRead(message.data[5], 4); 
          senzor[idRoata].ultimulMesaj = millis(); 
      }
    }
  }

  // 2. AFISARE TABLOU DE BORD (O data la 2 secunde)
  if (millis() - timerAfisare >= 200) {
    timerAfisare = millis();
    
    Serial.println("\n--- [ STATUS TPMS ] ---");
    
    for (int i = 1; i <= 4; i++) {
      const char *nume = numeRoti[i];

      Serial.print("["); Serial.print(nume); Serial.print("] ");

      // CALCULUL TIMPULUI SCURS - Aici era eroarea
      unsigned long acum = millis();
      unsigned long diff = acum - senzor[i].ultimulMesaj;

      if (senzor[i].ultimulMesaj == 0 || diff > TIMEOUT_SENZOR) {
          senzor[i].presiune = 0;
          senzor[i].baterie = 0;
          Serial.println("OFFLINE (Scoateti bateria / Lipsa semnal)");
      } 
      else {
          Serial.print("Pres: "); Serial.print(senzor[i].presiune, 2); Serial.print(" Bar | ");
          Serial.print("Temp: "); Serial.print(senzor[i].temp); Serial.print(" C | ");
          Serial.print("Bat: "); Serial.print(senzor[i].baterie, 2); Serial.print(" V");

          if (senzor[i].scapaAer) Serial.print(" ALERTA AER!");
          if (senzor[i].tempExtrema) Serial.print(" ALERTA TEMP!");
          
          Serial.println(); 
      }
    }
    Serial.println("-----------------------");
  }
}
