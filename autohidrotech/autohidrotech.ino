/*************************************************************

Este projeto tem como objetivo automatizar e simplificar o cultivo 
de hortas hidropônicas utilizando o microcontrolador ESP32.Com a automação 
de tarefas como monitoramento de pH, condutividade elétrica, TDS e controle
de irrigação, você poderá desfrutar de uma horta eficiente e de fácil manutenção.

 *************************************************************/

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMxxxxxx"
#define BLYNK_TEMPLATE_NAME "Device"
#define BLYNK_AUTH_TOKEN "YourAuthToken"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "xxxxxxx";
char pass[] = "xxxxxxx";

/*******************/
BlynkTimer timer;
WidgetLED led1(V5);

/****MÓDULO RELAY****/
#define bombPin 25
unsigned long previousMillis = 0;
const unsigned long interval = 900000;  // 15 minutos em milissegundos
bool status_bomb = false;

/****LCD I2C****/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//Inicializa o display no endereco 0x27
LiquidCrystal_I2C lcd(0x3F, 16, 2);
bool set_lcd = 0;
bool set_phec = 0;

byte customChar[] = {
  B00100,
  B00100,
  B01110,
  B11111,
  B11101,
  B11001,
  B01110,
  B00100
};

/****pH METER****/
#include "DFRobot_PH.h"
#include <EEPROM.h>

#define PH_PIN 35
float voltage, phValue;
DFRobot_PH ph;

/****TDS METER****/
#define TdsSensorPin 34
#define VREF 3.3   // voltagem de referência analógica (Volt) do ADC
#define SCOUNT 30  // soma do ponto de amostragem

int analogBuffer[SCOUNT];  //armazene o valor analógico no array, leitura ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float ecValue = 0;
float temperature = 30;  // temperatura atual para compensação

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

BLYNK_WRITE(V0)  // Executa quando o valor do pino virtual 0 muda
{
  int pinValue = param.asInt();
  if (pinValue == 1) {
    // execute este código se o widget de troca estiver ativado
    digitalWrite(bombPin, HIGH);  // Defina o pino digital 25 ALTO
    led1.on();
    status_bomb = 1;
  } else {
    // execute este código se o widget de troca agora estiver DESLIGADO
    digitalWrite(bombPin, LOW);  // Defina o pino digital 25 BAIXO
    led1.off();
    status_bomb = 0;
  }
}

// Função ligar bomba
void ligar_bomba() {
  digitalWrite(bombPin, HIGH);
  Serial.println("Ligando a bomba d'água");
}
// Função desliga bomba d'água
void desligar_bomba() {
  digitalWrite(bombPin, LOW);
  Serial.println("Desligando a bomba d'água");
}


void dataTDS() {
  Serial.print("TDS Value:");
  Serial.print(tdsValue, 0);
  Serial.println("ppm");
  Serial.print("EC Value:");
  Serial.print(ecValue, 1);
  Serial.println("us/cm");
  Serial.print(phValue, 0);
  Serial.println("pH");
  Blynk.virtualWrite(V1, phValue);
  Blynk.virtualWrite(V2, tdsValue);
  Blynk.virtualWrite(V3, ecValue);
}


void print_logo() {

  lcd.clear();
  lcd.setBacklight(HIGH);
  lcd.setCursor(5, 0);
  lcd.print("HTech!!");
  lcd.setCursor(2, 1);
  lcd.print("Hidroponia");
}

// função que imprime os dados no lcd
void print_lcd() {
  if (set_lcd == 0) {
    set_lcd = 1;
    lcd.clear();
    lcd.setBacklight(HIGH);
    lcd.setCursor(0, 0);
    lcd.print("TDS: ");
    lcd.setCursor(5, 0);
    lcd.print(tdsValue);
    lcd.setCursor(12, 0);
    lcd.print("ppm");
    lcd.setCursor(0, 1);
    lcd.print("EC: ");
    lcd.setCursor(4, 1);
    lcd.print(ecValue);
    lcd.setCursor(10, 1);
    lcd.print("uS/cm");
  } else {
    set_lcd = 0;
    lcd.clear();
    lcd.setBacklight(HIGH);
    lcd.setCursor(5, 0);
    lcd.print("HTech!!");
    lcd.setCursor(1, 1);
    lcd.print("pH: ");
    lcd.setCursor(5, 1);
    lcd.print(phValue);
    lcd.setCursor(11, 1);
    lcd.write(0);
    lcd.setCursor(13, 1);

    if (status_bomb == 1) {
      lcd.print("ON");
    } else {
      lcd.print("OFF");
    }
  }
}

void read_sensor() {
  if (set_phec == 0) {
    /****TDS/EC SENSOR****/
    set_phec = 1;
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U) {  //every 40 milliseconds,read the analog value from the ADC
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  //read the analog value and store into the buffer
      analogBufferIndex++;
      if (analogBufferIndex == SCOUNT) {
        analogBufferIndex = 0;
      }
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U) {
      printTimepoint = millis();
      for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {

        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

        // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

        //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        //temperature compensation
        float compensationVoltage = averageVoltage / compensationCoefficient;

        //convert voltage value to tds value
        tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
        ecValue = (2 * tdsValue);
        //Serial.print("voltage:");
        //Serial.print(averageVoltage,2);
        //Serial.print("V   ");
      }
    }
  } else {
    // PH meter
    static unsigned long timepoint = millis();
    set_phec = 0;
    if (millis() - timepoint > 1000U) {  //time interval: 1s
      timepoint = millis();
      //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
      voltage = analogRead(PH_PIN) / 4096.0 * 5000;  // read the voltage
      phValue = ph.readPH(voltage, temperature);     // convert voltage to pH with temperature compensation
      Serial.print("temperature:");
      Serial.print(temperature, 1);
      Serial.print("^C  pH:");
      Serial.println(phValue, 2);
    }
    ph.calibration(voltage, temperature);  // calibration process by Serail CMD
  }
}

 /**** Timer Bomba****/
void timer_bomb() {
 
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (status_bomb) {
      desligar_bomba();
      led1.off();
      status_bomb = false;
    } else {
      ligar_bomba();
      led1.on();
      status_bomb = true;
    }
  }
}

BLYNK_CONNECTED() {

  Blynk.syncVirtual(V0, V1, V2, V3, V5);
}

void setup() {
  // Debug console
  Serial.begin(115200);
  lcd.init();
  ph.begin();
  lcd.createChar(0, customChar);
  pinMode(TdsSensorPin, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(bombPin, OUTPUT);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // You can also specify server:
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
  timer.setTimer(1000, print_logo, 1);
  timer.setInterval(1000L, dataTDS);
  timer.setInterval(4000L, print_lcd);
}

void loop() { 
  Blynk.run();
  timer.run();
  read_sensor();
  timer_bomb();
}
