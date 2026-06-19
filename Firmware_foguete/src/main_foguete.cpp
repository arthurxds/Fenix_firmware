#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <HardwareSerial.h>
#include <RH_RF95.h>
#include <TinyGPSPlus.h>
#include <MS5611.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP280.h>

// ─── PINOS ATUALIZADOS (ESP32-C6 SUPER MINI) ────────────────
#define PIN_SDA         6    
#define PIN_SCL         7    

#define PIN_SPI_SCK     0    
#define PIN_SPI_MOSI    1    
#define PIN_SPI_MISO    2    

#define PIN_LORA_NSS    3    
#define PIN_LORA_RST   18    
#define PIN_LORA_DIO0  19    

#define PIN_SD_CS       8    

#define PIN_GPS_RX     16    // Rx do GPS -> Rx do ESP
#define PIN_GPS_TX     17    // Tx do GPS -> Tx do ESP

#define PIN_BUZZER      5    
#define PIN_SQUIB      15    // Porta Gate do Mosfet 1
#define PIN_RBF         4    // Fio do RBF

// ─── CONSTANTES ─────────────────────────────────────────────
#define LOOP_MS          20      // 50 Hz
#define TELEMETRY_MS    500      // LoRa 2 Hz
#define SD_SYNC_MS     1000      // fsync 1 Hz

#define LAUNCH_ALT_M    10.0f    
#define LAUNCH_CONFIRM   3       
#define APOGEE_CONFIRM   3       
#define SQUIB_PULSE_MS  500      
#define LANDING_SPEED   0.5f     
#define LANDING_TIME_MS 3000     

// ─── MÁQUINA DE ESTADOS ─────────────────────────────────────
enum FlightState : uint8_t {
    IDLE    = 0,
    VOANDO  = 1,
    APOGEU  = 2,
    DESCIDA = 3,   
    SOLO    = 4
};

const char* ESTADOS[] = { "IDLE", "VOANDO", "APOGEU", "DESCIDA", "SOLO" };
FlightState currentState = IDLE;

// ─── VARIÁVEIS GLOBAIS DOS SENSORES ─────────────────────────
float padPressure = 101325.0f;
float ms_press = 0.0, ms_temp = 0.0, ms_alt = 0.0, ms_altPrev = 0.0;
float bmp_press = 0.0, bmp_temp = 0.0, bmp_alt = 0.0;
float vz = 0.0f, maxAlt = 0.0f;

float a1x = 0.0, a1y = 0.0, a1z = 0.0; // ADXL 1
float a2x = 0.0, a2y = 0.0, a2z = 0.0; // ADXL 2

double  gpsLat = 0.0, gpsLon = 0.0;
float   gpsAlt = 0.0f;
uint8_t gpsSats = 0;

bool gpsValid = false, lora_ok = false, sd_ok = false, rbfRemoved = false;
bool ms5611_ok = false, bmp_ok = false, adxl1_ok = false, adxl2_ok = false;

bool squibFired = false;
bool squibPinActive = false;

int  launchCount = 0, apogeeCount = 0;
unsigned long apogeeTimer = 0, squibTimer = 0, landingTimer = 0;
unsigned long lastLoop = 0, lastTele = 0, lastSync = 0;

// ─── OBJETOS ─────────────────────────────────────────────────
MS5611            baro_ms;            // Padrão MS5611
Adafruit_BMP280   baro_bmp;           // BMP280
Adafruit_ADXL375  accel1(1234);       // ADXL 1 - Apenas o ID falso aqui
Adafruit_ADXL375  accel2(5678);       // ADXL 2 - Apenas o ID falso aqui

RH_RF95           lora(PIN_LORA_NSS, PIN_LORA_DIO0);
TinyGPSPlus       gps;
HardwareSerial    gpsSerial(1);       // Usando UART1
File              logFile;

// ─── PROTÓTIPOS ──────────────────────────────────────────────
void readSensors();
void updateFSM();
void controlSquib();
void processData();
void checkRBF();
void buzzerBeep(int n, int onMs, int offMs = 150);

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n=== FENIX UFMG v4.0 — REDUNDANCIA DE SENSORES ==="));

    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_SQUIB,  OUTPUT);
    pinMode(PIN_RBF,    INPUT_PULLUP);

    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_SQUIB,  LOW);

    checkRBF();

    Wire.begin(PIN_SDA, PIN_SCL);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // Cartão SD
    if (SD.begin(PIN_SD_CS)) {
        logFile = SD.open("/fenix_log.csv", FILE_WRITE);
        if (logFile) {
            logFile.println(F("ms,estado,alt_ms,vz,press_ms,temp_ms,alt_bmp,press_bmp,temp_bmp,a1x,a1y,a1z,a2x,a2y,a2z,lat,lon,sats,rbf,squib"));
            logFile.flush();
            sd_ok = true;
            Serial.println(F("[OK] SD card"));
        }
    } else { Serial.println(F("[ERRO] SD card")); }

    // Sensores Barométricos
    if (baro_ms.begin()) {
        baro_ms.setOversampling(OSR_ULTRA_HIGH);
        ms5611_ok = true;
        Serial.println(F("[OK] MS5611"));
    } else { Serial.println(F("[ERRO] MS5611")); }

    if (baro_bmp.begin(0x76)) { // Força o endereço I2C do pino SDO no GND
        bmp_ok = true;
        Serial.println(F("[OK] BMP280 (0x76)"));
    } else { Serial.println(F("[ERRO] BMP280")); }

    // Sensores de Aceleração 
    if (accel1.begin(0x53)) { 
        accel1.setDataRate(ADXL343_DATARATE_100_HZ);
        adxl1_ok = true;
        Serial.println(F("[OK] ADXL375 #1 (0x53)"));
    } else { Serial.println(F("[ERRO] ADXL375 #1")); }

    if (accel2.begin(0x1D)) { 
        accel2.setDataRate(ADXL343_DATARATE_100_HZ);
        adxl2_ok = true;
        Serial.println(F("[OK] ADXL375 #2 (0x1D)"));
    } else { Serial.println(F("[ERRO] ADXL375 #2")); }

    // LoRa
    pinMode(PIN_LORA_RST, OUTPUT);
    digitalWrite(PIN_LORA_RST, LOW);  delay(10);
    digitalWrite(PIN_LORA_RST, HIGH); delay(10);
    
    if (lora.init()) {
        lora.setFrequency(915.0);
        lora.setTxPower(20, false);
        lora_ok = true;
        Serial.println(F("[OK] LoRa RFM95W"));
    } else { Serial.println(F("[ERRO] LoRa RFM95W")); }

    // Calibração do Barômetro Principal (MS5611) para Altitude Base
    Serial.println(F("[IDLE] Calibrando pad..."));
    if (ms5611_ok) {
        float soma = 0.0f;
        int n = 0;
        for (int i = 0; i < 30; i++) {
            if (baro_ms.read() == 0) { soma += baro_ms.getPressure(); n++; }
            delay(20);
        }
        if (n > 0) padPressure = soma / n;
    }
    
    Serial.printf("[IDLE] Pad: %.2f Pa | RBF: %s\n", padPressure, rbfRemoved ? "REMOVIDO" : "INSTALADO");
    Serial.println(F("[IDLE] Pronto!\n"));
    buzzerBeep(2, 300);
}

// ─────────────────────────────────────────────────────────────
//  LOOP PRINCIPAL
// ─────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();
    if (now - lastLoop < LOOP_MS) return;
    lastLoop = now;

    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    if (gps.location.isValid()) {
        gpsLat  = gps.location.lat();
        gpsLon  = gps.location.lng();
        gpsAlt  = gps.altitude.meters();
        gpsSats = gps.satellites.value();
        gpsValid = true;
    }

    readSensors();
    updateFSM();
    controlSquib();   
    processData();
}

// ─────────────────────────────────────────────────────────────
//  LEITURA DOS SENSORES
// ─────────────────────────────────────────────────────────────
void readSensors() {
    checkRBF();

    // Lê o Barômetro Mestre (O FSM usa este para tomar decisões)
    if (ms5611_ok && baro_ms.read() == 0) {
        ms_press = baro_ms.getPressure();
        ms_temp  = baro_ms.getTemperature();
        ms_altPrev = ms_alt;
        ms_alt   = 44330.0f * (1.0f - powf(ms_press / padPressure, 0.1903f));
        vz       = (ms_alt - ms_altPrev) / (LOOP_MS / 1000.0f);
        if (ms_alt > maxAlt) maxAlt = ms_alt;
    }

    // Lê o Barômetro Secundário (Apenas para gravação/telemetria)
    if (bmp_ok) {
        bmp_press = baro_bmp.readPressure();
        bmp_temp  = baro_bmp.readTemperature();
        bmp_alt   = 44330.0f * (1.0f - powf(bmp_press / padPressure, 0.1903f)); 
    }

    sensors_event_t event;
    
    // Lê o Acelerômetro 1
    if (adxl1_ok) {
        accel1.getEvent(&event);
        a1x = event.acceleration.x / 9.81f;
        a1y = event.acceleration.y / 9.81f;
        a1z = event.acceleration.z / 9.81f;
    }

    // Lê o Acelerômetro 2
    if (adxl2_ok) {
        accel2.getEvent(&event);
        a2x = event.acceleration.x / 9.81f;
        a2y = event.acceleration.y / 9.81f;
        a2z = event.acceleration.z / 9.81f;
    }
}

// ─────────────────────────────────────────────────────────────
//  MÁQUINA DE ESTADOS
// ─────────────────────────────────────────────────────────────
void updateFSM() {
    switch (currentState) {
        
        case IDLE:
            if (ms_alt > LAUNCH_ALT_M) launchCount++;
            else                       launchCount = 0;

            if (launchCount >= LAUNCH_CONFIRM) {
                currentState = VOANDO;
                launchCount  = 0;
                Serial.println(F("[FSM] LANCAMENTO!"));
                buzzerBeep(1, 200);
            }
            break;

        case VOANDO:
            if (vz < 0.0f) apogeeCount++;
            else           apogeeCount = 0;

            if (apogeeCount >= APOGEE_CONFIRM) {
                currentState = APOGEU;
                apogeeTimer  = millis();
                apogeeCount  = 0;
                Serial.printf("[FSM] APOGEU! Alt max: %.1fm\n", maxAlt);
                buzzerBeep(2, 200);
            }
            break;

        case APOGEU:
            // Atraso de 2 segundos (2000 ms) para atuar como Backup
            if (millis() - apogeeTimer >= 2000) {
                if (rbfRemoved && ms_alt >= 50.0f && !squibFired) {
                    digitalWrite(PIN_SQUIB, HIGH);
                    squibPinActive = true;
                    squibTimer     = millis();
                    squibFired     = true;
                    Serial.println(F("[SQUIB] Paraquedas Ativado (Backup ESP)!"));
                }
                currentState = DESCIDA;
                Serial.println(F("[FSM] APOGEU -> DESCIDA"));
            }
            break;

        case DESCIDA:
            if (fabsf(vz) < LANDING_SPEED) {
                if (landingTimer == 0) landingTimer = millis();
                if (millis() - landingTimer >= LANDING_TIME_MS) {
                    currentState = SOLO;
                    Serial.println(F("[FSM] POUSADO!"));
                    buzzerBeep(3, 300);
                }
            } else {
                landingTimer = 0;
            }
            break;

        case SOLO:
            if (millis() % 3000 < LOOP_MS) {
                buzzerBeep(3, 100, 100);
            }
            break;
    }
}

void controlSquib() {
    if (squibPinActive && (millis() - squibTimer >= SQUIB_PULSE_MS)) {
        digitalWrite(PIN_SQUIB, LOW);
        squibPinActive = false;
        Serial.println(F("[SQUIB] Pulso finalizado"));
    }
}

void checkRBF() {
    rbfRemoved = (digitalRead(PIN_RBF) == HIGH);
}

// ─────────────────────────────────────────────────────────────
//  PROCESSAMENTO DE DADOS (SD E LORA)
// ─────────────────────────────────────────────────────────────
void processData() {
    unsigned long now = millis();

    String csv = String(now)                   + "," +
                 String(ESTADOS[currentState]) + "," +
                 String(ms_alt,  1)            + "," +
                 String(vz,      1)            + "," +
                 String(ms_press,0)            + "," +
                 String(ms_temp, 1)            + "," +
                 String(bmp_alt, 1)            + "," +
                 String(bmp_press,0)           + "," +
                 String(bmp_temp, 1)           + "," +
                 String(a1x, 2) + "," + String(a1y, 2) + "," + String(a1z, 2) + "," +
                 String(a2x, 2) + "," + String(a2y, 2) + "," + String(a2z, 2) + "," +
                 String(gpsLat,  6)            + "," +
                 String(gpsLon,  6)            + "," +
                 String(gpsSats)               + "," +
                 String(rbfRemoved ? 1 : 0)    + "," +
                 String(squibFired ? 1 : 0);

    if (sd_ok) {
        logFile.println(csv);
        if (now - lastSync >= SD_SYNC_MS) {
            logFile.flush();
            lastSync = now;
        }
    }

    if (lora_ok && (now - lastTele >= TELEMETRY_MS)) {
        lora.send((uint8_t*)csv.c_str(), csv.length());
        lora.waitPacketSent();
        lastTele = now;
    }
}

void buzzerBeep(int n, int onMs, int offMs) {
    for (int i = 0; i < n; i++) {
        digitalWrite(PIN_BUZZER, HIGH); delay(onMs);
        digitalWrite(PIN_BUZZER, LOW);
        if (i < n - 1) delay(offMs);
    }
}
