
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h> 
#include <RHSoftwareSPI.h> 

// ─── PINOS (ARDUINO MEGA 2560) ────────────────────────────────
#define PIN_LORA_NSS      7   // Chip Select (CS) - Precisa de conversor de nível (5V -> 3.3V)
#define PIN_LORA_RST      9   // Pino de Reset 
#define PIN_LORA_DIO0     8   // Pino de interrupção (sinal de pacote recebido)

// Pinos de comunicação de Dados (Implementação via Software SPI)
#define PIN_SPI_MISO     11   // Master In Slave Out (Recebe dados do Rádio)
#define PIN_SPI_SCK      12   // Clock do barramento - Precisa de conversor de nível
#define PIN_SPI_MOSI     13   // Master Out Slave In (Envia comandos ao Rádio) - Precisa de conversor

// ─── CONSTANTES DE COMUNICAÇÃO E WATCHDOG ────────────────────
#define LORA_FREQ        915.0f
#define LORA_TX_POWER    5      // Potência reduzida (a base apenas recebe dados, pouca emissão)
#define NO_SIGNAL_WARN   5000   // Warning: 5 segundos sem receber pacotes do foguete
#define NO_SIGNAL_ALERT  30000  // Crítico: 30 segundos sem receber dados (Pode indicar perda total)
#define LORA_RETRY_MS    10000  // Tentativa de reiniciar o rádio da base a cada 10s se houver falha de hardware

// ─── INSTANCIAÇÃO DO SPI VIRTUAL E RÁDIO ─────────────────────
// Usamos SPI via Software (RHSoftwareSPI) por restrições de pinagem no Mega.
// A velocidade do software é perfeitamente suficiente para a taxa de dados de 2Hz do foguete.
RHSoftwareSPI spiVirtual;
RH_RF95 lora(PIN_LORA_NSS, PIN_LORA_DIO0, spiVirtual);

// ─── VARIÁVEIS GLOBAIS ───────────────────────────────────────
unsigned long lastPacketMs = 0;
unsigned long lastLoraRetry = 0;
unsigned long loopCount = 0;

bool loraReady = false;
bool warnSent = false;
bool alertActive = false;

String lastEstado = "";

// ─── PROTÓTIPOS ──────────────────────────────────────────────
void initLora();
void processaPacket();
void checaTimeout();
void alertaEstado(const String& estado);
void sendStatus(const char* codigo, const char* msg);

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    // Aguarda a porta serial conectar para não perder a primeira mensagem
    while (!Serial) { ; } 
    
    Serial.println(F("$STATUS,BOOT,Fenix UFMG Estacao de Solo v1.4"));
    
    // Injeta os pinos personalizados no objeto SPI Virtual
    spiVirtual.setPins(PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SCK);
    
    initLora();
    
    sendStatus("READY", "Estacao configurada via SPI por Software. Aguardando pacotes...");
    lastPacketMs = millis();
    lastLoraRetry = millis();
}

// ─────────────────────────────────────────────────────────────
//  LOOP PRINCIPAL
// ─────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // AUTO-RECUPERAÇÃO: Se o rádio LoRa travar ou for desconectado fisicamente,
    // ele vai tentar reiniciá-lo a cada 10 segundos sem travar o painel.
    if (!loraReady) {
        if (now - lastLoraRetry > LORA_RETRY_MS) {
            lastLoraRetry = now;
            sendStatus("LORA_RETRY", "Tentando reinicializar RFM95W...");
            initLora();
        }
        loopCount++;
        return; 
    }

    // Ouve o ambiente continuamente à procura de pacotes
    if (lora.available()) {
        processaPacket();
    }
    
    // Verifica sempre se o foguete parou de transmitir
    checaTimeout();
    loopCount++;
}

// ─────────────────────────────────────────────────────────────
//  INICIALIZAÇÃO DO RÁDIO
// ─────────────────────────────────────────────────────────────
void initLora() {
    pinMode(PIN_LORA_RST, OUTPUT);
    digitalWrite(PIN_LORA_RST, LOW);  delay(10);
    digitalWrite(PIN_LORA_RST, HIGH); delay(10);

    // Tenta inicializar. Se o conversor 5V->3.3V falhar, ele avisa o PC.
    if (!lora.init()) {
        sendStatus("LORA_ERR", "Falha no RFM95W. Verifique pinos D7, D8, D9, D11, D12, D13");
        loraReady = false;
        return; 
    }

    lora.setFrequency(LORA_FREQ);
    lora.setTxPower(LORA_TX_POWER, false);
    loraReady = true;
    
    Serial.print(F("$STATUS,LORA_OK,LoRa iniciado em "));
    Serial.print(LORA_FREQ);
    Serial.println(F(" MHz"));
}

// ─────────────────────────────────────────────────────────────
//  PROCESSAMENTO DOS PACOTES
// ─────────────────────────────────────────────────────────────
void processaPacket() {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Se a leitura falhar (pacote corrompido no ar), cancela
    if (!lora.recv(buf, &len)) return;
    
    unsigned long ms_rx = millis();
    lastPacketMs = ms_rx; // Reseta o Watchdog Timer
    warnSent = false;
    alertActive = false;

    // Converte os bytes recebidos num texto legível
    buf[len] = '\0'; 
    String payload = String((char*)buf);
    payload.trim();

    // Qualidade do Sinal
    // RSSI: Quão forte o sinal chegou (quanto mais perto de 0, melhor).
    // SNR: Relação Sinal/Ruído (quanto mais positivo, mais limpo o sinal).
    int16_t rssi = lora.lastRssi();
    float snr = lora.lastSNR();

    // Despeja a string no barramento serial (Cabo USB) formatada com o cabeçalho "$FENIX,".
    Serial.print(F("$FENIX,"));
    Serial.print(rssi);
    Serial.print(F(","));
    Serial.print(snr, 1);
    Serial.print(F(","));
    Serial.print(ms_rx);
    Serial.print(F(","));
    Serial.println(payload);

    // ANÁLISE DE STRINGS:
    // Procura o estado do FSM (que é o segundo item no CSV gerado pelo foguete)
    int firstComma = payload.indexOf(',');
    if (firstComma >= 0) {
        int secondComma = payload.indexOf(',', firstComma + 1);
        if (secondComma >= 0) {
            String estado = payload.substring(firstComma + 1, secondComma);
            estado.trim();
            alertaEstado(estado);
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  TIMEOUT E ALERTAS (WATCHDOG DE TELEMETRIA)
// ─────────────────────────────────────────────────────────────
void checaTimeout() {
    unsigned long elapsed = millis() - lastPacketMs;

    // Alerta crítico de 30 segundos
    if (elapsed >= NO_SIGNAL_ALERT) {
        if (!alertActive) {
            sendStatus("NO_SIGNAL_CRITICAL", "Sem sinal por 30s!");
            alertActive = true;
        }
        return;
    }

    // Aviso de 5 segundos
    if (elapsed >= NO_SIGNAL_WARN && !warnSent) {
        sendStatus("NO_SIGNAL_WARN", "Sem pacote ha 5s");
        warnSent = true;
    }
}

// Emite eventos de log separados quando a Máquina de Estados muda
void alertaEstado(const String& estado) {
    if (estado == lastEstado) return; 

    if (estado == "VOANDO") {
        sendStatus("EVENT_LAUNCH", "Lancamento detectado"); 
    } 
    else if (estado == "APOGEU") {
        sendStatus("EVENT_APOGEU", "Apogeu detectado"); 
    } 
    else if (estado == "DESCIDA") { 
        sendStatus("EVENT_SQUIB", "Paraquedas disparado"); 
    } 
    else if (estado == "SOLO") {
        sendStatus("EVENT_SOLO", "Foguete pousado"); 
    }

    lastEstado = estado;
}

void sendStatus(const char* codigo, const char* msg) {
    Serial.print(F("$STATUS,"));
    Serial.print(codigo);
    Serial.print(F(","));
    Serial.println(msg);
}
