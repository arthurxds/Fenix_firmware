# Fênix UFMG - Sistema de Aviônica e Telemetria 

Este repositório reúne o firmware do sistema eletrónico de controlo, aquisição de dados e telemetria, desenvolvido para os projetos de foguetimodelismo de alta potência da equipa Fênix UFMG. O projeto é composto por dois módulos principais que operam em conjunto via comunicação de radiofrequência (LoRa).

---

## Estrutura do projeto

O repositório foi unificado num único ambiente de desenvolvimento para facilitar o controlo de versão e a partilha de bibliotecas. Dentro da pasta `src/`, o código está dividido em dois ficheiros principais:

### 1. Computador de Voo (`main_foguete.cpp`)
Desenvolvido no ambiente PlatformIO para o microcontrolador ESP32 (perfil NodeMCU-32S / Super Mini). É o núcleo responsável pelo processamento de dados, execução da lógica de voo e acionamento do sistema de recuperação.
* **Máquina de Estados:** Gere os cinco estágios de voo (Idle, Voando, Apogeu, Descida e Solo).
* **Redundância (I2C):** Integração de sensores duplos (MS5611 + BMP280 para altitude; 2x ADXL375 para aceleração).
* **Segurança:** Implementação de barreira lógica no pino de RBF (Remove Before Flight) para prevenir disparos acidentais.
* **Telemetria e Logging:** Transmissão via LoRa RFM95W (915 MHz) e gravação de dados em cartão SD com sincronização física do sistema de ficheiros (fsync).

### 2. Estação de Solo (`main_estacao.cpp`)
Desenvolvido para o Arduino Mega 2560. Funciona como a interface de receção e processamento de telemetria.
* **Comunicação:** Implementação de Software SPI para operação com o rádio LoRa, contornando as limitações de hardware da placa.
* **Monitorização:** Sistema de *Timeout* que monitoriza a integridade da ligação, emitindo alertas visuais em caso de falha de telemetria.
* **Interface:** Conversão dos pacotes recebidos num fluxo serial limpo para leitura e análise em software de solo.

---

## Dependências e Bibliotecas

As principais bibliotecas utilizadas são geridas automaticamente pelo PlatformIO:
* **RadioHead** (Fixada na versão compatível com os *timers* da plataforma ESP32 6.5.0)
* **TinyGPSPlus** (Protocolo NMEA)
* **Adafruit ADXL375** e **Adafruit BMP280**
* **MS5611** (desenvolvida por Rob Tillaart)

---

## Procedimento de Compilação

O projeto inteiro é gerido pelo **PlatformIO** (extensão do VS Code). Graças ao uso de filtros de *build* no ficheiro `platformio.ini`, é possível compilar ambos os códigos a partir do mesmo projeto, sem misturar bibliotecas ou gerar erros de dependência cruzada.

1. Abre a pasta raiz do repositório no VS Code (certifica-te de que tens a extensão PlatformIO instalada).
2. Na barra inferior azul do VS Code, clica no nome do ambiente atual (ex: `env:nodemcu-32s`).
3. Aparecerá uma lista no topo do ecrã. Seleciona o ambiente que desejas compilar ou carregar:
   * Escolhe `env:nodemcu-32s` para gravar o código no **Foguete** (ESP32).
   * Escolhe `env:megaatmega2560` para gravar o código na **Estação de Solo** (Arduino Mega).
4. Clica no ícone de "✓" (*Build*) ou "➔" (*Upload*). O PlatformIO fará o download de todas as dependências corretas automaticamente e ignorará o código da outra placa.
