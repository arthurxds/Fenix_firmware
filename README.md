# Fênix UFMG - Sistema de Aviônica e Telemetria 

Este repositório reúne o firmware do sistema eletrônico de controle, aquisição de dados e telemetria, desenvolvido para os projetos de foguetimodelismo de alta potência da equipe Fênix UFMG. O projeto é composto por dois módulos principais que operam em conjunto via comunicação de radiofrequência (LoRa).

## Nota sobre o histórico de versionamento

Este repositório foi consolidado para a entrega 3 que marca a unificação do sistema e a introdução de redundância de sensores. Durante as etapas de desenvolvimento, os firmwares do foguete e da estação de solo foram tratados em ambientes de compilação isolados. Por essa razão, este repositório apresenta um histórico de commits unificado a partir deste ponto, garantindo que a versão apresentada seja o estado estável, testado e validado para voo, eliminando registros de rascunhos ou configurações de teste que não compõem o produto final.

---

## Estrutura do projeto

O repositório está organizado nos seguintes diretórios:

### 1. Firmware_foguete (Computador de Voo)
Desenvolvido no ambiente PlatformIO para o microcontrolador ESP32-C6 Super Mini (arquitetura RISC-V). É o núcleo responsável pelo processamento de dados, execução da lógica de voo e acionamento do sistema de recuperação.

* Máquina de Estados: Gerencia os cinco estágios de voo (Idle, Voando, Apogeu, Descida e Solo).
* Redundância (I2C): Integração de sensores duplos (MS5611 + BMP280 para altitude; 2x ADXL375 para aceleração).
* Segurança: Implementação de barreira lógica no pino de RBF (Remove Before Flight) para prevenir disparos acidentais.
* Telemetria e Logging: Transmissão via LoRa RFM95W (915 MHz) e gravação de dados em cartão SD com sincronização física de sistema de arquivos (fsync).

### 2. Estacao_Solo (Receptor e Ponte de Dados)
Desenvolvido para o Arduino Mega 2560. Funciona como a interface de recepção e processamento de telemetria.

* Comunicação: Implementação de Software SPI para operação com o rádio LoRa, contornando limitações de hardware da placa.
* Monitoramento: Sistema de Timeout que monitora a integridade da conexão, emitindo alertas visuais em caso de falha de telemetria.
* Interface: Conversão dos pacotes recebidos em um fluxo serial limpo para leitura e análise em software de solo.

---

## Dependências e Bibliotecas

As principais bibliotecas utilizadas são:

* RadioHead (Configurada com ajustes locais para compatibilidade de interrupções no ESP32-C6).
* TinyGPSPlus (Protocolo NMEA).
* Adafruit ADXL375 e Adafruit BMP280.
* MS5611 (desenvolvida por Rob Tillaart).

## Procedimento de Compilação

1. Computador de Voo: O projeto dentro da pasta Firmware_foguete deve ser aberto via VS Code com a extensão PlatformIO. O ambiente gerenciará automaticamente as dependências listadas no platformio.ini.
2. Estação de Solo: O código contido em Estacao_Solo deve ser compilado na Arduino IDE, utilizando o perfil do Arduino Mega 2560. Certifique-se de que a biblioteca RadioHead está instalada manualmente no gerenciador de bibliotecas da IDE.
