# LoRaMESH C++

Biblioteca C++ para comunicação com módulos **LoRaMESH (Radioenge)** via UART, projetada para ser o **core nativo do ecossistema LoRaMESH** em múltiplas plataformas.

Este projeto não foi pensado apenas para um único firmware ou para um único sistema operacional. A proposta é ter uma base técnica sólida, reutilizável e previsível, capaz de atender dois mundos ao mesmo tempo:

1. **microcontroladores e firmwares embarcados**
2. **Linux, Raspberry Pi, gateways e aplicações de alto nível**

Por isso, a biblioteca foi estruturada em torno de um núcleo C++ com uma abstração de transporte chamada `ILoraTransport`, permitindo reaproveitar a mesma lógica de protocolo em diferentes plataformas sem reescrever o comportamento principal do módulo a cada novo port.

Além disso, este core também é a base da SDK Python do projeto. A camada Python, instalada com:

```bash
pip install loramesh
```

replica a lógica desta biblioteca em uma interface mais amigável para scripts, automações, gateways Linux e Raspberry Pi. Em outras palavras, o **C++ é a base** e o **Python é a extensão de alto nível** construída em cima dessa base.

## Sumário

1. [Visão geral](#1-visão-geral)
2. [Por que esta biblioteca existe](#2-por-que-esta-biblioteca-existe)
3. [Objetivo do projeto](#3-objetivo-do-projeto)
4. [Relação com a SDK Python](#4-relação-com-a-sdk-python)
5. [Plataformas e direção do projeto](#5-plataformas-e-direção-do-projeto)
6. [Organização conceitual do projeto](#6-organização-conceitual-do-projeto)
7. [Arquitetura da solução](#7-arquitetura-da-solução)
8. [Conceitos importantes antes de começar](#8-conceitos-importantes-antes-de-começar)
9. [Interface de transporte `ILoraTransport`](#9-interface-de-transporte-iloratransport)
10. [Adapters por plataforma](#10-adapters-por-plataforma)
11. [Inicialização rápida](#11-inicialização-rápida)
12. [Fluxo recomendado de inicialização](#12-fluxo-recomendado-de-inicialização)
13. [Debug da comunicação](#13-debug-da-comunicação)
14. [Como o core monta e recebe frames](#14-como-o-core-monta-e-recebe-frames)
15. [CRC utilizado pela biblioteca](#15-crc-utilizado-pela-biblioteca)
16. [Configuração de rede](#16-configuração-de-rede)
17. [Configuração de rádio](#17-configuração-de-rádio)
18. [Classe LoRa e janela de recepção](#18-classe-lora-e-janela-de-recepção)
19. [Constantes principais](#19-constantes-principais)
20. [Informações e estados mantidos pelo core](#20-informações-e-estados-mantidos-pelo-core)
21. [Controle remoto de GPIO](#21-controle-remoto-de-gpio)
22. [Leitura analógica e medições auxiliares](#22-leitura-analógica-e-medições-auxiliares)
23. [Modo transparente e dupla UART](#23-modo-transparente-e-dupla-uart)
24. [Referência detalhada da API](#24-referência-detalhada-da-api)
25. [Exemplos completos](#25-exemplos-completos)
26. [Boas práticas](#26-boas-práticas)
27. [Erros comuns e diagnóstico](#27-erros-comuns-e-diagnóstico)
28. [ESP-IDF e STM32 HAL](#28-esp-idf-e-stm32-hal)
29. [Casos de uso reais](#29-casos-de-uso-reais)
30. [Licença](#30-licença)
31. [Contato e referências](#31-contato-e-referências)
32. [Considerações finais](#32-considerações-finais)

## 1. Visão geral

O **LoRaMESH C++** é o núcleo responsável por concentrar a lógica real de comunicação com os módulos LoRaMESH. É aqui que ficam as rotinas que efetivamente:

- montam frames de comando
- montam frames de modo transparente
- enviam bytes por um transporte abstrato
- recebem respostas do módulo
- validam CRC
- leem o estado local do rádio
- configuram `NetworkID`
- configuram senha
- leem e alteram parâmetros de rádio
- configuram classe e janela de recepção
- controlam GPIO remoto
- leem entradas digitais e analógicas remotas
- calculam resistência e temperatura a partir do ADC
- mantêm um fluxo único reaproveitável entre Arduino, Linux, Raspberry Pi e futuras integrações com ESP-IDF e STM32 HAL

A proposta desta biblioteca não é só encapsular um conjunto de comandos. Ela existe para ser o coração de um ecossistema maior, em que:

- o **core** fica em C++
- os **adapters** ligam o core à UART real da plataforma
- os **examples** mostram uso prático por ambiente
- a **SDK Python** replica essa base para automações e gateways

## 2. Por que esta biblioteca existe

Quem já trabalhou com rádio em aplicações reais sabe que o problema raramente é apenas enviar uma string. O desafio de verdade aparece quando você precisa:

- manter vários nós com configuração consistente
- acionar dispositivos remotos em campo
- ler sensores analógicos e digitais distribuídos
- portar a mesma lógica para firmware e para Linux
- depurar serial, framing, tempo de resposta e estado da rede
- evitar reimplementar o protocolo em cada projeto

Esta biblioteca nasce justamente para resolver isso com uma base única e reaproveitável.

Em vez de criar uma implementação isolada para Arduino, outra para Linux, outra para Raspberry Pi e outra para Python, a ideia do projeto é concentrar a inteligência principal no core C++, desacoplando apenas a camada que fala com a UART da plataforma.

Isso reduz retrabalho, melhora a manutenção, facilita a evolução e mantém o comportamento mais previsível entre ambientes diferentes.

## 3. Objetivo do projeto

O objetivo desta biblioteca é servir como base para sistemas reais com módulos LoRaMESH, permitindo construir aplicações como:

- automação rural
- controle de bombas e válvulas
- redes privadas de monitoramento
- leitura remota de sensores distribuídos
- controle remoto de saídas em campo
- gateways Linux ou Raspberry Pi
- provisionamento e manutenção de nós
- integração entre microcontroladores e aplicações de alto nível
- testes, debug e automação com a mesma lógica central

## 4. Relação com a SDK Python

Este ponto precisa ficar muito claro: **a SDK Python não substitui este core; ela replica sua lógica em uma interface mais amigável para Python**.

Na prática:

- o projeto em C++ é a base estrutural
- a SDK Python expõe essa base para automação, scripts e gateways
- a nomenclatura segue a mesma linha conceitual do core nativo
- o fluxo mental de uso continua alinhado com a versão C++
- evolução e manutenção continuam centradas no núcleo C++

A biblioteca Python já pode ser instalada com:

```bash
pip install loramesh
```

Isso reforça a proposta do projeto: o **C++ é a base** e o **Python é a camada de conveniência** para ambientes em que testes, automações e integrações de mais alto nível fazem sentido.

## 5. Plataformas e direção do projeto

A biblioteca foi pensada para funcionar em mais de um ambiente desde o início.

### 5.1 Arduino

Arduino é um dos alvos principais da biblioteca. A proposta aqui é permitir uso direto em firmwares, com inicialização da UART da placa, adapter leve e integração simples ao fluxo de controle do módulo.

Esse cenário cobre, por exemplo:

- placas AVR
- placas ARM compatíveis com Arduino
- ESP32 rodando framework Arduino
- ambientes com `Serial`, `Serial1`, `Serial2` ou derivados de `Stream`

### 5.2 Linux e Raspberry Pi

Linux é outro alvo central do projeto. Aqui a biblioteca pode ser usada diretamente em C++, principalmente em gateways, ferramentas de provisionamento, utilitários de diagnóstico e aplicações que falam com módulos LoRaMESH por serial.

No caso do Raspberry Pi, existem duas abordagens naturais:

- usar o core C++ diretamente com adapter Linux
- usar a SDK Python `loramesh`, construída sobre esse mesmo core

### 5.3 ESP-IDF

ESP-IDF já faz parte da direção do projeto e deve existir como integração própria. No momento, essa frente pode ficar em **standby**, mas o README já precisa reservar espaço para ela porque a arquitetura foi desenhada exatamente para isso.

### 5.4 STM32 HAL

STM32 HAL também faz parte da direção natural da biblioteca. Assim como no caso de ESP-IDF, esta parte pode permanecer em **standby por enquanto**, mas precisa estar contemplada na documentação porque o core já foi pensado para ser portável.

### 5.5 Um ponto importante sobre os exemplos

Este README evita congelar uma árvore rígida de arquivos no texto, porque a organização do repositório pode evoluir. O que importa aqui é a ideia estrutural:

- existe um **core C++**
- existe uma **interface de transporte**
- existem **adapters por plataforma**
- existem **examples por ambiente**
- existe uma **camada Python** no mesmo ecossistema

## 6. Organização conceitual do projeto

Em vez de documentar uma árvore fixa de paths que pode mudar com o tempo, o mais importante é entender os blocos conceituais do projeto.

### 6.1 Core

O core é onde mora a classe `LoRaMESH` e toda a lógica de protocolo, framing, CRC, configuração, leitura de estado do módulo e operações remotas.

### 6.2 Transporte abstrato

A interface `ILoraTransport` define o contrato mínimo que qualquer plataforma precisa oferecer ao core.

### 6.3 Adapters por plataforma

Cada plataforma implementa sua forma concreta de:

- escrever bytes
- verificar bytes disponíveis
- ler dados
- fornecer uma base temporal em milissegundos

### 6.4 Examples

Os exemplos servem para mostrar uso real da biblioteca em cada ambiente, cobrindo desde inicialização básica até controle de múltiplos slaves.

### 6.5 Camada Python

A parte Python existe como extensão do ecossistema. Ela não compete com o core C++, mas sim o reaproveita para uso em scripts, automações e gateways.

## 7. Arquitetura da solução

A arquitetura do projeto foi desenhada para reuso.

```text
Aplicação do usuário
    ↓
LoRaMESH (core C++)
    ↓
ILoraTransport
    ↓
Adapter da plataforma
    ↓
UART / serial real da plataforma
    ↓
Módulo LoRaMESH
```

Quando a camada Python entra na jogada, o desenho fica assim:

```text
Aplicação Python
    ↓
Bindings Python
    ↓
LoRaMESH (core C++)
    ↓
ILoraTransport / serial Linux
    ↓
UART
    ↓
Módulo LoRaMESH
```

### Vantagens dessa abordagem

- a lógica principal do protocolo fica centralizada
- cada plataforma implementa apenas o mínimo necessário para transporte
- o comportamento tende a ser consistente entre ambientes
- a SDK Python não precisa reinventar o protocolo
- novos adapters podem ser adicionados sem reescrever o core

## 8. Conceitos importantes antes de começar

Antes de usar a biblioteca, alguns pontos precisam ficar muito claros.

### 8.1 O projeto separa transporte de protocolo

A classe `LoRaMESH` não fala diretamente com `Serial`, `HardwareSerial`, `termios`, HAL do STM32 ou drivers do ESP-IDF. Ela fala com uma abstração chamada `ILoraTransport`.

É isso que torna o projeto realmente portável.

### 8.2 O core aceita dois transports

O construtor recebe:

```cpp
LoRaMESH(ILoraTransport* cmdTransport, ILoraTransport* transpTransport)
```

Ou seja, a arquitetura já foi pensada para separar:

- o canal de **comandos**
- o canal de **modo transparente**

Esses dois transports podem ser:

- o mesmo objeto
- dois objetos diferentes
- duas UARTs separadas
- duas interfaces lógicas na mesma plataforma

### 8.3 O debug em C++ depende de callback

No core C++, o debug depende de uma função callback registrada com `setDebug()`.

Isso significa que:

- `begin(true)` sozinho não é suficiente para imprimir mensagens
- o dump hexadecimal só aparece se houver callback configurado
- em ambientes embarcados, o callback normalmente aponta para `Serial.println`

### 8.4 O adapter Linux atual deve ser tratado com cuidado no baudrate

No estado atual do código, o construtor de `LinuxSerialTransport` recebe `device` e `baud`, mas a configuração do `termios` foi implementada com `B9600` fixo.

Na prática, isso significa que:

- o adapter Linux atual deve ser tratado como **9600** até que a conversão completa do argumento `baud` seja implementada
- exemplos C++ usando `LinuxSerialTransport` fazem mais sentido em `9600`
- isso é diferente da SDK Python, que pode usar outro backend de serial

### 8.5 LoRa não é comunicação instantânea

Ao controlar nós remotos, existe latência natural. Ela depende de fatores como:

- `BW`
- `SF`
- `CR`
- topologia da rede
- quantidade de hops
- ruído
- classe do dispositivo

### 8.6 Sempre valide o módulo antes de sair configurando

O fluxo mais seguro é sempre:

1. inicializar a UART da plataforma
2. criar o adapter
3. criar a instância de `LoRaMESH`
4. configurar debug, se necessário
5. chamar `begin()`
6. insistir em `localRead()` até obter resposta válida
7. só então aplicar configurações e comandos

## 9. Interface de transporte `ILoraTransport`

A base da portabilidade da biblioteca está aqui:

```cpp
class ILoraTransport
{
public:
    virtual ~ILoraTransport() {}
    virtual size_t write(const uint8_t* data, size_t len) = 0;
    virtual size_t available() = 0;
    virtual size_t read(uint8_t* buffer, size_t maxLen) = 0;
    virtual uint32_t millis() = 0;
};
```

### Responsabilidades de cada método

#### `write(const uint8_t* data, size_t len)`

Envia bytes pela interface física da plataforma.

#### `available()`

Retorna quantos bytes estão disponíveis para leitura.

#### `read(uint8_t* buffer, size_t maxLen)`

Lê dados disponíveis para o buffer informado.

#### `millis()`

Fornece uma base temporal em milissegundos, usada pelo core para controlar timeouts de recepção.

### O que isso significa na prática

Se uma plataforma consegue implementar corretamente esses quatro métodos, ela já consegue ser conectada ao core da biblioteca.

## 10. Adapters por plataforma

### 10.1 Arduino

O adapter Arduino é um wrapper fino sobre `Stream`, o que permite uso com objetos como:

- `Serial`
- `Serial1`
- `Serial2`
- `HardwareSerial`
- implementações derivadas de `Stream`

Essa escolha é muito boa porque mantém o adapter simples e compatível com o ecossistema Arduino.

### 10.2 Linux

O adapter Linux usa chamadas típicas de ambiente POSIX, como:

- `open()`
- `termios`
- `ioctl(FIONREAD)`
- `clock_gettime(CLOCK_MONOTONIC)`

Isso torna o core utilizável em PCs, SBCs e gateways Linux.

### 10.3 Raspberry Pi

No Raspberry Pi, você pode optar por duas estratégias:

1. usar o core C++ diretamente com o adapter Linux
2. usar a SDK Python `loramesh`, que já replica esse core para Python

As duas abordagens fazem sentido. A escolha depende mais do tipo de aplicação do que da biblioteca em si.

### 10.4 ESP-IDF

A arquitetura já reserva espaço para um adapter dedicado para ESP-IDF. Mesmo estando em standby por enquanto, ele faz parte da direção técnica do projeto.

### 10.5 STM32 HAL

STM32 HAL também já está contemplado na arquitetura. A ideia aqui é seguir a mesma linha: um adapter fino sobre a UART/HAL, reaproveitando integralmente o core C++.

## 11. Inicialização rápida

O uso mais básico da biblioteca em C++ começa com:

1. inicializar a UART da plataforma
2. criar o transport
3. instanciar `LoRaMESH`
4. opcionalmente configurar debug
5. chamar `begin()`

### Exemplo mínimo em Arduino

```cpp
#include "LoRaMESH.h"

#define LORA_RX   16
#define LORA_TX   17
#define LORA_BAUD 9600

ArduinoSerialTransport transportCmd(Serial2);
LoRaMESH lora(&transportCmd, &transportCmd);

static void debugPrint(const char* msg)
{
    Serial.println(msg);
}

void setup()
{
    Serial.begin(115200);

    Serial2.begin(
        LORA_BAUD,
        SERIAL_8N1,
        LORA_RX,
        LORA_TX
    );

    lora.setDebug(debugPrint, true);
    lora.begin(true);
}

void loop()
{
}
```

### Exemplo mínimo em Linux / Raspberry Pi com C++

```cpp
#include "LoRaMESH.h"
#include "adapters/linux/LinuxSerialTransport.h"
#include <iostream>
#include <thread>
#include <chrono>

static void debugPrint(const char* msg)
{
    std::cout << msg << std::endl;
}

int main()
{
    LinuxSerialTransport transportCmd("/dev/ttyACM0", 9600);
    LoRaMESH lora(&transportCmd, &transportCmd);

    lora.setDebug(debugPrint, true);
    lora.begin(true);

    while (!lora.localRead())
    {
        std::cout << "Aguardando modulo..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Modulo pronto" << std::endl;
    return 0;
}
```

## 12. Fluxo recomendado de inicialização

Pelo comportamento atual do core, um fluxo robusto de inicialização costuma seguir esta ordem:

1. iniciar a UART da plataforma
2. criar o adapter correspondente
3. criar a instância de `LoRaMESH`
4. configurar debug, se quiser enxergar TX/RX
5. chamar `begin()`
6. insistir em `localRead()` até o módulo responder
7. validar ou ajustar `NetworkID`
8. consultar a configuração de rádio
9. ajustar BPS se necessário
10. consultar classe atual
11. ajustar classe e janela se necessário
12. ajustar senha, se necessário
13. só então iniciar operações remotas

### Exemplo de função de inicialização robusta em Arduino

```cpp
void initializeRadio()
{
    Serial.println("Aguardando modulo...");

    while (!lora.localRead())
    {
        Serial.println("Modulo nao respondeu ainda...");
        delay(500);
    }

    Serial.println("Modulo pronto");

    if (lora.localId != ID)
    {
        Serial.println("Configurando NetworkID");

        for (int i = 0; i < 3; i++)
        {
            if (lora.setNetworkID(ID))
                break;

            delay(200);
        }
    }

    for (int i = 0; i < 3; i++)
        if (lora.getBPS())
            break;

    if (lora.BW != BW500 || lora.SF != SF7 || lora.CR != CR4_5)
    {
        Serial.println("Configurando BPS");

        for (int i = 0; i < 3; i++)
            if (lora.setBPS(BW500, SF7, CR4_5))
                break;
    }

    for (int i = 0; i < 3; i++)
        if (lora.getClass())
            break;

    if (lora.LoRa_class != CLASS_C || lora.LoRa_window != WINDOW_5s)
    {
        Serial.println("Configurando classe");

        for (int i = 0; i < 3; i++)
            if (lora.setClass(CLASS_C, WINDOW_5s))
                break;
    }

    if (lora.registered_password != 123)
    {
        Serial.println("Configurando senha");

        for (int i = 0; i < 3; i++)
            if (lora.setPassword(123))
                break;
    }

    Serial.println("Configuracao concluida");
}
```

## 13. Debug da comunicação

### 13.1 Registrando debug no core C++

O debug é configurado por:

```cpp
void setDebug(DebugFn fn, bool enable_hex_dump)
```

Exemplo:

```cpp
static void debugPrint(const char* msg)
{
    Serial.println(msg);
}

lora.setDebug(debugPrint, true);
```

### 13.2 O que `begin(true)` realmente faz

Na implementação atual, `begin(true)` faz basicamente duas coisas:

- habilita o dump hexadecimal
- chama `localRead()` uma vez

Isso significa que:

- `begin(true)` não substitui um loop robusto com `while(!localRead())`
- sem callback configurado, nada será impresso
- o comportamento seguro ainda é validar o módulo explicitamente

### 13.3 Prefixos usados no debug

O core usa prefixos como:

```text
TX CMD:
RX CMD:
TX TRP:
RX TRP:
```

Isso é muito útil porque permite visualizar:

- o frame real enviado
- a resposta real recebida
- o fluxo de comando separado do fluxo transparente
- diferenças entre o que a aplicação pretende enviar e o que de fato foi serializado

## 14. Como o core monta e recebe frames

A biblioteca separa internamente a lógica entre:

- `prepareFrameCommand()`
- `PrepareFrameTransp()`
- `sendPacket()`
- `receivePacketCommand()`
- `receivePacketTransp()`

### 14.1 Frame de comando

Na prática, o frame de comando segue esta ideia:

```text
[id_low][id_high][cmd][payload ...][crc_low][crc_high]
```

### 14.2 Frame transparente

No modo transparente, a ideia é esta:

```text
[id_low][id_high][payload ...]
```

### 14.3 O que isso mostra sobre a arquitetura

O core já foi desenhado para tratar comando e transparente como fluxos distintos, inclusive com possibilidade de transports separados. Isso é uma das partes mais fortes do projeto.

## 15. CRC utilizado pela biblioteca

O CRC usado no core atual é calculado com:

- seed inicial `0xC181`
- operação XOR byte a byte
- polinômio aplicado como `0xA001` no bit menos significativo

Isso importa porque:

- frames inválidos são descartados
- respostas corrompidas não devem ser tratadas como válidas
- o debug hexadecimal ajuda a entender quando a falha está no framing e não na lógica de alto nível

## 16. Configuração de rede

A configuração de rede é uma das primeiras etapas depois que o módulo responde corretamente.

### 16.1 Definição do `NetworkID`

```cpp
lora.setNetworkID(0);
```

No core atual, IDs acima de `2047` são rejeitados.

Na prática, é comum usar:

- `0` para gateway, master ou nó principal
- outros valores para nós remotos conforme a organização da rede

### 16.2 Definição de senha

```cpp
lora.setPassword(123);
```

A senha faz parte da identidade operacional do módulo na rede.

### 16.3 Observação importante sobre a senha no estado atual do core

Pelo comportamento observado no código atual, a verificação de `setPassword()` reconstrói a comparação usando a parte de 16 bits refletida na leitura local. Na prática, os exemplos atuais fazem mais sentido com senhas de até 16 bits.

Por isso, o exemplo de Arduino usa:

```cpp
123
```

como valor simples e previsível.

## 17. Configuração de rádio

A configuração de rádio define alcance, robustez, taxa de transmissão e latência da rede.

### 17.1 Leitura da configuração atual

```cpp
if (lora.getBPS())
{
    Serial.println("BW=" + String(lora.BW));
    Serial.println("SF=" + String(lora.SF));
    Serial.println("CR=" + String(lora.CR));
}
```

A chamada `getBPS()` atualiza internamente os campos:

- `BW`
- `SF`
- `CR`

### 17.2 Definição de configuração

```cpp
lora.setBPS(BW500, SF7, CR4_5);
```

### 17.3 Interpretação prática

- bandwidth maior tende a aumentar taxa de transmissão
- spreading factor maior tende a aumentar robustez e tempo no ar
- coding rate maior tende a aumentar redundância e robustez

### 17.4 Validações feitas pelo core

O core atual rejeita:

- bandwidth acima de `BW500`
- spreading factor fora de `SF_FSK` ou `SF7` a `SF12`
- coding rate fora de `CR4_5` a `CR4_8`

## 18. Classe LoRa e janela de recepção

### 18.1 Leitura da classe atual

```cpp
if (lora.getClass())
{
    Serial.println("Class=" + String(lora.LoRa_class));
    Serial.println("Window=" + String(lora.LoRa_window));
}
```

### 18.2 Definição da classe

```cpp
lora.setClass(CLASS_C, WINDOW_5s);
```

### 18.3 Interpretação prática

- `CLASS_A` tende a ser mais econômica
- `CLASS_C` mantém recepção contínua e é útil em sistemas energizados continuamente
- a janela define o comportamento temporal implementado no módulo conforme os valores aceitos pelo core

### 18.4 Validações feitas pelo core

No estado atual da implementação:

- apenas `CLASS_A` e `CLASS_C` são aceitas
- janelas acima de `WINDOW_15s` são rejeitadas

## 19. Constantes principais

Assim como no README da camada Python, vale documentar as principais constantes de forma explícita.

### 19.1 Bandwidth

```cpp
BW125 = 0x00
BW250 = 0x01
BW500 = 0x02
```

Leitura prática:

- `BW125` privilegia robustez relativa e menor taxa
- `BW250` fica no meio-termo
- `BW500` privilegia maior taxa de transmissão

### 19.2 Spreading Factor

```cpp
SF_FSK = 0x00

SF7  = 0x07
SF8  = 0x08
SF9  = 0x09
SF10 = 0x0A
SF11 = 0x0B
SF12 = 0x0C
```

Leitura prática:

- `SF7` tende a ser mais rápido
- `SF12` tende a ser mais robusto e mais lento
- valores intermediários servem para equilibrar alcance e tempo no ar
- `SF_FSK` representa operação em FSK, não em LoRa tradicional

### 19.3 Coding Rate

```cpp
CR4_5 = 0x01
CR4_6 = 0x02
CR4_7 = 0x03
CR4_8 = 0x04
```

Leitura prática:

- `CR4_5` tende a ser mais eficiente
- `CR4_8` tende a ser mais robusto

### 19.4 Classe

```cpp
CLASS_A = 0x00
CLASS_C = 0x02
```

### 19.5 Janela

```cpp
WINDOW_5s  = 0x00
WINDOW_10s = 0x01
WINDOW_15s = 0x02
```

### 19.6 Pull de GPIO

```cpp
LNOT_PULL = 0x00
LPULLUP   = 0x01
LPULLDOWN = 0x02
```

### 19.7 Modos de IO

```cpp
INOUT_DIGITAL_INPUT  = 0x00
INOUT_DIGITAL_OUTPUT = 0x01
INOUT_ANALOG_INPUT   = 0x03
```

### 19.8 Compatibilidade com estilo Arduino

O core também aceita uma forma de uso mais próxima de Arduino:

```cpp
INPUT
OUTPUT
INPUT_PULLUP
INPUT_PULLDOWN
```

Internamente, esses aliases são convertidos para os modos e pulls corretos.

## 20. Informações e estados mantidos pelo core

Uma característica importante da biblioteca atual é que ela mantém alguns estados que podem ser lidos após operações bem-sucedidas.

Entre os mais úteis no uso prático estão:

- `localId`
- `localUniqueId`
- `registered_password`
- `BW`
- `SF`
- `CR`
- `LoRa_class`
- `LoRa_window`

Esses campos são úteis para:

- debug
- verificação de provisionamento
- logs de inventário
- decisão condicional na inicialização
- confirmação do estado real do módulo após leitura

### Exemplo de leitura desses estados

```cpp
if (lora.localRead())
{
    Serial.println("Local ID: " + String(lora.localId));
    Serial.println("Password: " + String(lora.registered_password));
    Serial.println("UID: " + String((unsigned long)lora.localUniqueId));
}
```

## 21. Controle remoto de GPIO

Um dos maiores diferenciais do projeto é permitir tratar um nó remoto como extensão física do sistema principal.

### 21.1 Configuração de pino

Antes de ler ou escrever um GPIO remoto, ele precisa ser configurado.

```cpp
lora.pinMode(1, 3, OUTPUT);
```

Ou, de forma mais explícita com constantes internas:

```cpp
lora.pinMode(1, 3, INOUT_DIGITAL_OUTPUT);
```

### 21.2 Escrita digital remota

```cpp
lora.digitalWrite(1, 3, 1);
lora.digitalWrite(1, 3, 0);
```

Interpretação:

- primeiro parâmetro: ID do nó remoto
- segundo parâmetro: GPIO remoto
- terceiro parâmetro: nível lógico

### 21.3 Leitura digital remota

```cpp
uint8_t estado = lora.digitalRead(1, 4);
```

### 21.4 Controle direto de slaves

Você pode atuar diretamente em qualquer nó remoto conhecido pelo ID.

```cpp
lora.digitalWrite(2, 0, 1);
```

Nesse caso:

- `2` é o ID do slave
- `0` é o GPIO
- `1` representa nível alto

### 21.5 Observação importante sobre validação

No estado atual do core:

- `gpio > 0x07` é rejeitado
- `digitalWrite()` tenta confirmar a escrita verificando o retorno do módulo

### 21.6 Particularidade dos GPIOs 5 e 6

Existe uma nuance importante no core atual: quando determinados fluxos configuram os GPIOs 5 e 6 como entrada, o comportamento passa a ser tratado como entrada analógica e `analogEnabled` é ativado.

Isso impacta diretamente:

- `analogRead()`
- `digitalRead()` nesses canais
- `digitalWrite()`, que passa a ser bloqueado nesses GPIOs quando o modo analógico estiver ativo

Essa é uma característica importante do estado atual da biblioteca e precisa ser conhecida por quem vai usar IO remoto de forma mais avançada.

## 22. Leitura analógica e medições auxiliares

### 22.1 Leitura analógica remota

```cpp
uint16_t adc = lora.analogRead(1, 5);
```

A resposta é um valor bruto do ADC reconstruído a partir dos bytes recebidos do módulo.

### 22.2 Leitura digital derivada de analógico nos canais híbridos

Quando `analogEnabled` está ativo e o canal cai na regra dos GPIOs analógicos, `digitalRead()` usa `analogRead()` e aplica um limiar de meia escala.

Na prática, isso significa que a leitura digital nesses canais é derivada da leitura analógica.

### 22.3 Cálculo de resistência

```cpp
uint16_t adc = lora.analogRead(1, 5);
int r1 = lora.getR1(adc, 10000);
```

Essa função é útil quando o ADC está lendo um divisor resistivo.

### 22.4 Cálculo de temperatura com NTC

```cpp
uint16_t adc = lora.analogRead(1, 5);
double temp = lora.getTemp(adc, 3950, 10000, 10000);
```

Essa função é útil para sensores resistivos baseados em NTC.

### 22.5 Comportamentos especiais do core

No estado atual da implementação:

- `getR1(0, R2)` retorna um valor muito alto como sentinela
- `getR1(rawADC >= 4096, R2)` retorna `0`
- `getTemp()` retorna um valor próximo de `-273.15` quando a resistência calculada é inválida

## 23. Modo transparente e dupla UART

Mesmo que muitos exemplos atuais estejam mais focados em comandos, o core já foi desenhado para separar claramente:

- fluxo de comando
- fluxo transparente

### 23.1 Dois transports separados

A arquitetura permite:

```cpp
LoRaMESH lora(&transportCmd, &transportTransp);
```

### 23.2 Mesmo transport para os dois fluxos

Também é possível usar o mesmo objeto nos dois papéis:

```cpp
LoRaMESH lora(&transportCmd, &transportCmd);
```

Foi exatamente essa a abordagem usada no exemplo Arduino atual.

### 23.3 O que isso abre de possibilidade

- uma UART dedicada a configuração e gerenciamento
- outra UART dedicada a payload transparente
- debug mais limpo por fluxo
- arquiteturas em que gateway e módulo convivem em dois modos distintos

### 23.4 Observação importante para quem for expandir o projeto

Mesmo que sua aplicação inicial use só o canal de comando, vale manter em mente que a arquitetura já foi preparada para crescimento em direção ao modo transparente.

## 24. Referência detalhada da API

Abaixo está a referência organizada com base no comportamento observado no core atual.

### 24.1 Construtor

```cpp
LoRaMESH(ILoraTransport* cmdTransport, ILoraTransport* transpTransport)
```

Descrição:

- cria a instância principal da biblioteca
- associa o transporte de comandos
- associa o transporte de modo transparente

Uso típico quando ambos compartilham a mesma UART:

```cpp
ArduinoSerialTransport transportCmd(Serial2);
LoRaMESH lora(&transportCmd, &transportCmd);
```

### 24.2 `setDebug()`

```cpp
void setDebug(DebugFn fn, bool enable_hex_dump)
```

Descrição:

- registra a função callback de debug
- habilita ou desabilita dump hexadecimal

### 24.3 `begin()`

```cpp
void begin(bool debug)
```

Descrição:

- habilita dump hexadecimal quando `debug == true`
- chama `localRead()` uma vez internamente

Observação importante:

- isso não substitui um `while(!localRead())`
- isso não imprime nada sem callback registrado

### 24.4 `localRead()`

```cpp
bool localRead()
```

Descrição:

- envia o comando de leitura local
- atualiza `localId`
- atualiza `registered_password`
- atualiza `localUniqueId`

### 24.5 `setNetworkID()`

```cpp
bool setNetworkID(uint16_t id)
```

Descrição:

- configura o `NetworkID` lógico do módulo
- rejeita IDs acima de `2047`

### 24.6 `setPassword()`

```cpp
bool setPassword(uint32_t password)
```

Descrição:

- envia o comando de configuração de senha
- executa `localRead()` após a escrita
- compara o valor retornado para verificar consistência

### 24.7 `getBPS()`

```cpp
bool getBPS(bool ignore_cmd = false)
```

Descrição:

- consulta a configuração atual de rádio
- atualiza `BW`, `SF` e `CR`

### 24.8 `setBPS()`

```cpp
bool setBPS(uint8_t bandwidth, uint8_t spreading_factor, uint8_t coding_rate)
```

Descrição:

- valida os parâmetros
- envia a configuração
- chama `getBPS(true)`
- confirma se o estado interno corresponde ao solicitado

### 24.9 `getClass()`

```cpp
bool getClass(bool ignore_cmd = false)
```

Descrição:

- consulta classe e janela
- atualiza `LoRa_class` e `LoRa_window`

### 24.10 `setClass()`

```cpp
bool setClass(uint8_t lora_class, uint8_t lora_window)
```

Descrição:

- valida classe e janela
- envia o comando
- chama `getClass(true)`
- confirma se o estado interno corresponde ao solicitado

### 24.11 `pinMode()`

```cpp
bool pinMode(uint16_t id, uint8_t gpio, uint8_t inout, uint8_t logical_level = 0)
```

Descrição:

- configura o comportamento de um pino remoto
- aceita aliases estilo Arduino e constantes internas do protocolo

### 24.12 `digitalWrite()`

```cpp
bool digitalWrite(int16_t id, uint8_t gpio, uint8_t logical_level)
```

Descrição:

- escreve em um GPIO remoto
- tenta confirmar a escrita com base no retorno do módulo
- internamente já executa mais de uma tentativa de confirmação

### 24.13 `digitalRead()`

```cpp
uint8_t digitalRead(int16_t id, uint8_t gpio)
```

Descrição:

- consulta o estado lógico de um pino remoto
- pode derivar a leitura a partir do ADC nos canais híbridos

### 24.14 `analogRead()`

```cpp
uint16_t analogRead(int16_t id, uint8_t gpio)
```

Descrição:

- consulta o valor analógico remoto
- tenta múltiplas leituras antes de desistir

### 24.15 `getNoise()`

```cpp
int getNoise(uint16_t id, uint8_t select)
```

Descrição:

- consulta ruído do nó
- aceita `select` até `2`
- retorna `255` em caso de erro

### 24.16 `getR1()`

```cpp
int getR1(uint16_t rawADC, int R2)
```

Descrição:

- calcula resistência equivalente a partir de um divisor resistivo

### 24.17 `getTemp()`

```cpp
double getTemp(uint16_t rawADC, int beta, int Rt, int R2)
```

Descrição:

- estima temperatura a partir do ADC e dos parâmetros do NTC

## 25. Exemplos completos

Esta seção é uma das mais importantes do projeto porque a biblioteca foi desenhada para existir em mais de um ambiente.

### 25.1 Exemplo mínimo funcional em Arduino / ESP32 Arduino

```cpp
#include "LoRaMESH.h"

#define LORA_RX   16
#define LORA_TX   17
#define LORA_BAUD 9600

ArduinoSerialTransport transportCmd(Serial2);
LoRaMESH lora(&transportCmd, &transportCmd);

static void debugPrint(const char* msg)
{
    Serial.println(msg);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial2.begin(
        LORA_BAUD,
        SERIAL_8N1,
        LORA_RX,
        LORA_TX
    );

    lora.setDebug(debugPrint, true);
    lora.begin(true);

    while (!lora.localRead())
    {
        Serial.println("Aguardando modulo...");
        delay(500);
    }

    Serial.println("Modulo pronto");
}

void loop()
{
}
```

### 25.2 Exemplo lendo informações locais do módulo

```cpp
if (lora.localRead())
{
    Serial.println("Local ID: " + String(lora.localId));
    Serial.println("Password: " + String(lora.registered_password));
    Serial.println("UID: " + String((unsigned long)lora.localUniqueId));
}
```

### 25.3 Exemplo configurando rede e rádio

```cpp
void setupRadio()
{
    if (lora.localId != 0)
        lora.setNetworkID(0);

    lora.setPassword(123);
    lora.setBPS(BW500, SF7, CR4_5);
    lora.setClass(CLASS_C, WINDOW_5s);

    lora.getBPS();
    lora.getClass();

    Serial.println("BW=" + String(lora.BW));
    Serial.println("SF=" + String(lora.SF));
    Serial.println("CR=" + String(lora.CR));
    Serial.println("CLASS=" + String(lora.LoRa_class));
    Serial.println("WINDOW=" + String(lora.LoRa_window));
}
```

### 25.4 Exemplo controlando uma saída remota

```cpp
void setup(){
    ...
    lora.pinMode(1, 0, INOUT_DIGITAL_OUTPUT);
}

void loop()
{
    Serial.println("Ligando GPIO0 do slave 1");
    lora.digitalWrite(1, 0, 1);
    delay(1000);

    Serial.println("Desligando GPIO0 do slave 1");
    lora.digitalWrite(1, 0, 0);
    delay(1000);
}
```

### 25.5 Exemplo lendo um pino digital remoto

```cpp
void setup(){
    ...
    lora.pinMode(1, 4, INOUT_DIGITAL_INPUT);
}

void loop()
{
    uint8_t estado = lora.digitalRead(1, 4);
    Serial.println("Estado do GPIO4 remoto: " + String(estado));
    delay(1000);
}
```

### 25.6 Exemplo lendo canal analógico remoto

```cpp
void setup(){
    ...
    lora.pinMode(1, 5, INOUT_ANALOG_INPUT);
}

void loop()
{
    uint16_t adc = lora.analogRead(1, 5);
    Serial.println("ADC remoto: " + String(adc));
    delay(1000);
}
```

### 25.7 Exemplo calculando resistência e temperatura

```cpp
void setup(){
    ...
    lora.pinMode(1, 5, INOUT_ANALOG_INPUT);
}

void loop()
{
    uint16_t adc = lora.analogRead(1, 5);
    int r1 = lora.getR1(adc, 10000);
    double temp = lora.getTemp(adc, 3950, 10000, 10000);

    Serial.println("ADC: " + String(adc));
    Serial.println("R1: " + String(r1));
    Serial.println("Temp: " + String(temp));
    delay(1000);
}
```

### 25.8 Exemplo de blink em múltiplos slaves

Este exemplo segue a mesma linha do material atual do projeto: um master alterna o estado do GPIO 0 de múltiplos nós remotos.

```cpp
#include "LoRaMESH.h"

#define LORA_RX                 16
#define LORA_TX                 17
#define LORA_BAUD               9600

#define MAX_REPLICATE_IN_SLAVES 3
#define DELAY_BETWEEN_SLAVES    250
#define MAX_RETRY               3

uint8_t ID = 0;

ArduinoSerialTransport transportCmd(Serial2);
LoRaMESH lora(&transportCmd, &transportCmd);

static void debugPrint(const char* msg)
{
  Serial.println(msg);
}

void initializeRadio()
{
  Serial.println("Aguardando modulo...");

  while(!lora.localRead())
  {
    Serial.println("Modulo nao respondeu ainda...");
    delay(500);
  }

  Serial.println("Modulo pronto");

  if(lora.localId != ID)
  {
    Serial.println("Configurando NetworkID");

    for(int i=0;i<3;i++)
    {
      if(lora.setNetworkID(ID))
        break;

      delay(200);
    }
  }

  for(int i=0;i<3;i++)
    if(lora.getBPS())
      break;

  if(lora.BW != BW500 || lora.SF != SF7 || lora.CR != CR4_5)
  {
    Serial.println("Configurando BPS");

    for(int i=0;i<3;i++)
      if(lora.setBPS(BW500,SF7,CR4_5))
        break;
  }

  for(int i=0;i<3;i++)
    if(lora.getClass())
      break;

  if(lora.LoRa_class != CLASS_C || lora.LoRa_window != WINDOW_5s)
  {
    Serial.println("Configurando classe");

    for(int i=0;i<3;i++)
      if(lora.setClass(CLASS_C,WINDOW_5s))
        break;
  }

  if(lora.registered_password != 123)
  {
    Serial.println("Configurando senha");

    for(int i=0;i<3;i++)
      if(lora.setPassword(123))
        break;
  }

  Serial.println("Configuracao concluida");
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("Inicializando LoRaMesh...");

  Serial2.begin(
      LORA_BAUD,
      SERIAL_8N1,
      LORA_RX,
      LORA_TX
  );

  lora.setDebug(debugPrint, true);
  lora.begin(true);

  initializeRadio();

  Serial.println("Master pronto");
}

void loop()
{
  static bool state = false;

  state = !state;

  for (uint16_t i = 1; i <= MAX_REPLICATE_IN_SLAVES; ++i)
  {
    for (uint8_t j = 0; j < MAX_RETRY; ++j)
    {
      if (lora.digitalWrite(i, 0, state ? 1 : 0))
        break;
      else
        delay(1000 + random(500, 2500));
    }

    if (state)
      Serial.println("GPIO0 SLAVE:" + String(i) + "=ON");
    else
      Serial.println("GPIO0 SLAVE:" + String(i) + "=OFF");

    if (MAX_REPLICATE_IN_SLAVES > 1)
      delay(DELAY_BETWEEN_SLAVES);
  }

  if (MAX_REPLICATE_IN_SLAVES == 1)
    delay(DELAY_BETWEEN_SLAVES);
}
```

### 25.9 Exemplo em Linux / Raspberry Pi usando C++ diretamente

```cpp
#include "LoRaMESH.h"
#include "adapters/linux/LinuxSerialTransport.h"
#include <iostream>
#include <thread>
#include <chrono>

static void debugPrint(const char* msg)
{
    std::cout << msg << std::endl;
}

int main()
{
    LinuxSerialTransport transportCmd("/dev/ttyACM0", 9600);
    LoRaMESH lora(&transportCmd, &transportCmd);

    lora.setDebug(debugPrint, true);
    lora.begin(true);

    while (!lora.localRead())
    {
        std::cout << "Aguardando modulo..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (lora.localId != 0)
        lora.setNetworkID(0);

    lora.setPassword(123);
    lora.setBPS(BW500, SF7, CR4_5);
    lora.setClass(CLASS_C, WINDOW_5s);

    std::cout << "Modulo pronto" << std::endl;
    return 0;
}
```

### 25.10 Exemplo em Raspberry Pi usando a SDK Python do mesmo ecossistema

Mesmo este sendo o README do core C++, vale documentar o uso natural em Raspberry Pi através da camada Python, já que ela existe exatamente para reaproveitar este core em um ambiente mais produtivo para gateways e automações.

```python
import loramesh
import time
import random

PORT = "/dev/ttyACM0"
BAUD = 9600

ID = 0

MAX_REPLICATE_IN_SLAVES = 3
DELAY_BETWEEN_SLAVES = 0.250
MAX_RETRY = 3

BW500 = 0x02
SF7 = 0x07
CR4_5 = 0x01

CLASS_C = 0x02
WINDOW_5s = 0x00

PASSWORD = 123


def initialize_radio(mesh):
    print("Aguardando modulo...")

    while not mesh.localRead():
        print("Modulo nao respondeu ainda...")
        time.sleep(0.5)

    print("Modulo pronto")
    if mesh.getLocalID() != ID:
        print("Configurando NetworkID")
        for _ in range(3):
            if mesh.setNetworkID(ID):
                break
            time.sleep(0.2)

    for _ in range(3):
        if mesh.getBPS():
            break
    if mesh.getBW() != BW500 or mesh.getSF() != SF7 or mesh.getCR() != CR4_5:
        print("Configurando BPS")
        for _ in range(3):
            if mesh.setBPS(BW500, SF7, CR4_5):
                break

    for _ in range(3):
        if mesh.getClass():
            break

    if mesh.getClassValue() != CLASS_C or mesh.getWindow() != WINDOW_5s:
        print("Configurando classe")
        for _ in range(3):
            if mesh.setClass(CLASS_C, WINDOW_5s):
                break

    if mesh.setPassword(PASSWORD):
        print("Senha configurada")
    print("Configuracao concluida")


def main():
    print("Inicializando LoRaMesh...")
    mesh = loramesh.LoRaMESH(PORT, BAUD)
    mesh.begin(True, True)
    initialize_radio(mesh)
    print("Master pronto")
    state = False

    while True:
        state = not state
        for i in range(1, MAX_REPLICATE_IN_SLAVES + 1):
            for j in range(MAX_RETRY):
                if mesh.digitalWrite(i, 0, 1 if state else 0):
                    break
                else:
                    delay = 1 + random.uniform(0.5, 2.5)
                    time.sleep(delay)

            if state:
                print(f"GPIO0 SLAVE:{i}=ON")
            else:
                print(f"GPIO0 SLAVE:{i}=OFF")
            if MAX_REPLICATE_IN_SLAVES > 1:
                time.sleep(DELAY_BETWEEN_SLAVES)

        if MAX_REPLICATE_IN_SLAVES == 1:
            time.sleep(DELAY_BETWEEN_SLAVES)


if __name__ == "__main__":
    main()
```

### 25.11 Exemplo reservado para ESP-IDF

Esta seção fica reservada para a integração futura com ESP-IDF. Quando essa frente estiver consolidada, o ideal é documentar aqui:

- inicialização da UART no ESP-IDF
- criação do adapter da plataforma
- callback de debug
- laço principal de leitura e escrita
- exemplos equivalentes aos de Arduino

### 25.12 Exemplo reservado para STM32 HAL

Esta seção fica reservada para a integração futura com STM32 HAL. Quando essa parte estiver consolidada, o ideal é documentar aqui:

- inicialização da UART via HAL
- adapter sobre `UART_HandleTypeDef`
- integração de `millis()` ou time base equivalente
- callback de debug
- exemplos equivalentes aos de Arduino

## 26. Boas práticas

### 26.1 Sempre aguarde `localRead()` responder

Esse é o ponto mais importante para evitar inicialização falsa.

### 26.2 Não envie comandos em sequência sem respirar a rede

Pequenos delays ajudam bastante, principalmente em redes reais.

### 26.3 Leia o estado atual antes de sobrescrever tudo

Consultar `BW`, `SF`, `CR`, `LoRa_class`, `LoRa_window` e `registered_password` ajuda muito em diagnóstico e provisionamento.

### 26.4 Use retry quando a operação for importante

Principalmente em acionamento remoto de GPIO.

### 26.5 Deixe o debug ativo durante desenvolvimento

Em sistemas seriais e de rádio, enxergar o TX/RX bruto economiza muito tempo.

### 26.6 Documente a configuração real usada em campo

Registre sempre:

- baudrate
- `NetworkID`
- senha
- `BW`
- `SF`
- `CR`
- classe
- janela
- IDs dos nós

### 26.7 Em Linux, trate o adapter atual como 9600 até nova revisão

No estado atual do código, essa é a abordagem mais segura para evitar diagnóstico enganoso.

## 27. Erros comuns e diagnóstico

### 27.1 `localRead()` nunca responde

Verifique:

- porta serial correta
- alimentação do módulo
- baudrate real do enlace
- permissões de acesso à serial
- fiação UART
- callback de debug configurado corretamente
- se o módulo realmente inicializou

### 27.2 O comando de escrita remota não parece funcionar

Verifique:

- se o `device_id` está correto
- se o pino foi configurado antes com `pinMode()`
- se o nó remoto está ativo na rede
- se os dois lados têm configuração de rádio compatível
- se a latência natural da rede está sendo respeitada

### 27.3 A leitura analógica parece incoerente

Verifique:

- o canal real usado
- se o GPIO entrou em fluxo analógico
- resolução efetiva do ADC do módulo
- circuito ligado ao pino
- valores usados em `getR1()` e `getTemp()`

### 27.4 O comportamento dos GPIOs 5 e 6 parece estranho

Verifique se o fluxo ativou `analogEnabled`. No estado atual do core, isso altera o comportamento de leitura e bloqueia escrita nesses canais quando passam a ser tratados como analógicos.

### 27.5 O debug não imprime nada

Verifique:

- se `setDebug()` foi chamado
- se a função callback realmente imprime a string recebida
- se `begin(true)` foi usado quando você deseja dump hexadecimal

## 28. ESP-IDF e STM32 HAL

Estas duas plataformas já fazem parte da direção do projeto e não devem ficar invisíveis no README, mesmo estando em standby por enquanto.

### 28.1 ESP-IDF

A integração esperada com ESP-IDF deve seguir a mesma filosofia do restante do projeto:

- adapter fino sobre UART do framework
- implementação de `write()`, `available()`, `read()` e `millis()`
- reaproveitamento integral do core `LoRaMESH`
- exemplos equivalentes aos de Arduino

### 28.2 STM32 HAL

Para STM32 HAL, a ideia é exatamente a mesma:

- adapter fino sobre HAL UART
- time base compatível para `millis()`
- reaproveitamento do core sem reinventar o protocolo
- exemplos equivalentes aos de Arduino

### 28.3 Por que reservar essas seções desde já

Porque o projeto já foi desenhado com essa ambição multiplataforma. Deixar isso explícito no README ajuda a manter o direcionamento do repositório coeso mesmo antes da finalização dessas integrações.

## 29. Casos de uso reais

O valor desta biblioteca aparece quando ela sai do exemplo simples e entra em aplicações reais. Alguns cenários muito naturais para o projeto são:

- automação rural com bombas e válvulas
- leitura remota de sensores analógicos e digitais
- monitoramento distribuído em áreas sem infraestrutura tradicional de rede
- gateways Linux ou Raspberry Pi para provisionamento e supervisão
- controle de saídas remotas em nós distribuídos
- telemetria de baixa taxa em redes privadas
- prototipagem rápida com a camada Python sobre o mesmo core
- ferramentas de manutenção, diagnóstico e inventário de módulos

## 30. Licença
MIT License

## 31. Contato e referências
### 31.1 Desenvolvedor da biblioteca
Autor: Gustavo Cereza
- GitHub: [https://github.com/GustavoCereza](https://github.com/GustavoCereza)  
- LinkedIn: [https://www.linkedin.com/in/gustavo-cereza/](https://www.linkedin.com/in/gustavo-cereza/)  
- Site: [https://elcereza.com](https://elcereza.com)

### 31.2 Fabricante do módulo
Empresa: Radioenge
- Site: [https://www.radioenge.com.br](https://www.radioenge.com.br)
- Documentação e outras informações: [https://www.radioenge.com.br/produto/modulo-loramesh/](https://www.radioenge.com.br/produto/modulo-loramesh/)
- Produto LoRaMesh: [https://meli.la/2dSTW3C](https://meli.la/2dSTW3C)

## 32. Considerações finais
O **LoRaMESH C++** não foi pensado como uma biblioteca superficial. Ele existe para ser a base técnica real do projeto, concentrando a lógica de protocolo, configuração, leitura de estado do módulo, controle remoto de IO e reaproveitamento entre plataformas.

Essa é justamente a força da biblioteca: você mantém uma base nativa sólida e coerente, mas consegue expandir esse mesmo núcleo para Arduino, Linux, Raspberry Pi, ESP-IDF, STM32 HAL e Python sem reinventar a lógica principal a cada novo ambiente.

Se a intenção é construir aplicações reais com módulos LoRaMESH, controlar nós remotos, ler sensores, acionar GPIOs, provisionar dispositivos e integrar firmware com gateways ou automações, este core C++ é o ponto central do projeto.