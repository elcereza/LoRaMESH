# LoRaMESH MicroPython

Biblioteca **LoRaMESH para MicroPython** voltada para comunicação com módulos **LoRaMESH (Radioenge)** em ambientes embarcados leves, principalmente quando a ideia é subir testes rápidos, integrações simples, prototipagem e experimentação diretamente em placas compatíveis com MicroPython.

Este port existe para levar a proposta do ecossistema LoRaMESH para um ambiente mais leve e mais direto de testar em hardware, mantendo a mesma linha conceitual do projeto principal:

- o **core nativo em C++** continua sendo a base técnica do ecossistema
- a **SDK Python para Linux/Raspberry Pi** funciona como camada de alto nível
- a **variação MicroPython** tenta levar essa mesma ideia para microcontroladores compatíveis com runtime interpretado

Em outras palavras, esta biblioteca não nasce como um projeto isolado. Ela faz parte da mesma família do **LoRaMESH C++** e do **LoRaMESH Python**, mas com foco em ambientes mais limitados, onde praticidade de teste e deploy contam muito.

## Aviso importante sobre o estado deste port

Este README precisa ser transparente sobre um ponto importante: **a variação MicroPython ainda deve ser tratada como experimental**.

Eu não sou uma pessoa que trabalha com MicroPython como base principal de desenvolvimento. Minha experiência com esse ecossistema ainda é relativamente baixa, principalmente se comparada ao meu trabalho com C, C++, firmware embarcado e integração em hardware real.

Na prática, meu contato com MicroPython veio muito mais por dois caminhos:

- uso no **Sipeed MaixBit**
- produção de conteúdo e experimentação para o site envolvendo **Raspberry Pi Pico**

Por isso, este port pode apresentar:

- pontos ainda pouco refinados
- API ainda em evolução
- comportamento não tão otimizado quanto a base em C++
- limitações específicas do runtime MicroPython
- incompatibilidades que só aparecem em determinados firmwares ou placas

Então a melhor leitura possível é esta: **o projeto é útil, promissor e funcional para vários cenários, mas ainda não deve ser tratado com a mesma maturidade do core em C++**.

## Direção do projeto

Neste momento, o foco prático do port está em **ESP32 com MicroPython**, porque é um caminho mais natural para validar a proposta, testar UART com facilidade e subir exemplos de forma rápida.

Ao mesmo tempo, existe intenção de portabilidade futura para:

- **Raspberry Pi Pico**
- **Sipeed MaixBit**

Essas duas plataformas são especialmente relevantes para o projeto por motivos diferentes:

- o **Raspberry Pi Pico** é uma plataforma muito acessível, popular e excelente para conteúdo prático
- o **MaixBit da Sipeed** tem valor estratégico no laboratório por já ter sido usado em experimentações e por abrir espaço para aplicações mais específicas

Ainda assim, essa portabilidade futura deve ser lida como **direção do projeto**, não como promessa de compatibilidade fechada e estável neste exato momento.

## Sumário

1. [Visão geral](#1-visão-geral)
2. [Por que este port existe](#2-por-que-este-port-existe)
3. [Estado atual e transparência sobre estabilidade](#3-estado-atual-e-transparência-sobre-estabilidade)
4. [Relação com o ecossistema LoRaMESH](#4-relação-com-o-ecossistema-loramesh)
5. [Plataformas atuais e direção futura](#5-plataformas-atuais-e-direção-futura)
6. [Quando usar esta versão](#6-quando-usar-esta-versão)
7. [Quando não usar esta versão](#7-quando-não-usar-esta-versão)
8. [Arquitetura conceitual da solução](#8-arquitetura-conceitual-da-solução)
9. [Conceitos importantes antes de começar](#9-conceitos-importantes-antes-de-começar)
10. [Preparando o ESP32 com MicroPython](#10-preparando-o-esp32-com-micropython)
11. [Instalando ferramentas no host](#11-instalando-ferramentas-no-host)
12. [Organizando o projeto para upload](#12-organizando-o-projeto-para-upload)
13. [Subindo a biblioteca para a placa com ampy](#13-subindo-a-biblioteca-para-a-placa-com-ampy)
14. [Atualizando apenas o `main.py`](#14-atualizando-apenas-o-mainpy)
15. [Removendo arquivos antigos da placa](#15-removendo-arquivos-antigos-da-placa)
16. [Arquitetura esperada da biblioteca em MicroPython](#16-arquitetura-esperada-da-biblioteca-em-micropython)
17. [Constantes principais](#17-constantes-principais)
18. [Inicialização rápida](#18-inicialização-rápida)
19. [Fluxo recomendado de inicialização](#19-fluxo-recomendado-de-inicialização)
20. [Configuração de rede](#20-configuração-de-rede)
21. [Configuração de rádio](#21-configuração-de-rádio)
22. [Classe LoRa e janela de recepção](#22-classe-lora-e-janela-de-recepção)
23. [Controle remoto de GPIO](#23-controle-remoto-de-gpio)
24. [Leitura analógica e medições auxiliares](#24-leitura-analógica-e-medições-auxiliares)
25. [Comunicação transparente entre nós](#25-comunicação-transparente-entre-nós)
26. [Exemplos completos](#26-exemplos-completos)
27. [Boas práticas](#27-boas-práticas)
28. [Limitações atuais](#28-limitações-atuais)
29. [Erros comuns e diagnóstico](#29-erros-comuns-e-diagnóstico)
30. [Portabilidade futura para Raspberry Pi Pico e MaixBit](#30-portabilidade-futura-para-raspberry-pi-pico-e-maixbit)
31. [Casos de uso reais](#31-casos-de-uso-reais)
32. [Licença](#32-licença)
33. [Contato e referências](#33-contato-e-referências)
34. [Considerações finais](#34-considerações-finais)

## 1. Visão geral

O **LoRaMESH MicroPython** busca entregar uma forma objetiva de trabalhar com módulos LoRaMESH em ambientes embarcados interpretados, preservando a lógica do ecossistema principal e permitindo:

- testes rápidos em hardware
- automações simples
- prototipagem em ESP32
- validação de configuração de rádio
- leitura e escrita de IO remoto
- envio de payloads entre nós
- experimentação em placas com suporte a `machine.UART`

Ele é especialmente interessante quando a prioridade é:

- reduzir o atrito para subir testes em campo
- validar integrações sem recompilar firmware nativo a cada alteração
- acelerar prova de conceito
- publicar conteúdo técnico prático
- experimentar variações do ecossistema LoRaMESH em placas acessíveis

## 2. Por que este port existe

O port de MicroPython nasce de uma necessidade bem prática: em muitos cenários, compilar e ajustar firmware nativo não é o melhor caminho para um teste rápido, uma validação simples de rede ou uma prova de conceito.

Ter a opção de subir a biblioteca em MicroPython abre espaço para:

- testar módulos LoRaMESH de forma mais rápida
- construir exemplos didáticos
- reduzir o tempo entre ideia e hardware em funcionamento
- validar configurações de rede e rádio sem entrar imediatamente em um firmware completo
- criar conteúdo técnico voltado para placas populares

Ao mesmo tempo, este port também tem valor estratégico dentro do projeto maior. Ele mostra que a ideia da biblioteca não está presa a uma única linguagem ou a um único fluxo de desenvolvimento.

## 3. Estado atual e transparência sobre estabilidade

Esta seção existe de forma proposital. O objetivo é ser honesto com quem vai usar a biblioteca.

Hoje, esta variação deve ser tratada como:

- **experimental**
- **funcional para vários cenários de teste**
- **ainda sujeita a ajustes**
- **menos madura que o core em C++**
- **menos validada do que a camada Python para Linux/Raspberry Pi**

Os principais motivos são:

- minha experiência com MicroPython ainda é baixa
- meu foco principal de trabalho não é MicroPython
- o projeto ainda está amadurecendo em relação a organização, abstrações e comportamento em múltiplos runtimes
- diferenças entre firmwares e placas podem impactar o funcionamento de forma mais sensível do que no C++

Isso não invalida o projeto. Pelo contrário: apenas posiciona corretamente o nível de maturidade para quem vai usar.

## 4. Relação com o ecossistema LoRaMESH

Este port deve ser entendido dentro do ecossistema maior do projeto:

### 4.1 Core em C++

O **LoRaMESH C++** é a base técnica do ecossistema. É nele que está a lógica nativa mais sólida e reutilizável do projeto.

### 4.2 SDK Python

O **LoRaMESH Python** replica essa base em uma camada mais amigável para scripts, gateways Linux, Raspberry Pi e aplicações de alto nível.

### 4.3 Variação MicroPython

O **LoRaMESH MicroPython** tenta trazer essa mesma proposta para placas com runtime interpretado, priorizando rapidez de iteração e facilidade de teste em hardware compatível.

Em resumo:

- **C++** é a base mais sólida
- **Python** é a extensão de alto nível para Linux
- **MicroPython** é a adaptação leve e experimental para placas interpretadas

## 5. Plataformas atuais e direção futura

### 5.1 Plataforma principal neste momento

Atualmente, o caminho mais natural deste port é:

- **ESP32 rodando MicroPython**

Isso faz sentido por vários motivos:

- bom suporte de comunidade
- presença de `machine.UART`
- processo relativamente simples de flash
- facilidade de testes via USB
- ótimo custo-benefício para laboratório, conteúdo e prototipagem

### 5.2 Plataformas previstas para evolução futura

Existe intenção de ampliar a portabilidade para:

- **Raspberry Pi Pico**
- **Sipeed MaixBit**

### 5.3 Observação importante

Essa portabilidade futura exige cuidado porque:

- Raspberry Pi Pico usa um ambiente diferente do ESP32 em vários detalhes práticos
- o MaixBit pode envolver diferenças importantes dependendo do firmware e do dialeto utilizado
- nem todo comportamento de `machine`, `UART`, ticks e temporização se transfere de forma idêntica entre plataformas

## 6. Quando usar esta versão

Esta versão faz bastante sentido quando você quer:

- testar comunicação LoRaMESH sem entrar de imediato no firmware nativo
- montar protótipos rápidos
- validar uma topologia simples
- criar conteúdo técnico ou exemplos didáticos
- experimentar GPIO remoto ou payload transparente
- usar ESP32 com um ciclo de iteração mais rápido
- fazer testes exploratórios em laboratório

## 7. Quando não usar esta versão

Esta versão talvez não seja a melhor escolha quando o cenário exige:

- máxima robustez
- comportamento muito previsível em produção
- otimização agressiva
- uso intensivo de memória e desempenho
- integração mais crítica e refinada
- controle fino de hardware em aplicações finais

Nesses casos, a recomendação natural continua sendo:

- **LoRaMESH C++** para core embarcado e firmware final
- **LoRaMESH Python** para gateways e automações Linux

## 8. Arquitetura conceitual da solução

A proposta desta variação normalmente segue uma separação parecida com esta:

```text
Aplicação MicroPython
    ↓
LoRaMESH em MicroPython
    ↓
Transport / Adapter baseado em machine.UART
    ↓
Backend de tempo / ticks
    ↓
UART física
    ↓
Módulo LoRaMESH
```

Quando a organização do projeto está separada em módulos, a ideia costuma envolver componentes como:

- `loramesh.core.LoRaMESH`
- `loramesh.constants`
- `loramesh.adapters.machine_uart`
- `loramesh.transport.UARTTransport`
- `loramesh.backend.MicroPythonTicks`

Essa separação é importante porque ajuda a manter:

- o código mais organizado
- a lógica menos acoplada à placa
- a portabilidade futura mais viável
- a semelhança conceitual com o projeto principal em C++

## 9. Conceitos importantes antes de começar

Antes de usar a biblioteca, vale entender alguns pontos que influenciam diretamente a experiência prática.

### 9.1 Esta não é a versão mais madura do projeto

Se você já vem do C++ ou do Python, entre sabendo que a camada MicroPython ainda está amadurecendo.

### 9.2 A UART precisa estar correta

A maioria dos problemas em LoRaMESH não nasce do protocolo em si, mas da camada física:

- TX/RX invertidos ou incorretos
- baudrate errado
- GND ausente
- alimentação instável
- módulo ainda não pronto
- porta errada no host durante upload

### 9.3 Runtime interpretado tem custo

MicroPython entrega muita agilidade, mas cobra preço em:

- desempenho
- previsibilidade temporal
- sensibilidade a memória
- overhead de interpretação

### 9.4 Nem tudo que está ótimo em ESP32 estará automaticamente pronto em Pico ou MaixBit

Essa é uma das razões pelas quais a portabilidade futura deve ser tratada com cautela.

## 10. Preparando o ESP32 com MicroPython

O fluxo prático informado para o projeto começa apagando a flash e gravando a imagem MicroPython no ESP32.

### 10.1 Apagando a flash

```bash
esptool.py --chip esp32 --port /dev/ttyACM0 erase_flash
```

### 10.2 Gravando o firmware MicroPython

```bash
esptool.py --chip esp32 --port /dev/ttyACM0 write_flash -z 0x1000 workspace/ESP32_GENERIC-20251209-v1.27.0.bin
```

### 10.3 O que isso faz

- apaga o conteúdo anterior da flash
- grava a imagem do firmware MicroPython
- prepara a placa para receber a biblioteca e o `main.py`

### 10.4 Observações importantes

- confirme a porta serial correta antes de executar
- ajuste o caminho do `.bin` conforme o local real no seu ambiente
- após o flash, reinicie a placa se necessário
- algumas placas podem reaparecer com outro identificador de porta depois da regravação

## 11. Instalando ferramentas no host

### 11.1 Instalando o `ampy`

```bash
pip install ampy
```

O `ampy` é usado para:

- criar diretórios na placa
- enviar arquivos `.py`
- listar conteúdo da flash
- remover arquivos antigos

## 12. Organizando o projeto para upload

O fluxo informado por você pressupõe algo como:

1. entrar na pasta da variação MicroPython
2. copiar um exemplo de `main.py` para a raiz de trabalho
3. enviar a biblioteca `loramesh`
4. enviar o `main.py`

### 12.1 Entrando na pasta da variação MicroPython

```bash
cd micropython
```

### 12.2 Copiando um exemplo para a raiz de trabalho

```bash
cp exemples/you_board/main.py ./main.py
```

Na prática, isso deixa o projeto preparado para subir o `main.py` principal junto com a biblioteca.

### 12.3 Observação

A pasta de exemplos deve ser ajustada conforme a estrutura real usada no seu repositório. O importante aqui é a lógica:

- escolher um exemplo da sua placa
- promovê-lo temporariamente a `main.py`
- subir esse arquivo junto com a biblioteca

## 13. Subindo a biblioteca para a placa com ampy

A sequência abaixo é útil quando você quer mandar a biblioteca inteira para a placa de forma relativamente automatizada.

### 13.1 Definindo a porta

```bash
PORT=/dev/ttyACM0
```

### 13.2 Criando a pasta principal da biblioteca

```bash
echo "mkdir loramesh"
ampy --port "$PORT" mkdir loramesh
```

### 13.3 Criando os subdiretórios necessários

```bash
find loramesh -type d ! -name "__pycache__" | sort | while read -r d; do
  [ "$d" = "loramesh" ] && continue
  echo "mkdir $d"
  ampy --port "$PORT" mkdir "$d" 2>/dev/null || true
done
```

### 13.4 Enviando os arquivos `.py` da biblioteca

```bash
find loramesh -type f -name "*.py" ! -path "*/__pycache__/*" | sort | while read -r f; do
  echo "put $f"
  ampy --port "$PORT" put "$f" "$f" || break
done
```

### 13.5 Enviando o `main.py`

```bash
echo "put main.py"
ampy --port "$PORT" put main.py
```

### 13.6 Listando o conteúdo enviado

```bash
echo "ls"
ampy --port "$PORT" ls
```

### 13.7 O que essa sequência resolve

Esse fluxo permite:

- criar a estrutura da biblioteca na placa
- enviar todos os módulos Python necessários
- colocar um `main.py` pronto para execução
- verificar se o conteúdo realmente foi gravado

## 14. Atualizando apenas o `main.py`

Quando você só quer alterar o comportamento principal da aplicação e já enviou a biblioteca anteriormente, o fluxo pode ser mais curto.

### 14.1 Removendo o `main.py` anterior

```bash
ampy --port /dev/ttyACM0 rm main.py
```

### 14.2 Enviando o novo `main.py`

```bash
ampy --port /dev/ttyACM0 put main.py
```

### 14.3 Listando arquivos, se quiser conferir

```bash
ampy --port /dev/ttyACM0 ls
```

Esse `ls` não é obrigatório. Ele só ajuda a conferir rapidamente se o arquivo está no lugar.

## 15. Removendo arquivos antigos da placa

Quando você precisa limpar completamente a biblioteca da flash antes de um novo upload, pode seguir o fluxo abaixo.

### 15.1 Removendo arquivos individuais

```bash
ampy --port /dev/ttyACM0 rm loramesh/__init__.py
ampy --port /dev/ttyACM0 rm loramesh/constants.py
ampy --port /dev/ttyACM0 rm loramesh/adapters/__init__.py
ampy --port /dev/ttyACM0 rm loramesh/adapters/machine_uart.py
ampy --port /dev/ttyACM0 rm main.py
```

### 15.2 Removendo diretórios

```bash
ampy --port /dev/ttyACM0 rmdir loramesh/adapters
ampy --port /dev/ttyACM0 rmdir loramesh
```

### 15.3 Quando fazer isso

Use esse fluxo quando:

- quiser garantir que não sobrou arquivo antigo
- mudou a estrutura da biblioteca
- suspeita de conflito entre versões
- quer testar upload limpo

## 16. Arquitetura esperada da biblioteca em MicroPython

Dentro da proposta atual do projeto, a organização lógica tende a se dividir assim:

### 16.1 `loramesh.core.LoRaMESH`

Responsável pela lógica principal da biblioteca em MicroPython.

### 16.2 `loramesh.constants`

Módulo de constantes de rádio, classe, janela, GPIO e demais valores compartilhados.

### 16.3 `loramesh.adapters.machine_uart`

Camada de adaptação entre a UART nativa do MicroPython e a biblioteca.

### 16.4 `loramesh.transport.UARTTransport`

Camada de transporte usada pelo core para enviar e receber bytes sem depender diretamente do `machine.UART`.

### 16.5 `loramesh.backend.MicroPythonTicks`

Camada ligada ao tempo e temporização, importante para timeouts, polling e controle interno.

Essa organização é valiosa porque ajuda a manter a versão MicroPython parecida com a forma de pensar do projeto maior.

## 17. Constantes principais

Um dos pontos mais importantes do README é deixar claras as configurações disponíveis, assim como foi feito nas outras variações do projeto.

### 17.1 Bandwidth

```python
BW125 = 0x00
BW250 = 0x01
BW500 = 0x02
```

Leitura prática:

- `BW125` privilegia maior robustez relativa e menor taxa
- `BW250` fica como meio-termo
- `BW500` privilegia maior taxa de transmissão

### 17.2 Spreading Factor

```python
SF_FSK = 0x00

SF7  = 0x07
SF8  = 0x08
SF9  = 0x09
SF10 = 0x0A
SF11 = 0x0B
SF12 = 0x0C
```

Leitura prática:

- `SF7` tende a entregar mais velocidade
- `SF12` tende a entregar mais robustez e maior tempo no ar
- os valores intermediários equilibram velocidade e alcance
- `SF_FSK` representa operação em FSK

### 17.3 Coding Rate

```python
CR4_5 = 0x01
CR4_6 = 0x02
CR4_7 = 0x03
CR4_8 = 0x04
```

Leitura prática:

- `CR4_5` tende a ser mais eficiente
- `CR4_8` tende a ser mais robusto

### 17.4 Classe

```python
CLASS_A = 0x00
CLASS_C = 0x02
```

Leitura prática:

- `CLASS_A` costuma ser mais econômica
- `CLASS_C` mantém recepção contínua e costuma fazer mais sentido para aplicações alimentadas continuamente

### 17.5 Janela

```python
WINDOW_5s  = 0x00
WINDOW_10s = 0x01
WINDOW_15s = 0x02
```

### 17.6 Pull de GPIO

```python
LNOT_PULL = 0x00
LPULLUP   = 0x01
LPULLDOWN = 0x02
```

### 17.7 Modos de IO

```python
INOUT_DIGITAL_INPUT  = 0x00
INOUT_DIGITAL_OUTPUT = 0x01
INOUT_ANALOG_INPUT   = 0x03
```

### 17.8 Compatibilidade com estilo Arduino

Se a versão MicroPython também preserva os nomes mais familiares, você pode encontrar ou definir algo nessa linha:

```python
INPUT          = 0
OUTPUT         = 1
INPUT_PULLUP   = 7
INPUT_PULLDOWN = 8
```

## 18. Inicialização rápida

Um fluxo típico de uso em MicroPython pode começar com algo assim:

```python
from machine import UART
import time

from loramesh.adapters.machine_uart import MachineUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.backend.MicroPythonTicks import MicroPythonTicks
from loramesh.core.LoRaMESH import LoRaMESH

uart = UART(2, baudrate=9600, tx=17, rx=16)
adapter = MachineUARTAdapter(uart)
transport = UARTTransport(adapter, MicroPythonTicks())

mesh = LoRaMESH(transport)
mesh.begin()
```

### Observação importante

A forma exata do construtor pode variar conforme o estado atual da sua implementação. O mais importante aqui é a ideia:

- criar a UART
- adaptá-la para a biblioteca
- montar o transporte
- inicializar o `LoRaMESH`

## 19. Fluxo recomendado de inicialização

Um fluxo mais completo e seguro seria:

```python
from machine import UART
import time

from loramesh.adapters.machine_uart import MachineUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.backend.MicroPythonTicks import MicroPythonTicks
from loramesh.core.LoRaMESH import LoRaMESH
from loramesh.constants import BW500, SF7, CR4_5, CLASS_C, WINDOW_5s

ID = 0
PASSWORD = 123

uart = UART(2, baudrate=9600, tx=17, rx=16)
adapter = MachineUARTAdapter(uart)
transport = UARTTransport(adapter, MicroPythonTicks())

mesh = LoRaMESH(transport)
mesh.begin()

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
```

Esse fluxo é importante porque evita sair disparando comandos antes de validar que a comunicação realmente está funcional.

## 20. Configuração de rede

### 20.1 Definição do `NetworkID`

```python
mesh.setNetworkID(0)
```

### 20.2 Definição de senha

```python
mesh.setPassword(123)
```

### 20.3 Observações práticas

- faça isso depois de `localRead()`
- em muitos cenários o gateway trabalha com ID `0`
- mantenha coerência entre os nós da rede
- se estiver testando muito, registre os IDs usados para não se perder em provisionamento

## 21. Configuração de rádio

### 21.1 Leitura da configuração atual

```python
mesh.getBPS()

print("BW:", mesh.getBW())
print("SF:", mesh.getSF())
print("CR:", mesh.getCR())
```

### 21.2 Definindo a configuração

```python
mesh.setBPS(BW500, SF7, CR4_5)
```

### 21.3 Interpretação prática

- `BW` impacta taxa e comportamento do enlace
- `SF` impacta alcance, robustez e tempo no ar
- `CR` impacta redundância e robustez

## 22. Classe LoRa e janela de recepção

### 22.1 Configurando classe e janela

```python
mesh.setClass(CLASS_C, WINDOW_5s)
```

### 22.2 Interpretação prática

- `CLASS_A` tende a fazer mais sentido quando consumo importa muito
- `CLASS_C` tende a ser mais confortável para laboratório, testes contínuos e cenários energizados

## 23. Controle remoto de GPIO

Uma das partes mais valiosas do projeto continua sendo o controle remoto de IO.

### 23.1 Configuração de pino

```python
mesh.pinMode(1, 3, INOUT_DIGITAL_OUTPUT)
```

### 23.2 Escrita digital remota

```python
mesh.digitalWrite(1, 3, 1)
mesh.digitalWrite(1, 3, 0)
```

### 23.3 Leitura digital remota

```python
estado = mesh.digitalRead(1, 4)
print("Estado:", estado)
```

### 23.4 Observação prática

Em redes reais, é recomendável:

- usar retry
- colocar pequenos delays
- evitar rajadas sem controle
- validar se o nó remoto realmente está configurado

## 24. Leitura analógica e medições auxiliares

### 24.1 Leitura analógica

```python
adc = mesh.analogRead(1, 2)
print("ADC:", adc)
```

### 24.2 Cálculo de resistência

```python
r1 = mesh.getR1(rawADC=adc, R2=10000)
print("R1:", r1)
```

### 24.3 Cálculo de temperatura com NTC

```python
temp = mesh.getTemp(rawADC=adc, beta=3950)
print("Temperatura:", temp)
```

### 24.4 Leitura de ruído

```python
noise = mesh.getNoise(1)
print("Noise:", noise)
```

## 25. Comunicação transparente entre nós

Além de IO remoto, a biblioteca também faz sentido para troca de payload arbitrário.

```python
mesh.sendTransparent(2, b"START_PUMP")
```

Isso é útil para:

- protocolos próprios
- mensagens de aplicação
- comandos curtos
- telemetria compacta
- integração entre nós sem depender apenas das funções de IO

## 26. Exemplos completos

### 26.1 Exemplo mínimo funcional

```python
from machine import UART
import time

from loramesh.adapters.machine_uart import MachineUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.backend.MicroPythonTicks import MicroPythonTicks
from loramesh.core.LoRaMESH import LoRaMESH

uart = UART(2, baudrate=9600, tx=17, rx=16)
adapter = MachineUARTAdapter(uart)
transport = UARTTransport(adapter, MicroPythonTicks())
mesh = LoRaMESH(transport)

mesh.begin()

while not mesh.localRead():
    print("Aguardando modulo...")
    time.sleep(0.5)

print("Modulo pronto")
```

### 26.2 Exemplo configurando rádio e rede

```python
from machine import UART
import time

from loramesh.adapters.machine_uart import MachineUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.backend.MicroPythonTicks import MicroPythonTicks
from loramesh.core.LoRaMESH import LoRaMESH
from loramesh.constants import BW500, SF7, CR4_5, CLASS_C, WINDOW_5s

uart = UART(2, baudrate=9600, tx=17, rx=16)
adapter = MachineUARTAdapter(uart)
transport = UARTTransport(adapter, MicroPythonTicks())
mesh = LoRaMESH(transport)

mesh.begin()

while not mesh.localRead():
    time.sleep(0.5)

mesh.setNetworkID(0)
mesh.setPassword(123)
mesh.setBPS(BW500, SF7, CR4_5)
mesh.setClass(CLASS_C, WINDOW_5s)

mesh.getBPS()

print("BW:", mesh.getBW())
print("SF:", mesh.getSF())
print("CR:", mesh.getCR())
```

### 26.3 Exemplo controlando múltiplos slaves

```python
from machine import UART
import time
import urandom

from loramesh.adapters.machine_uart import MachineUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.backend.MicroPythonTicks import MicroPythonTicks
from loramesh.core.LoRaMESH import LoRaMESH
from loramesh.constants import BW500, SF7, CR4_5, CLASS_C, WINDOW_5s

MAX_REPLICATE_IN_SLAVES = 3
DELAY_BETWEEN_SLAVES = 250
MAX_RETRY = 3

ID = 0
PASSWORD = 123

def random_retry_delay():
    return 1500 + (urandom.getrandbits(16) % 2000)

uart = UART(2, baudrate=9600, tx=17, rx=16)
adapter = MachineUARTAdapter(uart)
transport = UARTTransport(adapter, MicroPythonTicks())
mesh = LoRaMESH(transport)

mesh.begin()

while not mesh.localRead():
    print("Modulo nao respondeu ainda...")
    time.sleep(0.5)

mesh.setNetworkID(ID)
mesh.setPassword(PASSWORD)
mesh.setBPS(BW500, SF7, CR4_5)
mesh.setClass(CLASS_C, WINDOW_5s)

state = False

while True:
    state = not state

    for i in range(1, MAX_REPLICATE_IN_SLAVES + 1):
        for _ in range(MAX_RETRY):
            if mesh.digitalWrite(i, 0, 1 if state else 0):
                break
            time.sleep_ms(random_retry_delay())

        print("GPIO0 SLAVE:{}={}".format(i, "ON" if state else "OFF"))

        if MAX_REPLICATE_IN_SLAVES > 1:
            time.sleep_ms(DELAY_BETWEEN_SLAVES)

    if MAX_REPLICATE_IN_SLAVES == 1:
        time.sleep_ms(DELAY_BETWEEN_SLAVES)
```

### 26.4 Exemplo lendo entrada digital remota

```python
while True:
    estado = mesh.digitalRead(1, 4)
    print("Estado do pino remoto:", estado)
    time.sleep(1)
```

### 26.5 Exemplo lendo canal analógico e calculando temperatura

```python
while True:
    adc = mesh.analogRead(1, 2)
    r1 = mesh.getR1(rawADC=adc, R2=10000)
    temp = mesh.getTemp(rawADC=adc, beta=3950)

    print("ADC:", adc)
    print("R1:", r1)
    print("Temperatura:", temp)

    time.sleep(1)
```

### 26.6 Exemplo de envio transparente

```python
mesh.sendTransparent(2, b"START_PUMP")
```

## 27. Boas práticas

### 27.1 Trate esta versão como port experimental

Mesmo que funcione bem no seu teste, não assuma automaticamente maturidade de produção.

### 27.2 Sempre valide com `localRead()`

Esse é o ponto mais importante de inicialização.

### 27.3 Use delays entre operações

Isso é especialmente importante em MicroPython, onde o ambiente interpretado já adiciona seu próprio custo.

### 27.4 Use retry em acionamento remoto

Principalmente quando o comando é importante.

### 27.5 Prefira exemplos simples primeiro

Antes de montar uma aplicação maior, valide:

- leitura local
- leitura de parâmetros
- escrita digital em um slave
- leitura digital
- leitura analógica

### 27.6 Documente o que funcionou no seu hardware

Anote sempre:

- placa usada
- firmware MicroPython usado
- UART utilizada
- pinos TX/RX
- baudrate
- configuração de rádio
- IDs da rede

## 28. Limitações atuais

Hoje, é prudente assumir que esta variação pode apresentar limitações como:

- cobertura incompleta em relação ao ecossistema principal
- diferenças de comportamento entre placas
- necessidade de ajustes manuais no deploy
- pouca validação em cenários extensos
- maturidade inferior à base em C++

## 29. Erros comuns e diagnóstico

### 29.1 A placa não aceita upload com `ampy`

Verifique:

- porta correta
- firmware MicroPython realmente gravado
- cabo USB funcional
- se a placa não está travada em reboot
- se outro programa não está segurando a porta

### 29.2 `localRead()` não responde

Verifique:

- pinos TX/RX
- GND comum
- baudrate correto
- alimentação do módulo LoRaMESH
- UART correta na placa

### 29.3 O exemplo roda, mas o slave não responde

Verifique:

- `NetworkID`
- senha
- configuração de rádio em todos os nós
- classe e janela
- ID do slave
- tempo entre tentativas

### 29.4 O código funciona no ESP32, mas não no futuro target

Esse tipo de diferença é esperado. Pico e MaixBit não devem ser tratados como substituição automática sem validação específica.

## 30. Portabilidade futura para Raspberry Pi Pico e MaixBit

Existe interesse real em levar esta variação para:

- **Raspberry Pi Pico**
- **Sipeed MaixBit**

Essa evolução faz sentido porque dialoga com dois mundos que já fazem parte do laboratório:

- o Pico como plataforma acessível, popular e muito interessante para conteúdo técnico
- o MaixBit como plataforma que já entrou em experimentação prática

Ainda assim, é importante deixar claro:

- isso é uma direção futura
- a compatibilidade não deve ser assumida sem testes
- pode haver ajustes específicos por plataforma
- parte da arquitetura talvez precise ser refinada para suportar bem esses ambientes

## 31. Casos de uso reais

Mesmo experimental, esta variação já conversa muito bem com cenários como:

- testes rápidos de rede LoRaMESH
- validação de provisionamento
- protótipos de automação
- leitura remota de sensores
- acionamento de saídas em laboratório
- criação de conteúdo técnico
- experimentação em ESP32
- futuras provas de conceito com Pico e MaixBit

## 32. Licença

MIT License

## 33. Contato e referências

### 33.1 Desenvolvedor da biblioteca

Autor: Gustavo Cereza

- GitHub: [https://github.com/GustavoCereza](https://github.com/GustavoCereza)
- LinkedIn: [https://www.linkedin.com/in/gustavo-cereza/](https://www.linkedin.com/in/gustavo-cereza/)
- Site: [https://elcereza.com](https://elcereza.com)

### 33.2 Fabricante do módulo

Empresa: Radioenge

- Site: [https://www.radioenge.com.br](https://www.radioenge.com.br)
- Documentação e informações: [https://www.radioenge.com.br/produto/modulo-loramesh/](https://www.radioenge.com.br/produto/modulo-loramesh/)
- Produto LoRaMESH: [https://meli.la/2dSTW3C](https://meli.la/2dSTW3C)

## 34. Considerações finais

O **LoRaMESH MicroPython** é uma extensão experimental e honesta do ecossistema LoRaMESH.

Ele não tenta competir com a base em C++ em maturidade. O objetivo aqui é outro: abrir espaço para testes mais rápidos, protótipos leves, conteúdo técnico, experimentação e futuras possibilidades em placas acessíveis.

Ao mesmo tempo, ele preserva algo muito importante: a identidade do projeto. Mesmo em MicroPython, a ideia continua sendo trabalhar com LoRaMESH de forma real, indo além do envio superficial de bytes e buscando configuração, controle remoto de IO, leitura analógica e integração prática com hardware.

Se a sua intenção é explorar o ecossistema LoRaMESH em um ambiente mais leve e mais rápido de iterar, esta variação já entrega um caminho promissor. Só vale manter a expectativa correta: **trate esta camada como um port em evolução, útil para laboratório, testes e prototipagem, mas ainda amadurecendo em robustez e refinamento**.