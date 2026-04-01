# LoRaMESH Python

Biblioteca Python para comunicação com módulos **LoRaMESH (Radioenge)** via interface serial, com bindings em C++ usando `pybind11`.

Este projeto já pode ser instalado diretamente pelo PyPI com:

```bash
pip install loramesh
```

A proposta desta biblioteca é entregar em Python uma interface prática, objetiva e próxima da biblioteca original em C++, preservando a mesma ideia de uso, a mesma organização conceitual e uma nomenclatura coerente com o core nativo. Em outras palavras, a API Python funciona como uma **replicação da biblioteca em C++ para uso em scripts, automações, gateways Linux e aplicações de alto nível**, sem perder a base técnica do projeto original.

Ela foi pensada para quem quer trabalhar com LoRaMESH de forma real, indo além de um simples envio de bytes. Aqui o foco é permitir configuração do rádio, leitura de informações do módulo, controle remoto de GPIO, leitura analógica, envio de payload em modo transparente, debug da serial e construção de sistemas completos com nós distribuídos.

## Sumário

1. [Visão geral](#1-visão-geral)
2. [Por que esta biblioteca existe](#2-por-que-esta-biblioteca-existe)
3. [Instalação](#3-instalação)
4. [Arquitetura da solução](#4-arquitetura-da-solução)
5. [Relação com a biblioteca em C++](#5-relação-com-a-biblioteca-em-c)
6. [Conceitos importantes antes de começar](#6-conceitos-importantes-antes-de-começar)
7. [Inicialização rápida](#7-inicialização-rápida)
8. [Fluxo recomendado de inicialização](#8-fluxo-recomendado-de-inicialização)
9. [Debug da comunicação](#9-debug-da-comunicação)
10. [Leitura local do módulo](#10-leitura-local-do-módulo)
11. [Configuração de rede](#11-configuração-de-rede)
12. [Configuração de rádio](#12-configuração-de-rádio)
13. [Classe LoRa e janela de recepção](#13-classe-lora-e-janela-de-recepção)
14. [Constantes principais](#14-constantes-principais)
15. [Controle remoto de GPIO](#15-controle-remoto-de-gpio)
16. [Leitura analógica e medições auxiliares](#16-leitura-analógica-e-medições-auxiliares)
17. [Comunicação transparente entre nós](#17-comunicação-transparente-entre-nós)
18. [Recebimento de dados](#18-recebimento-de-dados)
19. [Informações do dispositivo](#19-informações-do-dispositivo)
20. [Referência detalhada de funções](#20-referência-detalhada-de-funções)
21. [Exemplos completos](#21-exemplos-completos)
22. [Boas práticas](#22-boas-práticas)
23. [Erros comuns e diagnóstico](#23-erros-comuns-e-diagnóstico)
24. [Casos de uso reais](#24-casos-de-uso-reais)
25. [Licença](#25-licença)
26. [Contato](#26-licença)

## 1. Visão geral

O **LoRaMESH Python** permite comunicar com módulos LoRaMESH via UART a partir de aplicações Python, mantendo a lógica central da biblioteca em C++ e expondo uma interface mais amigável para:

* scripts rápidos de teste
* gateways rodando em Linux ou Raspberry Pi
* automações e integrações industriais
* sensoriamento remoto
* acionamento de saídas em nós remotos
* criação de protocolos próprios em cima da malha
* debug de integração durante desenvolvimento

Principais recursos já cobertos pela biblioteca e pelo material atual do projeto:

* criação da interface serial para o módulo
* inicialização da comunicação
* leitura local para validar o rádio
* definição de `NetworkID`
* definição de senha da rede
* leitura e escrita de parâmetros de rádio
* ajuste de largura de banda, spreading factor e coding rate
* configuração da classe LoRa
* configuração da janela de recepção
* leitura de informações do dispositivo
* controle remoto de pinos digitais
* leitura remota digital
* leitura remota analógica
* leitura de ruído
* cálculo auxiliar de resistência a partir do ADC
* cálculo auxiliar de temperatura via NTC
* envio de payload arbitrário em modo transparente
* recepção de dados quando disponível
* modo de debug com exibição de frames

## 2. Por que esta biblioteca existe

Muitas vezes o uso de LoRa acaba restrito a exemplos superficiais que apenas enviam um texto ou um pacote simples. Na prática, redes LoRaMESH podem ser usadas para construir sistemas completos, com nós remotos realizando leitura e acionamento em campo.

Esta biblioteca nasce justamente dessa necessidade de ter uma API em Python que seja realmente útil para integração de verdade.

A ideia aqui não é mascarar o comportamento do rádio, mas facilitar o uso sem perder o controle técnico. Por isso, a biblioteca em Python replica a lógica da versão em C++ e preserva a estrutura mental do projeto original. Isso é importante porque:

* reduz a distância entre o core nativo e a camada Python
* facilita manutenção e evolução conjunta
* evita criar uma API “bonita”, mas desconectada da realidade do módulo
* deixa o comportamento previsível para quem também trabalha com a versão em C++
* permite usar Python como camada de automação, teste, prototipagem e gateway

## 3. Instalação

### 3.1 Instalação via PyPI

A biblioteca já está publicada no PyPI, então a forma mais simples de instalar é:

```bash
pip install loramesh
```

### 3.2 Instalação local a partir do projeto

Se você estiver desenvolvendo a biblioteca ou testando localmente a partir do código-fonte:

```bash
pip install .
```

### 3.3 Instalação em modo de desenvolvimento

Para desenvolvimento, testes e alterações frequentes:

```bash
pip install -e .
```

### 3.4 Dependências úteis para build e publicação

```bash
pip install pybind11 setuptools build twine
```

### 3.5 Requisitos práticos

* Python 3
* acesso à porta serial do sistema
* módulo LoRaMESH conectado corretamente
* permissões adequadas para acessar `/dev/ttyACM*` ou `/dev/ttyUSB*`
* ambiente compatível com a extensão C++ gerada no build

Em Linux, muitas vezes é necessário garantir que o usuário tenha permissão para acessar a serial, normalmente via grupo como `dialout`.

## 4. Arquitetura da solução

A arquitetura segue uma divisão clara entre a camada de alto nível e o core nativo.

```text
Python API
    ↓
Bindings em pybind11
    ↓
Core C++ (LoRaMESH)
    ↓
Transporte serial (LinuxSerialTransport)
    ↓
UART / Porta serial
    ↓
Módulo LoRaMESH
```

Essa organização traz algumas vantagens importantes:

* o protocolo principal continua no core C++
* a camada Python fica mais limpa e objetiva
* a API em Python pode evoluir sem reescrever a lógica crítica
* a biblioteca pode ser usada em Linux, Raspberry Pi e ambientes próximos
* o comportamento tende a ser consistente com a implementação nativa

## 5. Relação com a biblioteca em C++

Este ponto precisa ficar muito claro: **a biblioteca Python segue a biblioteca em C++ como base estrutural e conceitual**.

Ela não foi pensada como uma implementação totalmente independente, mas como uma forma de expor em Python o mesmo núcleo funcional já existente em C++.

Na prática, isso significa que:

* o core principal está em C++
* o binding em Python serve como ponte para esse core
* a nomenclatura das funções acompanha a lógica da biblioteca nativa
* a forma de pensar o uso da biblioteca em Python continua alinhada com a versão em C++
* a manutenção e expansão da API podem continuar centradas na base nativa

Se você já conhece ou trabalha com a versão em C++, a curva de adaptação para Python tende a ser muito pequena.

## 6. Conceitos importantes antes de começar

Antes de usar a biblioteca, vale entender alguns pontos que influenciam diretamente a experiência prática.

### 6.1 A UART precisa estar correta

A biblioteca conversa com o módulo pela serial. Isso significa que cabo, alimentação, baudrate, porta correta e integridade da comunicação precisam estar corretos.

### 6.2 Nem toda falha é erro da biblioteca

Se `localRead()` não responde, o problema pode estar em:

* porta serial errada
* baudrate incompatível
* módulo não inicializado
* alimentação inadequada
* permissões de acesso à serial
* cabeamento ou adaptador USB serial

### 6.3 LoRa não é comunicação instantânea

Ao controlar um nó remoto, existe latência natural. Ela depende de fatores como:

* spreading factor
* bandwidth
* coding rate
* quantidade de hops
* topologia da malha
* nível de ruído
* classe do dispositivo

### 6.4 Sempre valide o módulo antes de sair configurando

O fluxo mais seguro é sempre:

1. abrir a biblioteca
2. iniciar a comunicação
3. aguardar `localRead()` responder
4. só então aplicar configurações ou comandos

## 7. Inicialização rápida

O uso mais básico começa criando o objeto da biblioteca com a porta serial e o baudrate.

```python
from loramesh import LoRaMESH

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin()
```

Você também pode usar a forma via módulo:

```python
import loramesh

mesh = loramesh.LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin()
```

### Observação sobre baudrate

O baudrate padrão segundo a fabricante é de 9600, então sempre teste nesse baudrate. 

## 8. Fluxo recomendado de inicialização

Um fluxo robusto de inicialização normalmente fica assim:

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin(debug=True, hex_dump=True)

while not mesh.localRead():
    print("Aguardando módulo...")
    time.sleep(0.5)

print("Módulo pronto")
```

Depois disso, você pode seguir com:

* leitura de ID local
* leitura do UID do módulo
* leitura de parâmetros de rádio
* ajuste de `NetworkID`
* ajuste de senha
* configuração de classe
* configuração de pinos remotos
* envio de payload

## 9. Debug da comunicação

Para integração real, debug é uma das partes mais úteis da biblioteca.

Ative assim:

```python
mesh.begin(debug=True, hex_dump=True)
```

### O que esse modo ajuda a enxergar

* frames TX enviados ao módulo
* frames RX recebidos do módulo
* payload em hexadecimal
* comportamento real da comunicação
* diferenças entre o que a aplicação acha que enviou e o que realmente foi transmitido

### Exemplo de saída esperada

```text
TX CMD: 00 00 E2 ...
RX CMD: ...
```

### Quando usar

Use debug principalmente quando estiver:

* subindo a integração pela primeira vez
* validando framing
* testando mudanças em funções do core
* comparando comportamento entre C++ e Python
* investigando falhas intermitentes de serial
* validando resposta do módulo após comando de configuração

## 10. Leitura local do módulo

Antes de qualquer operação relevante, a aplicação deve confirmar que o módulo está respondendo.

```python
while not mesh.localRead():
    print("Aguardando módulo...")
```

### O que `localRead()` representa

Essa função é usada para consultar o estado local do módulo e validar que a comunicação está funcionando. Na prática, ela é o ponto de sincronização inicial entre o software e o rádio.

### Por que isso é importante

Sem esse passo, sua aplicação pode:

* tentar configurar um módulo que ainda não respondeu
* assumir que a serial está funcionando quando não está
* entrar em uma sequência de comandos inválidos
* gerar debug enganoso

## 11. Configuração de rede

### 11.1 Definição do `NetworkID`

```python
mesh.setNetworkID(0)
```

O `NetworkID` define o identificador lógico do nó dentro da rede. Em muitos cenários, o gateway trabalha com `0`, enquanto os nós remotos operam com outros IDs.

### 11.2 Definição de senha

```python
mesh.setPassword(123)
```

A senha é um parâmetro importante para padronizar o comportamento esperado da rede. Todos os dispositivos que precisam operar juntos devem seguir a configuração definida para o ambiente.

### 11.3 Quando aplicar essas configurações

Normalmente logo após o `localRead()` bem-sucedido e antes de iniciar acionamentos ou troca de payloads.

## 12. Configuração de rádio

A configuração de rádio define alcance, robustez, taxa de transmissão e latência da rede.

### 12.1 Lendo a configuração atual

```python
mesh.getBPS()

bw = mesh.getBW()
sf = mesh.getSF()
cr = mesh.getCR()

print("BW:", bw)
print("SF:", sf)
print("CR:", cr)
```

A chamada `getBPS()` normalmente atualiza internamente as informações necessárias para que `getBW()`, `getSF()` e `getCR()` reflitam a configuração atual.

### 12.2 Definindo a configuração

```python
mesh.setBPS(bw=0x02, sf=0x07, cr=0x01)
```

Esse exemplo equivale a:

* `BW500`
* `SF7`
* `CR4_5`

### 12.3 Interpretação prática

* bandwidth maior tende a aumentar a taxa de transmissão
* spreading factor maior tende a aumentar alcance e robustez, mas também aumenta o tempo no ar
* coding rate maior tende a aumentar redundância e robustez, com custo de eficiência

## 13. Classe LoRa e janela de recepção

A biblioteca também permite ajustar classe e janela.

### 13.1 Exemplo de configuração

```python
mesh.setClass(lora_class=0x02, window=0x00)
```

### 13.2 Interpretação prática

* `CLASS_A` é voltada a menor consumo e funcionamento mais econômico
* `CLASS_C` mantém recepção contínua, sendo útil em cenários alimentados continuamente e com necessidade de resposta mais imediata

A janela de recepção define o comportamento temporal esperado conforme os valores implementados pela lógica do módulo.

## 14. Constantes principais

Como a biblioteca em Python replica a base em C++, a documentação fica muito mais útil quando os valores são apresentados de forma explícita.

### 14.1 Bandwidth

```python
BW125 = 0x00
BW250 = 0x01
BW500 = 0x02
```

Leitura prática:

* `BW125` privilegia alcance e robustez relativa, com menor taxa
* `BW250` fica no meio-termo
* `BW500` privilegia maior taxa, com trade-off de robustez e comportamento de enlace

### 14.2 Spreading Factor

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

* `SF7` tende a ser mais rápido
* `SF12` tende a ser mais robusto e mais lento
* valores intermediários podem ser escolhidos conforme o compromisso entre velocidade e alcance
* `SF_FSK` representa operação em FSK, não em LoRa tradicional

### 14.3 Coding Rate

```python
CR4_5 = 0x01
CR4_6 = 0x02
CR4_7 = 0x03
CR4_8 = 0x04
```

Leitura prática:

* `CR4_5` tende a ser mais eficiente
* `CR4_8` tende a ser mais robusto

### 14.4 Classe

```python
CLASS_A = 0x00
CLASS_C = 0x02
```

### 14.5 Janela

```python
WINDOW_5s  = 0x00
WINDOW_10s = 0x01
WINDOW_15s = 0x02
```

### 14.6 Pull de GPIO

```python
LNOT_PULL = 0x00
LPULLUP   = 0x01
LPULLDOWN = 0x02
```

### 14.7 Modos de IO

```python
INOUT_DIGITAL_INPUT  = 0x00
INOUT_DIGITAL_OUTPUT = 0x01
INOUT_ANALOG_INPUT   = 0x03
```

### 14.8 Compatibilidade com estilo Arduino

Também aparecem definições pensadas para aproximar o uso do estilo Arduino:

```python
INPUT          = 0
OUTPUT         = 1
INPUT_PULLUP   = 7
INPUT_PULLDOWN = 8
```

### 14.9 Observação importante sobre as constantes em Python

Dependendo do estado atual do binding, algumas constantes podem estar definidas no core C++ e não necessariamente expostas diretamente como atributos do módulo Python.

Por isso, este README documenta tanto os nomes quanto os valores. Assim você pode:

* usar os valores numéricos diretamente
* criar um `constants.py`
* expor as constantes via `pybind11`
* manter a documentação coerente com a biblioteca em C++

## 15. Controle remoto de GPIO

Um dos principais diferenciais do uso de LoRaMESH é poder tratar um nó remoto como uma extensão física da aplicação principal.

### 15.1 Configuração de pino

Antes de usar um pino, ele precisa ser configurado.

```python
mesh.pinMode(id=1, gpio=0, mode=1)
```

Exemplo mais semântico:

```python
mesh.pinMode(1, 3, INOUT_DIGITAL_OUTPUT)
```

### 15.2 Escrita digital remota

```python
mesh.digitalWrite(id=1, gpio=0, val=1)
```

Exemplo:

```python
mesh.digitalWrite(1, 3, 1)
mesh.digitalWrite(1, 3, 0)
```

Interpretação:

* primeiro parâmetro: ID do dispositivo remoto
* segundo parâmetro: pino remoto
* terceiro parâmetro: valor lógico aplicado

### 15.3 Leitura digital remota

```python
value = mesh.digitalRead(id=1, gpio=0)
```

Exemplo:

```python
estado = mesh.digitalRead(1, 4)
print("Estado do pino:", estado)
```

### 15.4 Controle direto de slave

A biblioteca trabalha com endereçamento por `device_id`, então você pode atuar diretamente em qualquer nó remoto conhecido.

```python
mesh.digitalWrite(2, 1, 1)
```

Nesse caso:

* `2` é o ID do nó remoto
* `1` é o pino
* `1` é o nível lógico alto

## 16. Leitura analógica e medições auxiliares

### 16.1 Leitura analógica remota

```python
adc = mesh.analogRead(id=1, gpio=0)
```

Exemplo:

```python
valor = mesh.analogRead(1, 2)
print("Valor analógico:", valor)
```

### 16.2 Observações sobre o ADC

O valor retornado depende do hardware e da implementação do módulo. Na prática, espere um inteiro cuja escala deve ser interpretada conforme a resolução efetiva do ADC e o circuito ligado ao pino.

### 16.3 Leitura de ruído

```python
noise = mesh.getNoise(id=1)
```

Isso pode ser útil para diagnóstico de rede, análise ambiental e monitoramento de qualidade de enlace.

### 16.4 Cálculo de resistência

```python
r1 = mesh.getR1(rawADC=adc, R2=10000)
```

Esse tipo de função é útil quando o canal analógico está lendo um divisor resistivo e você deseja reconstruir o valor do resistor variável medido.

### 16.5 Cálculo de temperatura com NTC

```python
temp = mesh.getTemp(rawADC=adc, beta=3950)
```

Esse recurso ajuda bastante quando o nó remoto é usado para leitura de sensores resistivos baseados em NTC.

### 16.6 Exemplo combinado

```python
adc = mesh.analogRead(1, 2)
r1 = mesh.getR1(rawADC=adc, R2=10000)
temp = mesh.getTemp(rawADC=adc, beta=3950)

print("ADC:", adc)
print("R1:", r1)
print("Temperatura:", temp)
```

## 17. Comunicação transparente entre nós

Além dos comandos de configuração e IO, a biblioteca permite envio de payload arbitrário entre dispositivos.

```python
mesh.sendTransparent(device_id, payload)
```

Exemplo:

```python
mesh.sendTransparent(2, b"START_PUMP")
```

### Quando isso é útil

* protocolos próprios
* comandos customizados
* envio de telemetria compacta
* encapsulamento de mensagens entre nós
* integração com aplicações externas que já têm seu próprio formato de dados

## 18. Recebimento de dados

No material atual da biblioteca, aparece o seguinte fluxo de recepção:

```python
if mesh.available():
    data = mesh.receive()
    print("Recebido:", data)
```

Isso sugere a seguinte lógica:

* `available()` indica se existe dado pronto para leitura
* `receive()` retorna o payload recebido

Se a sua versão do binding já expõe esses métodos, esse é o fluxo esperado. Se estiver trabalhando em uma etapa em que parte da recepção ainda esteja em evolução, esta documentação continua válida como direção de uso da API.

## 19. Informações do dispositivo

Após a leitura local do módulo, a biblioteca permite consultar diversas informações relevantes.

```python
mesh.getLocalID()
mesh.getUniqueID()
mesh.getBW()
mesh.getSF()
mesh.getCR()
mesh.getClassValue()
mesh.getWindow()
```

Essas chamadas são úteis para:

* diagnóstico
* verificação de inicialização
* confirmação de configuração real do rádio
* logs de inventário de dispositivos
* automação de provisionamento

## 20. Referência detalhada de funções

Abaixo está uma referência organizada das funções citadas no material atual do projeto.

### 20.1 Construtor

```python
LoRaMESH(device, baud)
```

Descrição:

* cria a instância da biblioteca
* associa a porta serial desejada
* define o baudrate de comunicação com o módulo

Parâmetros:

* `device`: porta serial, como `/dev/ttyACM0` ou `/dev/ttyUSB0`
* `baud`: baudrate usado na UART

### 20.2 `begin()`

```python
mesh.begin(debug=False, hex_dump=False)
```

Descrição:

* inicializa a comunicação da biblioteca
* pode habilitar logs de debug
* pode exibir payload em hexadecimal

Parâmetros:

* `debug`: ativa logs de depuração
* `hex_dump`: ativa exibição de bytes em hexadecimal

### 20.3 `localRead()`

```python
mesh.localRead()
```

Descrição:

* consulta o módulo local
* valida se a comunicação está funcionando
* deve ser usada antes das demais operações

Retorno esperado:

* booleano indicando sucesso ou falha da leitura

### 20.4 `setNetworkID()`

```python
mesh.setNetworkID(id)
```

Descrição:

* define o ID lógico do nó na rede

### 20.5 `setPassword()`

```python
mesh.setPassword(password)
```

Descrição:

* define a senha configurada no módulo

### 20.6 `getBPS()`

```python
mesh.getBPS()
```

Descrição:

* lê a configuração atual de rádio
* normalmente prepara os valores para consulta posterior via getters específicos

### 20.7 `getBW()`, `getSF()`, `getCR()`

```python
mesh.getBW()
mesh.getSF()
mesh.getCR()
```

Descrição:

* retornam os parâmetros atuais de bandwidth, spreading factor e coding rate

### 20.8 `setBPS()`

```python
mesh.setBPS(bw, sf, cr)
```

Descrição:

* altera os parâmetros principais do rádio

### 20.9 `setClass()`

```python
mesh.setClass(lora_class, window)
```

Descrição:

* define classe e janela de recepção

### 20.10 `getClassValue()` e `getWindow()`

```python
mesh.getClassValue()
mesh.getWindow()
```

Descrição:

* retornam a classe atual e a janela configurada

### 20.11 `getLocalID()`

```python
mesh.getLocalID()
```

Descrição:

* retorna o ID local do módulo

### 20.12 `getUniqueID()`

```python
mesh.getUniqueID()
```

Descrição:

* retorna o identificador único do dispositivo

### 20.13 `pinMode()`

```python
mesh.pinMode(id, gpio, mode)
```

Descrição:

* configura o comportamento de um pino remoto

Parâmetros:

* `id`: ID do dispositivo remoto
* `gpio`: número do pino ou canal
* `mode`: modo de operação do pino

### 20.14 `digitalWrite()`

```python
mesh.digitalWrite(id, gpio, val)
```

Descrição:

* escreve um valor lógico em um pino remoto configurado como saída

### 20.15 `digitalRead()`

```python
mesh.digitalRead(id, gpio)
```

Descrição:

* lê o estado lógico de um pino remoto

### 20.16 `analogRead()`

```python
mesh.analogRead(id, gpio)
```

Descrição:

* lê o valor analógico de um canal remoto

### 20.17 `getNoise()`

```python
mesh.getNoise(id)
```

Descrição:

* lê ou calcula informação relacionada ao ruído do nó

### 20.18 `getR1()`

```python
mesh.getR1(rawADC, R2)
```

Descrição:

* calcula resistência com base em um valor ADC e em um resistor conhecido

### 20.19 `getTemp()`

```python
mesh.getTemp(rawADC, beta)
```

Descrição:

* estima a temperatura de um NTC com base no ADC e no valor beta informado

### 20.20 `sendTransparent()`

```python
mesh.sendTransparent(device_id, payload)
```

Descrição:

* envia payload arbitrário para outro nó da rede

### 20.21 `available()`

```python
mesh.available()
```

Descrição:

* indica se existe dado recebido pronto para leitura

### 20.22 `receive()`

```python
mesh.receive()
```

Descrição:

* retorna o conteúdo recebido quando disponível

## 21. Exemplos completos

### 21.1 Exemplo mínimo funcional

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin()

while not mesh.localRead():
    print("Aguardando módulo...")
    time.sleep(0.5)

print("Módulo pronto")
```

### 21.2 Exemplo com debug e leitura de configuração

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin(debug=True, hex_dump=True)

while not mesh.localRead():
    time.sleep(0.5)

mesh.getBPS()

print("Local ID:", mesh.getLocalID())
print("Unique ID:", mesh.getUniqueID())
print("BW:", mesh.getBW())
print("SF:", mesh.getSF())
print("CR:", mesh.getCR())
print("Class:", mesh.getClassValue())
print("Window:", mesh.getWindow())
```

### 21.3 Exemplo configurando rede e rádio

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 115200)
mesh.begin(debug=True)

while not mesh.localRead():
    time.sleep(0.5)

mesh.setNetworkID(0)
mesh.setPassword(123)
mesh.setBPS(0x02, 0x07, 0x01)
mesh.setClass(0x02, 0x00)

mesh.getBPS()

print("BW final:", mesh.getBW())
print("SF final:", mesh.getSF())
print("CR final:", mesh.getCR())
```

### 21.4 Exemplo controlando uma saída remota

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin(debug=True)

while not mesh.localRead():
    time.sleep(1)

mesh.setNetworkID(0)
mesh.setPassword(123)
mesh.setBPS(0x02, 0x07, 0x01)
mesh.setClass(0x02, 0x00)

# deviceId = 1
# pin = 3
# mode = OUTPUT (0x01)

mesh.pinMode(1, 3, 0x01)
while True:
    print("Ligando saída remota")
    mesh.digitalWrite(1, 3, 1)
    time.sleep(1)
    print("Desligando saída remota")
    mesh.digitalWrite(1, 3, 0)
    time.sleep(1)
```

### 21.5 Exemplo lendo um pino digital remoto

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin()

while not mesh.localRead():
    time.sleep(0.5)

mesh.pinMode(1, 4, 0x00)  # INPUT

while True:
    estado = mesh.digitalRead(1, 4)
    print("Estado do pino remoto:", estado)
    time.sleep(1)
```

### 21.6 Exemplo lendo um canal analógico remoto

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin()

while not mesh.localRead():
    time.sleep(0.5)
mesh.pinMode(1, 2, 0x03)
while True:
    adc = mesh.analogRead(1, 2)
    print("ADC:", adc)
    time.sleep(1)
```

### 21.7 Exemplo calculando resistência e temperatura

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin()

while not mesh.localRead():
    time.sleep(0.5)
mesh.pinMode(1, 2, 0x03)
while True:
    adc = mesh.analogRead(1, 2)
    r1 = mesh.getR1(rawADC=adc, R2=10000) # Conversão para resistência (divisor resistivo)
    temp = mesh.getTemp(rawADC=adc, beta=3950) # Conversão para temperatura (NTC)
    print("ADC:", adc)
    print("R1:", r1)
    print("Temperatura:", temp)
    time.sleep(1)
```

### 21.8 Exemplo de envio transparente

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin(debug=True)

while not mesh.localRead():
    time.sleep(0.5)

mesh.sendTransparent(2, b"START_PUMP")
```

### 21.9 Exemplo de recepção

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin()

while not mesh.localRead():
    time.sleep(0.5)

while True:
    if mesh.available():
        data = mesh.receive()
        print("Recebido:", data)

    time.sleep(0.1)
```

### 21.10 Exemplo mais robusto com retry

```python
from loramesh import LoRaMESH
import time

mesh = LoRaMESH("/dev/ttyACM0", 9600)
mesh.begin(debug=True, hex_dump=True)

while not mesh.localRead():
    print("Aguardando módulo...")
    time.sleep(0.5)

mesh.setNetworkID(0)
mesh.setPassword(123)
mesh.pinMode(1, 3, 0x01)

ok = False
for _ in range(3):
    if mesh.digitalWrite(1, 3, 1):
        ok = True
        break
    time.sleep(0.2)

print("Comando executado:", ok)
```

### 21.10 Exemplo de uso simultâneo das UARTs (Command + Transparent)

O módulo LoRaMESH permite utilizar duas interfaces UART independentes:

- Uma para comandos (configuração e controle)
- Outra para envio/recebimento de dados em modo transparente

Abaixo um exemplo prático utilizando duas instâncias separadas para cada interface:

```python
from loramesh import LoRaMESH
import serial
import time

mesh_cmd = LoRaMESH("/dev/ttyACM0", 9600)
mesh_cmd.begin(debug=True)
uart_data = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

while not mesh_cmd.localRead():
    print("Aguardando módulo...")
    time.sleep(0.5)

mesh_cmd.setNetworkID(0)
mesh_cmd.setPassword(123)
mesh_cmd.setBPS(0x02, 0x07, 0x01)
mesh_cmd.setClass(0x02, 0x00)

print("Módulo configurado")
while True:
    uart_data.write(b"PING\n")
    if uart_data.in_waiting:
        data = uart_data.read(uart_data.in_waiting)
        print("RX transparente:", data)

    estado = mesh_cmd.digitalRead(1, 4)
    print("GPIO remoto:", estado)
    time.sleep(2)
```

## 22. Boas práticas

### 22.1 Sempre aguarde `localRead()`

Esse é o ponto mais importante para evitar inicialização falsa.

### 22.2 Não envie comandos em sequência sem respirar a rede

Pequenos delays ajudam bastante, especialmente em integração real.

```python
time.sleep(0.1)
```

### 22.3 Use retry quando a ação for importante

Principalmente em acionamento remoto.

### 22.4 Leia o estado atual antes de sobrescrever tudo

Isso ajuda em diagnóstico e evita configuração cega.

### 22.5 Deixe o debug ativo durante desenvolvimento

Em sistemas de comunicação, enxergar TX e RX poupa muito tempo.

### 22.6 Documente a configuração real usada em campo

Registre sempre:

* baudrate
* bandwidth
* spreading factor
* coding rate
* classe
* janela
* senha e política de provisionamento
* IDs dos nós

## 23. Erros comuns e diagnóstico

### 23.1 `localRead()` nunca responde

Verifique:

* porta correta
* alimentação do módulo
* baudrate real
* permissões de acesso à serial
* fiação UART
* se o módulo realmente inicializou

### 23.2 O comando de escrita remota não parece funcionar

Verifique:

* se o `device_id` está correto
* se o pino foi configurado antes via `pinMode()`
* se o nó remoto está ativo na rede
* se o rádio está configurado corretamente em ambos os lados
* se a latência da rede está sendo respeitada

### 23.3 A leitura analógica parece incoerente

Verifique:

* resolução real do ADC
* circuito conectado ao pino
* referência usada no módulo
* necessidade de calibração
* fórmula usada em `getR1()` e `getTemp()` conforme o hardware real

### 23.4 A recepção transparente não chega

Verifique:

* endereçamento do destino
* se existe leitura periódica no lado receptor
* se a topologia da rede está correta
* se o payload está no formato esperado pela aplicação

## 24. Casos de uso reais

O valor desta biblioteca aparece de verdade quando ela sai do exemplo simples e entra em campo. Alguns cenários muito naturais para o projeto são:

* automação rural com válvulas e bombas
* leitura remota de sensores analógicos e digitais
* monitoramento distribuído em áreas sem infraestrutura de rede tradicional
* integração entre gateway Linux e nós remotos LoRaMESH
* controle de saídas para acionamento local em campo
* telemetria industrial de baixa taxa
* prototipagem rápida de sistemas embarcados usando Python como camada de orquestração
* criação de ferramentas de provisionamento, diagnóstico e manutenção

## 25. Licença

MIT License

## 26. Contato e referências
### 26.1 Desenvolvedor da Biblioteca
Autor: Gustavo Cereza
- GitHub: [https://github.com/GustavoCereza](https://github.com/GustavoCereza)  
- LinkedIn: [https://www.linkedin.com/in/gustavo-cereza/](https://www.linkedin.com/in/gustavo-cereza/)  
- Site: [https://elcereza.com](https://elcereza.com)

### 26.2 Fabricante do módulo
Empresa: Radioenge
- Site: [https://www.radioenge.com.br](https://www.radioenge.com.br)
- Documentação e outras informações: [https://www.radioenge.com.br/produto/modulo-loramesh/](https://www.radioenge.com.br/produto/modulo-loramesh/)
- Produto LoRaMesh: [https://meli.la/2dSTW3C](https://meli.la/2dSTW3C)

## Considerações finais
O **LoRaMESH Python** não é apenas um wrapper superficial. Ele existe para levar a base da biblioteca em C++ para um ambiente em que testes, automações, gateways e integrações ficam muito mais rápidos de desenvolver.
Ao mesmo tempo, ele mantém a identidade técnica do projeto original. Isso é o que torna a biblioteca útil de verdade: você continua trabalhando sobre a mesma lógica central, mas com a agilidade do Python.
Se a intenção é construir aplicações reais com módulos LoRaMESH, controlar nós remotos, ler sensores, acionar GPIOs e transportar payloads sobre uma rede privada, esta biblioteca te dá uma base sólida para isso.
