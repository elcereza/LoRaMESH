# LoRaMESH MicroPython

Biblioteca **LoRaMESH para MicroPython** voltada para comunicação com módulos **LoRaMESH (Radioenge)** em ambientes embarcados leves, principalmente quando a ideia é subir testes rápidos, integrações simples, prototipagem e aplicações práticas diretamente em placas compatíveis com MicroPython.

Este port existe para levar a proposta do ecossistema LoRaMESH para um ambiente mais leve e mais direto de testar em hardware, mantendo a mesma linha conceitual do projeto principal:

- o **core nativo em C++** continua sendo a base técnica do ecossistema
- a **SDK Python para Linux/Raspberry Pi** funciona como camada de alto nível
- a **variação MicroPython** leva essa mesma ideia para microcontroladores compatíveis com runtime interpretado

Em outras palavras, esta biblioteca não nasce como um projeto isolado. Ela faz parte da mesma família do **LoRaMESH C++** e do **LoRaMESH Python**, mas com foco em ambientes mais limitados, onde praticidade de teste e deploy contam muito.

## Aviso importante sobre o estado deste port

Este README precisa ser transparente sobre um ponto importante: **a variação MicroPython já saiu de `1.0.0.b1` e passa a ser tratada como versão `1.0.0`**.

Essa mudança não acontece por suposição. Ela acontece porque este port já foi **testado e validado em três hardwares completamente diferentes**:

- **ESP32**
- **Raspberry Pi Pico**
- **[Sipeed MaixBit](https://s.click.aliexpress.com/e/_c4OlNQap)**

Isso significa que esta camada não deve mais ser apresentada como um experimento puramente conceitual. Ela já demonstrou funcionamento real em plataformas diferentes, com propostas bem distintas entre si.

Ao mesmo tempo, eu faço questão de deixar claro outro ponto: **MicroPython não é a linguagem que uso na maioria dos meus trabalhos com microcontroladores**. Minha base principal continua muito mais ligada a C, C++, firmware embarcado e integração de hardware em nível mais nativo.

Na prática, isso quer dizer o seguinte:

- a biblioteca já se mostrou funcional e viável em hardware real
- o port já tem maturidade suficiente para sair de `1.0.0.b1` e ir para **`1.0.0`**
- ainda assim, a organização interna, a ergonomia da API e algumas otimizações podem continuar evoluindo com o tempo

A melhor leitura possível hoje é esta: **o port MicroPython já foi validado na prática e é útil de verdade**, mas ainda pode amadurecer bastante em refinamento interno porque MicroPython não é o meu foco principal de desenvolvimento.

## Direção do projeto

Neste momento, o port já conta com validação prática em **ESP32**, **Raspberry Pi Pico** e **Sipeed MaixBit**.

Cada uma dessas placas ajuda a provar um ponto importante do projeto:

- o **ESP32** mostra um caminho muito natural para testes rápidos, UART simples e iteração frequente
- o **Raspberry Pi Pico** mostra que a proposta também funciona em uma plataforma extremamente acessível e muito relevante para conteúdo técnico
- o **MaixBit da Sipeed** mostra que o ecossistema consegue chegar também a um ambiente diferente, inclusive com fluxo baseado em microSD

Além disso, essa validação em placas distintas fortalece a proposta geral do ecossistema: a biblioteca não está presa a um único hardware nem a uma única forma de deploy.

Ainda assim, a direção do projeto continua aberta para expansão futura em outras placas compatíveis com MicroPython, sempre respeitando as particularidades de cada runtime, cada firmware e cada forma de acesso à UART.

## Sumário

1. [Visão geral](#1-visão-geral)
2. [Por que este port existe](#2-por-que-este-port-existe)
3. [Estado atual e transparência sobre maturidade](#3-estado-atual-e-transparência-sobre-maturidade)
4. [Relação com o ecossistema LoRaMESH](#4-relação-com-o-ecossistema-loramesh)
5. [Placas testadas e direção do projeto](#5-placas-testadas-e-direção-do-projeto)
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
30. [Placas testadas e expansão futura](#30-placas-testadas-e-expansão-futura)
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

No estado atual do projeto, esse port já chega com validação em **ESP32**, **Raspberry Pi Pico** e **Sipeed MaixBit**.

## 2. Por que este port existe

O port de MicroPython nasce de uma necessidade bem prática: em muitos cenários, compilar e ajustar firmware nativo não é o melhor caminho para um teste rápido, uma validação simples de rede ou uma prova de conceito.

Ter a opção de subir a biblioteca em MicroPython abre espaço para:

- testar módulos LoRaMESH de forma mais rápida
- construir exemplos didáticos
- reduzir o tempo entre ideia e hardware em funcionamento
- validar configurações de rede e rádio sem entrar imediatamente em um firmware completo
- criar conteúdo técnico voltado para placas populares

Ao mesmo tempo, este port também tem valor estratégico dentro do projeto maior. Ele mostra que a ideia da biblioteca não está presa a uma única linguagem ou a um único fluxo de desenvolvimento.

## 3. Estado atual e transparência sobre maturidade

Esta seção existe de forma proposital. O objetivo é posicionar corretamente a biblioteca para quem vai usar.

Hoje, esta variação deve ser entendida como:

- **validada em hardware real**
- **já tratada como versão `1.0.0`**
- **funcional em placas diferentes**
- **coerente com o ecossistema principal do projeto**
- **ainda aberta a melhorias de organização e otimização**

A mudança de `1.0.0.b1` para **`1.0.0`** não foi apenas simbólica. Ela acontece porque o port já foi testado e validado em:

- **ESP32**
- **Raspberry Pi Pico**
- **Sipeed MaixBit**

Isso mostra que a proposta funciona em cenários reais e não apenas em um único hardware muito específico.

Ao mesmo tempo, também é importante ser honesto sobre outro ponto:

- eu **não uso MicroPython na maioria dos meus trabalhos com microcontroladores**
- minha experiência principal continua muito mais concentrada em **C, C++ e firmware embarcado nativo**
- por isso, mesmo com o port validado, ainda existe espaço natural para evolução em ergonomia, organização e refinamento interno

Então a leitura correta é esta: **a biblioteca já foi validada na prática e merece a versão `1.0.0`**, mas isso não impede que continue amadurecendo bastante com o tempo.

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
- **MicroPython** é a adaptação leve para placas interpretadas, já validada em hardware real

## 5. Placas testadas e direção do projeto

### 5.1 Placas já testadas e validadas

Até o momento, este port já foi testado e validado em:

- **ESP32**
- **Raspberry Pi Pico**
- **Sipeed MaixBit**

Esse ponto é importante porque mostra que a biblioteca não ficou restrita a um único ambiente.

### 5.2 ESP32

O ESP32 continua sendo um dos caminhos mais naturais para este port porque oferece:

- bom suporte de comunidade
- presença de `machine.UART`
- processo simples de gravação de firmware
- deploy direto por USB
- excelente custo-benefício para laboratório, testes e prototipagem

### 5.3 Raspberry Pi Pico

O Raspberry Pi Pico é especialmente relevante para o projeto porque:

- é uma plataforma muito acessível
- faz bastante sentido para conteúdo técnico e exemplos didáticos
- já foi validado com sucesso com esta biblioteca

No caso do Pico, o fluxo de upload da biblioteca com `ampy` também se encaixa bem na proposta atual.

### 5.4 Sipeed MaixBit

O MaixBit também já foi validado com sucesso, mas seu fluxo de uso merece atenção especial porque, no caso dessa placa, o caminho mais prático adotado no projeto é trabalhar com **microSD**.

Isso muda a forma de deploy da biblioteca e também a forma de preparar o ambiente de import.

### 5.5 Direção do projeto

Mesmo com essas três placas já validadas, a direção do projeto continua aberta para:

- refinamento da biblioteca em MicroPython
- melhoria de organização interna
- polimento dos exemplos por placa
- expansão futura para outros hardwares compatíveis

## 6. Quando usar esta versão

Esta versão faz bastante sentido quando você quer:

- testar comunicação LoRaMESH sem entrar de imediato no firmware nativo
- montar protótipos rápidos
- validar uma topologia simples
- criar conteúdo técnico ou exemplos didáticos
- experimentar GPIO remoto ou payload transparente
- usar ESP32, Pico ou MaixBit com um ciclo de iteração mais rápido
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

### 9.1 Esta versão já foi validada, mas ainda pode evoluir bastante

Se você já vem do C++ ou do Python, a leitura correta não é mais tratar esta camada como beta ou puramente experimental. O correto hoje é entendê-la como uma variação **já validada em hardware real**, mas que ainda pode crescer em refinamento porque MicroPython não é minha base principal de trabalho.

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

### 9.4 A biblioteca `micropython/loramesh` precisa ser transferida por completo

Em todos os hardwares validados neste projeto, a regra prática é a mesma: **a pasta `/micropython/loramesh` precisa ser transferida por completo para o microcontrolador em questão**.

Isso vale para:

- **ESP32**
- **Raspberry Pi Pico**
- **Sipeed MaixBit**

O que muda entre eles não é a necessidade da biblioteca, mas o método de deploy.

### 9.5 ESP32 e Raspberry Pi Pico seguem bem com `ampy`

Nos casos de **ESP32** e **Raspberry Pi Pico**, o fluxo com `ampy` funciona bem para criação de diretórios, envio da biblioteca e atualização do `main.py`.

### 9.6 MaixBit exige cuidado especial com microSD

No **Sipeed MaixBit**, o caminho adotado é colocar a biblioteca em um **microSD** e ajustar o `sys.path` antes de importar os módulos do projeto. Sem isso, o import pode não localizar a biblioteca corretamente.

## 10. Preparando o ESP32 com MicroPython

O fluxo prático informado para o projeto começa apagando a flash e gravando a imagem MicroPython no ESP32. Esse continua sendo um dos caminhos mais práticos para colocar a placa em um estado limpo antes de subir a biblioteca.

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
3. garantir que a biblioteca `loramesh` será transferida por completo
4. enviar a biblioteca
5. enviar o `main.py`

### 12.1 Entrando na pasta da variação MicroPython

```bash
cd micropython
```

### 12.2 Copiando um exemplo para a raiz de trabalho

```bash
cp exemples/you_board/main.py ./main.py
```

Na prática, isso deixa o projeto preparado para subir o `main.py` principal junto com a biblioteca.

### 12.3 A biblioteca `loramesh` deve ir inteira para a placa

Independentemente da placa usada, a ideia correta é transferir **toda a pasta `micropython/loramesh`** para o hardware de destino.

Isso é importante porque o projeto depende da estrutura completa de módulos, e não apenas de um arquivo isolado.

### 12.4 ESP32 e Pico

No **ESP32** e no **Raspberry Pi Pico**, o fluxo com `ampy` é o caminho natural para:

- criar diretórios
- enviar a biblioteca inteira
- subir o `main.py`
- atualizar arquivos individualmente quando necessário

### 12.5 MaixBit

No **Sipeed MaixBit**, o processo muda: em vez de mandar a biblioteca com `ampy` para a flash interna da mesma forma que no ESP32 e no Pico, o caminho prático adotado é colocar a biblioteca em um **microSD**.

Antes de importar a biblioteca no MaixBit, é importante preparar o ambiente assim:

```python
import sys
import time
import urandom

if "/sd" not in sys.path:
    sys.path.append("/sd")
```

Só depois disso faz sentido importar os módulos da biblioteca a partir do cartão.

### 12.6 Observação

A pasta de exemplos deve ser ajustada conforme a estrutura real usada no seu repositório. O importante aqui é a lógica:

- escolher um exemplo da sua placa
- promovê-lo temporariamente a `main.py`
- transferir a biblioteca `loramesh` completa
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

### 13.8 Em quais placas este fluxo com `ampy` funciona diretamente

Esse fluxo com `ampy` se encaixa bem em:

- **ESP32**
- **Raspberry Pi Pico**

No caso do **MaixBit**, a lógica de organização da biblioteca continua valendo, mas o deploy prático deve ser feito via **microSD**.

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

No caso do MaixBit, essa mesma estrutura precisa estar disponível no caminho acessível pelo sistema de arquivos do cartão, normalmente sob `/sd`, para que os imports funcionem corretamente após o ajuste do `sys.path`.

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

### 26.6 Exemplo mínimo de preparação para MaixBit via microSD

No MaixBit, antes de importar a biblioteca, prepare o caminho do cartão:

```python
import sys
import time
import urandom

if "/sd" not in sys.path:
    sys.path.append("/sd")

from loramesh.adapters.machine_uart import MachineUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.backend.MicroPythonTicks import MicroPythonTicks
from loramesh.core.LoRaMESH import LoRaMESH
```

Esse trecho é importante porque, nesse cenário, a biblioteca foi colocada no microSD e precisa estar acessível pelo `sys.path`.

### 26.7 Exemplo de envio transparente

```python
mesh.sendTransparent(2, b"START_PUMP")
```

## 27. Boas práticas

### 27.1 Comece pelo fluxo mais básico e validado

Antes de expandir o projeto, valide primeiro:

- leitura local
- leitura de parâmetros
- escrita digital em um slave
- leitura digital
- leitura analógica

### 27.2 Sempre valide com `localRead()`

Esse é o ponto mais importante de inicialização.

### 27.3 Use delays entre operações

Isso é especialmente importante em MicroPython, onde o ambiente interpretado já adiciona seu próprio custo.

### 27.4 Use retry em acionamento remoto

Principalmente quando o comando é importante.

### 27.5 Transfira a biblioteca `loramesh` completa

Evite subir só arquivos soltos sem a estrutura correta. O caminho mais seguro é sempre garantir que a pasta `loramesh` esteja completa no dispositivo ou no microSD.

### 27.6 No MaixBit, ajuste o `sys.path` antes dos imports

Sem isso, a biblioteca pode não ser localizada corretamente quando estiver no cartão.

### 27.7 Documente o que funcionou no seu hardware

Anote sempre:

- placa usada
- firmware MicroPython usado
- UART utilizada
- pinos TX/RX
- baudrate
- configuração de rádio
- IDs da rede
- forma de deploy usada

## 28. Limitações atuais

Hoje, é prudente assumir que esta variação pode apresentar limitações como:

- diferenças de comportamento entre placas e firmwares
- necessidade de ajustes manuais no deploy conforme o hardware
- menor nível de otimização em comparação com uma implementação nativa em C++
- evolução contínua de organização interna e ergonomia da API

Isso não invalida a biblioteca. Apenas posiciona corretamente o tipo de maturidade esperado de uma camada em MicroPython dentro de um ecossistema cujo núcleo principal continua sendo o C++.

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

### 29.4 O código funciona em uma placa, mas não em outra

Mesmo já tendo validação em ESP32, Raspberry Pi Pico e MaixBit, ainda vale lembrar que diferenças de firmware, pinos, UART e método de deploy podem gerar comportamentos específicos por placa.

Verifique especialmente:

- mapeamento de UART
- método de transferência da biblioteca
- uso de flash interna ou microSD
- necessidade de ajuste de `sys.path`
- firmware MicroPython ou ambiente compatível usado na placa

## 30. Placas testadas e expansão futura

Atualmente, esta variação já foi testada e validada em:

- **ESP32**
- **Raspberry Pi Pico**
- **Sipeed MaixBit**

Esse ponto é importante porque mostra que o port já atravessou ambientes bem diferentes entre si.

### 30.1 ESP32

No ESP32, o fluxo com `esptool.py` e `ampy` se encaixa muito bem e torna a iteração rápida.

### 30.2 Raspberry Pi Pico

No Raspberry Pi Pico, a biblioteca também se mostrou funcional, reforçando que o projeto não está preso ao ESP32.

### 30.3 Sipeed MaixBit

No MaixBit, a validação também foi bem-sucedida, com a diferença prática de que o caminho adotado passa pelo uso de **microSD** e pelo ajuste do `sys.path` para localizar a biblioteca.

### 30.4 Expansão futura

A partir dessa base, o projeto pode continuar evoluindo para:

- refinamento dos exemplos por hardware
- melhoria de organização da biblioteca
- polimento de deploy
- expansão para outras placas compatíveis com MicroPython

## 31. Casos de uso reais

Com a validação já feita em hardwares diferentes, esta variação conversa muito bem com cenários como:

- testes rápidos de rede LoRaMESH
- validação de provisionamento
- protótipos de automação
- leitura remota de sensores
- acionamento de saídas em laboratório
- criação de conteúdo técnico
- experimentação em ESP32
- aplicações leves em Raspberry Pi Pico
- integrações específicas com MaixBit
- futuras provas de conceito em outros hardwares compatíveis

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

O **LoRaMESH MicroPython** não deve mais ser visto como um simples port beta ou apenas uma prova de conceito.

A mudança de `1.0.0.b1` para **`1.0.0`** acontece porque a biblioteca já foi testada e validada em **ESP32**, **Raspberry Pi Pico** e **Sipeed MaixBit**, mostrando que a proposta funciona de verdade em hardwares diferentes.

Ao mesmo tempo, eu faço questão de manter uma leitura honesta sobre o projeto: **MicroPython não é a base da maior parte do meu trabalho com microcontroladores**. Isso significa que ainda existe bastante espaço para evoluir em refinamento interno, ergonomia e otimização, mesmo com o port já validado.

Ainda assim, a identidade do projeto permanece muito clara. Mesmo em MicroPython, a ideia continua sendo trabalhar com LoRaMESH de forma real, indo além do envio superficial de bytes e buscando configuração de rede, controle remoto de IO, leitura analógica e integração prática com hardware.

Se a intenção é explorar o ecossistema LoRaMESH em um ambiente mais leve e mais rápido de iterar, esta variação já entrega uma base concreta, funcional e validada. E como o port já mostrou resultado em placas bem diferentes entre si, ele também passa a abrir um caminho mais sólido para evolução futura dentro do ecossistema.