#include "LoRaMESH.h"

#define LORA_RX                 16
#define LORA_TX                 17
#define LORA_BAUD               9600

#define MAX_REPLICATE_IN_SLAVES 3
#define DELAY_BETWEEN_SLAVES    250          // É importante testar na sua rede para ver qual é o melhor delay
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

  // Isso só vale se sua senha for de até 16bits, acima disso irá ser feito uma mudança a cada reinício
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
  static bool state=false;

  state = !state;

  if(state)
  {
    
    for(uint16_t i = 1; i <= MAX_REPLICATE_IN_SLAVES; ++i){
      for(uint8_t j = 0; j < MAX_RETRY; ++j){
        if(lora.digitalWrite(i,0,1))
          break;
        else
          delay(1000 + random(500, 2500));
      }
      Serial.println("GPIO0 SLAVE:" + String(i) + "=ON");
      if(MAX_REPLICATE_IN_SLAVES > 1)
        delay(DELAY_BETWEEN_SLAVES);
    }
  }
  else
  {
    for(uint16_t i = 1; i <= MAX_REPLICATE_IN_SLAVES; ++i){
      for(uint8_t j = 0; j < MAX_RETRY; ++j){
        if(lora.digitalWrite(i,0,0))
          break;
        else
           delay(1000 + random(500, 2500));
      }
      Serial.println("GPIO0 SLAVE:" + String(i) + "=OFF");
      if(MAX_REPLICATE_IN_SLAVES > 1)
        delay(DELAY_BETWEEN_SLAVES);
    }
  }
  if(MAX_REPLICATE_IN_SLAVES == 1)
    delay(DELAY_BETWEEN_SLAVES);
}
