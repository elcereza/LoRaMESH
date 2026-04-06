#define ESPIDF
#include "LoRaMESH.h"

extern "C" {
#include "driver/uart.h"
#include "esp_random.h"
#include "esp_err.h"
#include "esp_system.h"
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LORA_UART_NUM           UART_NUM_2
#define LORA_RX                 16
#define LORA_TX                 17
#define LORA_BAUD               9600

#define MAX_REPLICATE_IN_SLAVES 3
#define DELAY_BETWEEN_SLAVES    250
#define MAX_RETRY               3

static uint8_t ID = 0;

static EspIdfUartTransport transportCmd(LORA_UART_NUM);
static LoRaMESH lora(&transportCmd, &transportCmd);

static void delayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void debugPrint(const char* msg)
{
    printf("%s\n", msg);
}

static uint32_t randomBackoffMs()
{
    return 1000 + (random() % 2001); // 1000 a 3000 ms
}

static void setupLoRaUart()
{
    uart_config_t uartConfig = {};
    uartConfig.baud_rate = LORA_BAUD;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_DISABLE;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.rx_flow_ctrl_thresh = 0;

    ESP_ERROR_CHECK(uart_driver_install(
        LORA_UART_NUM,
        1024,
        1024,
        0,
        nullptr,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &uartConfig));

    ESP_ERROR_CHECK(uart_set_pin(
        LORA_UART_NUM,
        LORA_TX,
        LORA_RX,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));
}

static void initializeRadio()
{
    printf("Aguardando modulo...\n");

    while (!lora.localRead())
    {
        printf("Modulo nao respondeu ainda...\n");
        delayMs(500);
    }

    printf("Modulo pronto\n");

    if (lora.localId != ID)
    {
        printf("Configurando NetworkID\n");

        for (int i = 0; i < 3; i++)
        {
            if (lora.setNetworkID(ID))
                break;

            delayMs(200);
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (lora.getBPS())
            break;
    }

    if (lora.BW != BW500 || lora.SF != SF7 || lora.CR != CR4_5)
    {
        printf("Configurando BPS\n");

        for (int i = 0; i < 3; i++)
        {
            if (lora.setBPS(BW500, SF7, CR4_5))
                break;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (lora.getClass())
            break;
    }

    if (lora.LoRa_class != CLASS_C || lora.LoRa_window != WINDOW_5s)
    {
        printf("Configurando classe\n");

        for (int i = 0; i < 3; i++)
        {
            if (lora.setClass(CLASS_C, WINDOW_5s))
                break;
        }
    }

    // Isso só vale se sua senha for de até 16 bits
    if (lora.registered_password != 123)
    {
        printf("Configurando senha\n");

        for (int i = 0; i < 3; i++)
        {
            if (lora.setPassword(123))
                break;
        }
    }

    printf("Configuracao concluida\n");
}

extern "C" void app_main(void)
{
    delayMs(2000);

    printf("Inicializando LoRaMesh...\n");

    setupLoRaUart();

    lora.setDebug(debugPrint, true);
    lora.begin(true);

    initializeRadio();

    printf("Master pronto\n");

    bool state = false;

    while (true)
    {
        state = !state;

        if (state)
        {
            for (uint16_t i = 1; i <= MAX_REPLICATE_IN_SLAVES; ++i)
            {
                for (uint8_t j = 0; j < MAX_RETRY; ++j)
                {
                    if (lora.digitalWrite(i, 0, 1))
                        break;
                    else
                        delayMs(randomBackoffMs());
                }

                printf("GPIO0 SLAVE:%u=ON\n", i);

                if (MAX_REPLICATE_IN_SLAVES > 1)
                    delayMs(DELAY_BETWEEN_SLAVES);
            }
        }
        else
        {
            for (uint16_t i = 1; i <= MAX_REPLICATE_IN_SLAVES; ++i)
            {
                for (uint8_t j = 0; j < MAX_RETRY; ++j)
                {
                    if (lora.digitalWrite(i, 0, 0))
                        break;
                    else
                        delayMs(randomBackoffMs());
                }

                printf("GPIO0 SLAVE:%u=OFF\n", i);

                if (MAX_REPLICATE_IN_SLAVES > 1)
                    delayMs(DELAY_BETWEEN_SLAVES);
            }
        }

        if (MAX_REPLICATE_IN_SLAVES == 1)
            delayMs(DELAY_BETWEEN_SLAVES);
    }
}