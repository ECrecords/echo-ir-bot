#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "lwipopts.h"
#include "stdlib.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

#define SERVER_ADDR "192.168.0.163"

static QueueHandle_t rx_queue;

typedef struct RadarData_
{
    uint32_t distance;
    uint32_t angle;
} RadarData_t;

uint8_t buffer[8];

void SendDataToServer(RadarData_t *rx_data)
{
    printf("Distance: %d\n", rx_data->distance);
    printf("Angle: %d\n", rx_data->angle);
}

void xHandleReceive(void *pvParameters)
{

    RadarData_t rx_data;

    while (1)
    {
        if (xQueueReceive(rx_queue, &rx_data, portMAX_DELAY) == pdTRUE)
        {
            SendDataToServer(&rx_data);
        }
    }
}

void ConfigWifi(void *pvParameters)
{
    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect.\n");
        exit(1);
    }
    else
    {
        printf("Connected.\n");
    }

    ip_addr_t server_addr;
    ipaddr_aton(SERVER_ADDR, &server_addr);

    while (true)
    {
        // not much to do as LED is in another task, and we're using RAW (callback) lwIP API
        printf("Sending data to server...\n");
        vTaskDelay(100);
    }

    cyw43_arch_deinit();
}

#define SPI_RX_SCK 2
#define SPI_RX_MOSI 3
#define SPI_RX_MISO 4
#define SPI_RX_CS 5

int rx_channel;

void tx_handle() {
    printf("Transfer finished\nData: ");

    for (size_t i = 0; i < 8; i++) {
        printf(" 0x%0x", buffer[i]);
    }
    print("\n");

    dma_channel_set_write_addr(rx_channel, buffer, true);
    // NEED TO ACK INTERRUPT
}

void configure_spi_rx()
{
    gpio_init_mask((1 << SPI_RX_CS) | (1 << SPI_RX_MOSI) | (1 << SPI_RX_MISO) | (1 << SPI_RX_CS));
    gpio_set_function(SPI_RX_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX_CS, GPIO_FUNC_SPI);

    rx_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(rx_channel);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(spi0, false));
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);

    dma_channel_configure(rx_channel, &c,
                          buffer,
                          spi0_hw->dr,
                          8,
                          false);
}

int main()
{
    stdio_uart_init();

    config_i2c_slave();

    rx_queue = xQueueCreate(10, sizeof(RadarData_t));

    xTaskCreate(ConfigWifi, "ConfigWifi", 512, NULL, 1, NULL);
    xTaskCreate(xHandleReceive, "HandleReceive", 1024, NULL, 1, NULL);
    xTaskCreate(run_master, "RunMaster", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    cyw43_arch_deinit();
}
