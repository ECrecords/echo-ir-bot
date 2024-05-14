/**
 * @file main.c
 * @brief RADAR NAVIGATION SYSTEM implementation file.
 * 
 * This system interfaces with a radar sensor to receive navigation data over SPI,
 * processes this data, and sends it to a backend server via WiFi. The system uses
 * Raspberry Pi Pico SDK, FreeRTOS for task management, and LWIP for network communication.
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "stdlib.h"

#include "lwipopts.h"
#include "lwip/udp.h"
#include "lwip/ip.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

#define SERVER_ADDR "192.168.0.181"
#define SERVER_PORT 8008

#define SPI_SLAVE spi0
#define SPI_BAUD_RATE 1000 * 1000
#define SPI_SLAVE_SCK 18
#define SPI_SLAVE_MOSI 16
#define SPI_SLAVE_MISO 19
#define SPI_SLAVE_CS 17

#ifdef SPI_MASTER
#define SPI_TX spi0
#define SPI_TX_SCK 2
#define SPI_TX_MOSI 3
#define SPI_TX_MISO 4
#define SPI_TX_CS 5
#endif

/**
 * @struct RadarData_
 * @brief Structure to hold radar data.
 */
typedef struct RadarData_ {
    uint32_t distance; ///< Distance measurement from radar.
    uint32_t angle;    ///< Angle measurement from radar.
} RadarData_t;

/**
 * @union data_t
 * @brief Union to help with data manipulation.
 *
 * Allows access to radar data as both an array of uint32s or as RadarData_t struct.
 */
typedef union {
    uint32_t buff[2]; ///< Buffer as array of uint32s.
    RadarData_t rd;   ///< Radar data as struct.
} data_t;



static int rx_channel;
static data_t dma_data;
static QueueHandle_t rx_queue;

/**
 * @brief Sends radar data to the server over UDP.
 * @param rx_data Pointer to the radar data to send.
 * @return None
 */
void SendDataToServer(RadarData_t *rx_data) {
    struct udp_pcb *pcb;
    struct pbuf *p;
    ip_addr_t server_ip;
    err_t err;

    // Create new UDP control block
    pcb = udp_new();
    if (!pcb) {
        printf("Failed to create UDP PCB\n");
        return;
    }

    // Define the remote IP address and port
    IP4_ADDR(&server_ip, 192, 168, 0, 181);  // Server IP and port

    // Allocate pbuf (memory for packet)
    p = pbuf_alloc(PBUF_TRANSPORT, sizeof(RadarData_t), PBUF_RAM);
    if (!p) {
        printf("Failed to allocate pbuf\n");
        udp_remove(pcb);  // Clean up the PCB if pbuf allocation fails
        return;
    }

    // Copy radar data to pbuf
    memcpy(p->payload, rx_data, sizeof(RadarData_t));

    // Send UDP packet
    err = udp_sendto(pcb, p, &server_ip, SERVER_PORT);
    if (err != ERR_OK) {
        printf("Failed to send UDP packet\n");
    }

    // Free the pbuf
    pbuf_free(p);

    // Clean up the UDP control block
    udp_remove(pcb);
}

/**
 * @brief Configures and manages WiFi connection and data transmission.
 * @param pvParameters Pointer to task parameters (unused).
 * @return None
 */
void ConfigWifi(void *pvParameters)
{
    RadarData_t rx_data;

    uint connect_attempts = 0;

    if (cyw43_arch_init())
    {
        panic("Failed to initialise");
    }
    
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");

    while (connect_attempts < 5) {
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000))
        {
            printf("Failed to connect to Wi-Fi. Retrying... %d\n", connect_attempts);
            connect_attempts++;
        }
        else
        {
            printf("Connected.\n");
            break;
        }
    }

    while(1) {
        if (xQueueReceive(rx_queue, &rx_data, portMAX_DELAY) == pdTRUE)
        {
            // printf("Distance: %d, Angle: %d\n", rx_data.distance, rx_data.angle);
            if (cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_JOIN){
                SendDataToServer(&rx_data);
            }
        }
    }
    
    cyw43_arch_deinit();
}

/**
 * @brief SPI RX interrupt handler.
 * @param None
 * @return None
 */
void rx_handle()
{
    xQueueSendFromISR(rx_queue, &dma_data.rd, NULL);

    dma_channel_acknowledge_irq0(rx_channel);
    dma_channel_set_write_addr(rx_channel, dma_data.buff, true);
}

/**
 * @brief Configures SPI RX and DMA.
 * @param None
 * @return None
 */
void configure_spi_rx()
{
    gpio_init_mask((1 << SPI_SLAVE_CS) | (1 << SPI_SLAVE_MOSI) | (1 << SPI_SLAVE_MISO) | (1 << SPI_SLAVE_CS));
    gpio_set_function(SPI_SLAVE_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SLAVE_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SLAVE_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SLAVE_CS, GPIO_FUNC_SPI);

    uint actual_baud = spi_init(SPI_SLAVE, SPI_BAUD_RATE);

    if (actual_baud != SPI_BAUD_RATE)
    {
        printf("SPI baud rate %d requested, %d actual\n", SPI_BAUD_RATE, actual_baud);
    } else {
        printf("SPI baud rate %d\n", actual_baud);
    }


    spi_set_format(SPI_SLAVE, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(SPI_SLAVE, true);

    rx_channel = dma_claim_unused_channel(true);
    dma_channel_config rx_c = dma_channel_get_default_config(rx_channel);

    // RX DMA configuration
    channel_config_set_transfer_data_size(&rx_c, DMA_SIZE_32);
    channel_config_set_dreq(&rx_c, spi_get_dreq(spi0, false));
    channel_config_set_read_increment(&rx_c, false);
    channel_config_set_write_increment(&rx_c, true);

    dma_channel_configure(rx_channel, &rx_c,
                          dma_data.buff,
                          &spi_get_hw(SPI_SLAVE)->dr,
                          8,
                          false);

    irq_set_exclusive_handler(DMA_IRQ_0, rx_handle);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(rx_channel, true);

    dma_channel_start(rx_channel);
}

#ifdef SPI_MASTER
/**
 * @brief Task to simulate radar data transmission for testing in master mode.
 * @param pvParameters Pointer to task parameters (unused).
 */
void run_master(void *pvParameters)
{
    data_t data;

    gpio_init_mask((1 << SPI_TX_CS) | (1 << SPI_TX_MOSI) | (1 << SPI_TX_MISO) | (1 << SPI_TX_CS));
    gpio_set_function(SPI_TX_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX_CS, GPIO_FUNC_SPI);

    spi_init(spi0, SPI_BAUD_RATE);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(spi0, false);

    data.rd.distance = 0;
    data.rd.angle = 0;

    while (1)
    {

        spi_write_blocking(spi0, data.buff, 8);

        data.rd.angle++;
        data.rd.distance++;

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

/**
 * @brief Main function to initialize system and start scheduler.
 * @return int Program exit status.
 */
int main()
{
    stdio_uart_init();

    configure_spi_rx();

    rx_queue = xQueueCreate(512, sizeof(RadarData_t));

    xTaskCreate(ConfigWifi, "ConfigWifi", 512, NULL, 1, NULL);

#ifdef SPI_MASTER
    xTaskCreate(run_master, "RunMaster", 512, NULL, 1, NULL);
#endif

    vTaskStartScheduler();

    cyw43_arch_deinit();
}
