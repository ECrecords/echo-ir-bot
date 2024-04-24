#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "lwipopts.h"
#include "stdlib.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"



#define SERVER_ADDR "192.168.0.163"

static const uint I2C_BAUDRATE = 100 * 1000;

static const uint I2C_SLAVE_ADDRESS = 0x50;

static i2c_inst_t *I2C_SLAVE_INSTANCE = i2c1;
static const uint I2C_SLAVE_SDA_PIN = 2;
static const uint I2C_SLAVE_SCL_PIN = 3;

static i2c_inst_t *I2C_MASTER_INSTANCE = i2c0;
static const uint I2C_MASTER_SDA_PIN = 4;
static const uint I2C_MASTER_SCL_PIN = 5;

static QueueHandle_t rx_queue;

typedef struct RadarData_
{
    uint32_t distance;
    uint32_t angle;
} RadarData_t;

static struct {
    uint8_t mem[8];
    uint8_t mem_address;
    bool write_flag;
} context;

/**
 * @brief I2C slave event handler.
 *
 * This function is called when an I2C slave event occurs. It handles the following events:
 * - I2C_SLAVE_RECEIVE: A byte has been received. The byte is read and stored in the context's memory.
 * - I2C_SLAVE_REQUEST: A write request has been received. The write flag is set to true.
 * - I2C_SLAVE_FINISH: The transaction has finished. If the write flag is true, nothing happens. Otherwise, the received data is interpreted as radar data and sent to a queue.
 *
 * @param i2c The I2C instance that the event occurred on.
 * @param event The event that occurred.
 */
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    switch (event)
    {
     case I2C_SLAVE_RECEIVE:
        // A byte has been received. Read it and store it in the context's memory.
        context.write_flag = false;
        context.mem[context.mem_address++] = i2c_read_byte_raw(i2c);
        break;

    case I2C_SLAVE_REQUEST:
        // A write request has been received. Set the write flag to true.
        context.write_flag = true;
        break;

    case I2C_SLAVE_FINISH:
        // The transaction has finished.
        if (context.write_flag) {
            // If the write flag is true, do nothing.
            break;
        }

        // If the write flag is false, interpret the received data as radar data and send it to the queue.
        RadarData_t rx_data;
        rx_data.distance = (context.mem[0] << 24) | (context.mem[1] << 16) | (context.mem[2] << 8) | context.mem[3];
        rx_data.angle = (context.mem[4] << 24) | (context.mem[5] << 16) | (context.mem[6] << 8) | context.mem[7];
        xQueueSend(rx_queue, &rx_data, portMAX_DELAY);

        // Reset the context's memory and memory address.
        context.mem_address = 0;
        memset(context.mem, 0, sizeof(context.mem));

        break;

    default:
        break;
    }
}

/**
 * @brief Configures the I2C slave.
 *
 * This function configures the I2C slave by setting up the SDA and SCL pins and initializing the I2C slave.
 * - SDA and SCL pins are configured as I2C pins.
 * - I2C slave is initialized with the specified I2C instance, address, and event handler.
 * - The I2C slave is enabled.
 * 
 * @param None
*/
void config_i2c_slave()
{
    // Configure SDA and SCL pins
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    // Configure I2C slave
    i2c_init(I2C_SLAVE_INSTANCE, I2C_BAUDRATE);
    i2c_slave_init(I2C_SLAVE_INSTANCE, I2C_SLAVE_ADDRESS, &i2c_slave_handler);

    printf("I2C slave configured\n");
}


void run_master(void* pvParameters) {
    
    // Configure I2C master
    i2c_init(I2C_MASTER_INSTANCE, I2C_BAUDRATE);

    // Configure SDA and SCL pins
    gpio_init(I2C_MASTER_SDA_PIN);
    gpio_set_function(I2C_MASTER_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_MASTER_SDA_PIN);

    gpio_init(I2C_MASTER_SCL_PIN);
    gpio_set_function(I2C_MASTER_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_MASTER_SCL_PIN);

    uint32_t distance = 0;
    uint32_t angle = 0;

    while (1)
    {
        uint8_t data[8] = {
            (distance >> 24) & 0xFF,
            (distance >> 16) & 0xFF,
            (distance >> 8) & 0xFF,
            distance & 0xFF,
            (angle >> 24) & 0xFF,
            (angle >> 16) & 0xFF,
            (angle >> 8) & 0xFF,
            angle & 0xFF
        };
        
        distance++;
        angle++;


        i2c_write_blocking(I2C_MASTER_INSTANCE, I2C_SLAVE_ADDRESS, data, 8, false);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

}

void SendDataToServer(RadarData_t *rx_data) {
    printf("Distance: %d\n", rx_data->distance);
    printf("Angle: %d\n", rx_data->angle);
}

void xHandleReceive(void *pvParameters) {

    RadarData_t rx_data;

    while (1)
    {
        if (xQueueReceive(rx_queue, &rx_data, portMAX_DELAY) == pdTRUE)
        {
            SendDataToServer(&rx_data);
        }
    }

}

void ConfigWifi(void *pvParameters) {
    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        exit(1);
    } else {
        printf("Connected.\n");
    }

    ip_addr_t server_addr;
    ipaddr_aton(SERVER_ADDR, &server_addr);

    while(true) {
        // not much to do as LED is in another task, and we're using RAW (callback) lwIP API
        printf("Sending data to server...\n");  
        vTaskDelay(100);
    }

    cyw43_arch_deinit();
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
