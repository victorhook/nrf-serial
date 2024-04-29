#include <SPI.h>
#include "ArduinoQueue.h"
#include "RF24.h"
#include "hardware/flash.h"

// Uncomment below for master
//#define MASTER

// -- Pin definitions -- //
#define PIN_LED_GREEN 7
#define PIN_LED_RED 8
#define PIN_CSN  5
#define PIN_CE   6
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_SCK  2

// -- Other defines -- //
#define RADIO_ADDRESS_SIZE            5
#define RADIO_RX_PIPE                 0
#define RADIO_MAX_PACKET_SIZE         32
#define RADIO_DEFAULT_CHANNEL         90
#define RADIO_DEFAULT_DATARATE        RF24_2MBPS
#define RADIO_DEFAULT_PA_LEVEL        RF24_PA_MAX
#define RADIO_PACKET_MAX_PAYLOAD_SIZE 32

#define FLASH_SETTINGS_ADDR 0x101ff000

#ifdef DEBUG
    #define DEBUG_PRINT(msg) serial_debug->println(msg)
#else
    #define DEBUG_PRINT(msg)
#endif

// -- Typedefs -- //

typedef enum
{
    ERROR_RADIO_INIT = 0b1
} error_t;


typedef enum
{
    RADIO_MODE_RX,
    RADIO_MODE_TX
} radio_mode_t;

typedef struct
{
    uint8_t payload[RADIO_PACKET_MAX_PAYLOAD_SIZE];
}__attribute__((packed)) radio_packet_t;

typedef struct
{
    uint8_t   radio_address[RADIO_ADDRESS_SIZE];
    uint8_t   radio_pa_level;
    uint8_t   radio_channel;
    uint8_t   radio_datarate;
    uint32_t  serial_baudrate;
    uint8_t   serial_tunnel_port_usb;
}__attribute__((packed)) settings_t;


// -- Function definitions -- //
static void bootup_blink();
static void led_blink(const uint8_t pin, const int delay_ms);
static void go_to_error();
static void read_settings();
static void print_settings();
static void swap_rx();
static void swap_tx();


// -- Instantiations -- //
static RF24           radio(PIN_CE, PIN_CSN);
static uint32_t       status = 0;
static uint32_t       tx_len = 0;
static radio_packet_t tx_buf;
static bool           connected = false;
static uint32_t       last_connectivity_sign_timestamp = 0;
static bool           take_tx_packet_from_queue = true;

static radio_mode_t   mode = RADIO_MODE_RX;
static uint32_t       swapped_to_tx_timestamp = 0;
static uint32_t       swapped_to_rx_timestamp = 0;
static uint32_t       first_received_packet_since_swap = 0;
static uint32_t       packets_received_since_last_swap = 0;

static settings_t     settings;

// Queues
static mutex_t               mutex_radio;
static mutex_t               mutex_uart;
static ArduinoQueue<uint8_t> queue_radio(8192); // Extremely oversized buffers... :)
static ArduinoQueue<uint8_t> queue_uart(8192);
static uint32_t              radio_buffer_full = 0;


// -- Logic -- //

// If we got data to send, we'll send it for max 10ms before we'll swap
// to RX to check if there's any incoming data

HardwareSerial* serial_tunnel;
HardwareSerial* serial_debug;


void setup() {
    rp2040.idleOtherCore();

    mutex_init(&mutex_radio);
    mutex_init(&mutex_uart);

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);

    // Read settings from flash
    read_settings();

    if (settings.serial_tunnel_port_usb)
    {
        serial_tunnel = &Serial;
        serial_debug = &Serial1;
    }
    else
    {
        serial_tunnel = &Serial1;
        serial_debug = &Serial;
    }

    serial_tunnel->begin(settings.serial_baudrate);
    serial_debug->begin(115200);
    DEBUG_PRINT("Booting up!");

    // SPI & Radio init
    SPI.setTX(PIN_MOSI);
    SPI.setRX(PIN_MISO);
    SPI.setSCK(PIN_SCK);
    SPI.begin();
    if (!radio.begin(&SPI))
    {
        status |= ERROR_RADIO_INIT;
    }

    if (status != 0)
    {
        go_to_error();
    }

    // Configure radio
    radio.setAddressWidth(RADIO_ADDRESS_SIZE);
    radio.setPALevel(settings.radio_pa_level);
    radio.setChannel(settings.radio_channel);
    radio.setDataRate((rf24_datarate_e) settings.radio_datarate);
    radio.enableDynamicPayloads();
    radio.setAutoAck(true);
    radio.disableAckPayload();
    radio.setRetries(2, 15);

    radio.flush_tx();
    radio.flush_rx();

    // We'll always start as RX
    swap_rx();

    // Initialization OK
    bootup_blink();
    digitalWrite(PIN_LED_GREEN, HIGH);

    rp2040.resumeOtherCore();
}

void loop() {
    // Empty the RX buffer to serial
    mutex_enter_blocking(&mutex_uart);
    while (!queue_uart.isEmpty())
    {
        serial_tunnel->write(queue_uart.dequeue());
    }
    serial_tunnel->flush();
    mutex_exit(&mutex_uart);

    // Check if there's pending serial data we should send
    int pending_bytes = serial_tunnel->available();
    if (pending_bytes > 0)
    {
        mutex_enter_blocking(&mutex_radio);

        if (queue_radio.isFull())
        {
            radio_buffer_full++;
        }
        else
        {
            for (int i = 0; i < pending_bytes; i++)
            {
                queue_radio.enqueue(serial_tunnel->read());
            }
        }
        mutex_exit(&mutex_radio);
    }
}

void setup1()
{
    // We'll wait a bit for main core to initialize OK
    delay(300);
}

void loop1()
{
    uint8_t rx_buf[RADIO_PACKET_MAX_PAYLOAD_SIZE];
    uint8_t tx_buf[RADIO_PACKET_MAX_PAYLOAD_SIZE];

    switch (mode)
    {
        case RADIO_MODE_RX:
            if (radio.available())
            {
                uint8_t rx_len = radio.getDynamicPayloadSize();
                //DEBUG_SERIAL.printf("RX LEN: %d\n", rx_len);

                if (rx_len > 0)
                {
                    radio.read(rx_buf, rx_len);
                    last_connectivity_sign_timestamp = micros();

                    if (packets_received_since_last_swap == 0)
                    {
                        first_received_packet_since_swap = micros();
                    }
                    packets_received_since_last_swap++;

                    // Insert into RX buffer
                    mutex_enter_blocking(&mutex_uart);
                    for (int i = 0; i < rx_len; i++)
                    {
                        queue_uart.enqueue(rx_buf[i]);
                    }
                    mutex_exit(&mutex_uart);
                }
            }

            // Check if we should swap to TX
            if (
                ((millis() - swapped_to_rx_timestamp) > 10) &&                                                    // At least 10ms since we swapped
                (packets_received_since_last_swap ? ((millis() - first_received_packet_since_swap) > 10) : true) && // If we've gotten any packets, we'll have to listen for 10ms
                !queue_radio.isEmpty()                                                                            // We got data to send
               )
            {
                swap_tx();
            }
            break;
        case RADIO_MODE_TX:

            if (take_tx_packet_from_queue)
            {
                // Get data that's in the pending output buffer
                mutex_enter_blocking(&mutex_radio);
                tx_len = 0;
                while ((tx_len < RADIO_PACKET_MAX_PAYLOAD_SIZE) && !queue_radio.isEmpty())
                {
                    tx_buf[tx_len] = queue_radio.dequeue();
                    tx_len++;
                }
                mutex_exit(&mutex_radio);
            }

            if (tx_len > 0)
            {
                //DEBUG_SERIAL.printf("TX: %d\n", tx_len);
                bool tx_success = radio.write(tx_buf, tx_len);

                if (tx_success)
                {
                    last_connectivity_sign_timestamp = micros();
                    take_tx_packet_from_queue = true;
                }
                else
                {   // To avoid to getting unsynched if both devices try to transmit, we'll sleep a bit for a random duration here
                    const int max_sleep_us = 5000;
                    int sleep_delay_us = ((float) (float) rand() / (float) INT_MAX) * max_sleep_us;
                    delayMicroseconds(sleep_delay_us);

                    // Since the transmission failed we want to re-send it the next transmission instead of taking data from the queue
                    take_tx_packet_from_queue = false;
                }
            }

            // Check if it's time to swap to RX
            if ((millis() - swapped_to_tx_timestamp) > 10)
            {
                swap_rx();
            }

            break;
    }

}

static void swap_rx()
{
    radio.openReadingPipe(0, settings.radio_address);
    radio.startListening();
    swapped_to_rx_timestamp = millis();
    packets_received_since_last_swap = 0;
    mode = RADIO_MODE_RX;

    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
}

static void swap_tx()
{
    radio.stopListening();
    radio.openWritingPipe(settings.radio_address);
    swapped_to_tx_timestamp = millis();
    mode = RADIO_MODE_TX;

    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
}


static void led_blink(const uint8_t pin, const int delay_ms)
{
    digitalWrite(pin, HIGH);
    delay(delay_ms);
    digitalWrite(pin, LOW);
    delay(delay_ms);
}

static void bootup_blink()
{
    for (int i = 0; i < 3; i++)
    {
        #ifdef MASTER
            led_blink(PIN_LED_GREEN, 400);
        #else
            led_blink(PIN_LED_GREEN, 200);
        #endif
    }
}

static void go_to_error()
{
    while (1)
    {
        Serial.printf("Failed initialization: %d (%d s)\n", status, millis() / 1000);
        Serial1.printf("Failed initialization: %d (%d s)\n", status, millis() / 1000);
        while (Serial1.available())
        {
            Serial.write(Serial1.read());
        }
        led_blink(PIN_LED_GREEN, 100);
        led_blink(PIN_LED_GREEN, 100);
        led_blink(PIN_LED_GREEN, 500);
    }
}

static void read_settings()
{
    settings_t* flash_settings = (settings_t*) FLASH_SETTINGS_ADDR;
    memcpy(&settings, flash_settings, sizeof(settings_t));
}

static void print_settings()
{
    serial_debug->printf("Settings size: %d\n", sizeof(settings_t));
    serial_debug->printf("address: %02x%02x%02x%02x%02x\n", settings.radio_address[0], settings.radio_address[1], settings.radio_address[2], settings.radio_address[3], settings.radio_address[4], settings.radio_address[5]);
    serial_debug->printf("radio_pa_level: %d\n", settings.radio_pa_level);
    serial_debug->printf("radio_channel: %d\n", settings.radio_channel);
    serial_debug->printf("radio_datarate: %d\n", settings.radio_datarate);
    serial_debug->printf("serial_baudrate: %u\n", settings.serial_baudrate);
    serial_debug->printf("serial_port: %d\n", settings.serial_tunnel_port_usb);
    serial_debug->printf("\n");
}