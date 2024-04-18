#include <SPI.h>
#include "ArduinoQueue.h"
#include "RF24.h"

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
#define RADIO_POWER_LEVEL     RF24_PA_MAX
#define RADIO_DATA_RATE       RF24_2MBPS
#define RADIO_CHANNEL         90
#define RADIO_ADDRESS_SIZE    5
#define RADIO_RX_PIPE         1
#define RADIO_MAX_PACKET_SIZE 32
#define RADIO_PACKET_HEADER_SIZE      1
#define RADIO_PACKET_MAX_PAYLOAD_SIZE (RADIO_MAX_PACKET_SIZE - RADIO_PACKET_HEADER_SIZE)

#define MASTER_SEND_PERIOD_MS 1

#define CONNECTION_TIMEOUT_MS 500

#ifdef MASTER
    #define TUNNEL_SERIAL Serial
#else
    #define TUNNEL_SERIAL Serial1
#endif

#ifdef DEBUG
    #define DEBUG_PRINT(msg) TUNNEL_SERIAL.println(msg)
#else
    #define DEBUG_PRINT(msg)
#endif

// -- Typedefs -- //

typedef enum
{
    ERROR_RADIO_INIT = 0b1
} error_t;

typedef struct
{
    uint8_t type;
    uint8_t payload[RADIO_PACKET_MAX_PAYLOAD_SIZE];
}__attribute__((packed)) radio_packet_t;

// -- Function definitions -- //
static void bootup_blink();
static void led_blink(const uint8_t pin, const int delay_ms);
static void go_to_error();
static void connection_changed(const bool is_connected);

// -- Instantiations -- //
static RF24 radio(PIN_CE, PIN_CSN);
static uint8_t tx_address[RADIO_ADDRESS_SIZE]  = {0x42, 0x42, 0x42, 0x42, 0x01};
static uint8_t rx_address[RADIO_ADDRESS_SIZE]  = {0x42, 0x42, 0x42, 0x42, 0x01};
static uint32_t status = 0;

static uint64_t failed_transmissions = 0;
static uint64_t successful_transmissions = 0;
static mutex_t mutex_radio;
static mutex_t mutex_uart;
static ArduinoQueue<uint8_t> queue_radio(4096);
static ArduinoQueue<uint8_t> queue_uart(4096);
static bool connected = false;
static uint32_t last_connectivity_sign_timestamp = 0;

// -- Logic -- //

void setup() {
    rp2040.idleOtherCore();

    mutex_init(&mutex_radio);
    mutex_init(&mutex_uart);

    Serial.begin(115200);
    TUNNEL_SERIAL.begin(230400);
    DEBUG_PRINT("Booting up!");
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);

    SPI.setTX(PIN_MOSI);
    SPI.setRX(PIN_MISO);
    SPI.setSCK(PIN_SCK);
    SPI.begin();

    // Radio
    if (!radio.begin(&SPI))
    {
        status |= ERROR_RADIO_INIT;
    }

    if (status != 0)
    {
        go_to_error();
    }

    // Configure radio
    radio.setPALevel(RADIO_POWER_LEVEL);
    radio.setAddressWidth(RADIO_ADDRESS_SIZE);
    radio.setChannel(RADIO_CHANNEL);
    radio.setDataRate(RADIO_DATA_RATE);
    radio.enableDynamicPayloads();
    radio.setAutoAck(true);
    radio.enableAckPayload();

    #ifdef MASTER
        radio.openWritingPipe(tx_address);
    #else
        radio.openReadingPipe(RADIO_RX_PIPE, rx_address);
        radio.startListening();
    #endif

    radio.flush_tx();
    radio.flush_rx();


    // Initialization OK
    bootup_blink();
    delay(1000);
    digitalWrite(PIN_LED_GREEN, HIGH);

    rp2040.resumeOtherCore();
}

void loop() {
    // Empty the RX buffer to serial
    mutex_enter_blocking(&mutex_uart);
    while (!queue_uart.isEmpty())
    {
        TUNNEL_SERIAL.write(queue_uart.dequeue());
    }
    mutex_exit(&mutex_uart);

    // Check if there's pending serial data we should send
    int pending_bytes = TUNNEL_SERIAL.available();
    if (pending_bytes > 0)
    {
        mutex_enter_blocking(&mutex_radio);
        for (int i = 0; i < pending_bytes; i++)
        {
            queue_radio.enqueue(TUNNEL_SERIAL.read());
        }
        mutex_exit(&mutex_radio);

        Serial.printf("Buffer size: %d\n", queue_radio.itemCount());
    }
}

void setup1()
{
    // We'll wait a bit for main core to initialize OK
    delay(300);
}

void loop1()
{
    radio_packet_t rx_buf;
    radio_packet_t tx_buf;
    uint8_t tx_len = 0;
    uint8_t rx_len = 0;

    bool is_connected = (millis() - last_connectivity_sign_timestamp) < CONNECTION_TIMEOUT_MS;
    if (is_connected != connected)
    {
        connection_changed(is_connected);
    }

    static uint32_t last_sent_packet = 0;
    bool time_to_send_packet = (queue_radio.itemCount() >= RADIO_PACKET_MAX_PAYLOAD_SIZE) || (millis() - last_sent_packet) > MASTER_SEND_PERIOD_MS;

    if (time_to_send_packet)
    {
        mutex_enter_blocking(&mutex_radio);
        while ((tx_len < RADIO_PACKET_MAX_PAYLOAD_SIZE) && !queue_radio.isEmpty())
        {
            tx_buf.payload[tx_len] = queue_radio.dequeue();
            tx_len++;
        }
        mutex_exit(&mutex_radio);


        #ifdef MASTER
            //uint32_t t00 = micros();
            bool tx_success = radio.write((uint8_t*) &tx_buf, tx_len + RADIO_PACKET_HEADER_SIZE);
            //uint32_t dtU = micros() - t00;
            //Serial.printf("DT: %d\n", dtU);
            last_sent_packet = millis();
            //Serial.printf("Success: %d (%d)\n", tx_success, tx_len);

            if (tx_success)
            {
                successful_transmissions++;
                last_connectivity_sign_timestamp = last_sent_packet;
            }
            else
            {
                // ACK not received...
                failed_transmissions++;
            }
        #else
            // As slave we can only load data into the ACK payload and wait for master to send a packet to us, which will trigger the transmission of the ACK, thus this payload.
            // The ACK FIFO holds up to 3 packets.
            last_sent_packet = millis();
            if (radio.writeAckPayload(RADIO_RX_PIPE, (uint8_t*) &tx_buf, tx_len + RADIO_PACKET_HEADER_SIZE))
            {
                successful_transmissions++;
                last_connectivity_sign_timestamp = last_sent_packet;
            }
            else
            {
                failed_transmissions++;
            }
        #endif
    }

    if (radio.available())
    {
        // ACK contains data
        last_connectivity_sign_timestamp = millis();
        rx_len = radio.getDynamicPayloadSize();
        radio.read((uint8_t*) &rx_buf, rx_len);
        //Serial.printf("RX: %d (%d) (%d)\n", rx_len, successful_transmissions, failed_transmissions);
    }


    if (rx_len > 0)
    {
        // Insert into RX buffer
        mutex_enter_blocking(&mutex_uart);
        for (int i = 0; i < (rx_len - RADIO_PACKET_HEADER_SIZE); i++)
        {
            queue_uart.enqueue(rx_buf.payload[i]);
        }
        mutex_exit(&mutex_uart);
    }
}

// -- Helper functions -- //

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

static void connection_changed(const bool is_connected)
{
    Serial.printf("Connected: %d\n", is_connected);
    connected = is_connected;
    if (connected)
    {
        led_blink(PIN_LED_RED, 75);
        led_blink(PIN_LED_RED, 75);
        led_blink(PIN_LED_RED, 75);
        digitalWrite(PIN_LED_RED, HIGH);
    }
    else
    {
        digitalWrite(PIN_LED_RED, LOW);
    }
}
