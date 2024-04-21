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
    #define RADIO_ADDRESS_SIZE     5
    #define RADIO_RX_PIPE          0
    #define RADIO_MAX_PACKET_SIZE  32
    #define RADIO_DEFAULT_CHANNEL  90
    #define RADIO_DEFAULT_DATARATE RF24_2MBPS
    #define RADIO_DEFAULT_PA_LEVEL RF24_PA_MAX
    #define RADIO_PACKET_HEADER_SIZE      1
    #define RADIO_PACKET_MAX_PAYLOAD_SIZE (RADIO_MAX_PACKET_SIZE - RADIO_PACKET_HEADER_SIZE)

    #define FLASH_SETTINGS_ADDR 0x101ff000

    #define RADIO_PUSH_TIMEOUT_US 700
    #define CONNECTION_TIMEOUT_MS 500


    #ifdef MASTER
        #define TUNNEL_SERIAL Serial
        #define DEBUG_SERIAL Serial1
    #else
        #define TUNNEL_SERIAL Serial1
        #define DEBUG_SERIAL Serial
    #endif

    #ifdef DEBUG
        #define DEBUG_PRINT(msg) DEBUG_SERIAL.println(msg)
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

    typedef struct
    {
        uint8_t tx_address[RADIO_ADDRESS_SIZE];
        uint8_t rx_address1[RADIO_ADDRESS_SIZE];
        uint8_t rx_address2[RADIO_ADDRESS_SIZE];
        uint8_t rx_address3[RADIO_ADDRESS_SIZE];
        uint8_t rx_address4[RADIO_ADDRESS_SIZE];
        uint8_t rx_address5[RADIO_ADDRESS_SIZE];
        uint8_t radio_pa_level;
        uint8_t radio_channel;
        uint8_t radio_datarate;
        uint32_t serial_baudrate;
    }__attribute__((packed)) settings_t;

    // -- Function definitions -- //
    static void bootup_blink();
    static void led_blink(const uint8_t pin, const int delay_ms);
    static void go_to_error();
    static void read_settings();
    static void print_settings();
    static void connection_changed(const bool is_connected);

    // -- Instantiations -- //
    static RF24 radio(PIN_CE, PIN_CSN);
    static uint32_t status = 0;
    static uint32_t rx_total = 0;
    static uint32_t tx_total = 0;
    static uint32_t rx_fifo_full = 0;
    static uint32_t tx_fifo_full = 0;

    static uint32_t failed_transmissions = 0;
    static uint32_t successful_transmissions = 0;
    static bool connected = false;
    static uint32_t last_connectivity_sign_timestamp = 0;

    static settings_t settings;

    // Queues
    static mutex_t mutex_radio;
    static mutex_t mutex_uart;
    static ArduinoQueue<uint8_t> queue_radio(4096);
    static ArduinoQueue<uint8_t> queue_uart(4096);
    static uint32_t radio_buffer_full = 0;


    // -- Logic -- //

    void setup() {
        rp2040.idleOtherCore();

        mutex_init(&mutex_radio);
        mutex_init(&mutex_uart);

        DEBUG_SERIAL.begin(115200);
        DEBUG_PRINT("Booting up!");
        pinMode(PIN_LED_GREEN, OUTPUT);
        pinMode(PIN_LED_RED, OUTPUT);

        // Read settings from flash
        read_settings();
        TUNNEL_SERIAL.begin(settings.serial_baudrate);

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
        radio.setAddressWidth(RADIO_ADDRESS_SIZE);
        radio.setPALevel(settings.radio_pa_level);
        radio.setChannel(settings.radio_channel);
        radio.setDataRate((rf24_datarate_e) settings.radio_datarate);
        radio.enableDynamicPayloads();
        radio.setAutoAck(true);
        radio.enableAckPayload();
        radio.setRetries(2, 15);

        #ifdef MASTER
            radio.openWritingPipe(settings.tx_address);
        #else
            radio.openReadingPipe(0, settings.rx_address1);
            radio.openReadingPipe(1, settings.rx_address2);
            radio.openReadingPipe(2, settings.rx_address3);
            radio.openReadingPipe(3, settings.rx_address4);
            radio.openReadingPipe(4, settings.rx_address5);
            radio.startListening();
        #endif

        radio.flush_tx();
        radio.flush_rx();

        //delay(5000);
        //print_settings();

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
            TUNNEL_SERIAL.write(queue_uart.dequeue());
        }
        mutex_exit(&mutex_uart);

        // Check if there's pending serial data we should send
        int pending_bytes = TUNNEL_SERIAL.available();
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
                    queue_radio.enqueue(TUNNEL_SERIAL.read());
                }
            }
            mutex_exit(&mutex_radio);

            //Serial.printf("Buffer size: %d\n", queue_radio.itemCount());
        }
    }

    void setup1()
    {
        // We'll wait a bit for main core to initialize OK
        delay(300);
    }



    void loop1()
    {
        static radio_packet_t tx_buf;
        static uint8_t        tx_len = 0;
        radio_packet_t        rx_buf;
        uint8_t               rx_len = 0;
        static bool           take_tx_data_from_queue = true;
        static uint32_t       last_sent_packet = 0;

        bool is_connected = (micros() - last_connectivity_sign_timestamp) < (CONNECTION_TIMEOUT_MS * 1000);
        if (is_connected != connected)
        {
            connection_changed(is_connected);
        }

        bool tx_fifo_is_full = radio.isFifo(true, false);
        bool rx_fifo_is_full = radio.isFifo(false, false);

        #ifdef MASTER
            bool time_to_send_packet = (queue_radio.itemCount() >= RADIO_PACKET_MAX_PAYLOAD_SIZE) || (micros() - last_sent_packet) > RADIO_PUSH_TIMEOUT_US;
            uint32_t ack_payload_timeout_us = 0;
        #else
            //bool time_to_send_packet
            static const uint32_t ack_payload_min_timeout_us       = 10  * 1000;
            static const uint32_t ack_payload_max_timeout_us       = 250 * 1000;
            static const uint32_t ack_payload_backoff_increment_us = 3000;
            static uint32_t ack_payload_timeout_us                 = ack_payload_min_timeout_us;

            // 1. Haven't sent last payload, so we'll try again
            // 2. Lots of data pending, so we'll try straight away
            // 3. At least 1 byte waiting, and we haven't sent for some time
            bool time_to_send_packet = (!take_tx_data_from_queue) ||
                                       (queue_radio.itemCount() >= RADIO_PACKET_MAX_PAYLOAD_SIZE) ||
                                       ((queue_radio.itemCount() > 0) && (micros() - last_sent_packet) > ack_payload_timeout_us);
        #endif


        if (time_to_send_packet)
        {
            if (take_tx_data_from_queue)
            {
                mutex_enter_blocking(&mutex_radio);
                tx_len = 0;
                while ((tx_len < RADIO_PACKET_MAX_PAYLOAD_SIZE) && !queue_radio.isEmpty())
                {
                    tx_buf.payload[tx_len] = queue_radio.dequeue();
                    tx_len++;
                }
                mutex_exit(&mutex_radio);
            }


            #ifdef MASTER
                //uint32_t t00 = micros();
                bool tx_success = radio.write((uint8_t*) &tx_buf, tx_len + RADIO_PACKET_HEADER_SIZE);
                tx_total += tx_len;
                //uint32_t dtU = micros() - t00;
                //Serial.printf("DT: %d\n", dtU);
                last_sent_packet = micros();
                //Serial.printf("Success: %d (%d, %d, %d)\n", tx_success, tx_len, successful_transmissions, failed_transmissions);

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
                // The ACK FIFO holds up to 3 packets, and we don't know if it's full or not.
                last_sent_packet = micros();
                if (radio.writeAckPayload(RADIO_RX_PIPE, (uint8_t*) &tx_buf, tx_len + RADIO_PACKET_HEADER_SIZE))
                {
                    successful_transmissions++;
                    tx_total += tx_len;
                    last_connectivity_sign_timestamp = last_sent_packet;
                    ack_payload_timeout_us = constrain(ack_payload_timeout_us - ack_payload_backoff_increment_us, ack_payload_min_timeout_us, ack_payload_max_timeout_us);
                    take_tx_data_from_queue = true;
                }
                else
                {
                    failed_transmissions++;
                    ack_payload_timeout_us = constrain(ack_payload_timeout_us + ack_payload_backoff_increment_us, ack_payload_min_timeout_us, ack_payload_max_timeout_us);
                    take_tx_data_from_queue = false;
                    delayMicroseconds(500);
                }
            #endif
        }

        //radio.printPrettyDetails();

        static uint32_t last_p = 0;
        if ((millis() - last_p) > 1000)
        {
            DEBUG_SERIAL.print("S:"); DEBUG_SERIAL.print(successful_transmissions); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("F:"); DEBUG_SERIAL.print(failed_transmissions); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("RX:"); DEBUG_SERIAL.print(rx_total); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("TX:"); DEBUG_SERIAL.print(tx_total); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("TxL:"); DEBUG_SERIAL.print(tx_len); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("QL:"); DEBUG_SERIAL.print(queue_radio.itemCount()); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("RxF:"); DEBUG_SERIAL.print(rx_fifo_full); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("TxF:"); DEBUG_SERIAL.print(tx_fifo_full); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("Full:"); DEBUG_SERIAL.print(radio_buffer_full); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("Ser:"); DEBUG_SERIAL.print(TUNNEL_SERIAL.available()); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("Ack:"); DEBUG_SERIAL.print(ack_payload_timeout_us); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print("\n");
            last_p = millis();
        }

        if (tx_fifo_is_full)
        {
            tx_fifo_full++;
        }
        if (rx_fifo_is_full)
        {
            rx_fifo_full++;
        }

        //DEBUG_SERIAL.printf("FIFO: %d, %d\n", , radio.isFifo(false, false));

        if (radio.available())
        {
            // ACK contains data
            last_connectivity_sign_timestamp = micros();
            rx_len = radio.getDynamicPayloadSize();
            if (rx_len > 0)
            {
                radio.read((uint8_t*) &rx_buf, rx_len);
                //DEBUG_SERIAL.printf("RX_LEN: %d, %d\n", rx_len, rx_total);
                rx_total += (rx_len - RADIO_PACKET_HEADER_SIZE);
                //Serial.printf("RX: %d (%d) (%d)\n", rx_len, successful_transmissions, failed_transmissions);
            }
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
        //Serial.printf("Connected: %d\n", is_connected);
        connected = is_connected;
        if (connected)
        {
            led_blink(PIN_LED_RED, 50);
            led_blink(PIN_LED_RED, 50);
            digitalWrite(PIN_LED_RED, HIGH);
        }
        else
        {
            digitalWrite(PIN_LED_RED, LOW);
        }
    }


    static void read_settings()
    {
        settings_t* flash_settings = (settings_t*) FLASH_SETTINGS_ADDR;
        memcpy(&settings, flash_settings, sizeof(settings_t));
    }

    static void print_settings()
    {
        DEBUG_SERIAL.printf("Settings size: %d\n", sizeof(settings_t));
        DEBUG_SERIAL.printf("%02x%02x%02x%02x%02x\n", settings.tx_address[0], settings.tx_address[1], settings.tx_address[2], settings.tx_address[3], settings.tx_address[4], settings.tx_address[5]);
        DEBUG_SERIAL.printf("%02x%02x%02x%02x%02x\n", settings.rx_address1[0], settings.rx_address1[1], settings.rx_address1[2], settings.rx_address1[3], settings.rx_address1[4], settings.rx_address1[5]);
        DEBUG_SERIAL.printf("%02x%02x%02x%02x%02x\n", settings.rx_address2[0], settings.rx_address2[1], settings.rx_address2[2], settings.rx_address2[3], settings.rx_address2[4], settings.rx_address2[5]);
        DEBUG_SERIAL.printf("%02x%02x%02x%02x%02x\n", settings.rx_address3[0], settings.rx_address3[1], settings.rx_address3[2], settings.rx_address3[3], settings.rx_address3[4], settings.rx_address3[5]);
        DEBUG_SERIAL.printf("%02x%02x%02x%02x%02x\n", settings.rx_address4[0], settings.rx_address4[1], settings.rx_address4[2], settings.rx_address4[3], settings.rx_address4[4], settings.rx_address4[5]);
        DEBUG_SERIAL.printf("%02x%02x%02x%02x%02x\n", settings.rx_address5[0], settings.rx_address5[1], settings.rx_address5[2], settings.rx_address5[3], settings.rx_address5[4], settings.rx_address5[5]);
        DEBUG_SERIAL.printf("%d\n", settings.radio_pa_level);
        DEBUG_SERIAL.printf("%d\n", settings.radio_channel);
        DEBUG_SERIAL.printf("%d\n", settings.radio_datarate);
        DEBUG_SERIAL.printf("%u\n", settings.serial_baudrate);
        DEBUG_SERIAL.printf("\n");
    }