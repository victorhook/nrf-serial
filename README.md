## Re-sending ACKs
It seems that sometimes ACK payload gets lost from slave to master.
In order to detect this sequence numbers are used.

The first byte of the 32 bytes payload consists of the sequence number.
```
typedef struct
{
    uint8_t seq_nbr: 7;
    uint8_t resend: 1;
    uint8_t payload[RADIO_PACKET_MAX_PAYLOAD_SIZE];
}__attribute__((packed)) radio_packet_t;
```

Master handles the sequence number `seq_nbr` which starts at 0 and increments after each transmitted packet. The slave simply copies the received sequence number into the payload. The bit `resend` tells the slave that it needs to re-send the given sequence number since the master hasn't received it. For this to work the slave must have a history of sent packets that it can use to re-send old packets.