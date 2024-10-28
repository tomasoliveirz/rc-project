#ifndef CONSTANTS_H
#define CONSTANTS_H

#define FLAG 0x7E
#define ESCAPE 0x7D

#define A_TRANSMITTER_COMMAND 0x03
#define A_RECEIVER_REPLY 0x03

#define A_RECEIVER_COMMAND 0x01
#define A_TRANSMITTER_REPLY 0x01

#define A_SENDER          0x03  // Endereço do transmissor
#define A_RECEIVER        0x01  // Endereço do receptor
#define A_SENDER_TO_RECEIVER 0x03
#define A_RECEIVER_TO_SENDER 0x01

#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55

#define C_I0 0x00
#define C_I1 0x80

#define TIMEOUT_SEC 1

#define MAX_FRAME_SIZE 2048 // ...

#define FALSE 0
#define TRUE 1

#define STUFFING_BYTE 0x20

typedef enum {
    frame_I,
    frame_S,
    frame_U
} FrameType;

typedef enum {
    START_STATE,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA_RCV,
    STOP_STATE
} State;


#endif // CONSTANTS_H