// link_layer.c

#include "link_layer.h"
#include "serial_port.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

// Misc
#define _POSIX_SOURCE 1 // POSIX compliant source

typedef struct {
    unsigned char flag;
    unsigned char a;
    unsigned char c;
    unsigned char bcc1;
} frame_header;

LinkLayer connectionParameters;

extern int fd;  // Serial port file descriptor
int expected_seq_n = 0; // Expected sequence number for reception
int seq_n = 0;          // Current sequence number for transmission

volatile int timeout_flag = 0;
int alarmCount = 0;

// Alarm handler
void alarm_handler(int signo)
{
    timeout_flag = 1;
    alarmCount++;
    printf("[ALARM] alarm_handler: Timeout #%d\n", alarmCount);
}

// Function to calculate BCC1
unsigned char BCC1(unsigned char a, unsigned char c)
{
    unsigned char bcc = a ^ c;
    printf("[BCC1] Calculated BCC1: %02X ^ %02X = %02X\n", a, c, bcc);
    return bcc;
}

// Function to calculate BCC2
unsigned char BCC2(const unsigned char *data, int size)
{
    unsigned char bcc2 = 0;
    printf("[BCC2] Calculating BCC2 for data size %d\n", size);
    for (int i = 0; i < size; i++)
    {
        bcc2 ^= data[i];
        printf("[BCC2] Intermediate BCC2 after byte %d (%02X): %02X\n", i, data[i], bcc2);
    }
    printf("[BCC2] Final BCC2: %02X\n", bcc2);
    return bcc2;
}

// Function for byte stuffing
int stuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int outputSize = 0;
    printf("[STUFFING] Starting byte stuffing. Input size: %d\n", inputSize);

    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == FLAG || input[i] == ESCAPE)
        {
            output[outputSize++] = ESCAPE;
            output[outputSize++] = input[i] ^ STUFFING_BYTE; // XOR with 0x20
            printf("[STUFFING] Byte %d (%02X) stuffed as ESCAPE followed by %02X\n", i, input[i], input[i] ^ STUFFING_BYTE);
        }
        else
        {
            output[outputSize++] = input[i];
            printf("[STUFFING] Byte %d (%02X) not stuffed\n", i, input[i]);
        }
    }
    printf("[STUFFING] Byte stuffing completed. Output size: %d\n", outputSize);
    return outputSize;
}

// Function for byte destuffing
int destuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int outputSize = 0;
    printf("[DESTUFFING] Starting byte destuffing. Input size: %d\n", inputSize);
    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == ESCAPE)
        {
            i++;
            if (i < inputSize)
            {
                output[outputSize++] = input[i] ^ STUFFING_BYTE;
                printf("[DESTUFFING] ESCAPE detected. Byte %d stuffed to %02X\n", i, input[i] ^ STUFFING_BYTE);
            }
            else
            {
                printf("[DESTUFFING] Error - ESCAPE at end of input\n");
                return -1; // Error: ESCAPE at end of input
            }
        }
        else
        {
            output[outputSize++] = input[i];
            printf("[DESTUFFING] Byte %d (%02X) destuffed\n", i, input[i]);
        }
    }
    printf("[DESTUFFING] Byte destuffing completed. Output size: %d\n", outputSize);
    return outputSize;
}

// Function to build frames
int frame_build(unsigned char *frame, unsigned char c)
{
    frame[0] = FLAG;
    printf("[FRAME_BUILD] FLAG set to %02X at position 0\n", FLAG);

    if (connectionParameters.role == LlTx)
    {
        if (c == C_SET || c == C_DISC)
        {
            frame[1] = A_TRANSMITTER_COMMAND;
            printf("[FRAME_BUILD] Role Tx: Set A to TRANSMITTER_COMMAND (%02X)\n", A_TRANSMITTER_COMMAND);
        }
        else
        {
            frame[1] = A_RECEIVER_REPLY;
            printf("[FRAME_BUILD] Role Tx: Set A to RECEIVER_REPLY (%02X)\n", A_RECEIVER_REPLY);
        }
    }
    else
    {
        if (c == C_UA || c == C_DISC)
        {
            frame[1] = A_RECEIVER_COMMAND;
            printf("[FRAME_BUILD] Role Rx: Set A to RECEIVER_COMMAND (%02X)\n", A_RECEIVER_COMMAND);
        }
        else
        {
            frame[1] = A_TRANSMITTER_REPLY;
            printf("[FRAME_BUILD] Role Rx: Set A to TRANSMITTER_REPLY (%02X)\n", A_TRANSMITTER_REPLY);
        }
    }

    frame[2] = c;
    printf("[FRAME_BUILD] Set C to %02X at position 2\n", c);

    frame[3] = BCC1(frame[1], frame[2]);
    printf("[FRAME_BUILD] Set BCC1 to %02X at position 3\n", frame[3]);

    frame[4] = FLAG;
    printf("[FRAME_BUILD] FLAG set to %02X at position 4\n", FLAG);

    printf("[FRAME_BUILD] Frame built successfully with size 5\n");
    return 5;
}

// Function to build information frames (I-frames)
int data_frame_build(unsigned char *frame, const unsigned char *data, int dataSize)
{
    printf("[DATA_FRAME_BUILD] Building data frame with data size %d\n", dataSize);
    frame[0] = FLAG;
    printf("[DATA_FRAME_BUILD] FLAG set to %02X at position 0\n", FLAG);
    frame[1] = A_TRANSMITTER_COMMAND;
    printf("[DATA_FRAME_BUILD] A set to TRANSMITTER_COMMAND (%02X) at position 1\n", A_TRANSMITTER_COMMAND);

    if (seq_n == 0)
    {
        frame[2] = C_I0;
        printf("[DATA_FRAME_BUILD] Sequence number 0: C set to C_I0 (%02X) at position 2\n", C_I0);
    }
    else
    {
        frame[2] = C_I1;
        printf("[DATA_FRAME_BUILD] Sequence number 1: C set to C_I1 (%02X) at position 2\n", C_I1);
    }

    frame[3] = BCC1(frame[1], frame[2]);
    printf("[DATA_FRAME_BUILD] BCC1 set to %02X at position 3\n", frame[3]);

    // Stuff data and BCC2
    unsigned char tempBuffer[2 * MAX_PAYLOAD_SIZE]; // Buffer to hold stuffed data
    int stuffedSize = stuffing(data, dataSize, tempBuffer);
    printf("[DATA_FRAME_BUILD] Data stuffed size: %d\n", stuffedSize);

    unsigned char bcc2 = BCC2(data, dataSize);
    printf("[DATA_FRAME_BUILD] Calculated BCC2: %02X\n", bcc2);

    // Stuff BCC2 if necessary
    unsigned char bcc2Stuffed[2];
    int bcc2Size = 1;
    if (bcc2 == FLAG || bcc2 == ESCAPE)
    {
        bcc2Stuffed[0] = ESCAPE;
        bcc2Stuffed[1] = bcc2 ^ STUFFING_BYTE;
        bcc2Size = 2;
        printf("[DATA_FRAME_BUILD] BCC2 (%02X) stuffed as ESCAPE (%02X) and %02X\n", bcc2, ESCAPE, bcc2Stuffed[1]);
    }
    else
    {
        bcc2Stuffed[0] = bcc2;
        printf("[DATA_FRAME_BUILD] BCC2 (%02X) not stuffed\n", bcc2);
    }

    // Ensure we don't exceed the frame buffer
    if (4 + stuffedSize + bcc2Size + 1 > MAX_FRAME_SIZE)
    {
        fprintf(stderr, "[DATA_FRAME_BUILD] Frame size exceeds MAX_FRAME_SIZE\n");
        return -1;
    }

    // Copy stuffed data into frame
    memcpy(frame + 4, tempBuffer, stuffedSize);
    printf("[DATA_FRAME_BUILD] Copied stuffed data to frame starting at position 4\n");

    memcpy(frame + 4 + stuffedSize, bcc2Stuffed, bcc2Size);
    printf("[DATA_FRAME_BUILD] Copied BCC2 to frame starting at position %d\n", 4 + stuffedSize);

    frame[4 + stuffedSize + bcc2Size] = FLAG;
    printf("[DATA_FRAME_BUILD] FLAG set to %02X at final position %d\n", FLAG, 4 + stuffedSize + bcc2Size);

    int frameSize = 4 + stuffedSize + bcc2Size + 1; // Header + Data + BCC2 + FLAG
    printf("[DATA_FRAME_BUILD] Data frame built successfully with size %d\n", frameSize);
    return frameSize;
}

// Alarm configuration
void alarm_config()
{
    printf("[ALARM_CONFIG] Configuring alarm handler\n");
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = alarm_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGALRM, &sa, NULL) == -1)
    {
        perror("[ALARM_CONFIG] sigaction");
        exit(1);
    }
    printf("[ALARM_CONFIG] Alarm handler configured successfully\n");
}

// Function to read a control frame
unsigned char readControlFrame(int fd)
{
    unsigned char byte, cField = 0;
    State state = START_STATE;
    frame_header temp_frame;

    printf("[READ_CONTROL_FRAME] Starting to read control frame\n");
    while (state != STOP_STATE && !timeout_flag)
    {
        int res = readByteSerialPort(&byte);
        if (res < 0)
        {
            if (errno == EINTR)
            {
                // Interrupted by signal, continue reading
                printf("[READ_CONTROL_FRAME] Read interrupted by signal\n");
                continue;
            }
            perror("[READ_CONTROL_FRAME] Error reading from serial port");
            return 0;
        }
        else if (res == 0)
        {
            continue; // No data read
        }

        switch (state)
        {
        case START_STATE:
            if (byte == FLAG)
            {
                state = FLAG_RCV;
                printf("[READ_CONTROL_FRAME] FLAG received, moving to FLAG_RCV state\n");
            }
            break;
        case FLAG_RCV:
            if (byte == A_TRANSMITTER_COMMAND || byte == A_RECEIVER_REPLY ||
                byte == A_RECEIVER_COMMAND || byte == A_TRANSMITTER_REPLY)
            {
                temp_frame.a = byte;
                state = A_RCV;
                printf("[READ_CONTROL_FRAME] A field received: %02X, moving to A_RCV state\n", byte);
            }
            else if (byte != FLAG)
            {
                state = START_STATE;
                printf("[READ_CONTROL_FRAME] Unexpected A field, resetting to START_STATE\n");
            }
            else
            {
                printf("[READ_CONTROL_FRAME] Repeated FLAG, staying in FLAG_RCV state\n");
            }
            break;
        case A_RCV:
            if (byte == C_UA || byte == C_SET || byte == C_DISC ||
                byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1)
            {
                temp_frame.c = byte;
                cField = byte;
                state = C_RCV;
                printf("[READ_CONTROL_FRAME] C field received: %02X, moving to C_RCV state\n", byte);
            }
            else if (byte == FLAG)
            {
                state = FLAG_RCV;
                printf("[READ_CONTROL_FRAME] FLAG received, moving back to FLAG_RCV state\n");
            }
            else
            {
                state = START_STATE;
                printf("[READ_CONTROL_FRAME] Unexpected C field, resetting to START_STATE\n");
            }
            break;
        case C_RCV:
            if (byte == BCC1(temp_frame.a, temp_frame.c))
            {
                state = BCC_OK;
                printf("[READ_CONTROL_FRAME] BCC1 correct, moving to BCC_OK state\n");
            }
            else if (byte == FLAG)
            {
                state = FLAG_RCV;
                printf("[READ_CONTROL_FRAME] FLAG received instead of BCC1, moving to FLAG_RCV state\n");
            }
            else
            {
                state = START_STATE;
                printf("[READ_CONTROL_FRAME] Incorrect BCC1, resetting to START_STATE\n");
            }
            break;
        case BCC_OK:
            if (byte == FLAG)
            {
                state = STOP_STATE;
                printf("[READ_CONTROL_FRAME] Final FLAG received, control frame successfully read\n");
                return cField;
            }
            else
            {
                state = START_STATE;
                printf("[READ_CONTROL_FRAME] Expected final FLAG, but received %02X. Resetting to START_STATE\n", byte);
            }
            break;
        default:
            state = START_STATE;
            printf("[READ_CONTROL_FRAME] Unknown state, resetting to START_STATE\n");
            break;
        }
    }
    printf("[READ_CONTROL_FRAME] Exiting readControlFrame due to timeout or STOP_STATE\n");
    return 0;
}

// Function to send a frame and wait for confirmation
int sendFrameAndWait(unsigned char *frame, int framesize)
{
    int attempts = 0;
    printf("[SEND_FRAME_AND_WAIT] Sending frame of size %d\n", framesize);
    while (attempts < connectionParameters.nRetransmissions)
    {
        int res = writeBytesSerialPort(frame, framesize);
        if (res < 0)
        {
            perror("[SEND_FRAME_AND_WAIT] Error writing frame to serial port");
            return -1;
        }
        printf("[SEND_FRAME_AND_WAIT] Frame sent, attempt %d\n", attempts + 1);

        // Set alarm
        timeout_flag = 0;
        alarm(connectionParameters.timeout);
        printf("[SEND_FRAME_AND_WAIT] Alarm set for %d seconds\n", connectionParameters.timeout);

        unsigned char cField = 0;

        // Wait for confirmation
        cField = readControlFrame(fd);
        printf("[SEND_FRAME_AND_WAIT] Received cField: %02X\n", cField);

        if (cField == C_UA || cField == C_RR0 || cField == C_RR1)
        {
            // Confirmation received
            alarm(0);
            printf("[SEND_FRAME_AND_WAIT] Confirmation received: %02X. Success.\n", cField);
            return 1;
        }
        else if (cField == C_REJ0 || cField == C_REJ1)
        {
            // Rejection received, retransmit
            alarm(0);
            attempts++;
            printf("[SEND_FRAME_AND_WAIT] REJ received: %02X. Retransmitting...\n", cField);
            continue;
        }
        else if (timeout_flag)
        {
            // Timeout, increment attempts
            attempts++;
            timeout_flag = 0;
            printf("[SEND_FRAME_AND_WAIT] Timeout occurred. Attempt %d/%d\n", attempts, connectionParameters.nRetransmissions);
            continue;
        }
        else
        {
            // Unexpected frame received, ignore
            printf("[SEND_FRAME_AND_WAIT] Unexpected frame received: %02X. Ignoring and continuing.\n", cField);
            continue;
        }
    }

    printf("[SEND_FRAME_AND_WAIT] Max attempts (%d) exceeded. Failed to send frame.\n", connectionParameters.nRetransmissions);
    return -1;
}

//////////////////////////////////////////////
// LLOPEN
//////////////////////////////////////////////
int llopen(LinkLayer cParams)
{
    printf("[LLOPEN] Initializing llopen with role %s\n",
           (cParams.role == LlTx) ? "LlTx" : "LlRx");
    connectionParameters = cParams;
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0)
    {
        perror("[LLOPEN] Error opening serial port");
        return -1;
    }
    printf("[LLOPEN] Serial port %s opened successfully\n", connectionParameters.serialPort);

    // Configure alarm
    alarm_config();

    if (connectionParameters.role == LlTx)
    {
        // Transmitter: send SET and wait for UA
        unsigned char set_frame[5];
        frame_build(set_frame, C_SET);
        printf("[LLOPEN] Transmitter: Sending SET frame\n");

        if (sendFrameAndWait(set_frame, 5) < 0)
        {
            fprintf(stderr, "[LLOPEN] Failed to send SET frame or receive UA frame\n");
            closeSerialPort();
            return -1;
        }

        // Connection established
        printf("[LLOPEN] Connection established (Transmitter)\n");
        return 1;
    }
    else if (connectionParameters.role == LlRx)
    {
        // Receiver: wait for SET and send UA
        unsigned char byte;
        State state = START_STATE;
        frame_header temp_frame;

        printf("[LLOPEN] Receiver: Waiting to receive SET frame\n");
        while (1)
        {
            int res = readByteSerialPort(&byte);
            if (res < 0)
            {
                perror("[LLOPEN (Receiver)] Error reading from serial port");
                return -1;
            }
            else if (res == 0)
            {
                continue; // No data read
            }

            switch (state)
            {
            case START_STATE:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    printf("[LLOPEN (Receiver)] FLAG received, moving to FLAG_RCV state\n");
                }
                break;
            case FLAG_RCV:
                if (byte == A_TRANSMITTER_COMMAND)
                {
                    temp_frame.a = byte;
                    state = A_RCV;
                    printf("[LLOPEN (Receiver)] A field received: %02X, moving to A_RCV state\n", byte);
                }
                else if (byte != FLAG)
                {
                    state = START_STATE;
                    printf("[LLOPEN (Receiver)] Unexpected A field, resetting to START_STATE\n");
                }
                break;
            case A_RCV:
                if (byte == C_SET)
                {
                    temp_frame.c = byte;
                    state = C_RCV;
                    printf("[LLOPEN (Receiver)] C field received: %02X (C_SET), moving to C_RCV state\n", byte);
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    printf("[LLOPEN (Receiver)] FLAG received, moving back to FLAG_RCV state\n");
                }
                else
                {
                    state = START_STATE;
                    printf("[LLOPEN (Receiver)] Unexpected C field, resetting to START_STATE\n");
                }
                break;
            case C_RCV:
                if (byte == BCC1(temp_frame.a, temp_frame.c))
                {
                    state = BCC_OK;
                    printf("[LLOPEN (Receiver)] BCC1 correct, moving to BCC_OK state\n");
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    printf("[LLOPEN (Receiver)] FLAG received instead of BCC1, moving to FLAG_RCV state\n");
                }
                else
                {
                    state = START_STATE;
                    printf("[LLOPEN (Receiver)] Incorrect BCC1, resetting to START_STATE\n");
                }
                break;
            case BCC_OK:
                if (byte == FLAG)
                {
                    // SET frame received, send UA
                    unsigned char ua_frame[5];
                    frame_build(ua_frame, C_UA);
                    int res_write = writeBytesSerialPort(ua_frame, 5);
                    if (res_write < 0)
                    {
                        perror("[LLOPEN (Receiver)] Error sending UA frame");
                        closeSerialPort();
                        return -1;
                    }
                    printf("[LLOPEN (Receiver)] SET frame received. UA frame sent successfully.\n");

                    // Connection established
                    printf("[LLOPEN (Receiver)] Connection established (Receiver)\n");
                    return 1;
                }
                else
                {
                    state = START_STATE;
                    printf("[LLOPEN (Receiver)] Expected final FLAG, but received %02X. Resetting to START_STATE\n", byte);
                }
                break;
            default:
                state = START_STATE;
                printf("[LLOPEN (Receiver)] Unknown state, resetting to START_STATE\n");
                break;
            }
        }
    }

    printf("[LLOPEN] Role not recognized. Failed to open connection.\n");
    return -1;
}

//////////////////////////////////////////////
// LLWRITE
//////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    printf("[LLWRITE] Preparing to write %d bytes\n", bufSize);
    if (bufSize > MAX_PAYLOAD_SIZE)
    {
        fprintf(stderr, "[LLWRITE] bufSize exceeds MAX_PAYLOAD_SIZE\n");
        return -1;
    }

    // Build data frame
    unsigned char frame[MAX_FRAME_SIZE];
    int frameSize = data_frame_build(frame, buf, bufSize);
    if (frameSize < 0)
    {
        fprintf(stderr, "[LLWRITE] Error building data frame\n");
        return -1;
    }

    printf("[LLWRITE] Data frame built with size %d\n", frameSize);

    // Send frame and wait for confirmation
    if (sendFrameAndWait(frame, frameSize) < 0)
    {
        fprintf(stderr, "[LLWRITE] Error sending data frame or receiving acknowledgment\n");
        return -1;
    }

    // Update sequence number
    printf("[LLWRITE] Acknowledgment received. Updating sequence number from %d to %d\n", seq_n, 1 - seq_n);
    seq_n = 1 - seq_n;

    printf("[LLWRITE] llwrite completed successfully\n");
    return bufSize;
}

//////////////////////////////////////////////
// LLREAD
//////////////////////////////////////////////
int llread(unsigned char *packet)
{
    printf("[LLREAD] Starting to read a frame\n");
    unsigned char dataBuffer[2 * MAX_PAYLOAD_SIZE]; // To accommodate possible stuffing
    int dataSize = 0;
    State state = START_STATE;
    unsigned char byte;
    unsigned char a_field = 0, c_field = 0, bcc1 = 0;

    while (1)
    {
        int res = readByteSerialPort(&byte);
        if (res < 0)
        {
            perror("[LLREAD] Error reading from serial port");
            return -1;
        }
        else if (res == 0)
        {
            continue; // No data read
        }

        switch (state)
        {
        case START_STATE:
            if (byte == FLAG)
            {
                state = FLAG_RCV;
                printf("[LLREAD] FLAG received, moving to FLAG_RCV state\n");
            }
            break;
        case FLAG_RCV:
            if (byte == A_TRANSMITTER_COMMAND)
            {
                a_field = byte;
                state = A_RCV;
                printf("[LLREAD] A field received: %02X, moving to A_RCV state\n", byte);
            }
            else if (byte == FLAG)
            {
                printf("[LLREAD] Repeated FLAG received, staying in FLAG_RCV state\n");
                // Stay in FLAG_RCV
            }
            else
            {
                state = START_STATE;
                printf("[LLREAD] Unexpected A field: %02X, resetting to START_STATE\n", byte);
            }
            break;
        case A_RCV:
            if (byte == C_I0 || byte == C_I1)
            {
                c_field = byte;
                state = C_RCV;
                printf("[LLREAD] C field received: %02X, moving to C_RCV state\n", byte);
            }
            else if (byte == FLAG)
            {
                state = FLAG_RCV;
                printf("[LLREAD] FLAG received, moving back to FLAG_RCV state\n");
            }
            else
            {
                state = START_STATE;
                printf("[LLREAD] Unexpected C field: %02X, resetting to START_STATE\n", byte);
            }
            break;
        case C_RCV:
            if (byte == BCC1(a_field, c_field))
            {
                bcc1 = byte;
                state = BCC_OK;
                printf("[LLREAD] BCC1 correct: %02X, moving to BCC_OK state\n", byte);
            }
            else if (byte == FLAG)
            {
                state = FLAG_RCV;
                printf("[LLREAD] FLAG received instead of BCC1, moving to FLAG_RCV state\n");
            }
            else
            {
                state = START_STATE;
                printf("[LLREAD] Incorrect BCC1: %02X, resetting to START_STATE\n", byte);
            }
            break;
        case BCC_OK:
            if (byte == FLAG)
            {
                // Empty frame, ignore
                state = FLAG_RCV;
                printf("[LLREAD] Empty frame detected, moving to FLAG_RCV state\n");
            }
            else
            {
                if (dataSize >= sizeof(dataBuffer))
                {
                    fprintf(stderr, "[LLREAD] Data buffer overflow\n");
                    return -1;
                }
                dataBuffer[dataSize++] = byte;
                state = DATA_RCV;
                printf("[LLREAD] Starting to receive data. First data byte: %02X\n", byte);
            }
            break;
        case DATA_RCV:
            if (byte == FLAG)
            {
                // Frame complete
                state = STOP_STATE;
                printf("[LLREAD] Final FLAG received, frame complete\n");
            }
            else
            {
                if (dataSize >= sizeof(dataBuffer))
                {
                    fprintf(stderr, "[LLREAD] Data buffer overflow while receiving data\n");
                    return -1;
                }
                dataBuffer[dataSize++] = byte;
                printf("[LLREAD] Received data byte: %02X\n", byte);
            }
            break;
        default:
            state = START_STATE;
            printf("[LLREAD] Unknown state, resetting to START_STATE\n");
            break;
        }

        if (state == STOP_STATE)
        {
            printf("[LLREAD] Processing received frame\n");
            // Process received frame
            // Destuff data
            unsigned char destuffedData[MAX_PAYLOAD_SIZE];
            int destuffedSize = destuffing(dataBuffer, dataSize, destuffedData);
            if (destuffedSize < 0)
            {
                // Error in destuffing
                fprintf(stderr, "[LLREAD] Error in destuffing data\n");
                // Send REJ
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                printf("[LLREAD] Sending REJ frame: %02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0)
                {
                    perror("[LLREAD] Error sending REJ frame");
                }
                state = START_STATE;
                continue;
            }

            // Separate BCC2
            if (destuffedSize < 1)
            {
                fprintf(stderr, "[LLREAD] Destuffed data too small\n");
                // Send REJ
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                printf("[LLREAD] Sending REJ frame: %02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0)
                {
                    perror("[LLREAD] Error sending REJ frame");
                }
                state = START_STATE;
                continue;
            }

            unsigned char receivedBCC2 = destuffedData[destuffedSize - 1];
            unsigned char calculatedBCC2 = BCC2(destuffedData, destuffedSize - 1);

            if (receivedBCC2 != calculatedBCC2)
            {
                // BCC2 error, send REJ
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                printf("[LLREAD] BCC2 mismatch. Sending REJ frame: %02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0)
                {
                    perror("[LLREAD] Error sending REJ frame");
                }
                printf("[LLREAD] BCC2 error, sent REJ frame\n");
                state = START_STATE;
                continue;
            }
            else
            {
                // BCC2 correct
                int received_seq_n = (c_field == C_I0) ? 0 : 1;
                printf("[LLREAD] BCC2 correct. Received sequence number: %d\n", received_seq_n);

                if (received_seq_n == expected_seq_n)
                {
                    // Expected sequence number, send RR and deliver data
                    unsigned char rr_frame[5];
                    unsigned char rr_c = (expected_seq_n == 0) ? C_RR1 : C_RR0;
                    frame_build(rr_frame, rr_c);
                    printf("[LLREAD] Sequence number matches. Sending RR frame: %02X\n", rr_c);
                    if (writeBytesSerialPort(rr_frame, 5) < 0)
                    {
                        perror("[LLREAD] Error sending RR frame");
                    }

                    // Update expected sequence number
                    expected_seq_n = 1 - expected_seq_n;
                    printf("[LLREAD] Updated expected sequence number to %d\n", expected_seq_n);

                    // Copy data to packet
                    memcpy(packet, destuffedData, destuffedSize - 1);
                    printf("[LLREAD] Data copied to packet. Data size: %d bytes\n", destuffedSize - 1);
                    return destuffedSize - 1;
                }
                else
                {
                    // Duplicate frame, resend last RR
                    unsigned char rr_frame[5];
                    unsigned char rr_c = (received_seq_n == 0) ? C_RR1 : C_RR0;
                    frame_build(rr_frame, rr_c);
                    printf("[LLREAD] Duplicate frame received. Resending RR frame: %02X\n", rr_c);
                    if (writeBytesSerialPort(rr_frame, 5) < 0)
                    {
                        perror("[LLREAD] Error sending RR frame");
                    }
                    printf("[LLREAD] Duplicate frame detected, RR frame sent\n");
                    state = START_STATE;
                    continue;
                }
            }
        }
    }
}

//////////////////////////////////////////////
// LLCLOSE
//////////////////////////////////////////////
int llclose(int showStatistics)
{
    printf("[LLCLOSE] Initiating llclose with showStatistics=%d\n", showStatistics);
    if (connectionParameters.role == LlTx)
    {
        // Transmitter: send DISC and wait for DISC, then send UA
        unsigned char disc_frame[5];
        frame_build(disc_frame, C_DISC);
        printf("[LLCLOSE] Transmitter: Sending DISC frame\n");

        int attempts = 0;
        while (attempts < connectionParameters.nRetransmissions)
        {
            int res = writeBytesSerialPort(disc_frame, 5);
            if (res < 0)
            {
                perror("[LLCLOSE] Error sending DISC frame");
                return -1;
            }
            printf("[LLCLOSE] DISC frame sent, attempt %d\n", attempts + 1);

            // Set alarm
            timeout_flag = 0;
            alarm(connectionParameters.timeout);
            printf("[LLCLOSE] Alarm set for %d seconds while waiting for DISC\n", connectionParameters.timeout);

            unsigned char cField = readControlFrame(fd);
            printf("[LLCLOSE] Received cField: %02X\n", cField);

            if (cField == C_DISC)
            {
                // Received DISC from receiver, send UA
                unsigned char ua_frame[5];
                frame_build(ua_frame, C_UA);
                res = writeBytesSerialPort(ua_frame, 5);
                if (res < 0)
                {
                    perror("[LLCLOSE] Error sending UA frame");
                    return -1;
                }
                printf("[LLCLOSE] DISC frame received. UA frame sent successfully. Connection closed (Transmitter)\n");

                // Close serial port
                if (closeSerialPort() < 0)
                {
                    perror("[LLCLOSE] Error closing serial port");
                    return -1;
                }

                return 1;
            }
            else if (timeout_flag)
            {
                // Timeout, increment attempts
                attempts++;
                timeout_flag = 0;
                printf("[LLCLOSE] Timeout occurred while waiting for DISC. Attempt %d/%d\n", attempts, connectionParameters.nRetransmissions);
                continue;
            }
            else
            {
                // Unexpected frame received, ignore
                printf("[LLCLOSE] Unexpected frame received: %02X. Ignoring and continuing.\n", cField);
                continue;
            }
        }

        printf("[LLCLOSE] Max attempts (%d) exceeded while sending DISC. Failed to close connection.\n", connectionParameters.nRetransmissions);
        return -1;
    }
    else if (connectionParameters.role == LlRx)
    {
        // Receiver: wait for DISC, send DISC, then wait for UA
        unsigned char byte;
        State state = START_STATE;
        frame_header temp_frame;
        int attempts = 0;

        printf("[LLCLOSE] Receiver: Waiting to receive DISC frame\n");
        while (1)
        {
            int res = readByteSerialPort(&byte);
            if (res < 0)
            {
                perror("[LLCLOSE (Receiver)] Error reading from serial port");
                return -1;
            }
            else if (res == 0)
            {
                continue; // No data read
            }

            switch (state)
            {
            case START_STATE:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    printf("[LLCLOSE (Receiver)] FLAG received, moving to FLAG_RCV state\n");
                }
                break;
            case FLAG_RCV:
                if (byte == A_TRANSMITTER_COMMAND)
                {
                    temp_frame.a = byte;
                    state = A_RCV;
                    printf("[LLCLOSE (Receiver)] A field received: %02X, moving to A_RCV state\n", byte);
                }
                else if (byte != FLAG)
                {
                    state = START_STATE;
                    printf("[LLCLOSE (Receiver)] Unexpected A field: %02X, resetting to START_STATE\n", byte);
                }
                break;
            case A_RCV:
                if (byte == C_DISC)
                {
                    temp_frame.c = byte;
                    state = C_RCV;
                    printf("[LLCLOSE (Receiver)] C field received: %02X (C_DISC), moving to C_RCV state\n", byte);
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    printf("[LLCLOSE (Receiver)] FLAG received, moving back to FLAG_RCV state\n");
                }
                else
                {
                    state = START_STATE;
                    printf("[LLCLOSE (Receiver)] Unexpected C field: %02X, resetting to START_STATE\n", byte);
                }
                break;
            case C_RCV:
                if (byte == BCC1(temp_frame.a, temp_frame.c))
                {
                    state = BCC_OK;
                    printf("[LLCLOSE (Receiver)] BCC1 correct: %02X, moving to BCC_OK state\n", byte);
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    printf("[LLCLOSE (Receiver)] FLAG received instead of BCC1, moving to FLAG_RCV state\n");
                }
                else
                {
                    state = START_STATE;
                    printf("[LLCLOSE (Receiver)] Incorrect BCC1: %02X, resetting to START_STATE\n", byte);
                }
                break;
            case BCC_OK:
                if (byte == FLAG)
                {
                    // DISC frame received, send DISC
                    unsigned char disc_frame[5];
                    frame_build(disc_frame, C_DISC);
                    res = writeBytesSerialPort(disc_frame, 5);
                    if (res < 0)
                    {
                        perror("[LLCLOSE (Receiver)] Error sending DISC frame");
                        return -1;
                    }
                    printf("[LLCLOSE (Receiver)] DISC frame received. DISC frame sent successfully (Receiver)\n");

                    // Wait for UA from transmitter
                    timeout_flag = 0;
                    alarm(connectionParameters.timeout);
                    printf("[LLCLOSE (Receiver)] Alarm set for %d seconds while waiting for UA\n", connectionParameters.timeout);

                    unsigned char cField = readControlFrame(fd);
                    printf("[LLCLOSE (Receiver)] Received cField: %02X\n", cField);

                    if (cField == C_UA)
                    {
                        // UA received, connection closed
                        printf("[LLCLOSE (Receiver)] UA received. Connection closed successfully (Receiver)\n");
                        if (closeSerialPort() < 0)
                        {
                            perror("[LLCLOSE (Receiver)] Error closing serial port");
                            return -1;
                        }
                        return 1;
                    }
                    else if (timeout_flag)
                    {
                        // Timeout occurred
                        printf("[LLCLOSE (Receiver)] Timeout occurred while waiting for UA. Closing connection.\n");
                        if (closeSerialPort() < 0)
                        {
                            perror("[LLCLOSE (Receiver)] Error closing serial port");
                            return -1;
                        }
                        return 1;
                    }
                    else
                    {
                        // Unexpected frame received, ignore
                        printf("[LLCLOSE (Receiver)] Unexpected frame received: %02X. Ignoring and continuing.\n", cField);
                        continue;
                    }
                }
                else
                {
                    state = START_STATE;
                    printf("[LLCLOSE (Receiver)] Expected final FLAG, but received %02X. Resetting to START_STATE\n", byte);
                }
                break;
            default:
                state = START_STATE;
                printf("[LLCLOSE (Receiver)] Unknown state, resetting to START_STATE\n");
                break;
            }

            if (attempts >= connectionParameters.nRetransmissions)
            {
                printf("[LLCLOSE (Receiver)] Max attempts (%d) exceeded while waiting for DISC\n", connectionParameters.nRetransmissions);
                return -1;
            }
        }
    }

    printf("[LLCLOSE] Role not recognized. Failed to close connection.\n");
    return -1;
}
