// link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

// misc
#define _POSIX_SOURCE 1 // posix compliant source

typedef struct {
    unsigned char flag;
    unsigned char a;
    unsigned char c;
    unsigned char bcc1;
    unsigned char data[MAX_FRAME_SIZE];
    unsigned char bcc2;
} frame;

LinkLayer connectionParameters;

extern int fd;  // descritor da porta série
int expected_seq_n = 0; // número de sequência esperado para recepção
int seq_n = 0; // número de sequência atual para transmissão

volatile int timeout_flag = 0;
int alarmCount = 0;

// handler do alarme
void alarm_handler(int signo)
{
    timeout_flag = 1;
    alarmCount++;
    printf("alarm_handler: Timeout #%d\n", alarmCount);
}

void clean_frame(frame *f)
{
    f->flag = 0;
    f->a = 0;
    f->c = 0;
    f->bcc1 = 0;
}

// função para calcular o BCC1
unsigned char BCC1(unsigned char a, unsigned char c)
{
    return a ^ c;
}

// calcular o BCC2
unsigned char BCC2(const unsigned char *data, int size)
{
    unsigned char bcc2 = 0;
    for (int i = 0; i < size; i++)
    {
        bcc2 ^= data[i];
    }
    return bcc2;
}

// faz o stuffing dos dados
int stuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int outputSize = 0;

    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == FLAG || input[i] == ESCAPE)
        {
            output[outputSize++] = ESCAPE;
            output[outputSize++] = input[i] ^ STUFFING_BYTE; // xor com 0x20
        }
        else
        {
            output[outputSize++] = input[i];
        }
    }
    printf("stuffing: inputSize=%d, outputSize=%d\n", inputSize, outputSize); // debug
    return outputSize;
}

// faz destuffing dos dados para retirar os bytes ESCAPE
int destuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int outputSize = 0;
    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == ESCAPE)
        {
            i++;
            if (i < inputSize) {
                output[outputSize++] = input[i] ^ STUFFING_BYTE;
            }
            else {
                printf("destuffing: Error - ESCAPE at end of input\n");
                return -1; // erro: ESCAPE no final do input
            }
        }
        else
        {
            output[outputSize++] = input[i];
        }
    }
    printf("destuffing: inputSize=%d, outputSize=%d\n", inputSize, outputSize);
    return outputSize;
}

// função para construir os frames
int frame_build(unsigned char *frame, unsigned char c)
{
    frame[0] = FLAG;

    if (connectionParameters.role == LlTx)
    {
        if (c == C_SET || c == C_DISC || c == C_I0 || c == C_I1)
        {
            // transmissor envia comandos
            frame[1] = 0x03; // A_TRANSMITTER_COMMAND
        }
        else
        {
            // transmissor envia respostas
            frame[1] = 0x01; // A_TRANSMITTER_REPLY
        }
    }
    else
    {
        // receptor
        if (c == C_SET || c == C_DISC || c == C_I0 || c == C_I1)
        {
            // receptor envia comandos
            frame[1] = 0x01; // A_RECEIVER_COMMAND
        }
        else
        {
            // receptor envia respostas
            frame[1] = 0x03; // A_RECEIVER_REPLY
        }
    }

    frame[2] = c;
    frame[3] = BCC1(frame[1], frame[2]);
    frame[4] = FLAG;

    printf("frame_build: A=%02X, C=%02X\n", frame[1], frame[2]);
    printf("BCC1=%02X\n", frame[3]);
    return 5;
}

// função para construir os information frames
int data_frame_build(unsigned char *frame, const unsigned char *data, int dataSize)
{
    frame[0] = FLAG;
    frame[1] = 0x03; // A_TRANSMITTER_COMMAND

    if (seq_n == 0)
    {
        frame[2] = C_I0;
    }
    else
    {
        frame[2] = C_I1;
    }

    frame[3] = BCC1(frame[1], frame[2]);
    printf("data_frame_build: A=%02X, C=%02X, seq_n=%d\n", frame[1], frame[2], seq_n);

    unsigned char stuffedData[MAX_FRAME_SIZE];
    int stuffedSize = stuffing(data, dataSize, stuffedData);

    memcpy(frame + 4, stuffedData, stuffedSize);

    unsigned char bcc2 = BCC2(data, dataSize);
    printf("data_frame_build: BCC2=%02X\n", bcc2);

    if (bcc2 == FLAG || bcc2 == ESCAPE)
    {
        frame[4 + stuffedSize] = ESCAPE;
        frame[5 + stuffedSize] = bcc2 ^ STUFFING_BYTE;
        stuffedSize += 2;
    }
    else
    {
        frame[4 + stuffedSize] = bcc2;
        stuffedSize += 1;
    }

    frame[4 + stuffedSize] = FLAG;
    stuffedSize += 1;

    printf("data_frame_build: frameSize=%d\n", 4 + stuffedSize);
    return 4 + stuffedSize;
}

// configuração do alarme
void alarm_config()
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = alarm_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGALRM, &sa, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
}

// função para ler um frame de controle
unsigned char readControlFrame(int fd)
{
    unsigned char byte, cField = 0;
    State state = start_state;
    frame temp_frame;
    clean_frame(&temp_frame);

    while (state != data_rcv_state && !timeout_flag)
    {
        if (readByteSerialPort(&byte) > 0)
        {
            switch (state)
            {
            case start_state:
                if (byte == FLAG)
                {
                    state = flag_rcv_state;
                    temp_frame.flag = byte;
                    printf("readControlFrame: FLAG received - byte=%02X\n", byte);
                }
                break;
            case flag_rcv_state:
                if (byte == A_RECEIVER_REPLY || byte == A_TRANSMITTER_REPLY)
                {
                    temp_frame.a = byte;
                    state = a_rcv_state;
                    printf("readControlFrame: A=%02X received\n", byte);
                }
                else if (byte != FLAG)
                {
                    state = start_state;
                    printf("readControlFrame: Invalid A received - byte=%02X\n", byte);
                }
                break;
            case a_rcv_state:
                if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == C_UA || byte == C_DISC)
                {
                    temp_frame.c = byte;
                    cField = byte;
                    state = c_rcv_state;
                    printf("readControlFrame: cField=%02X received\n", cField);
                }
                else if (byte == FLAG){
                    clean_frame(&temp_frame);
                    state = flag_rcv_state;
                    printf("readControlFrame: FLAG received\n");
                }
                else
                {
                    clean_frame(&temp_frame);
                    state = start_state;
                    printf("readControlFrame: Invalid cField received - byte=%02X\n", byte);
                }
                break;
            case c_rcv_state:
                if (byte == BCC1(temp_frame.a, cField))
                {
                    temp_frame.bcc1 = byte;
                    state = bcc1_rcv_state;
                    printf("readControlFrame: BCC1=%02X received\n", byte);
                }
                else if (byte == FLAG){
                    clean_frame(&temp_frame);
                    state = flag_rcv_state;
                    printf("readControlFrame: FLAG received\n");
                }
                else
                {
                    clean_frame(&temp_frame);
                    printf("readControlFrame: Invalid BCC1 received - byte=%02X\n Expected=%02X\n", byte, BCC1(A_RECEIVER_REPLY, cField));
                    state = start_state;
                }
                break;
            case bcc1_rcv_state:
                if (byte == FLAG)
                {
                    state = data_rcv_state;
                    clean_frame(&temp_frame);
                    return cField;
                }
                else
                {
                    clean_frame(&temp_frame);
                    state = start_state;
                }
                break;
            default:
                clean_frame(&temp_frame);
                state = start_state;
                break;
            }
        }
    }
    return 0;
}

// função para enviar um frame e aguardar por confirmação
int sendFrameAndWait(unsigned char *frame, int framesize)
{
    int attempts = 0;
    while (attempts < connectionParameters.nRetransmissions)
    {
        if (writeBytesSerialPort(frame, framesize) < 0)
        {
            perror("sendFrameAndWait: Error writing frame to serial port");
            return -1;
        }
        printf("sendFrameAndWait: Frame sent, attempt %d\n", attempts + 1);

        // configurar alarme
        timeout_flag = 0;
        alarm(connectionParameters.timeout);

        unsigned char cField = 0;

        // aguardar confirmação
        cField = readControlFrame(fd);

        if (cField == C_UA || cField == C_RR0 || cField == C_RR1)
        {
            // confirmação recebida
            alarm(0);
            printf("sendFrameAndWait: Confirmation received, cField=%02X\n", cField);
            return 1;
        }
        else if (cField == C_REJ0 || cField == C_REJ1)
        {
            // rejeição recebida, retransmitir
            alarm(0);
            attempts++;
            printf("sendFrameAndWait: REJ received, cField=%02X\n", cField);
            continue;
        }
        else if (timeout_flag)
        {
            // timeout, incrementar tentativas
            attempts++;
            timeout_flag = 0;
            printf("sendFrameAndWait: Timeout occurred\n");
            continue;
        }
    }

    printf("sendFrameAndWait: Max attempts exceeded\n");
    return -1;
}

//////////////////////////////////////////////
// LLOPEN
//////////////////////////////////////////////
int llopen(LinkLayer cParams)
{
    connectionParameters = cParams;
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0)
    {
        perror("llopen: Error opening serial port");
        return -1;
    }
    printf("llopen: Serial port %s opened\n", connectionParameters.serialPort);

    // configura o alarme
    alarm_config();

    if (connectionParameters.role == LlTx)
    {
        // transmissor: enviar SET e aguardar UA
        unsigned char set_frame[5];
        frame_build(set_frame, C_SET);

        if (sendFrameAndWait(set_frame, 5) < 0)
        {
            perror("llopen: Failed to send SET frame or receive UA frame");
            closeSerialPort();
            return -1;
        }

        // conexão estabelecida
        printf("llopen: Connection established (Transmitter)\n");
        return 1;
    }
    else if (connectionParameters.role == LlRx)
    {
        // receptor: aguardar SET e enviar UA
        State state = start_state;
        unsigned char byte;

        while (1)
        {
            if (readByteSerialPort(&byte) > 0)
            {
                switch (state)
                {
                case start_state:
                    if (byte == FLAG)
                        state = flag_rcv_state;
                    break;
                case flag_rcv_state:
                    if (byte == 0x03) // A_TRANSMITTER_COMMAND
                    {
                        state = a_rcv_state;
                        printf("llopen (Receiver): A=%02X received\n", byte);
                    }
                    else if (byte != FLAG)
                    {
                        state = start_state;
                    }
                    break;
                case a_rcv_state:
                    if (byte == C_SET)
                        state = c_rcv_state;
                    else if (byte == FLAG)
                        state = flag_rcv_state;
                    else
                        state = start_state;
                    break;
                case c_rcv_state:
                    if (byte == BCC1(0x03, C_SET)) // A_TRANSMITTER_COMMAND
                        state = bcc1_rcv_state;
                    else if (byte == FLAG)
                        state = flag_rcv_state;
                    else
                        state = start_state;
                    break;
                case bcc1_rcv_state:
                    if (byte == FLAG)
                    {
                        // frame SET recebido, enviar UA
                        unsigned char ua_frame[5];
                        frame_build(ua_frame, C_UA);
                        if (writeBytesSerialPort(ua_frame, 5) < 0)
                        {
                            perror("llopen: Error sending UA frame");
                            closeSerialPort();
                            return -1;
                        }
                        // conexão estabelecida
                        printf("llopen: Connection established (Receiver)\n");
                        return 1;
                    }
                    else
                        state = start_state;
                    break;
                default:
                    state = start_state;
                    break;
                }
            }
        }
    }

    return -1;
}

//////////////////////////////////////////////
// LLWRITE
//////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    if (bufSize > MAX_PAYLOAD_SIZE)
    {
        fprintf(stderr, "llwrite: bufSize exceeds MAX_PAYLOAD_SIZE\n");
        return -1;
    }

    // construir o frame de dados
    unsigned char frame[MAX_FRAME_SIZE];
    int frameSize = data_frame_build(frame, buf, bufSize);
    printf("llwrite: frameSize=%d\n", frameSize);

    // enviar o frame e aguardar confirmação
    if (sendFrameAndWait(frame, frameSize) < 0)
    {
        perror("llwrite: Error sending data frame or receiving acknowledgment");
        return -1;
    }

    // atualizar número de sequência
    seq_n = 1 - seq_n;
    printf("llwrite: Data frame sent and acknowledged. seq_n updated to %d\n", seq_n);

    return bufSize;
}

//////////////////////////////////////////////
// LLREAD
//////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char buffer[MAX_FRAME_SIZE];
    int bufferSize = 0;
    State state = start_state;
    unsigned char byte;

    while (1)
    {
        if (readByteSerialPort(&byte) > 0)
        {
            switch (state)
            {
            case start_state:
                if (byte == FLAG)
                {
                    bufferSize = 0;
                    buffer[bufferSize++] = byte;
                    state = flag_rcv_state;
                }
                break;
            case flag_rcv_state:
                if (byte == 0x03) // A_TRANSMITTER_COMMAND
                {
                    buffer[bufferSize++] = byte;
                    state = a_rcv_state;
                }
                else if (byte != FLAG)
                {
                    state = start_state;
                }
                break;
            case a_rcv_state:
                if (byte == C_I0 || byte == C_I1)
                {
                    buffer[bufferSize++] = byte;
                    state = c_rcv_state;
                }
                else if (byte == FLAG)
                {
                    state = flag_rcv_state;
                }
                else
                {
                    state = start_state;
                }
                break;
            case c_rcv_state:
                if (byte == BCC1(0x03, buffer[bufferSize - 1])) // A_TRANSMITTER_COMMAND
                {
                    buffer[bufferSize++] = byte;
                    state = bcc1_rcv_state;
                }
                else if (byte == FLAG)
                {
                    state = flag_rcv_state;
                }
                else
                {
                    state = start_state;
                }
                break;
            case bcc1_rcv_state:
                if (byte != FLAG)
                {
                    buffer[bufferSize++] = byte;
                    state = data_rcv_state;
                }
                else
                {
                    state = start_state;
                }
                break;
            case data_rcv_state:
                buffer[bufferSize++] = byte;
                if (byte == FLAG)
                {
                    // frame completo recebido
                    printf("llread: Complete frame received. bufferSize=%d\n", bufferSize);
                    // processar dados
                    int dataSize = bufferSize - 5;
                    unsigned char stuffedData[MAX_FRAME_SIZE];
                    memcpy(stuffedData, &buffer[4], dataSize);

                    unsigned char destuffedData[MAX_FRAME_SIZE];
                    int destuffedSize = destuffing(stuffedData, dataSize, destuffedData);
                    if (destuffedSize < 0)
                    {
                        // erro no destuffing
                        fprintf(stderr, "llread: Error in destuffing data\n");
                        state = start_state;
                        break;
                    }

                    // BCC2 é o último byte antes da FLAG
                    unsigned char receivedBCC2 = destuffedData[destuffedSize - 1];
                    unsigned char calculatedBCC2 = BCC2(destuffedData, destuffedSize - 1);
                    printf("llread: receivedBCC2=%02X, calculatedBCC2=%02X\n", receivedBCC2, calculatedBCC2);

                    if (receivedBCC2 != calculatedBCC2)
                    {
                        // erro no BCC2, enviar REJ
                        unsigned char rej_frame[5];
                        unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                        frame_build(rej_frame, rej_c);
                        writeBytesSerialPort(rej_frame, 5);
                        printf("llread: BCC2 error, sent REJ frame\n");
                        state = start_state;
                        bufferSize = 0;
                    }
                    else
                    {
                        // BCC2 correto
                        int received_seq_n = (buffer[2] == C_I0) ? 0 : 1;
                        printf("llread: received_seq_n=%d, expected_seq_n=%d\n", received_seq_n, expected_seq_n);
                        if (received_seq_n == expected_seq_n)
                        {
                            // número de sequência esperado, enviar RR e entregar dados
                            unsigned char rr_frame[5];
                            unsigned char rr_c = (expected_seq_n == 0) ? C_RR1 : C_RR0;
                            frame_build(rr_frame, rr_c);
                            writeBytesSerialPort(rr_frame, 5);
                            printf("llread: Correct sequence number, sent RR frame\n");

                            // atualizar número de sequência esperado
                            expected_seq_n = 1 - expected_seq_n;

                            // copiar dados para o packet
                            memcpy(packet, destuffedData, destuffedSize - 1);
                            return destuffedSize - 1;
                        }
                        else
                        {
                            // número de sequência incorreto, enviar RR com número esperado
                            unsigned char rr_frame[5];
                            unsigned char rr_c = (expected_seq_n == 0) ? C_RR0 : C_RR1;
                            frame_build(rr_frame, rr_c);
                            writeBytesSerialPort(rr_frame, 5);
                            printf("llread: Wrong sequence number, sent RR frame with expected_seq_n=%d\n", expected_seq_n);
                            state = start_state;
                            bufferSize = 0;
                        }
                    }
                    state = start_state;
                }
                break;
            default:
                state = start_state;
                break;
            }
        }
    }

    return -1;
}

//////////////////////////////////////////////
// LLCLOSE
//////////////////////////////////////////////
int llclose(int showStatistics)
{
    if (connectionParameters.role == LlTx)
    {
        // transmissor: enviar DISC e aguardar DISC ou UA
        unsigned char disc_frame[5];
        frame_build(disc_frame, C_DISC);

        int attempts = 0;
        while (attempts < connectionParameters.nRetransmissions)
        {
            if (writeBytesSerialPort(disc_frame, 5) < 0)
            {
                perror("llclose: Error sending DISC frame");
                return -1;
            }
            printf("llclose: DISC frame sent, attempt %d\n", attempts + 1);

            // configurar alarme
            timeout_flag = 0;
            alarm(connectionParameters.timeout);

            unsigned char cField = readControlFrame(fd);
            printf("llclose: cField=%02X\n", cField);
            if (cField == C_DISC)
            {
                // recebeu DISC do receptor, enviar UA
                unsigned char ua_frame[5];
                frame_build(ua_frame, C_UA);
                if (writeBytesSerialPort(ua_frame, 5) < 0)
                {
                    perror("llclose: Error sending UA frame");
                    return -1;
                }
                printf("llclose: UA frame sent, connection closed (Transmitter)\n");

                // fechar porta serial
                if (closeSerialPort() < 0)
                {
                    perror("llclose: Error closing serial port");
                    return -1;
                }

                return 1;
            }
            else if (timeout_flag)
            {
                // timeout, incrementar tentativas
                attempts++;
                timeout_flag = 0;
                printf("llclose: Timeout occurred\n");
                continue;
            }
            else
            {
                // recebeu outro frame inesperado, ignorar e continuar
                printf("llclose: Unexpected frame received, cField=%02X\n", cField);
                continue;
            }
        }

        printf("llclose: Max attempts exceeded\n");
        return -1;
    }
    else if (connectionParameters.role == LlRx)
    {
        // receptor: aguardar DISC e enviar DISC, então aguardar UA
        State state = start_state;
        unsigned char byte;
        int attempts = 0;
        while (1)
        {
            if (readByteSerialPort(&byte) > 0)
            {
                switch (state)
                {
                case start_state:
                    if (byte == FLAG)
                        state = flag_rcv_state;
                    break;
                case flag_rcv_state:
                    if (byte == 0x03) // A_TRANSMITTER_COMMAND (DISC)
                    {
                        state = a_rcv_state;
                        printf("llclose (Receiver): A=%02X received\n", byte);
                    }
                    else if (byte != FLAG)
                    {
                        state = start_state;
                    }
                    break;
                case a_rcv_state:
                    if (byte == C_DISC)
                        state = c_rcv_state;
                    else if (byte == FLAG)
                        state = flag_rcv_state;
                    else
                        state = start_state;
                    break;
                case c_rcv_state:
                    if (byte == BCC1(0x03, C_DISC))
                        state = bcc1_rcv_state;
                    else if (byte == FLAG)
                        state = flag_rcv_state;
                    else
                        state = start_state;
                    break;
                case bcc1_rcv_state:
                    if (byte == FLAG)
                    {
                        // frame DISC recebido, enviar DISC em resposta
                        unsigned char disc_frame[5];
                        frame_build(disc_frame, C_DISC);
                        if (writeBytesSerialPort(disc_frame, 5) < 0)
                        {
                            perror("llclose: Error sending DISC frame");
                            return -1;
                        }
                        printf("llclose: DISC frame sent (Receiver)\n");

                        // aguardar UA do transmissor
                        timeout_flag = 0;
                        alarm(connectionParameters.timeout);

                        unsigned char cField = readControlFrame(fd);

                        if (cField == C_UA)
                        {
                            // recebeu UA, conexão encerrada
                            printf("llclose: UA received, connection closed (Receiver)\n");
                            if (closeSerialPort() < 0)
                            {
                                perror("llclose: Error closing serial port");
                                return -1;
                            }
                            return 1;
                        }
                        else if (cField == C_DISC)
                        {
                            // recebeu outro DISC, enviar UA
                            unsigned char ua_frame[5];
                            frame_build(ua_frame, C_UA);
                            if (writeBytesSerialPort(ua_frame, 5) < 0)
                            {
                                perror("llclose: Error sending UA frame");
                                return -1;
                            }
                            printf("llclose: UA frame sent in response to DISC (Receiver)\n");

                            // fechar porta serial
                            if (closeSerialPort() < 0)
                            {
                                perror("llclose: Error closing serial port");
                                return -1;
                            }

                            return 1;
                        }
                        else if (timeout_flag)
                        {
                            // timeout ocorrido
                            printf("llclose: Timeout occurred while waiting for UA (Receiver)\n");
                            if (closeSerialPort() < 0)
                            {
                                perror("llclose: Error closing serial port");
                                return -1;
                            }
                            return 1;
                        }
                        else
                        {
                            // recebeu outro frame inesperado, ignorar e continuar
                            printf("llclose: Unexpected frame received, cField=%02X (Receiver)\n", cField);
                            continue;
                        }
                    }
                    else
                        state = start_state;
                    break;
                default:
                    state = start_state;
                    break;
                }
            }

            if (attempts >= connectionParameters.nRetransmissions)
            {
                printf("llclose: Max attempts exceeded (Receiver)\n");
                return -1;
            }
        }
    }

    return -1;
}
