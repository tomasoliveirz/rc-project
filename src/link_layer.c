#include "link_layer.h"
#include "serial_port.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>

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

volatile sig_atomic_t timeout_flag = 0;
int alarmCount = 0;

// Função para obter o timestamp atual com precisão de milissegundos
void get_current_timestamp(char *buffer, size_t size) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    struct tm *tm_info;
    tm_info = localtime(&tv.tv_sec);

    int millisec = tv.tv_usec / 1000;
    strftime(buffer, size, "%H:%M:%S", tm_info);
    snprintf(buffer + strlen(buffer), size - strlen(buffer), ".%03d", millisec);
}

// Macro para facilitar a inclusão do timestamp em printf
#define LOG_PRINTF(...) do { \
    char timestamp[16]; \
    get_current_timestamp(timestamp, sizeof(timestamp)); \
    printf("[%s] ", timestamp); \
    printf(__VA_ARGS__); \
} while(0)

// Alarm handler
void alarm_handler(int signo)
{
    timeout_flag = 1;
    alarmCount++;
    LOG_PRINTF("[ALARM] alarm_handler: Timeout #%d\n", alarmCount);
}

// Function to calculate BCC1
unsigned char BCC1(unsigned char a, unsigned char c)
{
    unsigned char bcc = a ^ c;
    return bcc;
}

// Function to calculate BCC2
unsigned char BCC2(const unsigned char *data, int size)
{
    unsigned char bcc2 = 0;
    for (int i = 0; i < size; i++)
    {
        bcc2 ^= data[i];
    }
    return bcc2;
}

// Function for byte stuffing
int stuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int outputSize = 0;
    LOG_PRINTF("[STUFFING] Dados para stuffing (%d bytes):", inputSize);

    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == FLAG || input[i] == ESCAPE)
        {
            output[outputSize++] = ESCAPE;
            output[outputSize++] = input[i] ^ STUFFING_BYTE; // XOR with 0x20
        }
        else
        {
            output[outputSize++] = input[i];
        }
    }
    LOG_PRINTF("\n");
    LOG_PRINTF("[STUFFING] Dados stuffeds (%d bytes):", outputSize);

    return outputSize;
}

// Function for byte destuffing
int destuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int outputSize = 0;
    LOG_PRINTF("[DESTUFFING] Dados para destuffing (%d bytes):", inputSize);
    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == ESCAPE)
        {
            i++;
            if (i < inputSize)
            {
                output[outputSize++] = input[i] ^ STUFFING_BYTE;
            }
            else
            {
                return -1; // Error: ESCAPE at end of input
            }
        }
        else
        {
            output[outputSize++] = input[i];
        }
    }
    
    LOG_PRINTF("\n");
    LOG_PRINTF("[DESTUFFING] Dados destuffeds (%d bytes):\n", outputSize);
    return outputSize;
}

// Function to build frames
int frame_build(unsigned char *frame, unsigned char c)
{
    frame[0] = FLAG;
    LOG_PRINTF("[FRAME_BUILD] FLAG set to %02X at position 0\n", FLAG);

    if (connectionParameters.role == LlTx)
    {
        if (c == C_SET || c == C_DISC)
        {
            frame[1] = A_TRANSMITTER_COMMAND;
            LOG_PRINTF("[FRAME_BUILD] Role Tx: Set A to TRANSMITTER_COMMAND (%02X)\n", A_TRANSMITTER_COMMAND);
        }
        else
        {
            frame[1] =  A_TRANSMITTER_REPLY;
            LOG_PRINTF("[FRAME_BUILD] Role Tx: Set A to RECEIVER_REPLY (%02X)\n", A_RECEIVER_REPLY);
        }
    }
    else
    {
        if (c == C_SET || c == C_DISC)
        {
            frame[1] = A_RECEIVER_COMMAND;
            LOG_PRINTF("[FRAME_BUILD] Role Rx: Set A to RECEIVER_COMMAND (%02X)\n", A_RECEIVER_COMMAND);
        }
        else
        {
            frame[1] = A_RECEIVER_REPLY;
            LOG_PRINTF("[FRAME_BUILD] Role Rx: Set A to TRANSMITTER_REPLY (%02X)\n", A_TRANSMITTER_REPLY);
        }
    }

    frame[2] = c;
    LOG_PRINTF("[FRAME_BUILD] Set C to %02X at position 2\n", c);

    frame[3] = BCC1(frame[1], frame[2]);
    LOG_PRINTF("[FRAME_BUILD] Set BCC1 to %02X at position 3\n", frame[3]);

    frame[4] = FLAG;
    LOG_PRINTF("[FRAME_BUILD] FLAG set to %02X at position 4\n", FLAG);

    LOG_PRINTF("[FRAME_BUILD] Frame built successfully with size 5\n");
    return 5;
}

// Function to build information frames (I-frames)
int data_frame_build(unsigned char *frame, const unsigned char *data, int dataSize)
{
    LOG_PRINTF("[DATA_FRAME_BUILD] Building data frame with data size %d\n", dataSize);
    frame[0] = FLAG;
    LOG_PRINTF("[DATA_FRAME_BUILD] FLAG set to %02X at position 0\n", FLAG);
    frame[1] = A_TRANSMITTER_COMMAND;
    LOG_PRINTF("[DATA_FRAME_BUILD] A set to TRANSMITTER_COMMAND (%02X) at position 1\n", A_TRANSMITTER_COMMAND);

    if (seq_n == 0)
    {
        frame[2] = C_I0;
        LOG_PRINTF("[DATA_FRAME_BUILD] Sequence number 0: C set to C_I0 (%02X) at position 2\n", C_I0);
    }
    else
    {
        frame[2] = C_I1;
        LOG_PRINTF("[DATA_FRAME_BUILD] Sequence number 1: C set to C_I1 (%02X) at position 2\n", C_I1);
    }

    frame[3] = BCC1(frame[1], frame[2]);
    LOG_PRINTF("[DATA_FRAME_BUILD] BCC1 set to %02X at position 3\n", frame[3]);

    // Stuff data and BCC2
    unsigned char tempBuffer[2 * MAX_PAYLOAD_SIZE]; // Buffer to hold stuffed data
    int stuffedSize = stuffing(data, dataSize, tempBuffer);
    LOG_PRINTF("[DATA_FRAME_BUILD] Data stuffed size: %d\n", stuffedSize);

    unsigned char bcc2 = BCC2(data, dataSize);
    LOG_PRINTF("[DATA_FRAME_BUILD] Calculated BCC2: %02X\n", bcc2);

    // Stuff BCC2 if necessary
    unsigned char bcc2Stuffed[2];
    int bcc2Size = 1;
    if (bcc2 == FLAG || bcc2 == ESCAPE)
    {
        bcc2Stuffed[0] = ESCAPE;
        bcc2Stuffed[1] = bcc2 ^ STUFFING_BYTE;
        bcc2Size = 2;
    }
    else
    {
        bcc2Stuffed[0] = bcc2;
    }

    // Ensure we don't exceed the frame buffer
    if (4 + stuffedSize + bcc2Size + 1 > MAX_FRAME_SIZE)
    {
        LOG_PRINTF("[DATA_FRAME_BUILD] Frame size exceeds MAX_FRAME_SIZE\n");
        return -1;
    }

    // Copy stuffed data into frame
    memcpy(frame + 4, tempBuffer, stuffedSize);
    LOG_PRINTF("[DATA_FRAME_BUILD] Copied stuffed data to frame starting at position 4\n");

    memcpy(frame + 4 + stuffedSize, bcc2Stuffed, bcc2Size);
    LOG_PRINTF("[DATA_FRAME_BUILD] Copied BCC2 to frame starting at position %d\n", 4 + stuffedSize);

    frame[4 + stuffedSize + bcc2Size] = FLAG;
    LOG_PRINTF("[DATA_FRAME_BUILD] FLAG set to %02X at final position %d\n", FLAG, 4 + stuffedSize + bcc2Size);

    int frameSize = 4 + stuffedSize + bcc2Size + 1; // Header + Data + BCC2 + FLAG
    LOG_PRINTF("[DATA_FRAME_BUILD] Data frame built successfully with size %d\n", frameSize);
    return frameSize;
}

// Alarm configuration
void alarm_config()
{
    LOG_PRINTF("[ALARM_CONFIG] Configuring alarm handler\n");
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = alarm_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGALRM, &sa, NULL) == -1)
    {
        perror("[ALARM_CONFIG] sigaction");
        exit(1);
    }
    LOG_PRINTF("[ALARM_CONFIG] Alarm handler configured successfully\n");
}

// Function to read a control frame
unsigned char readControlFrame(int fd)
{
    unsigned char byte, cField = 0;
    State state = START_STATE;
    frame_header temp_frame;

    LOG_PRINTF("[READ_CONTROL_FRAME] Starting to read control frame\n");
    while (state != STOP_STATE && !timeout_flag)
    {
        int res = readByteSerialPort(&byte);
        if (res < 0)
        {
            if (errno == EINTR)
            {
                // Interrupted by signal, continue reading
                LOG_PRINTF("[READ_CONTROL_FRAME] Read interrupted by signal\n");
                continue;
            }
            perror("[READ_CONTROL_FRAME] Error reading from serial port");
            return 0;
        }
        else if (res == 0)
        {
            LOG_PRINTF("[READ_CONTROL_FRAME] No data read\n");
            continue; // No data read
        }

        switch (state)
        {
        case START_STATE:
            if (byte == FLAG)
            {
                state = FLAG_RCV;
                LOG_PRINTF("[READ_CONTROL_FRAME] FLAG received, moving to FLAG_RCV state\n");
            }
            break;
        case FLAG_RCV:
            if (byte == A_TRANSMITTER_COMMAND || byte == A_RECEIVER_REPLY ||
                byte == A_RECEIVER_COMMAND || byte == A_TRANSMITTER_REPLY)
            {
                temp_frame.a = byte;
                state = A_RCV;
                LOG_PRINTF("[READ_CONTROL_FRAME] A field received: %02X, moving to A_RCV state\n", byte);
            }
            else if (byte != FLAG)
            {
                state = START_STATE;
                LOG_PRINTF("[READ_CONTROL_FRAME] Unexpected A field, resetting to START_STATE\n");
            }
            else
            {
                LOG_PRINTF("[READ_CONTROL_FRAME] Repeated FLAG, staying in FLAG_RCV state\n");
            }
            break;
        case A_RCV:
            if (byte == C_UA || byte == C_SET || byte == C_DISC ||
                byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1)
            {
                temp_frame.c = byte;
                cField = byte;
                state = C_RCV;
                LOG_PRINTF("[READ_CONTROL_FRAME] C field received: %02X, moving to C_RCV state\n", byte);
            }
            else if (byte == FLAG)
            {
                state = FLAG_RCV;
                LOG_PRINTF("[READ_CONTROL_FRAME] FLAG received, moving back to FLAG_RCV state\n");
            }
            else
            {
                state = START_STATE;
                LOG_PRINTF("[READ_CONTROL_FRAME] Unexpected C field, resetting to START_STATE\n");
            }
            break;
        case C_RCV:
            if (byte == BCC1(temp_frame.a, temp_frame.c))
            {
                state = BCC_OK;
                LOG_PRINTF("[READ_CONTROL_FRAME] BCC1 correct, moving to BCC_OK state\n");
            }
            else if (byte == FLAG)
            {
                state = FLAG_RCV;
                LOG_PRINTF("[READ_CONTROL_FRAME] FLAG received instead of BCC1, moving to FLAG_RCV state\n");
            }
            else
            {
                state = START_STATE;
                LOG_PRINTF("[READ_CONTROL_FRAME] Incorrect BCC1, resetting to START_STATE\n");
            }
            break;
        case BCC_OK:
            if (byte == FLAG)
            {
                state = STOP_STATE;
                LOG_PRINTF("[READ_CONTROL_FRAME] Final FLAG received, control frame successfully read\n");
                return cField;
            }
            else
            {
                state = START_STATE;
                LOG_PRINTF("[READ_CONTROL_FRAME] Expected final FLAG, but received %02X. Resetting to START_STATE\n", byte);
            }
            break;
        default:
            state = START_STATE;
            LOG_PRINTF("[READ_CONTROL_FRAME] Unknown state, resetting to START_STATE\n");
            break;
        }
    }
    LOG_PRINTF("[READ_CONTROL_FRAME] Exiting readControlFrame due to timeout or STOP_STATE\n");
    return 0;
}

// esta função envia o frame e espera a confirmação
// retorna 1 em caso de sucesso e -1 em caso de erro
int sendFrameAndWait(unsigned char *frame, int framesize)
{
    int attempts = 0;
    int rej_count = 0; // Contador de REJ recebidos
    LOG_PRINTF("[SEND_FRAME_AND_WAIT] Enviando frame de tamanho %d\n", framesize);
    
    // Cria uma cópia do frame para evitar modificações durante retransmissões
    unsigned char frame_copy[MAX_FRAME_SIZE];
    memcpy(frame_copy, frame, framesize);
    
    while (attempts < connectionParameters.nRetransmissions)
    {
        int res = writeBytesSerialPort(frame_copy, framesize);
        if (res < 0)
        {
            perror("[SEND_FRAME_AND_WAIT] Erro ao escrever frame na porta serial");
            return -1;
        }
        LOG_PRINTF("[SEND_FRAME_AND_WAIT] Frame enviado, tentativa %d\n", attempts + 1);

        timeout_flag = 0;
        alarm(connectionParameters.timeout);
        LOG_PRINTF("[SEND_FRAME_AND_WAIT] Alarme configurado para %d segundos\n", connectionParameters.timeout);

        unsigned char cField = 0;

        // Espera pela confirmação
        cField = readControlFrame(fd);
        LOG_PRINTF("[SEND_FRAME_AND_WAIT] cField recebido: %02X\n", cField);

        if (cField == C_UA || cField == C_RR0 || cField == C_RR1)
        {
            // Confirmação recebida
            alarm(0);
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] Confirmação recebida: %02X. Sucesso.\n", cField);
            return 1;
        }
        else if (cField == C_REJ0 || cField == C_REJ1)
        {
            // REJ recebido, incrementar contador de REJ
            alarm(0);
            rej_count++;
            attempts++;
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] REJ recebido: %02X. Retransmitindo... (REJ count: %d)\n", cField, rej_count);
            
            // Reiniciar o timeout
            timeout_flag = 0;
            alarm(connectionParameters.timeout);
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] Alarme reiniciado para %d segundos após REJ\n", connectionParameters.timeout);

            // Continua para a próxima tentativa de envio
            continue;
        }
        else if (timeout_flag)
        {
            // Timeout ocorreu, incrementar tentativas
            attempts++;
            timeout_flag = 0;
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] Timeout ocorrido. Tentativa %d/%d\n", attempts, connectionParameters.nRetransmissions);
            continue;
        }
        else
        {
            // Frame inesperado recebido, ignorar
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] Frame inesperado recebido: %02X. Ignorando e continuando.\n", cField);
            continue;
        }
    }

    LOG_PRINTF("[SEND_FRAME_AND_WAIT] Número máximo de tentativas (%d) excedido. Falha ao enviar frame.\n", connectionParameters.nRetransmissions);
    return -1;
}

//////////////////////////////////////////////
// LLOPEN
//////////////////////////////////////////////
// de um modo geral, a função llopen inicializa a conexão
// primeiro, ela inicializa os parametros da conexão
// depois, ela abre a serial port e configura o alarme
// se o role for tx, ela envia o SET e espera o UA
// se o role for rx, ela espera o SET e envia o UA
// retorna -1 em caso de erro e 1 em caso de sucesso
int llopen(LinkLayer cParams)
{
    // inicializa os parametros da conexão
    LOG_PRINTF("[LLOPEN] Initializing llopen with role %s\n",
           (cParams.role == LlTx) ? "LlTx" : "LlRx");
    connectionParameters = cParams;
    // fd é o descritor do arquivo que representa a serial port
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0)
    {
        perror("[LLOPEN] Error opening serial port");
        return -1;
    }
    LOG_PRINTF("[LLOPEN] Serial port %s opened successfully\n", connectionParameters.serialPort);

    // configura o alarme
    // este alarme é usado para detectar se o timeout ocorreu
    alarm_config();

    if (connectionParameters.role == LlTx)
    {
        // Transmitter: send SET and wait for UA
        unsigned char set_frame[5];
        frame_build(set_frame, C_SET);
        LOG_PRINTF("[LLOPEN] Transmitter: Sending SET frame\n");

        if (sendFrameAndWait(set_frame, 5) < 0)
        {
            LOG_PRINTF("[LLOPEN] Failed to send SET frame or receive UA frame\n");
            closeSerialPort();
            return -1;
        }

        // Connection established
        LOG_PRINTF("[LLOPEN] Connection established (Transmitter)\n");
        return 1;
    }
    else if (connectionParameters.role == LlRx)
    {
        // Receiver: wait for SET and send UA
        unsigned char byte;
        State state = START_STATE;
        frame_header temp_frame;

        LOG_PRINTF("[LLOPEN] Receiver: Waiting to receive SET frame\n");
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
                    LOG_PRINTF("[LLOPEN (Receiver)] FLAG received, moving to FLAG_RCV state\n");
                }
                break;
            case FLAG_RCV:
                if (byte == A_TRANSMITTER_COMMAND)
                {
                    temp_frame.a = byte;
                    state = A_RCV;
                    LOG_PRINTF("[LLOPEN (Receiver)] A field received: %02X, moving to A_RCV state\n", byte);
                }
                else if (byte != FLAG)
                {
                    state = START_STATE;
                    LOG_PRINTF("[LLOPEN (Receiver)] Unexpected A field, resetting to START_STATE\n");
                }
                break;
            case A_RCV:
                if (byte == C_SET)
                {
                    temp_frame.c = byte;
                    state = C_RCV;
                    LOG_PRINTF("[LLOPEN (Receiver)] C field received: %02X (C_SET), moving to C_RCV state\n", byte);
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    LOG_PRINTF("[LLOPEN (Receiver)] FLAG received, moving back to FLAG_RCV state\n");
                }
                else
                {
                    state = START_STATE;
                    LOG_PRINTF("[LLOPEN (Receiver)] Unexpected C field, resetting to START_STATE\n");
                }
                break;
            case C_RCV:
                if (byte == BCC1(temp_frame.a, temp_frame.c))
                {
                    state = BCC_OK;
                    LOG_PRINTF("[LLOPEN (Receiver)] BCC1 correct, moving to BCC_OK state\n");
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    LOG_PRINTF("[LLOPEN (Receiver)] FLAG received instead of BCC1, moving to FLAG_RCV state\n");
                }
                else
                {
                    state = START_STATE;
                    LOG_PRINTF("[LLOPEN (Receiver)] Incorrect BCC1, resetting to START_STATE\n");
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
                    LOG_PRINTF("[LLOPEN (Receiver)] SET frame received. UA frame sent successfully.\n");

                    // Connection established
                    LOG_PRINTF("[LLOPEN (Receiver)] Connection established (Receiver)\n");
                    return 1;
                }
                else
                {
                    state = START_STATE;
                    LOG_PRINTF("[LLOPEN (Receiver)] Expected final FLAG, but received %02X. Resetting to START_STATE\n", byte);
                }
                break;
            default:
                state = START_STATE;
                LOG_PRINTF("[LLOPEN (Receiver)] Unknown state, resetting to START_STATE\n");
                break;
            }
        }
    }

    LOG_PRINTF("[LLOPEN] Role not recognized. Failed to open connection.\n");
    return -1;
}

//////////////////////////////////////////////
// LLWRITE
//////////////////////////////////////////////
// esta é a função que envia os dados para a serial port
// de uma forma resumida:
// 1. cria o frame de dados
// 2. envia o frame e espera a confirmação
// 3. atualiza o numero de sequencia
// retorna o tamanho do buffer em caso de sucesso e -1 em caso de erro
int llwrite(const unsigned char *buf, int bufSize)
{
    LOG_PRINTF("[LLWRITE] Preparing to write %d bytes\n", bufSize);
    if (bufSize > MAX_PAYLOAD_SIZE)
    {
        LOG_PRINTF("[LLWRITE] bufSize exceeds MAX_PAYLOAD_SIZE\n");
        return -1;
    }

    // Cria o data frame
    unsigned char frame[MAX_FRAME_SIZE];
    int frameSize = data_frame_build(frame, buf, bufSize);
    if (frameSize < 0)
    {
        LOG_PRINTF("[LLWRITE] Error building data frame\n");
        return -1;
    }
    LOG_PRINTF("[LLWRITE] Data frame built with size %d\n", frameSize);

    // Envia o frame e espera a confirmação
    if (sendFrameAndWait(frame, frameSize) < 0)
    {
        LOG_PRINTF("[LLWRITE] Error sending data frame or receiving acknowledgment\n");
        return -1;
    }

    // Update sequence number
    LOG_PRINTF("[LLWRITE] Acknowledgment received. Updating sequence number from %d to %d\n", seq_n, 1 - seq_n);
    seq_n = 1 - seq_n;

    LOG_PRINTF("[LLWRITE] llwrite completed successfully\n");
    return bufSize;
}

//////////////////////////////////////////////
// LLREAD
//////////////////////////////////////////////
int llread(unsigned char *packet)
{
    LOG_PRINTF("[LLREAD] Iniciando a leitura de um frame\n");
    unsigned char dataBuffer[2 * MAX_PAYLOAD_SIZE]; // Para acomodar possível stuffing
    int dataSize = 0;
    State state = START_STATE;
    unsigned char byte;
    unsigned char a_field = 0, c_field = 0, bcc1 = 0;
    int tentativa_leitura = 0;
    const int MAX_TENTATIVAS = 5; // Número máximo de tentativas após timeout

    while (tentativa_leitura < MAX_TENTATIVAS)
    {
        int res = readByteSerialPort(&byte);
        if (res < 0)
        {
            perror("[LLREAD] Erro ao ler da porta serial");
            return -1;
        }
        else if (res == 0)
        {
            // Nenhum dado lido, verificar timeout
            if (timeout_flag)
            {
                LOG_PRINTF("[LLREAD] Timeout atingido durante a leitura.\n");
                timeout_flag = 0;
                tentativa_leitura++;
                state = START_STATE;
                dataSize = 0;
                memset(dataBuffer, 0, sizeof(dataBuffer));
                LOG_PRINTF("[LLREAD] Reinicializando o estado para nova tentativa (%d/%d)\n", tentativa_leitura, MAX_TENTATIVAS);
                continue;
            }
            continue; // Continuar esperando dados
        }

        switch (state)
        {
        case START_STATE:
            if (byte == FLAG)
            {
                state = FLAG_RCV;
                LOG_PRINTF("[LLREAD] FLAG recebido, mudando para estado FLAG_RCV\n");
            }
            break;
        case FLAG_RCV:
            if (byte == A_TRANSMITTER_COMMAND)
            {
                a_field = byte;
                state = A_RCV;
                LOG_PRINTF("[LLREAD] Campo A recebido: %02X, mudando para estado A_RCV\n", byte);
            }
            else if (byte == FLAG)
            {
                LOG_PRINTF("[LLREAD] FLAG repetido recebido, permanecendo em estado FLAG_RCV\n");
                // Permanecer em FLAG_RCV
            }
            else
            {
                state = START_STATE;
                LOG_PRINTF("[LLREAD] Campo A inesperado: %02X, retornando para estado START_STATE\n", byte);
            }
            break;
        case A_RCV:
            if (byte == C_I0 || byte == C_I1)
            {
                c_field = byte;
                state = C_RCV;
                LOG_PRINTF("[LLREAD] Campo C recebido: %02X, mudando para estado C_RCV\n", byte);
            }
            else if (byte == FLAG)
            {
                state = FLAG_RCV;
                LOG_PRINTF("[LLREAD] FLAG recebido, mudando para estado FLAG_RCV\n");
            }
            else
            {
                state = START_STATE;
                LOG_PRINTF("[LLREAD] Campo C inesperado: %02X, retornando para estado START_STATE\n", byte);
            }
            break;
        case C_RCV:
            if (byte == BCC1(a_field, c_field))
            {
                bcc1 = byte;
                state = BCC_OK;
                LOG_PRINTF("[LLREAD] BCC1 correto: %02X, mudando para estado BCC_OK\n", byte);
            }
            else if (byte == FLAG)
            {
                state = FLAG_RCV;
                LOG_PRINTF("[LLREAD] FLAG recebido em vez de BCC1, mudando para estado FLAG_RCV\n");
            }
            else
            {
                state = START_STATE;
                LOG_PRINTF("[LLREAD] BCC1 incorreto: %02X, retornando para estado START_STATE\n", byte);
            }

            // Verificação adicional para identificar o tipo de frame
            if (c_field == C_REJ0 || c_field == C_REJ1 || c_field == C_RR0 || c_field == C_RR1)
            {
                // Frame de controle recebido, responder adequadamente
                LOG_PRINTF("[LLREAD] Frame de controle recebido: %02X\n", c_field);
                // Implementar lógica para frames de controle, se necessário
                state = START_STATE;
                continue;
            }
            break;
        case BCC_OK:
            if (byte == FLAG)
            {
                // Frame vazio, ignorar
                state = FLAG_RCV;
                LOG_PRINTF("[LLREAD] Frame vazio detectado, mudando para estado FLAG_RCV\n");
            }
            else
            {
                if (dataSize >= sizeof(dataBuffer))
                {
                    LOG_PRINTF("[LLREAD] Overflow no buffer de dados\n");
                    return -1;
                }
                // se não for um data frame, tem que ser lido o header de novo
                dataBuffer[dataSize++] = byte;
                state = DATA_RCV;
                LOG_PRINTF("[LLREAD] Iniciando recepção de dados. Primeiro byte de dados: %02X\n", byte);
            }
            break;
        case DATA_RCV:
            if (byte == FLAG)
            {
                // Frame completo
                state = STOP_STATE;
                LOG_PRINTF("[LLREAD] FLAG final recebido, frame completo\n");
            }
            else
            {
                if (dataSize >= sizeof(dataBuffer))
                {
                    LOG_PRINTF("[LLREAD] Overflow no buffer de dados enquanto recebe dados\n");
                    // enviar REJ
                    unsigned char rej_frame[5];
                    unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                    frame_build(rej_frame, rej_c);
                    LOG_PRINTF("[LLREAD] Enviando frame REJ: %02X\n", rej_c);
                    if (writeBytesSerialPort(rej_frame, 5) < 0)
                    {
                        perror("[LLREAD] Erro ao enviar frame REJ");
                    }
                    state = START_STATE;
                    continue;
                }
                dataBuffer[dataSize++] = byte;
            }
            break;
        default:
            state = START_STATE;
            LOG_PRINTF("[LLREAD] Estado desconhecido, retornando para estado START_STATE\n");
            break;
        }

        if (state == STOP_STATE)
        {
            LOG_PRINTF("[LLREAD] Processando frame recebido\n");
            // Processar frame recebido
            // Desfazer stuffing
            unsigned char destuffedData[MAX_PAYLOAD_SIZE];
            int destuffedSize = destuffing(dataBuffer, dataSize, destuffedData);
            
            if (destuffedSize < 0)
            {
                // Erro no destuffing
                LOG_PRINTF("[LLREAD] Erro ao desfazer o stuffing dos dados\n");
                // Enviar REJ
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                LOG_PRINTF("[LLREAD] Enviando frame REJ: %02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0)
                {
                    perror("[LLREAD] Erro ao enviar frame REJ");
                }
                state = START_STATE;
                continue;
            }

            // Separar BCC2
            if (destuffedSize < 1)
            {
                LOG_PRINTF("[LLREAD] Dados destuffed muito pequenos\n");
                // Enviar REJ
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                LOG_PRINTF("[LLREAD] Enviando frame REJ: %02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0)
                {
                    perror("[LLREAD] Erro ao enviar frame REJ");
                }
                state = START_STATE;
                continue;
            }

            unsigned char receivedBCC2 = destuffedData[destuffedSize - 1];
            unsigned char calculatedBCC2 = BCC2(destuffedData, destuffedSize - 1);

            if ((receivedBCC2 != calculatedBCC2) && (destuffedData[1] != C_RR0 && destuffedData[1] != C_RR1))
            {
                // Erro no BCC2, enviar REJ
                LOG_PRINTF("[LLREAD] Mismatch no BCC2. Recebido: %02X, Calculado: %02X\n", receivedBCC2, calculatedBCC2);
                // limpar buffer que recebeu o frame errado
                memset(destuffedData, 0, sizeof(destuffedData));
                dataSize = 0;
                // enviar REJ
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                LOG_PRINTF("[LLREAD] Enviando frame REJ: %02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0)
                {
                    perror("[LLREAD] Erro ao enviar frame REJ");
                }
                LOG_PRINTF("[LLREAD] Frame REJ enviado devido a erro no BCC2\n");
                state = START_STATE;
                continue;
            }
            else
            {
                // BCC2 correto
                int received_seq_n = (c_field == C_I0) ? 0 : 1;
                LOG_PRINTF("[LLREAD] BCC2 correto. Número de sequência recebido: %d\n", received_seq_n);

                if (received_seq_n == expected_seq_n)
                {
                    // Número de sequência esperado, enviar RR e entregar dados
                    unsigned char rr_frame[5];
                    unsigned char rr_c = (expected_seq_n == 0) ? C_RR1 : C_RR0;
                    frame_build(rr_frame, rr_c);
                    LOG_PRINTF("[LLREAD] Número de sequência corresponde. Enviando frame RR: %02X\n", rr_c);
                    if (writeBytesSerialPort(rr_frame, 5) < 0)
                    {
                        perror("[LLREAD] Erro ao enviar frame RR");
                    }

                    // Atualizar número de sequência esperado
                    expected_seq_n = 1 - expected_seq_n;
                    LOG_PRINTF("[LLREAD] Número de sequência esperado atualizado para %d\n", expected_seq_n);

                    // Copiar dados para o pacote
                    memcpy(packet, destuffedData, destuffedSize - 1);
                    LOG_PRINTF("[LLREAD] Dados copiados para o pacote. Tamanho dos dados: %d bytes\n", destuffedSize - 1);
                    return destuffedSize - 1;
                }
                else
                {
                    // Frame duplicado, reenviar último RR
                    unsigned char rr_frame[5];
                    unsigned char rr_c = (received_seq_n == 0) ? C_RR1 : C_RR0;
                    frame_build(rr_frame, rr_c);
                    LOG_PRINTF("[LLREAD] Frame duplicado recebido. Reenviando frame RR: %02X\n", rr_c);
                    if (writeBytesSerialPort(rr_frame, 5) < 0)
                    {
                        perror("[LLREAD] Erro ao reenviar frame RR");
                    }
                    LOG_PRINTF("[LLREAD] Frame RR reenviado para confirmação do último frame correto\n");
                    state = START_STATE;
                    continue;
                }
            }
        }
    }

    LOG_PRINTF("[LLREAD] Número máximo de tentativas de leitura (%d) excedido. Falha na leitura do frame.\n", MAX_TENTATIVAS);
    return -1;
}

//////////////////////////////////////////////
// LLCLOSE
//////////////////////////////////////////////
int llclose(int showStatistics)
{
    LOG_PRINTF("[LLCLOSE] Initiating llclose with showStatistics=%d\n", showStatistics);
    if (connectionParameters.role == LlTx)
    {
        // Transmitter: send DISC and wait for DISC, then send UA
        unsigned char disc_frame[5];
        frame_build(disc_frame, C_DISC);
        LOG_PRINTF("[LLCLOSE] Transmitter: Sending DISC frame\n");

        int attempts = 0;
        while (attempts < connectionParameters.nRetransmissions)
        {
            int res = writeBytesSerialPort(disc_frame, 5);
            if (res < 0)
            {
                perror("[LLCLOSE] Error sending DISC frame");
                return -1;
            }
            LOG_PRINTF("[LLCLOSE] DISC frame sent, attempt %d\n", attempts + 1);

            // Set alarm
            timeout_flag = 0;
            alarm(connectionParameters.timeout);
            LOG_PRINTF("[LLCLOSE] Alarm set for %d seconds while waiting for DISC\n", connectionParameters.timeout);

            unsigned char cField = readControlFrame(fd);
            LOG_PRINTF("[LLCLOSE] Received cField: %02X\n", cField);

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
                LOG_PRINTF("[LLCLOSE] DISC frame received. UA frame sent successfully. Connection closed (Transmitter)\n");

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
                // Timeout, incrementar attempts
                attempts++;
                timeout_flag = 0;
                LOG_PRINTF("[LLCLOSE] Timeout occurred while waiting for DISC. Attempt %d/%d\n", attempts, connectionParameters.nRetransmissions);
                continue;
            }
            else
            {
                // Unexpected frame received, ignore
                LOG_PRINTF("[LLCLOSE] Unexpected frame received: %02X. Ignoring and continuing.\n", cField);
                continue;
            }
        }

        LOG_PRINTF("[LLCLOSE] Max attempts (%d) exceeded while sending DISC. Failed to close connection.\n", connectionParameters.nRetransmissions);
        return -1;
    }
    else if (connectionParameters.role == LlRx)
    {
        // Receiver: wait for DISC, send DISC, then wait for UA
        unsigned char byte;
        State state = START_STATE;
        frame_header temp_frame;
        int attempts = 0;

        LOG_PRINTF("[LLCLOSE] Receiver: Waiting to receive DISC frame\n");
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
                    LOG_PRINTF("[LLCLOSE (Receiver)] FLAG received, moving to FLAG_RCV state\n");
                }
                break;
            case FLAG_RCV:
                if (byte == A_TRANSMITTER_COMMAND)
                {
                    temp_frame.a = byte;
                    state = A_RCV;
                    LOG_PRINTF("[LLCLOSE (Receiver)] A field received: %02X, moving to A_RCV state\n", byte);
                }
                else if (byte != FLAG)
                {
                    state = START_STATE;
                    LOG_PRINTF("[LLCLOSE (Receiver)] Unexpected A field: %02X, resetting to START_STATE\n", byte);
                }
                break;
            case A_RCV:
                if (byte == C_DISC)
                {
                    temp_frame.c = byte;
                    state = C_RCV;
                    LOG_PRINTF("[LLCLOSE (Receiver)] C field received: %02X (C_DISC), moving to C_RCV state\n", byte);
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    LOG_PRINTF("[LLCLOSE (Receiver)] FLAG received, moving back to FLAG_RCV state\n");
                }
                else
                {
                    state = START_STATE;
                    LOG_PRINTF("[LLCLOSE (Receiver)] Unexpected C field: %02X, resetting to START_STATE\n", byte);
                }
                break;
            case C_RCV:
                if (byte == BCC1(temp_frame.a, temp_frame.c))
                {
                    state = BCC_OK;
                    LOG_PRINTF("[LLCLOSE (Receiver)] BCC1 correct: %02X, moving to BCC_OK state\n", byte);
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                    LOG_PRINTF("[LLCLOSE (Receiver)] FLAG received instead of BCC1, moving to FLAG_RCV state\n");
                }
                else
                {
                    state = START_STATE;
                    LOG_PRINTF("[LLCLOSE (Receiver)] Incorrect BCC1: %02X, resetting to START_STATE\n", byte);
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
                    LOG_PRINTF("[LLCLOSE (Receiver)] DISC frame received. DISC frame sent successfully (Receiver)\n");

                    // Wait for UA from transmitter
                    timeout_flag = 0;
                    alarm(connectionParameters.timeout);
                    LOG_PRINTF("[LLCLOSE (Receiver)] Alarm set for %d segundos enquanto espera por UA\n", connectionParameters.timeout);

                    unsigned char cField = readControlFrame(fd);
                    LOG_PRINTF("[LLCLOSE (Receiver)] Received cField: %02X\n", cField);

                    if (cField == C_UA)
                    {
                        // UA recebido, conexão encerrada
                        LOG_PRINTF("[LLCLOSE (Receiver)] UA recebido. Conexão encerrada com sucesso (Receiver)\n");
                        if (closeSerialPort() < 0)
                        {
                            perror("[LLCLOSE (Receiver)] Error closing serial port");
                            return -1;
                        }
                        return 1;
                    }
                    else if (timeout_flag)
                    {
                        // Timeout ocorreu
                        LOG_PRINTF("[LLCLOSE (Receiver)] Timeout ocorreu enquanto espera por UA. Encerrando conexão.\n");
                        if (closeSerialPort() < 0)
                        {
                            perror("[LLCLOSE (Receiver)] Error closing serial port");
                            return -1;
                        }
                        return 1;
                    }
                    else
                    {
                        // Frame inesperado recebido, ignorar
                        LOG_PRINTF("[LLCLOSE (Receiver)] Unexpected frame recebido: %02X. Ignoring and continuing.\n", cField);
                        continue;
                    }
                }
                else
                {
                    state = START_STATE;
                    LOG_PRINTF("[LLCLOSE (Receiver)] Expected final FLAG, but received %02X. Resetting to START_STATE\n", byte);
                }
                break;
            default:
                state = START_STATE;
                LOG_PRINTF("[LLCLOSE (Receiver)] Unknown state, resetting to START_STATE\n");
                break;
            }

            if (attempts >= connectionParameters.nRetransmissions)
            {
                LOG_PRINTF("[LLCLOSE (Receiver)] Max attempts (%d) exceeded while waiting for DISC\n", connectionParameters.nRetransmissions);
                return -1;
            }
        }
    }

    LOG_PRINTF("[LLCLOSE] Role not recognized. Failed to close connection.\n");
    return -1;
}
