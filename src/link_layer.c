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

// Definição dos Estados para as Máquinas de Estado
typedef enum {
    START_STATE,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA_RCV,
    STOP_STATE
} State;

// Estrutura de cabeçalho de frame
typedef struct {
    unsigned char flag;
    unsigned char a;
    unsigned char c;
    unsigned char bcc1;
} frame_header;

// Parâmetros de conexão
LinkLayer connectionParameters;
unsigned char readControlFrame(int fd);
extern int fd;  // Descritor da porta serial
int unexpecte_disc = 0;

int transmitter_reply = 0; 
int ua_received = 0;
int expected_seq_n = 0; // Número de sequência esperado para recepção
int seq_n = 0;           // Número de sequência atual para transmissão

volatile sig_atomic_t timeout_flag = 0;
int alarmCount = 0;

// Estrutura para Estatísticas com tempo de transmissão e recepção
typedef struct {
    // Estatísticas de Transmissão
    struct {
        // Total de Quadros Enviados
        int sent_I_frames;
        int sent_S_frames;
        int sent_U_frames;

        // Retransmissões
        int retransmissions_I_frames;
        int retransmissions_S_frames;

        // Timeouts
        int timeouts;

        // Tempo de transmissão
        struct timeval transmission_time;
    } transmission;

    // Estatísticas de Recepção
    struct {
        // Total de Quadros Recebidos
        int received_I_frames;
        int received_S_frames;
        int received_U_frames;

        // Quadros com Erros Detectados
        int bcc1_errors;
        int bcc2_errors;

        // Outros
        int duplicate_frames;
        int rejections_sent;

        // Tempo de recepção
        struct timeval reception_time;
    } reception;
} LinkLayerStats;
// Inicializando Estatísticas
LinkLayerStats stats = {0};

// Função para obter o timestamp atual com precisão de milissegundos
void get_current_timestamp(char *buffer, size_t size) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    struct tm *tm_info = localtime(&tv.tv_sec);
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

// Função para Imprimir Estatísticas
// Função para Imprimir Estatísticas com tempo de transmissão e recepção
void print_statistics() {
    LOG_PRINTF("\n===== Estatísticas da Conexão =====\n");

    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    double transmission_duration = 0.0;

    if (connectionParameters.role == LlTx) {
        LOG_PRINTF("===== Estatísticas de Transmissão =====\n");
        // Quadros Enviados
        LOG_PRINTF("Número Total de Quadros Enviados:\n");
        LOG_PRINTF("  I Frames (Quadros de Informação): %d\n", stats.transmission.sent_I_frames);
        LOG_PRINTF("  S Frames (Quadros de Supervisão): %d\n", stats.transmission.sent_S_frames);
        LOG_PRINTF("  U Frames (Quadros Não Numerados): %d\n", stats.transmission.sent_U_frames);

        // Retransmissões
        LOG_PRINTF("Retransmissões:\n");
        LOG_PRINTF("  Retransmissões de I Frames: %d\n", stats.transmission.retransmissions_I_frames);
        LOG_PRINTF("  Retransmissões de S Frames: %d\n", stats.transmission.retransmissions_S_frames);

        // Timeouts
        LOG_PRINTF("Timeouts Ocorridos: %d\n", stats.transmission.timeouts);

        // Tempo de transmissão
        transmission_duration = (current_time.tv_sec - stats.transmission.transmission_time.tv_sec) +
                                (current_time.tv_usec - stats.transmission.transmission_time.tv_usec) / 1e6;
        LOG_PRINTF("Tempo de Transmissão: %.3f segundos\n", transmission_duration);
    } else {
        LOG_PRINTF("===== Estatísticas de Recepção =====\n");
        // Quadros Recebidos
        LOG_PRINTF("Número Total de Quadros Recebidos:\n");
        LOG_PRINTF("  I Frames Recebidos: %d\n", stats.reception.received_I_frames);
        LOG_PRINTF("  S Frames Recebidos: %d\n", stats.reception.received_S_frames);
        LOG_PRINTF("  U Frames Recebidos: %d\n", stats.reception.received_U_frames);

        // Quadros com Erros Detectados
        LOG_PRINTF("Quadros com Erros Detectados:\n");
        LOG_PRINTF("  Erros no Cabeçalho (BCC1): %d\n", stats.reception.bcc1_errors);
        LOG_PRINTF("  Erros no Campo de Dados (BCC2): %d\n", stats.reception.bcc2_errors);

        // Outros
        LOG_PRINTF("Número de Quadros Duplicados Recebidos: %d\n", stats.reception.duplicate_frames);
        LOG_PRINTF("Número de Quadros Rejeitados (REJ) Enviados: %d\n", stats.reception.rejections_sent);

        // Tempo de recepção
        transmission_duration = (current_time.tv_sec - stats.reception.reception_time.tv_sec) +
                                (current_time.tv_usec - stats.reception.reception_time.tv_usec) / 1e6;
        LOG_PRINTF("Tempo de Recepção: %.3f segundos\n", transmission_duration);
    }

    LOG_PRINTF("=====================================\n\n");
}
// Manipulador de alarme
void alarm_handler(int signo) {
    timeout_flag = 1;
    alarmCount++;
    LOG_PRINTF("[ALARM] alarm_handler: Timeout #%d\n", alarmCount);
}

// Função para limpar o buffer de recepção de dados do receiver
void clean_buffer(unsigned char *buffer, size_t size, int *dataSize) {
    memset(buffer, 0, size);
    *dataSize = 0;
}

// Função para calcular BCC1
unsigned char BCC1(unsigned char a, unsigned char c) {
    return a ^ c;
}

// Função para calcular BCC2
unsigned char BCC2(const unsigned char *data, int size) {
    unsigned char bcc2 = 0;
    for (int i = 0; i < size; i++) {
        bcc2 ^= data[i];
    }
    return bcc2;
}

// Função para realizar byte stuffing
int stuffing(const unsigned char *input, int inputSize, unsigned char *output) {
    int outputSize = 0;
    LOG_PRINTF("[STUFFING] Dados para stuffing (%d bytes):\n", inputSize);

    for (int i = 0; i < inputSize; i++) {
        if (input[i] == FLAG || input[i] == ESCAPE) {
            output[outputSize++] = ESCAPE;
            output[outputSize++] = input[i] ^ STUFFING_BYTE; // XOR com 0x20
            LOG_PRINTF("[STUFFING] Byte 0x%02X -> 0x%02X 0x%02X\n", input[i], ESCAPE, input[i] ^ STUFFING_BYTE);
        } else {
            output[outputSize++] = input[i];
            LOG_PRINTF("[STUFFING] Byte 0x%02X mantido\n", input[i]);
        }
    }

    LOG_PRINTF("[STUFFING] Dados stuffed (%d bytes)\n", outputSize);
    return outputSize;
}

// Função para realizar byte destuffing
int destuffing(const unsigned char *input, int inputSize, unsigned char *output) {
    int outputSize = 0;
    LOG_PRINTF("[DESTUFFING] Dados para destuffing (%d bytes):\n", inputSize);

    for (int i = 0; i < inputSize; i++) {
        if (input[i] == ESCAPE) {
            i++;
            if (i < inputSize) {
                output[outputSize++] = input[i] ^ STUFFING_BYTE;
                LOG_PRINTF("[DESTUFFING] Sequência escapada 0x%02X 0x%02X -> 0x%02X\n", ESCAPE, input[i], input[i] ^ STUFFING_BYTE);
            } else {
                LOG_PRINTF("[DESTUFFING] Erro: ESCAPE no final do input\n");
                return -1; // Erro: ESCAPE no final do input
            }
        } else {
            output[outputSize++] = input[i];
            LOG_PRINTF("[DESTUFFING] Byte 0x%02X mantido\n", input[i]);
        }
    }

    LOG_PRINTF("[DESTUFFING] Dados destuffed (%d bytes)\n", outputSize);
    return outputSize;
}

// Função para construir frames de controle
int frame_build(unsigned char *frame, unsigned char c) {
    frame[0] = FLAG;
    LOG_PRINTF("[FRAME_BUILD] FLAG set to 0x%02X at position 0\n", FLAG);

    if (connectionParameters.role == LlTx) {
        if (c == C_SET || c == C_DISC) {
            frame[1] = A_TRANSMITTER_COMMAND;
            LOG_PRINTF("[FRAME_BUILD] Role Tx: Set A to TRANSMITTER_COMMAND (0x%02X) at position 1\n", A_TRANSMITTER_COMMAND);
        } else {
            frame[1] = A_TRANSMITTER_REPLY;
            LOG_PRINTF("[FRAME_BUILD] Role Tx: Set A to TRANSMITTER_REPLY (0x%02X) at position 1\n", A_TRANSMITTER_REPLY);
        }
    } else {
        if (c == C_SET || c == C_DISC) {
            frame[1] = A_RECEIVER_COMMAND;
            LOG_PRINTF("[FRAME_BUILD] Role Rx: Set A to RECEIVER_COMMAND (0x%02X) at position 1\n", A_RECEIVER_COMMAND);
        } else {
            frame[1] = A_RECEIVER_REPLY;
            LOG_PRINTF("[FRAME_BUILD] Role Rx: Set A to RECEIVER_REPLY (0x%02X) at position 1\n", A_RECEIVER_REPLY);
        }
    }

    frame[2] = c;
    LOG_PRINTF("[FRAME_BUILD] Set C to 0x%02X at position 2\n", c);

    frame[3] = BCC1(frame[1], frame[2]);
    LOG_PRINTF("[FRAME_BUILD] Set BCC1 to 0x%02X at position 3\n", frame[3]);

    frame[4] = FLAG;
    LOG_PRINTF("[FRAME_BUILD] FLAG set to 0x%02X at position 4\n", FLAG);

    LOG_PRINTF("[FRAME_BUILD] Frame built successfully with size 5\n");
    return 5;
}

// Função para construir frames de dados (I-frames)
int data_frame_build(unsigned char *frame, const unsigned char *data, int dataSize) {
    LOG_PRINTF("[DATA_FRAME_BUILD] Building data frame with data size %d\n", dataSize);
    frame[0] = FLAG;
    LOG_PRINTF("[DATA_FRAME_BUILD] FLAG set to 0x%02X at position 0\n", FLAG);

    frame[1] = A_TRANSMITTER_COMMAND;
    LOG_PRINTF("[DATA_FRAME_BUILD] A set to TRANSMITTER_COMMAND (0x%02X) at position 1\n", A_TRANSMITTER_COMMAND);

    if (seq_n == 0) {
        frame[2] = C_I0;
        LOG_PRINTF("[DATA_FRAME_BUILD] Sequence number 0: C set to C_I0 (0x%02X) at position 2\n", C_I0);
    } else {
        frame[2] = C_I1;
        LOG_PRINTF("[DATA_FRAME_BUILD] Sequence number 1: C set to C_I1 (0x%02X) at position 2\n", C_I1);
    }

    frame[3] = BCC1(frame[1], frame[2]);
    LOG_PRINTF("[DATA_FRAME_BUILD] BCC1 set to 0x%02X at position 3\n", frame[3]);

    // Realizar stuffing nos dados e no BCC2
    unsigned char tempBuffer[2 * MAX_PAYLOAD_SIZE];
    int stuffedSize = stuffing(data, dataSize, tempBuffer);
    LOG_PRINTF("[DATA_FRAME_BUILD] Data stuffed size: %d\n", stuffedSize);

    unsigned char bcc2 = BCC2(data, dataSize);
    LOG_PRINTF("[DATA_FRAME_BUILD] Calculated BCC2: 0x%02X\n", bcc2);

    // Realizar stuffing no BCC2 se necessário
    unsigned char bcc2Stuffed[2];
    int bcc2Size = 1;
    if (bcc2 == FLAG || bcc2 == ESCAPE) {
        bcc2Stuffed[0] = ESCAPE;
        bcc2Stuffed[1] = bcc2 ^ STUFFING_BYTE;
        bcc2Size = 2;
        LOG_PRINTF("[DATA_FRAME_BUILD] BCC2 needs stuffing: 0x%02X -> 0x%02X 0x%02X\n", bcc2, ESCAPE, bcc2 ^ STUFFING_BYTE);
    } else {
        bcc2Stuffed[0] = bcc2;
        LOG_PRINTF("[DATA_FRAME_BUILD] BCC2 does not need stuffing: 0x%02X\n", bcc2);
    }

    // Verificar se o tamanho do frame excede o máximo permitido
    if (4 + stuffedSize + bcc2Size + 1 > MAX_FRAME_SIZE) {
        LOG_PRINTF("[DATA_FRAME_BUILD] Frame size exceeds MAX_FRAME_SIZE\n");
        return -1;
    }

    // Copiar dados stuffeds para o frame
    memcpy(frame + 4, tempBuffer, stuffedSize);
    LOG_PRINTF("[DATA_FRAME_BUILD] Copied stuffed data to frame starting at position 4\n");

    // Copiar BCC2 stuffeds para o frame
    memcpy(frame + 4 + stuffedSize, bcc2Stuffed, bcc2Size);
    LOG_PRINTF("[DATA_FRAME_BUILD] Copied BCC2 to frame starting at position %d\n", 4 + stuffedSize);

    frame[4 + stuffedSize + bcc2Size] = FLAG;
    LOG_PRINTF("[DATA_FRAME_BUILD] FLAG set to 0x%02X at final position %d\n", FLAG, 4 + stuffedSize + bcc2Size);

    int frameSize = 4 + stuffedSize + bcc2Size + 1; // Header + Dados + BCC2 + FLAG
    LOG_PRINTF("[DATA_FRAME_BUILD] Data frame built successfully with size %d\n", frameSize);
    return frameSize;
}

// Configuração do alarme
void alarm_config() {
    LOG_PRINTF("[ALARM_CONFIG] Configuring alarm handler\n");
    struct sigaction sa; // estrutura de ação do sinal
    memset(&sa, 0, sizeof(sa)); // limpar a estrutura
    sa.sa_handler = alarm_handler; // definir o manipulador de sinal - isto vai fazer com que o sinal seja ignorado
    if (sigaction(SIGALRM, &sa, NULL) == -1) {
        perror("[ALARM_CONFIG] sigaction");
        exit(1);
    }
    LOG_PRINTF("[ALARM_CONFIG] Alarm handler configured successfully\n");
}


// Função para enviar um frame e esperar a confirmação
// Função para enviar um frame e esperar a confirmação com registro de tempo
int sendFrameAndWait(unsigned char *frame, int framesize) {
    int attempts = 0;
    int rej_count = 0; // Contador de REJ recebidos
    LOG_PRINTF("[SEND_FRAME_AND_WAIT] Enviando frame de tamanho %d\n", framesize);

    // Identificar o tipo de frame enviado para atualizar estatísticas
    unsigned char frame_type = frame[2]; // Supondo que o campo C indica o tipo
    if (frame_type == C_I0 || frame_type == C_I1) {
        stats.transmission.sent_I_frames++;
    } else if (frame_type == C_SET || frame_type == C_DISC) {
        stats.transmission.sent_S_frames++;
    } else {
        stats.transmission.sent_U_frames++;
    }

    // Cria uma cópia do frame para evitar modificações durante retransmissões
    unsigned char frame_copy[MAX_FRAME_SIZE];
    memcpy(frame_copy, frame, framesize);

    while (attempts < connectionParameters.nRetransmissions) {
        int res = writeBytesSerialPort(frame_copy, framesize);
        if (res < 0) {
            perror("[SEND_FRAME_AND_WAIT] Erro ao escrever frame na porta serial");
            return -1;
        }
        LOG_PRINTF("[SEND_FRAME_AND_WAIT] Frame enviado, tentativa %d\n", attempts + 1);

        timeout_flag = 0;
        alarm(connectionParameters.timeout);
        LOG_PRINTF("[SEND_FRAME_AND_WAIT] Alarme configurado para %d segundos\n", connectionParameters.timeout);

        unsigned char cField = readControlFrame(fd);
        LOG_PRINTF("[SEND_FRAME_AND_WAIT] cField recebido: 0x%02X\n", cField);

        if (cField == C_UA || cField == C_RR0 || cField == C_RR1) {
            // Confirmação recebida
            alarm(0);
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] Confirmação recebida: 0x%02X. Sucesso.\n", cField);

            // Se for o transmissor e receber UA após enviar SET, armazena o tempo
            if (connectionParameters.role == LlTx && frame_type == C_SET && cField == C_UA) {
                gettimeofday(&stats.transmission.transmission_time, NULL);
                LOG_PRINTF("[SEND_FRAME_AND_WAIT] Tempo de recepção de UA armazenado.\n");
            }

            return 1;
        } else if (cField == C_REJ0 || cField == C_REJ1) {
            // REJ recebido, incrementar contador de REJ
            alarm(0);
            rej_count++;
            attempts = 0;
            if (frame_type == C_I0 || frame_type == C_I1) {
                stats.transmission.retransmissions_I_frames++;
            } else {
                stats.transmission.retransmissions_S_frames++;
            }
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] REJ recebido: 0x%02X. Retransmitindo... (REJ count: %d)\n", cField, rej_count);

            // Reiniciar o timeout
            timeout_flag = 0;
            alarm(connectionParameters.timeout);
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] Alarme reiniciado para %d segundos após REJ\n", connectionParameters.timeout);

            continue;
        } else if (timeout_flag) {
            // Timeout ocorreu, incrementar tentativas
            attempts++;
            stats.transmission.timeouts++;
            timeout_flag = 0;
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] Timeout ocorrido. Tentativa %d/%d\n", attempts, connectionParameters.nRetransmissions);
            continue;
        } else {
            // Frame inesperado recebido, ignorar
            LOG_PRINTF("[SEND_FRAME_AND_WAIT] Frame inesperado recebido: 0x%02X. Ignorando e continuando.\n", cField);
            continue;
        }
    }

    LOG_PRINTF("[SEND_FRAME_AND_WAIT] Número máximo de tentativas (%d) excedido. Falha ao enviar frame.\n", connectionParameters.nRetransmissions);
    return -1;
}
// Função para ler um frame de controle
unsigned char readControlFrame(int fd) {
    unsigned char byte, cField = 0;
    State state = START_STATE;
    frame_header temp_frame;

    LOG_PRINTF("[READ_CONTROL_FRAME] Iniciando leitura de frame de controle\n");
    while (state != STOP_STATE && !timeout_flag) {
        int res = readByteSerialPort(&byte);
        if (res < 0) {
            if (errno == EINTR) {
                LOG_PRINTF("[READ_CONTROL_FRAME] Leitura interrompida por sinal\n");
                continue;
            }
            perror("[READ_CONTROL_FRAME] Erro ao ler da porta serial");
            return 0;
        } else if (res == 0) {
            LOG_PRINTF("[READ_CONTROL_FRAME] Nenhum dado lido\n");
            continue;
        }

        switch (state) {
            case START_STATE:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    LOG_PRINTF("[READ_CONTROL_FRAME] FLAG recebido, mudando para FLAG_RCV state\n");
                }
                break;
            case FLAG_RCV:
                if (byte == A_TRANSMITTER_COMMAND || byte == A_RECEIVER_REPLY ||
                    byte == A_RECEIVER_COMMAND || byte == A_TRANSMITTER_REPLY) {
                    temp_frame.a = byte;
                    state = A_RCV;
                    LOG_PRINTF("[READ_CONTROL_FRAME] A field recebido: 0x%02X, mudando para A_RCV state\n", byte);
                } else if (byte != FLAG) {
                    state = START_STATE;
                    LOG_PRINTF("[READ_CONTROL_FRAME] Campo A inesperado, resetando para START_STATE\n");
                } else {
                    LOG_PRINTF("[READ_CONTROL_FRAME] FLAG repetido recebido, permanecendo em FLAG_RCV state\n");
                }
                break;
            case A_RCV:
                if (byte == C_UA || byte == C_SET || byte == C_DISC ||
                    byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1) {
                    temp_frame.c = byte;
                    cField = byte;
                    state = C_RCV;
                    LOG_PRINTF("[READ_CONTROL_FRAME] C field recebido: 0x%02X, mudando para C_RCV state\n", byte);
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                    LOG_PRINTF("[READ_CONTROL_FRAME] FLAG recebido, mudando para FLAG_RCV state\n");
                } else {
                    state = START_STATE;
                    LOG_PRINTF("[READ_CONTROL_FRAME] Campo C inesperado, resetando para START_STATE\n");
                }
                break;
            case C_RCV:
                if (byte == BCC1(temp_frame.a, temp_frame.c)) {
                    state = BCC_OK;
                    LOG_PRINTF("[READ_CONTROL_FRAME] BCC1 correto, mudando para BCC_OK state\n");
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                    LOG_PRINTF("[READ_CONTROL_FRAME] FLAG recebido no lugar de BCC1, mudando para FLAG_RCV state\n");
                } else {
                    state = START_STATE;
                    if (connectionParameters.role == LlTx) {
                        stats.reception.bcc1_errors++;
                    } else {
                        stats.reception.bcc1_errors++;
                    }
                    LOG_PRINTF("[READ_CONTROL_FRAME] BCC1 incorreto: 0x%02X, resetando para START_STATE\n", byte);
                }
                break;
            case BCC_OK:
                if (byte == FLAG) {
                    state = STOP_STATE;
                    LOG_PRINTF("[READ_CONTROL_FRAME] FLAG final recebido, frame de controle lido com sucesso\n");
                    return cField;
                } else {
                    state = START_STATE;
                    LOG_PRINTF("[READ_CONTROL_FRAME] FLAG final esperado, mas recebido 0x%02X. Resetando para START_STATE\n", byte);
                }
                break;
            default:
                state = START_STATE;
                LOG_PRINTF("[READ_CONTROL_FRAME] Estado desconhecido, resetando para START_STATE\n");
                break;
        }
    }
    LOG_PRINTF("[READ_CONTROL_FRAME] Saindo de readControlFrame devido a timeout ou STOP_STATE\n");
    return 0;
}

//////////////////////////////////////////////
// LL1
//////////////////////////////////////////////

// Função llopen modificada para registrar tempo de recepção
int llopen(LinkLayer cParams) {
    // Inicializa os parâmetros da conexão
    LOG_PRINTF("[LLOPEN] Inicializando llopen com role %s\n",
               (cParams.role == LlTx) ? "LlTx" : "LlRx");
    connectionParameters = cParams;

    // Abre a porta serial
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        perror("[LLOPEN] Erro ao abrir a porta serial");
        return -1;
    }
    LOG_PRINTF("[LLOPEN] Porta serial %s aberta com sucesso\n", connectionParameters.serialPort);

    // Configura o alarme
    alarm_config();

    if (connectionParameters.role == LlTx) {
        // Transmissor: envia SET e espera por UA
        unsigned char set_frame[5];
        frame_build(set_frame, C_SET);
        LOG_PRINTF("[LLOPEN] Transmissor: Enviando frame SET\n");

        if (sendFrameAndWait(set_frame, 5) < 0) {
            LOG_PRINTF("[LLOPEN] Falha ao enviar frame SET ou receber frame UA\n");
            closeSerialPort();
            return -1;
        }

        // Conexão estabelecida
        LOG_PRINTF("[LLOPEN] Conexão estabelecida (Transmissor)\n");
        return 1;
    } else if (connectionParameters.role == LlRx) {
        // Receptor: espera por SET e envia UA
        unsigned char byte;
        State state = START_STATE;
        frame_header temp_frame;

        LOG_PRINTF("[LLOPEN] Receptor: Esperando receber frame SET\n");
        while (1) {
            int res = readByteSerialPort(&byte);
            if (res < 0) {
                perror("[LLOPEN (Receiver)] Erro ao ler da porta serial");
                return -1;
            } else if (res == 0) {
                continue; // Nenhum dado lido
            }

            switch (state) {
                case START_STATE:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        LOG_PRINTF("[LLOPEN (Receiver)] FLAG recebido, mudando para FLAG_RCV state\n");
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_TRANSMITTER_COMMAND) {
                        temp_frame.a = byte;
                        state = A_RCV;
                        LOG_PRINTF("[LLOPEN (Receiver)] Campo A recebido: 0x%02X, mudando para A_RCV state\n", byte);
                    } else if (byte != FLAG) {
                        state = START_STATE;
                        LOG_PRINTF("[LLOPEN (Receiver)] Campo A inesperado: 0x%02X, resetando para START_STATE\n", byte);
                    }
                    break;
                case A_RCV:
                    if (byte == C_SET) {
                        temp_frame.c = byte;
                        state = C_RCV;
                        LOG_PRINTF("[LLOPEN (Receiver)] Campo C recebido: 0x%02X (C_SET), mudando para C_RCV state\n", byte);
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                        LOG_PRINTF("[LLOPEN (Receiver)] FLAG recebido, mudando para FLAG_RCV state\n");
                    } else {
                        state = START_STATE;
                        LOG_PRINTF("[LLOPEN (Receiver)] Campo C inesperado: 0x%02X, resetando para START_STATE\n", byte);
                    }
                    break;
                case C_RCV:
                    if (byte == BCC1(temp_frame.a, temp_frame.c)) {
                        state = BCC_OK;
                        LOG_PRINTF("[LLOPEN (Receiver)] BCC1 correto: 0x%02X, mudando para BCC_OK state\n", byte);
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                        LOG_PRINTF("[LLOPEN (Receiver)] FLAG recebido no lugar de BCC1, mudando para FLAG_RCV state\n");
                    } else {
                        state = START_STATE;
                        stats.reception.bcc1_errors++;
                        LOG_PRINTF("[LLOPEN (Receiver)] BCC1 incorreto: 0x%02X, resetando para START_STATE\n", byte);
                    }
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        // SET frame recebido, armazena o tempo
                        gettimeofday(&stats.reception.reception_time, NULL);
                        LOG_PRINTF("[LLOPEN (Receiver)] Tempo de recepção de SET armazenado.\n");

                        // Envia UA
                        unsigned char ua_frame[5];
                        frame_build(ua_frame, C_UA);
                        int res_write = writeBytesSerialPort(ua_frame, 5);
                        if (res_write < 0) {
                            perror("[LLOPEN (Receiver)] Erro ao enviar frame UA");
                            closeSerialPort();
                            return -1;
                        }
                        LOG_PRINTF("[LLOPEN (Receiver)] Frame SET recebido. Frame UA enviado com sucesso.\n");

                        // Conexão estabelecida
                        LOG_PRINTF("[LLOPEN (Receiver)] Conexão estabelecida (Receptor)\n");
                        return 1;
                    } else {
                        state = START_STATE;
                        LOG_PRINTF("[LLOPEN (Receiver)] FLAG final esperado, mas recebido 0x%02X. Resetando para START_STATE\n", byte);
                    }
                    break;
                default:
                    state = START_STATE;
                    LOG_PRINTF("[LLOPEN (Receiver)] Estado desconhecido, resetando para START_STATE\n");
                    break;
            }
        }
    }

    LOG_PRINTF("[LLOPEN] Role não reconhecida. Falha ao abrir conexão.\n");
    return -1;
}
//////////////////////////////////////////////
// LLWRITE
//////////////////////////////////////////////

int llwrite(const unsigned char *buf, int bufSize) {
    LOG_PRINTF("[LLWRITE] Preparando para escrever %d bytes\n", bufSize);
    if (bufSize > MAX_PAYLOAD_SIZE) {
        LOG_PRINTF("[LLWRITE] bufSize excede MAX_PAYLOAD_SIZE\n");
        return -1;
    }

    // Cria o data frame
    unsigned char frame[MAX_FRAME_SIZE];
    int frameSize = data_frame_build(frame, buf, bufSize);
    if (frameSize < 0) {
        LOG_PRINTF("[LLWRITE] Erro ao construir data frame\n");
        return -1;
    }
    LOG_PRINTF("[LLWRITE] Data frame construído com tamanho %d\n", frameSize);

    // Envia o frame e espera a confirmação
    if (sendFrameAndWait(frame, frameSize) < 0) {
        LOG_PRINTF("[LLWRITE] Erro ao enviar data frame ou receber confirmação\n");
        return -1;
    }

    // Atualiza o número de sequência
    LOG_PRINTF("[LLWRITE] Confirmação recebida. Atualizando número de sequência de %d para %d\n", seq_n, 1 - seq_n);
    seq_n = 1 - seq_n;

    LOG_PRINTF("[LLWRITE] llwrite concluído com sucesso\n");
    return bufSize;
}

//////////////////////////////////////////////
// LLREAD
//////////////////////////////////////////////

int llread(unsigned char *packet) {
    LOG_PRINTF("[LLREAD] Iniciando a leitura de um frame\n");
    unsigned char dataBuffer[2 * MAX_PAYLOAD_SIZE];
    int dataSize = 0;
    State state = START_STATE;
    unsigned char byte;
    unsigned char a_field = 0, c_field = 0;
    int tentativa_leitura = 0;
    const int MAX_TENTATIVAS = 5;

    while (tentativa_leitura < MAX_TENTATIVAS) {
        int res = readByteSerialPort(&byte);
        if (res < 0) {
            perror("[LLREAD] Erro ao ler da serial port");
            return -1;
        } else if (res == 0) {
            if (timeout_flag) {
                LOG_PRINTF("[LLREAD] Timeout atingido durante a leitura.\n");
                stats.transmission.timeouts++; 
                timeout_flag = 0;
                tentativa_leitura++;
                state = START_STATE;
                clean_buffer(dataBuffer, sizeof(dataBuffer), &dataSize);
                LOG_PRINTF("[LLREAD] A reinicializar o estado para uma nova tentativa (%d/%d)\n", tentativa_leitura, MAX_TENTATIVAS);
                continue;
            }
            continue; 
        }

        switch (state) {
            case START_STATE:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    LOG_PRINTF("[LLREAD] FLAG recebido, mudando para estado FLAG_RCV\n");
                }
                break;
            case FLAG_RCV:
                if (byte == A_TRANSMITTER_COMMAND || byte == A_RECEIVER_REPLY) {
                    a_field = byte;
                    state = A_RCV;
                    LOG_PRINTF("[LLREAD] Campo A recebido: 0x%02X, mudando para estado A_RCV\n", byte);
                } else if (byte != FLAG) {
                    state = START_STATE;
                    LOG_PRINTF("[LLREAD] Campo A inesperado: 0x%02X, resetando para START_STATE\n", byte);
                }
                // Se for FLAG, permanece em FLAG_RCV
                break;
            case A_RCV:
                if (byte == C_I0 || byte == C_I1) {
                    c_field = byte;
                    state = C_RCV;

                    LOG_PRINTF("[LLREAD] Campo C recebido: 0x%02X, mudando para estado C_RCV\n", byte);
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                    LOG_PRINTF("[LLREAD] FLAG recebido, mudando para estado FLAG_RCV\n");
                } else if (byte == C_DISC) {
                    unexpecte_disc = 1;
                    LOG_PRINTF("[LLREAD] C_DISC recebido, mudando para DISC_RCV state\n");
                    return -1;
                } else {
                    state = START_STATE;
                    LOG_PRINTF("[LLREAD] Campo C inesperado: 0x%02X, resetando para START_STATE\n", byte);
                }
                break;
            case C_RCV:
                if (byte == BCC1(a_field, c_field)) {
                    state = BCC_OK;
                    LOG_PRINTF("[LLREAD] BCC1 correto: 0x%02X, mudando para estado BCC_OK\n", byte);
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                    LOG_PRINTF("[LLREAD] FLAG recebido no lugar de BCC1, mudando para estado FLAG_RCV\n");
                } else {
                    state = START_STATE;
                    stats.reception.bcc1_errors++;
                    LOG_PRINTF("[LLREAD] BCC1 incorreto: 0x%02X, resetando para START_STATE\n", byte);
                }

                // Identificar tipo de frame
                if (c_field == C_REJ0 || c_field == C_REJ1 || c_field == C_RR0 || c_field == C_RR1) {
                    // resetar as retries
                    tentativa_leitura = 0;
                    LOG_PRINTF("[LLREAD] Frame de controle recebido: 0x%02X\n", c_field);
                    // Atualizar estatísticas de quadros recebidos
                    if (c_field == C_RR0 || c_field == C_RR1) {
                        stats.reception.received_S_frames++;
                    } else {
                        stats.reception.received_S_frames++;
                        stats.reception.rejections_sent++;
                    }
                    state = START_STATE;
                    continue;
                }
                break;
            case BCC_OK:
                if (byte == FLAG) {
                    // Frame vazio, ignorar
                    state = FLAG_RCV;
                    LOG_PRINTF("[LLREAD] Frame vazio detectado, mudando para estado FLAG_RCV\n");
                } else {
                    if (dataSize >= sizeof(dataBuffer)) {
                        LOG_PRINTF("[LLREAD] Overflow no buffer de dados\n");

                        unsigned char rej_frame[5];
                        unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                        frame_build(rej_frame, rej_c);
                        stats.reception.rejections_sent++;
                        LOG_PRINTF("[LLREAD] Enviando frame REJ: 0x%02X\n", rej_c);
                        if (writeBytesSerialPort(rej_frame, 5) < 0) {
                            perror("[LLREAD] Erro ao enviar frame REJ");
                        }
                        clean_buffer(dataBuffer, sizeof(dataBuffer), &dataSize);
                        LOG_PRINTF("[LLREAD] Buffer limpo após overflow\n");
                        state = START_STATE;
                        continue;
                    }

                    dataBuffer[dataSize++] = byte;
                    state = DATA_RCV;
                    LOG_PRINTF("[LLREAD] Iniciando recepção de dados. Primeiro byte de dados: 0x%02X\n", byte);
                }
                break;
            case DATA_RCV:
                if (byte == FLAG) {
                    // Frame completo
                    state = STOP_STATE;
                    LOG_PRINTF("[LLREAD] FLAG final recebido, frame completo\n");
                } else {
                    if (dataSize >= sizeof(dataBuffer)) {
                        LOG_PRINTF("[LLREAD] Overflow no buffer de dados enquanto recebe dados\n");
                        // Enviar REJ
                        unsigned char rej_frame[5];
                        unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                        frame_build(rej_frame, rej_c);
                        stats.reception.rejections_sent++;
                        LOG_PRINTF("[LLREAD] Enviando frame REJ: 0x%02X\n", rej_c);
                        if (writeBytesSerialPort(rej_frame, 5) < 0) {
                            perror("[LLREAD] Erro ao enviar frame REJ");
                        }
                        clean_buffer(dataBuffer, sizeof(dataBuffer), &dataSize);
                        LOG_PRINTF("[LLREAD] Buffer limpo após overflow\n");
                        state = START_STATE;
                        continue;
                    }
                    dataBuffer[dataSize++] = byte;
                }
                break;
            default:
                state = START_STATE;
                LOG_PRINTF("[LLREAD] Estado desconhecido, resetando para START_STATE\n");
                break;
        }

        if (state == STOP_STATE) {
            LOG_PRINTF("[LLREAD] Processando frame recebido\n");
            // Desfazer stuffing
            unsigned char destuffedData[MAX_PAYLOAD_SIZE];
            int destuffedSize = destuffing(dataBuffer, dataSize, destuffedData);

            if (destuffedSize < 0) {
                LOG_PRINTF("[LLREAD] Erro ao desfazer o stuffing dos dados\n");
                // Enviar REJ
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                stats.reception.rejections_sent++;
                LOG_PRINTF("[LLREAD] Enviando frame REJ: 0x%02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0) {
                    perror("[LLREAD] Erro ao enviar frame REJ");
                }
                clean_buffer(destuffedData, sizeof(destuffedData), &dataSize);
                LOG_PRINTF("[LLREAD] Buffer limpo devido a dados destuffed inválidos\n");
                state = START_STATE;
                continue;
            }

            // Separar BCC2
            if (destuffedSize < 1) {
                LOG_PRINTF("[LLREAD] Dados destuffed muito pequenos\n");
                // Enviar REJ
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                stats.reception.rejections_sent++;
                LOG_PRINTF("[LLREAD] Enviando frame REJ: 0x%02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0) {
                    perror("[LLREAD] Erro ao enviar frame REJ");
                }
                clean_buffer(destuffedData, sizeof(destuffedData), &dataSize);
                LOG_PRINTF("[LLREAD] Buffer limpo devido a dados destuffed inválidos\n");
                state = START_STATE;
                continue;
            }

            unsigned char receivedBCC2 = destuffedData[destuffedSize - 1];
            unsigned char calculatedBCC2 = BCC2(destuffedData, destuffedSize - 1);

            if (receivedBCC2 != calculatedBCC2) {
                // Erro no BCC2, enviar REJ
                LOG_PRINTF("[LLREAD] Mismatch no BCC2. Recebido: 0x%02X, Calculado: 0x%02X\n", receivedBCC2, calculatedBCC2);
                if (c_field == C_I0 || c_field == C_I1) {
                    stats.reception.bcc2_errors++;
                }
                unsigned char rej_frame[5];
                unsigned char rej_c = (expected_seq_n == 0) ? C_REJ0 : C_REJ1;
                frame_build(rej_frame, rej_c);
                LOG_PRINTF("[LLREAD] Enviando frame REJ: 0x%02X\n", rej_c);
                if (writeBytesSerialPort(rej_frame, 5) < 0) {
                    perror("[LLREAD] Erro ao enviar frame REJ");
                }
                clean_buffer(destuffedData, sizeof(destuffedData), &dataSize);
                LOG_PRINTF("[LLREAD] Buffer limpo devido a erro no BCC2\n");
                state = START_STATE;
                continue;
            } else {
                // BCC2 correto
                int received_seq_n = (c_field == C_I0) ? 0 : 1;
                LOG_PRINTF("[LLREAD] BCC2 correto. Número de sequência recebido: %d\n", received_seq_n);

                if (received_seq_n == expected_seq_n) {
                    // Número de sequência esperado, enviar RR e entregar dados
                    unsigned char rr_frame[5];
                    unsigned char rr_c = (expected_seq_n == 0) ? C_RR0 : C_RR1;
                    frame_build(rr_frame, rr_c);
                    if (c_field == C_I0 || c_field == C_I1) {
                        stats.reception.received_I_frames++;
                    }
                    LOG_PRINTF("[LLREAD] Número de sequência corresponde. Enviando frame RR: 0x%02X\n", rr_c);
                    if (writeBytesSerialPort(rr_frame, 5) < 0) {
                        perror("[LLREAD] Erro ao enviar frame RR");
                    }

                    // Atualizar número de sequência esperado
                    expected_seq_n = 1 - expected_seq_n;
                    LOG_PRINTF("[LLREAD] Número de sequência esperado atualizado para %d\n", expected_seq_n);

                    // Copiar dados para o pacote
                    memcpy(packet, destuffedData, destuffedSize - 1);
                    LOG_PRINTF("[LLREAD] Dados copiados para o pacote. Tamanho dos dados: %d bytes\n", destuffedSize - 1);
                    return destuffedSize - 1;
                } else {
                    // Frame duplicado, reenviar último RR
                    unsigned char rr_frame[5];
                    unsigned char rr_c = (received_seq_n == 0) ? C_RR0 : C_RR1;
                    frame_build(rr_frame, rr_c);
                    LOG_PRINTF("[LLREAD] Frame duplicado recebido. Reenviando frame RR: 0x%02X\n", rr_c);
                    if (writeBytesSerialPort(rr_frame, 5) < 0) {
                        perror("[LLREAD] Erro ao reenviar frame RR");
                    }
                    stats.reception.duplicate_frames++;
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

int llclose(int showStatistics) {
    LOG_PRINTF("[LLCLOSE] Iniciando llclose com showStatistics=%d\n", showStatistics);
    if (connectionParameters.role == LlTx) {
        // Transmissor: envia DISC e espera por DISC, depois envia UA
        unsigned char disc_frame[5];
        frame_build(disc_frame, C_DISC);
        LOG_PRINTF("[LLCLOSE] Transmissor: Enviando frame DISC\n");

        int attempts = 0;
        while (attempts < connectionParameters.nRetransmissions) {
            int res = writeBytesSerialPort(disc_frame, 5);
            if (res < 0) {
                perror("[LLCLOSE] Erro ao enviar frame DISC");
                return -1;
            }
            LOG_PRINTF("[LLCLOSE] DISC frame enviado, tentativa %d\n", attempts + 1);

            // Set alarm
            timeout_flag = 0;
            alarm(connectionParameters.timeout);
            LOG_PRINTF("[LLCLOSE] Alarme configurado para %d segundos enquanto espera por DISC\n", connectionParameters.timeout);

            unsigned char cField = readControlFrame(fd);
            LOG_PRINTF("[LLCLOSE] Received cField: 0x%02X\n", cField);

            if (cField == C_DISC) {
                // Recebeu DISC do receptor, envia UA
                unsigned char ua_frame[5];
                frame_build(ua_frame, C_UA);
                res = writeBytesSerialPort(ua_frame, 5);
                if (res < 0) {
                    perror("[LLCLOSE] Erro ao enviar frame UA");
                    return -1;
                }
                LOG_PRINTF("[LLCLOSE] DISC frame recebido. UA frame enviado com sucesso. Conexão encerrada (Transmissor)\n");

                // Atualizar estatísticas de frames recebidos
                stats.reception.received_S_frames++; // DISC é um frame de supervisão

                // Fecha a porta serial
                if (closeSerialPort() < 0) {
                    perror("[LLCLOSE] Erro ao fechar a porta serial");
                    return -1;
                }

                // Imprimir estatísticas se necessário
                if (showStatistics) {
                    print_statistics();
                }

                return 1;
            } else if (timeout_flag) {
                // Timeout, incrementar tentativas
                attempts++;
                stats.transmission.timeouts++;
                timeout_flag = 0;
                LOG_PRINTF("[LLCLOSE] Timeout ocorreu enquanto esperava por DISC. Tentativa %d/%d\n", attempts, connectionParameters.nRetransmissions);
                continue;
            } else {
                // Frame inesperado recebido, ignorar
                LOG_PRINTF("[LLCLOSE] Frame inesperado recebido: 0x%02X. Ignorando e continuando.\n", cField);
                continue;
            }
        }

        LOG_PRINTF("[LLCLOSE] Número máximo de tentativas (%d) excedido enquanto enviava DISC. Falha ao encerrar conexão.\n", connectionParameters.nRetransmissions);
        return -1;
    } else if (connectionParameters.role == LlRx) {
        // Receptor: espera por DISC, envia DISC, depois espera por UA
        unsigned char byte;
        State state = START_STATE;
        frame_header temp_frame;
        int attempts = 0;
        transmitter_reply = 0;
        ua_received = 0;
        LOG_PRINTF("[LLCLOSE] Receptor: Esperando receber frame DISC\n");
        while (1) {
            int res = readByteSerialPort(&byte);
            if (res < 0) {
                perror("[LLCLOSE (Receiver)] Erro ao ler da porta serial");
                return -1;
            } else if (res == 0) {
                continue; // Nenhum dado lido
            }

            switch (state) {
                case START_STATE:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        LOG_PRINTF("[LLCLOSE (Receiver)] FLAG recebido, mudando para estado FLAG_RCV\n");
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_TRANSMITTER_COMMAND) {
                        temp_frame.a = byte;
                        state = A_RCV;
                        LOG_PRINTF("[LLCLOSE (Receiver)] Campo A recebido: 0x%02X, mudando para estado A_RCV\n", byte);
                    } else if (byte == A_TRANSMITTER_REPLY) {
                        temp_frame.a = byte;
                        transmitter_reply = 1;
                        state = A_RCV;
                        LOG_PRINTF("[LLCLOSE (Receiver)] Campo A recebido: 0x%02X, mudando para estado A_RCV\n", byte);
                    } else if (byte != FLAG) {
                        state = START_STATE;
                        LOG_PRINTF("[LLCLOSE (Receiver)] Campo A inesperado: 0x%02X, resetando para START_STATE\n", byte);
                    }
                    break;
                case A_RCV:
                    if (byte == C_DISC) {
                        temp_frame.c = byte;
                        state = C_RCV;
                        LOG_PRINTF("[LLCLOSE (Receiver)] Campo C recebido: 0x%02X (C_DISC), mudando para estado C_RCV\n", byte);
                    } else if (byte == C_UA) {
                        temp_frame.c = byte;
                        ua_received = 1;
                        state = C_RCV;
                        LOG_PRINTF("[LLCLOSE (Receiver)] Campo C recebido: 0x%02X (C_UA), mudando para estado C_RCV\n", byte);
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                        LOG_PRINTF("[LLCLOSE (Receiver)] FLAG recebido, mudando para estado FLAG_RCV\n");
                    } else {
                        state = START_STATE;
                        LOG_PRINTF("[LLCLOSE (Receiver)] Campo C inesperado: 0x%02X, resetando para START_STATE\n", byte);
                    }
                    break;
                case C_RCV:
                    if (byte == BCC1(temp_frame.a, temp_frame.c)) {
                        state = BCC_OK;
                        LOG_PRINTF("[LLCLOSE (Receiver)] BCC1 correto: 0x%02X, mudando para estado BCC_OK\n", byte);
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                        LOG_PRINTF("[LLCLOSE (Receiver)] FLAG recebido no lugar de BCC1, mudando para estado FLAG_RCV\n");
                    } else {
                        state = START_STATE;
                        stats.reception.bcc1_errors++;
                        LOG_PRINTF("[LLCLOSE (Receiver)] BCC1 incorreto: 0x%02X, resetando para START_STATE\n", byte);
                    }
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        // DISC frame recebido, envia DISC
                        unsigned char disc_frame[5];
                        frame_build(disc_frame, C_DISC);
                        res = writeBytesSerialPort(disc_frame, 5);
                        if (res < 0) {
                            perror("[LLCLOSE (Receiver)] Erro ao enviar frame DISC");
                            return -1;
                        }
                        LOG_PRINTF("[LLCLOSE (Receiver)] DISC frame recebido. DISC frame enviado com sucesso (Receptor)\n");

                        // Atualizar estatísticas de frames recebidos
                        stats.reception.received_S_frames++; // DISC é um frame de supervisão

                        // Espera por UA do transmissor
                        timeout_flag = 0;
                        alarm(connectionParameters.timeout);
                        LOG_PRINTF("[LLCLOSE (Receiver)] Alarme configurado para %d segundos enquanto espera por UA\n", connectionParameters.timeout);

                        unsigned char cField = readControlFrame(fd);
                        LOG_PRINTF("[LLCLOSE (Receiver)] Received cField: 0x%02X\n", cField);

                        if (cField == C_UA || (ua_received && transmitter_reply)) {
                            if (ua_received && transmitter_reply){
                                printf("testestestest\n");
                            }
                            LOG_PRINTF("[LLCLOSE (Receiver)] ua_received = %d, transmitter_reply = %d\n", ua_received, transmitter_reply);
                            // UA recebido, conexão encerrada
                            LOG_PRINTF("[LLCLOSE (Receiver)] UA recebido. Conexão encerrada com sucesso (Receptor)\n");
                            if (closeSerialPort() < 0) {
                                perror("[LLCLOSE (Receiver)] Erro ao fechar a porta serial");
                                return -1;
                            }

                            // Imprimir estatísticas se necessário
                            if (showStatistics) {
                                print_statistics();
                            }

                            return 1;
                        } else if (timeout_flag) {
                            // Timeout ocorreu
                            LOG_PRINTF("[LLCLOSE (Receiver)] Timeout ocorreu enquanto espera por UA. Encerrando conexão.\n");
                            if (closeSerialPort() < 0) {
                                perror("[LLCLOSE (Receiver)] Erro ao fechar a porta serial");
                                return -1;
                            }

                            // Imprimir estatísticas se necessário
                            if (showStatistics) {
                                print_statistics();
                            }

                            return 1;
                        } else {
                            // Frame inesperado recebido, ignorar
                            LOG_PRINTF("[LLCLOSE (Receiver)] Frame inesperado recebido: 0x%02X. Ignorando e continuando.\n", cField);
                            continue;
                        }
                    } else {
                        state = START_STATE;
                        LOG_PRINTF("[LLCLOSE (Receiver)] FLAG final esperado, mas recebido 0x%02X. Resetando para START_STATE\n", byte);
                    }
                    break;
                default:
                    state = START_STATE;
                    LOG_PRINTF("[LLCLOSE (Receiver)] Estado desconhecido, resetando para START_STATE\n");
                    break;
            }

            if (attempts >= connectionParameters.nRetransmissions) {
                LOG_PRINTF("[LLCLOSE (Receiver)] Número máximo de tentativas (%d) excedido enquanto espera por DISC\n", connectionParameters.nRetransmissions);
                return -1;
            }
        }
    }

    LOG_PRINTF("[LLCLOSE] Role não reconhecida. Falha ao encerrar conexão.\n");
    return -1;
}
