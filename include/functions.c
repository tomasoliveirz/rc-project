#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include "constants.h"
#include "link_layer.h"


LinkLayer connectionParameters;
volatile sig_atomic_t timeout_flag = 0;
int alarmCount = 0;

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
    unsigned char is_clear;
    unsigned char a;
    unsigned char c;
    unsigned char bcc1;
    unsigned char error;
} frame_header;

int clean_frame_header(frame_header *frame) {
    frame->is_clear = 1;
    frame->a = 0;
    frame->c = 0;
    frame->bcc1 = 0;
    frame->error = 0;
    return 0;
}

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

// Manipulador de alarme
void alarm_handler(int signo) {
    timeout_flag = 1;
    alarmCount++;
    LOG_PRINTF("[ALARM] alarm_handler: Timeout #%d\n", alarmCount);
}


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
        } else {
            output[outputSize++] = input[i];
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
            } else {
                return -1; // Erro: ESCAPE no final do input
            }
        } else {
            output[outputSize++] = input[i];
        }
    }

    LOG_PRINTF("[DESTUFFING] Dados destuffed (%d bytes)\n", outputSize);
    return outputSize;
}