// application_layer.c

#include "application_layer.h"
#include "link_layer.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// função para enviar a imagem (transmissor)
int sendImage(const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("sendImage: Error opening file for reading");
        return -1;
    }

    // printar o tamanho do arquivo
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    printf("sendImage: File size: %ld bytes\n", fileSize);

    unsigned char buffer[MAX_PAYLOAD_SIZE];
    size_t bytesRead;
    int frameNumber = 0;

    while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
        printf("sendImage: Read %zu bytes from %s\n", bytesRead, filename);
        int bytesWritten = llwrite(buffer, bytesRead);
        if (bytesWritten < 0) {
            fprintf(stderr, "sendImage: Error writing data frame %d\n", frameNumber);
            fclose(file);
            llclose(0);
            return -1;
        }
        printf("sendImage: Data frame %d sent successfully\n", frameNumber);
        frameNumber++;
    }

    if (ferror(file)) {
        perror("sendImage: Error reading from file");
        fclose(file);
        llclose(0);
        return -1;
    }

    fclose(file);
    printf("sendImage: All data frames sent successfully\n");
    return 1;
}

// função para receber a imagem (receptor)
int receiveImage(const char *filename) {
    FILE *file = fopen(filename, "wb");
    if (!file) {
        perror("receiveImage: Error opening file for writing");
        return -1;
    }

    unsigned char buffer[MAX_PAYLOAD_SIZE];
    int bytesRead;
    int frameNumber = 0;

    while (1) {
        bytesRead = llread(buffer);
        if (bytesRead < 0) {
            fprintf(stderr, "receiveImage: Error reading data frame %d\n", frameNumber);
            fclose(file);
            llclose(0);
            return -1;
        }

        // verificar se a conexão foi encerrada
        if (bytesRead == 0) {
            printf("receiveImage: No more data frames to read\n");
            break;
        }

        // escrever os dados no arquivo
        size_t bytesWritten = fwrite(buffer, 1, bytesRead, file);
        if (bytesWritten != bytesRead) {
            perror("receiveImage: Error writing to file");
            fclose(file);
            llclose(0);
            return -1;
        }

        printf("receiveImage: Data frame %d received and written (%d bytes)\n", frameNumber, bytesRead);
        frameNumber++;

        if (bytesRead < MAX_PAYLOAD_SIZE) {
            printf("receiveImage: Last data frame received\n");
            break;
        }
    }

    fclose(file);
    printf("receiveImage: All data frames received and written successfully\n");
    return 1;
}

// função da camada de aplicação
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // inicializa os parâmetros da conexão
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    printf("applicationLayer: serialPort=%s, role=%s, baudRate=%d, nTries=%d, timeout=%d\n",
           serialPort, role, baudRate, nTries, timeout);

    // abre a conexão
    int connectionStatus = llopen(connectionParameters);
    if (connectionStatus < 0) {
        fprintf(stderr, "applicationLayer: Failed to open connection.\n");
        return;
    }

    if (connectionParameters.role == LlTx) {
        // transmissor: enviar a imagem
        printf("Transmissor: Enviando a imagem %s\n", filename);
        if (sendImage(filename) < 0) {
            fprintf(stderr, "applicationLayer: Error sending image.\n");
            llclose(0);
            return;
        }
        printf("Transmissor: Imagem enviada com sucesso.\n");
    } else {
        // receptor: receber a imagem
        printf("Receptor: Recebendo a imagem e salvando em %s\n", filename);
        if (receiveImage(filename) < 0) {
            fprintf(stderr, "applicationLayer: Error receiving image.\n");
            llclose(0);
            return;
        }
        printf("Receptor: Imagem recebida e salva com sucesso.\n");
    }

    // fecha a conexão
    if (llclose(1) < 0) {
        fprintf(stderr, "applicationLayer: Failed to close connection.\n");
    } else {
        printf("applicationLayer: Connection closed successfully.\n");
    }
}
