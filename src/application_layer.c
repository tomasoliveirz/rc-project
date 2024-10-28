// application_layer.c

#include "application_layer.h"
#include "link_layer.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h> // For uint8_t, uint16_t

#define START 1
#define DATA 2
#define END 3

#define FILE_SIZE 0
#define FILE_NAME 1

#define MAX_SEQUENCE_NUMBER 256 // As per 1-byte sequence number

// Function to build a control packet
int build_control_packet(unsigned char *packet, int C, const char *filename, long filesize)
{
    printf("[BUILD_CONTROL_PACKET] Building control packet with C=%d\n", C);
    int index = 0;
    packet[index++] = C; // Control field
    printf("[BUILD_CONTROL_PACKET] Control field set to %d at position 0\n", C);

    // First parameter: FILE_SIZE
    packet[index++] = FILE_SIZE; // T1
    printf("[BUILD_CONTROL_PACKET] T1 set to FILE_SIZE (%d) at position %d\n", FILE_SIZE, index - 1);
    packet[index++] = sizeof(filesize); // L1
    printf("[BUILD_CONTROL_PACKET] L1 set to %lu at position %d\n", sizeof(filesize), index - 1);
    // Copy filesize into packet
    memcpy(&packet[index], &filesize, sizeof(filesize));
    printf("[BUILD_CONTROL_PACKET] Copied filesize (%ld) into packet starting at position %d\n", filesize, index);
    index += sizeof(filesize);

    // Second parameter: FILE_NAME
    if (filename != NULL)
    {
        packet[index++] = FILE_NAME; // T2
        printf("[BUILD_CONTROL_PACKET] T2 set to FILE_NAME (%d) at position %d\n", FILE_NAME, index - 1);
        uint8_t name_length = strlen(filename);
        if (name_length > MAX_PAYLOAD_SIZE - index - 2) // Ensure no overflow
        {
            fprintf(stderr, "[BUILD_CONTROL_PACKET] Filename too long\n");
            return -1;
        }
        packet[index++] = name_length; // L2
        printf("[BUILD_CONTROL_PACKET] L2 set to %d at position %d\n", name_length, index - 1);
        memcpy(&packet[index], filename, name_length);
        printf("[BUILD_CONTROL_PACKET] Copied filename \"%s\" into packet starting at position %d\n", filename, index);
        index += name_length;
    }

    printf("[BUILD_CONTROL_PACKET] Control packet built successfully with size %d\n", index);
    return index; // Total packet size
}

// Function to parse a control packet
int parse_control_packet(unsigned char *packet, int size, int *C, char *filename, long *filesize)
{
    printf("[PARSE_CONTROL_PACKET] Parsing control packet of size %d\n", size);
    int index = 0;
    *C = packet[index++]; // Control field
    printf("[PARSE_CONTROL_PACKET] Control field: %d\n", *C);

    while (index < size)
    {
        uint8_t T = packet[index++];
        uint8_t L = packet[index++];
        printf("[PARSE_CONTROL_PACKET] Parameter T=%d, L=%d\n", T, L);
        switch (T)
        {
            case FILE_SIZE:
                memcpy(filesize, &packet[index], L);
                printf("[PARSE_CONTROL_PACKET] FILE_SIZE parameter: %ld\n", *filesize);
                index += L;
                break;
            case FILE_NAME:
                memcpy(filename, &packet[index], L);
                filename[L] = '\0'; // Null-terminate the string
                printf("[PARSE_CONTROL_PACKET] FILE_NAME parameter: \"%s\"\n", filename);
                index += L;
                break;
            default:
                // Unknown parameter, skip
                printf("[PARSE_CONTROL_PACKET] Unknown parameter T=%d. Skipping %d bytes.\n", T, L);
                index += L;
                break;
        }
    }
    printf("[PARSE_CONTROL_PACKET] Control packet parsed successfully\n");
    return 0;
}

// Function to build a data packet
int build_data_packet(unsigned char *packet, int sequence_number, unsigned char *data, int data_size)
{
    printf("[BUILD_DATA_PACKET] Building data packet with sequence_number=%d and data_size=%d\n", sequence_number, data_size);
    int index = 0;
    packet[index++] = DATA; // C field
    printf("[BUILD_DATA_PACKET] C field set to DATA (%d) at position 0\n", DATA);
    packet[index++] = sequence_number % MAX_SEQUENCE_NUMBER; // S field (0-255)
    printf("[BUILD_DATA_PACKET] S field set to %d at position 1\n", sequence_number % MAX_SEQUENCE_NUMBER);

    // L2, L1
    packet[index++] = (data_size >> 8) & 0xFF; // L2
    packet[index++] = data_size & 0xFF;        // L1
    printf("[BUILD_DATA_PACKET] L2 set to %d and L1 set to %d at positions 2 and 3\n", (data_size >> 8) & 0xFF, data_size & 0xFF);

    // Data
    memcpy(&packet[index], data, data_size);
    printf("[BUILD_DATA_PACKET] Copied %d bytes of data into packet starting at position %d\n", data_size, index);
    index += data_size;

    printf("[BUILD_DATA_PACKET] Data packet built successfully with size %d\n", index);
    return index; // Total packet size
}

// Function to parse a data packet
int parse_data_packet(unsigned char *packet, int size, int *sequence_number, unsigned char *data, int *data_size)
{
    printf("[PARSE_DATA_PACKET] Parsing data packet of size %d\n", size);
    int index = 0;
    if (packet[index++] != DATA)
    {
        printf("[PARSE_DATA_PACKET] Not a DATA packet. Found C=%d\n", packet[0]);
        return -1; // Not a data packet
    }
    printf("[PARSE_DATA_PACKET] C field verified as DATA\n");

    *sequence_number = packet[index++];
    printf("[PARSE_DATA_PACKET] Sequence number: %d\n", *sequence_number);

    uint16_t L2 = packet[index++];
    uint16_t L1 = packet[index++];
    *data_size = (L2 << 8) | L1;
    printf("[PARSE_DATA_PACKET] Data size: %d (L2=%d, L1=%d)\n", *data_size, L2, L1);

    if (*data_size > size - index)
    {
        // Data size exceeds packet size
        printf("[PARSE_DATA_PACKET] Data size exceeds packet size. Expected %d, available %d\n", *data_size, size - index);
        return -1;
    }

    memcpy(data, &packet[index], *data_size);
    printf("[PARSE_DATA_PACKET] Copied %d bytes of data into buffer\n", *data_size);
    index += *data_size;

    printf("[PARSE_DATA_PACKET] Data packet parsed successfully\n");
    return index; // Total number of bytes processed
}

// Function to send the image (transmitter)
int sendImage(const char *filename)
{
    printf("[SEND_IMAGE] Starting to send image: %s\n", filename);
    FILE *file = fopen(filename, "rb");
    if (!file)
    {
        perror("[SEND_IMAGE] Error opening file for reading");
        return -1;
    }

    // Get the file size
    fseek(file, 0, SEEK_END);
    long filesize = ftell(file);
    fseek(file, 0, SEEK_SET);
    printf("[SEND_IMAGE] File size: %ld bytes\n", filesize);

    // Send START control packet
    unsigned char control_packet[MAX_PAYLOAD_SIZE];
    int control_packet_size = build_control_packet(control_packet, START, filename, filesize);
    if (control_packet_size < 0)
    {
        fprintf(stderr, "[SEND_IMAGE] Error building START control packet\n");
        fclose(file);
        llclose(0);
        return -1;
    }

    printf("[SEND_IMAGE] Sending START control packet\n");
    int bytesWritten = llwrite(control_packet, control_packet_size);
    if (bytesWritten < 0)
    {
        fprintf(stderr, "[SEND_IMAGE] Error sending START control packet\n");
        fclose(file);
        llclose(0);
        return -1;
    }
    printf("[SEND_IMAGE] START control packet sent successfully\n");

    // Now send data packets
    unsigned char file_buffer[MAX_PAYLOAD_SIZE - 4]; // Leave room for C, S, L2, L1
    size_t bytesRead;
    int sequence_number = 0;

    printf("[SEND_IMAGE] Starting to send data packets\n");
    while ((bytesRead = fread(file_buffer, 1, sizeof(file_buffer), file)) > 0)
    {
        unsigned char data_packet[MAX_FRAME_SIZE];
        int data_packet_size = build_data_packet(data_packet, sequence_number, file_buffer, bytesRead);

        if (data_packet_size < 0)
        {
            fprintf(stderr, "[SEND_IMAGE] Error building data packet\n");
            fclose(file);
            llclose(0);
            return -1;
        }

        printf("[SEND_IMAGE] Sending data packet with sequence number %d\n", sequence_number);
        int bytesWritten = llwrite(data_packet, data_packet_size);
        if (bytesWritten < 0)
        {
            fprintf(stderr, "[SEND_IMAGE] Error writing data packet with sequence number %d\n", sequence_number);
            fclose(file);
            llclose(0);
            return -1;
        }
        printf("[SEND_IMAGE] Data packet %d sent successfully (%d bytes)\n", sequence_number, bytesWritten);
        sequence_number = (sequence_number + 1) % MAX_SEQUENCE_NUMBER;
    }

    if (ferror(file))
    {
        perror("[SEND_IMAGE] Error reading from file");
        fclose(file);
        llclose(0);
        return -1;
    }

    // Send END control packet
    control_packet_size = build_control_packet(control_packet, END, filename, filesize);
    if (control_packet_size < 0)
    {
        fprintf(stderr, "[SEND_IMAGE] Error building END control packet\n");
        fclose(file);
        llclose(0);
        return -1;
    }

    printf("[SEND_IMAGE] Sending END control packet\n");
    bytesWritten = llwrite(control_packet, control_packet_size);
    if (bytesWritten < 0)
    {
        fprintf(stderr, "[SEND_IMAGE] Error sending END control packet\n");
        fclose(file);
        llclose(0);
        return -1;
    }
    printf("[SEND_IMAGE] END control packet sent successfully\n");

    fclose(file);
    printf("[SEND_IMAGE] File transfer completed successfully\n");
    return 1;
}

// Function to receive the image (receiver)
int receiveImage(const char *filename)
{
    printf("[RECEIVE_IMAGE] Starting to receive image and save to: %s\n", filename);
    // Receive START control packet
    unsigned char control_packet[MAX_PAYLOAD_SIZE];
    int bytesRead = llread(control_packet);
    if (bytesRead < 0)
    {
        fprintf(stderr, "[RECEIVE_IMAGE] Error reading START control packet\n");
        llclose(0);
        return -1;
    }

    int C;
    char received_filename[256];
    long filesize;
    parse_control_packet(control_packet, bytesRead, &C, received_filename, &filesize);

    if (C != START)
    {
        fprintf(stderr, "[RECEIVE_IMAGE] Expected START control packet, but received different packet (C=%d)\n", C);
        llclose(0);
        return -1;
    }

    printf("[RECEIVE_IMAGE] START control packet received\n");
    printf("[RECEIVE_IMAGE] File size: %ld bytes\n", filesize);
    printf("[RECEIVE_IMAGE] File name: %s\n", received_filename);

    // Open file for writing
    FILE *file = fopen(filename, "wb");
    if (!file)
    {
        perror("[RECEIVE_IMAGE] Error opening file for writing");
        llclose(0);
        return -1;
    }

    // Now receive data packets
    int sequence_number = 0;
    long total_bytes_received = 0;

    printf("[RECEIVE_IMAGE] Starting to receive data packets\n");
    while (1)
    {
        unsigned char packet_buffer[MAX_PAYLOAD_SIZE];
        bytesRead = llread(packet_buffer);
        if (bytesRead < 0)
        {
            fprintf(stderr, "[RECEIVE_IMAGE] Error reading packet\n");
            fclose(file);
            llclose(0);
            return -1;
        }

        // Check if it's an END control packet
        if (packet_buffer[0] == END)
        {
            // Parse END control packet
            int C_end;
            char filename_end[256];
            long filesize_end;
            parse_control_packet(packet_buffer, bytesRead, &C_end, filename_end, &filesize_end);

            if (C_end != END)
            {
                fprintf(stderr, "[RECEIVE_IMAGE] Expected END control packet, but received different packet (C=%d)\n", C_end);
                fclose(file);
                llclose(0);
                return -1;
            }

            // Verify that START and END packets match
            if (filesize != filesize_end || strcmp(received_filename, filename_end) != 0)
            {
                fprintf(stderr, "[RECEIVE_IMAGE] END control packet does not match START control packet\n");
                fclose(file);
                llclose(0);
                return -1;
            }

            printf("[RECEIVE_IMAGE] END control packet received, file transfer complete\n");
            break;
        }
        else if (packet_buffer[0] == DATA)
        {
            // Parse data packet
            int seq_num;
            unsigned char data[MAX_PAYLOAD_SIZE];
            int data_size;

            if (parse_data_packet(packet_buffer, bytesRead, &seq_num, data, &data_size) < 0)
            {
                fprintf(stderr, "[RECEIVE_IMAGE] Error parsing data packet\n");
                fclose(file);
                llclose(0);
                return -1;
            }

            // Verify sequence number
            if (seq_num != sequence_number)
            {
                fprintf(stderr, "[RECEIVE_IMAGE] Unexpected sequence number. Expected %d, got %d\n", sequence_number, seq_num);
                // For simplicity, we'll accept it and update the sequence number
                sequence_number = seq_num;
            }

            // Write data to file
            size_t bytesWritten = fwrite(data, 1, data_size, file);
            if (bytesWritten != data_size)
            {
                perror("[RECEIVE_IMAGE] Error writing to file");
                fclose(file);
                llclose(0);
                return -1;
            }

            total_bytes_received += data_size;
            printf("[RECEIVE_IMAGE] Data packet %d received (%d bytes)\n", seq_num, data_size);

            sequence_number = (sequence_number + 1) % MAX_SEQUENCE_NUMBER;
        }
        else
        {
            fprintf(stderr, "[RECEIVE_IMAGE] Unknown packet type: %d\n", packet_buffer[0]);
            fclose(file);
            llclose(0);
            return -1;
        }
    }

    fclose(file);
    printf("[RECEIVE_IMAGE] File received successfully, total bytes: %ld\n", total_bytes_received);
    return 1;
}

// Application layer function
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    printf("[APPLICATION_LAYER] Initializing application layer\n");
    // Initialize connection parameters
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    printf("[APPLICATION_LAYER] Configuration - serialPort=%s, role=%s, baudRate=%d, nTries=%d, timeout=%d\n",
           serialPort, role, baudRate, nTries, timeout);

    // Open the connection
    int connectionStatus = llopen(connectionParameters);
    if (connectionStatus < 0)
    {
        fprintf(stderr, "[APPLICATION_LAYER] Failed to open connection.\n");
        return;
    }
    printf("[APPLICATION_LAYER] Connection opened successfully\n");

    if (connectionParameters.role == LlTx)
    {
        // Transmitter: send the image
        printf("[APPLICATION_LAYER] Transmitter: Sending the image %s\n", filename);
        if (sendImage(filename) < 0)
        {
            fprintf(stderr, "[APPLICATION_LAYER] Error sending image.\n");
            llclose(0);
            return;
        }
        printf("[APPLICATION_LAYER] Image sent successfully.\n");
    }
    else
    {
        // Receiver: receive the image
        printf("[APPLICATION_LAYER] Receiver: Receiving the image and saving to %s\n", filename);
        if (receiveImage(filename) < 0)
        {
            fprintf(stderr, "[APPLICATION_LAYER] Error receiving image.\n");
            llclose(0);
            return;
        }
        printf("[APPLICATION_LAYER] Image received and saved successfully.\n");
    }

    // Close the connection
    if (llclose(1) < 0)
    {
        fprintf(stderr, "[APPLICATION_LAYER] Failed to close connection.\n");
    }
    else
    {
        printf("[APPLICATION_LAYER] Connection closed successfully.\n");
    }
}
