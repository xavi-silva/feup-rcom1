#include "../include/link_layer.h"
#include "../include/serial_port.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <signal.h>


// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 256

#define FLAG 0x7E
#define A_SENDER 0x03
#define A_RECEIVER 0x01
#define C_SET 0x03
#define C_UA 0x07

#define SET_START_STATE 0
#define SET_FLAG_RCV_STATE 1
#define SET_A_RCV_STATE 2
#define SET_C_RCV_STATE 3
#define SET_BCC_OK_STATE 4
#define SET_STOP_STATE 5

volatile int STOP = FALSE;

unsigned int alarmCount = 0;
int alarmEnabled = FALSE;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters){
    if (connectionParameters.role == LlRx) {
        int fd = openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate);

        if (fd < 0){
            return -1;
        }

        struct termios oldtio;
        struct termios newtio;

        // Save current port settings
        if (tcgetattr(fd, &oldtio) == -1){
            perror("tcgetattr");
            exit(-1);
        }

        // Clear struct for new port settings
        memset(&newtio, 0, sizeof(newtio));

        newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;

        // Set input mode (non-canonical, no echo,...)
        newtio.c_lflag = 0;
        newtio.c_cc[VTIME] = 0; // Inter-character timer unused
        newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

        // VTIME e VMIN should be changed in order to protect with a
        // timeout the reception of the following character(s)

        // Now clean the line and activate the settings for the port
        // tcflush() discards data written to the object referred to
        // by fd but not transmitted, or data received but not read,
        // depending on the value of queue_selector:
        //   TCIFLUSH - flushes data received but not read.
        tcflush(fd, TCIOFLUSH);

        // Set new port settings
        if (tcsetattr(fd, TCSANOW, &newtio) == -1){
            perror("tcsetattr");
            exit(-1);
        }

        printf("New termios structure set\n");

        // Loop for input
        unsigned char buf[BUF_SIZE] = {0};

        int state = SET_START_STATE;

        while (state != SET_STOP_STATE){
            int bytes = read(fd, buf, 1);

            if (bytes > 0) {
                switch (state) {
                        case SET_START_STATE :
                            printf("Started set\n");
                            if (buf[0] == FLAG) {
                                printf("Received flag\n");
                                state = SET_FLAG_RCV_STATE;
                            }
                            break;
                            if (buf[0] == A_SENDER) {
                                printf("Received address\n");
                                state = SET_A_RCV_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_A_RCV_STATE :
                            if (buf[0] == C_SET) {
                                printf("Received control\n");
                                state = SET_C_RCV_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_C_RCV_STATE :
                            if (buf[0] == A_SENDER ^ C_SET) {
                                printf("Received bcc\n");
                                state = SET_BCC_OK_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_BCC_OK_STATE :
                            if (buf[0] == FLAG) {
                                printf("Received final flag\n");
                                state = SET_STOP_STATE;
                                unsigned char buf[BUF_SIZE] = {0};

                                buf[0] = FLAG;
                                buf[1] = C_UA;
                                buf[2] = A_SENDER;
                                buf[3] = buf[3] ^ buf[2];
                                buf[4] = FLAG;
                                int bytes1 = write(fd, buf, BUF_SIZE);
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        default :
                            printf("Error state\n");
                            break;
                }      
            }
        }

        printf("Connection established\n");

        /* Restore the old port settings
        if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
        {
            perror("tcsetattr");
            exit(-1);
        }

        close(fd);
        */
        }
        return 0;
    }
    
    else if (connectionParameters.role == LlTx){
        (void)signal(SIGALRM, alarmHandler);
    
        const char *serialPortName = connectionParameters.serialPort;
        
        // Open serial port device for reading and writing, and not as controlling tty
        // because we don't want to get killed if linenoise sends CTRL-C.
        int fd = open(serialPortName, O_RDWR | O_NOCTTY);
        
        if (fd < 0)
        {
            perror(serialPortName);
            exit(-1);
        }
        
        struct termios oldtio;
        struct termios newtio;

        // Save current port settings
        if (tcgetattr(fd, &oldtio) == -1)
        {
            perror("tcgetattr");
            exit(-1);
        }
        
        // Clear struct for new port settings
        memset(&newtio, 0, sizeof(newtio));

        newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;

        // Set input mode (non-canonical, no echo,...)
        newtio.c_lflag = 0;
        newtio.c_cc[VTIME] = 0; // Inter-character timer unused
        newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

        // VTIME e VMIN should be changed in order to protect with a
        // timeout the reception of the following character(s)

        // Now clean the line and activate the settings for the port
        // tcflush() discards data written to the object referred to
        // by fd but not transmitted, or data received but not read,
        // depending on the value of queue_selector:
        //   TCIFLUSH - flushes data received but not read.
        tcflush(fd, TCIOFLUSH);

        // Set new port settings
        if (tcsetattr(fd, TCSANOW, &newtio) == -1)
        {
            perror("tcsetattr");
            exit(-1);
        }

        printf("New termios structure set\n");

        // Create string to send
        unsigned char buf[BUF_SIZE] = {0};

        buf[0] = FLAG;
        buf[1] = C_SET;
        buf[2] = A_SENDER;
        buf[3] = buf[3] ^ buf[2];
        buf[4] = FLAG;

        while (STOP == FALSE && alarmCount < 3){
    
            if (alarmEnabled == FALSE)
                {
                    int bytes = write(fd, buf, BUF_SIZE);
                    sleep(1);
                    printf("%d bytes written\n", bytes);
                    
                    alarm(3); // Set alarm to be triggered in 3s
                    alarmEnabled = TRUE;
                    
                    unsigned char buf[BUF_SIZE + 1] = {0};

                    int state = SET_START_STATE;

                    while (state != SET_STOP_STATE && alarmEnabled)
                    {
                        int bytes = read(fd, buf, 1);

                        if (bytes > 0) {
                            switch (state) {
                                    case SET_START_STATE :
                                        printf("Started set\n");
                                        if (buf[0] == FLAG) {
                                            printf("Received flag\n");
                                            state = SET_FLAG_RCV_STATE;
                                        }
                                        break;
                                    case SET_FLAG_RCV_STATE :
                                        if (buf[0] == A_SENDER) {
                                            printf("Received address\n");
                                            state = SET_A_RCV_STATE;
                                        }
                                        else if (buf[0] == FLAG) {
                                            state = SET_FLAG_RCV_STATE;
                                        }
                                        else {
                                            state = SET_START_STATE;
                                        }
                                        break;
                                    case SET_A_RCV_STATE :
                                        if (buf[0] == C_SET) {
                                            printf("Received control\n");
                                            state = SET_C_RCV_STATE;
                                        }
                                        else if (buf[0] == FLAG) {
                                            state = SET_FLAG_RCV_STATE;
                                        }
                                        else {
                                            state = SET_START_STATE;
                                        }
                                        break;
                                    case SET_C_RCV_STATE :
                                        if (buf[0] == (A_SENDER ^ C_SET)) {
                                            printf("Received bcc\n");
                                            state = SET_BCC_OK_STATE;
                                        }
                                        else if (buf[0] == FLAG) {
                                            state = SET_FLAG_RCV_STATE;
                                        }
                                        else {
                                            state = SET_START_STATE;
                                        }
                                        break;
                                    case SET_BCC_OK_STATE :
                                        if (buf[0] == FLAG) {
                                            printf("Received final flag\n");
                                            state = SET_STOP_STATE;
                                            STOP = TRUE;
                                        }
                                        else {
                                            state = SET_START_STATE;
                                        }
                                        break;
                                    default :
                                        printf("Error state\n");
                                        break;
                            }      
                        }
                    }
                }
        }
        
        alarm(0);
    
    /*
    
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    }
    
    */
    
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
//////////////////////////////////////////////// 
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    while (state != SET_STOP_STATE){
            int bytes = read(fd, buf, 1);

            if (bytes > 0) {
                switch (state) {
                        case SET_START_STATE :
                            printf("Started set\n");
                            if (buf[0] == FLAG) {
                                printf("Received flag\n");
                                state = SET_FLAG_RCV_STATE;
                            }
                            break;
                            if (buf[0] == A_SENDER) {
                                printf("Received address\n");
                                state = SET_A_RCV_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_A_RCV_STATE :
                            if (buf[0] == C_SET) {
                                printf("Received control\n");
                                state = SET_C_RCV_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_C_RCV_STATE :
                            if (buf[0] == A_SENDER ^ C_SET) {
                                printf("Received bcc\n");
                                state = SET_BCC_OK_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_BCC_OK_STATE :
                            if (buf[0] == FLAG) {
                                printf("Received final flag\n");
                                state = SET_STOP_STATE;
                                unsigned char buf[BUF_SIZE] = {0};

                                buf[0] = FLAG;
                                buf[1] = C_UA;
                                buf[2] = A_SENDER;
                                buf[3] = buf[3] ^ buf[2];
                                buf[4] = FLAG;
                                int bytes1 = write(fd, buf, BUF_SIZE);
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        default :
                            printf("Error state\n");
                            break;
                }      
            }
        }

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
if (connectionParameters.role == LlRx){

        int fd = openSerialPort(connectionParameters.serialPort,
                        connectionParameters.baudRate);

        if (fd < 0){
            return -1;
        }

        struct termios oldtio;
        struct termios newtio;

        // Save current port settings
        if (tcgetattr(fd, &oldtio) == -1){
            perror("tcgetattr");
            exit(-1);
        }

        // Clear struct for new port settings
        memset(&newtio, 0, sizeof(newtio));

        newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;

        // Set input mode (non-canonical, no echo,...)
        newtio.c_lflag = 0;
        newtio.c_cc[VTIME] = 0; // Inter-character timer unused
        newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

        // VTIME e VMIN should be changed in order to protect with a
        // timeout the reception of the following character(s)

        // Now clean the line and activate the settings for the port
        // tcflush() discards data written to the object referred to
        // by fd but not transmitted, or data received but not read,
        // depending on the value of queue_selector:
        //   TCIFLUSH - flushes data received but not read.
        tcflush(fd, TCIOFLUSH);

        // Set new port settings
        if (tcsetattr(fd, TCSANOW, &newtio) == -1){
            perror("tcsetattr");
            exit(-1);
        }

        printf("New termios structure set\n");

        // Loop for input
        unsigned char buf[BUF_SIZE] = {0};

        int state = SET_START_STATE;

        while (state != SET_STOP_STATE){
            int bytes = read(fd, buf, 1);

            if (bytes > 0) {
                switch (state) {
                        case SET_START_STATE :
                            printf("Started set\n");
                            if (buf[0] == FLAG) {
                                printf("Received flag\n");
                                state = SET_FLAG_RCV_STATE;
                            }
                            break;
                            if (buf[0] == A_SENDER) {
                                printf("Received address\n");
                                state = SET_A_RCV_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_A_RCV_STATE :
                            if (buf[0] == C_DISC) {
                                printf("Received control\n");
                                state = SET_C_RCV_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_C_RCV_STATE :
                            if (buf[0] == A_SENDER ^ C_DISC) {
                                printf("Received bcc\n");
                                state = SET_BCC_OK_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_BCC_OK_STATE :
                            if (buf[0] == FLAG) {
                                printf("Received final flag\n");
                                state = SET_STOP_STATE;
                                unsigned char buf[BUF_SIZE] = {0};
                                
                                printf("Received DISC\n");
                                buf[0] = FLAG;
                                buf[1] = C_DISC;
                                buf[2] = A_SENDER;
                                buf[3] = buf[3] ^ buf[2];
                                buf[4] = FLAG;
                                int bytes1 = write(fd, buf, BUF_SIZE);
                                printf("Sent DISC\n");
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        default :
                            printf("Error state\n");
                            break;
                }      
            }
        }


        state = SET_START_STATE;

        while (state != SET_STOP_STATE){
            int bytes = read(fd, buf, 1);

            if (bytes > 0) {
                switch (state) {
                        case SET_START_STATE :
                            printf("Started set\n");
                            if (buf[0] == FLAG) {
                                printf("Received flag\n");
                                state = SET_FLAG_RCV_STATE;
                            }
                            break;
                            if (buf[0] == A_SENDER) {
                                printf("Received address\n");
                                state = SET_A_RCV_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_A_RCV_STATE :
                            if (buf[0] == C_UA) {
                                printf("Received control\n");
                                state = SET_C_RCV_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_C_RCV_STATE :
                            if (buf[0] == A_SENDER ^ C_UA) {
                                printf("Received bcc\n");
                                state = SET_BCC_OK_STATE;
                            }
                            else if (buf[0] == FLAG) {
                                state = SET_FLAG_RCV_STATE;
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        case SET_BCC_OK_STATE :
                            if (buf[0] == FLAG) {
                                printf("Received final flag\n");
                                state = SET_STOP_STATE;
                                unsigned char buf[BUF_SIZE] = {0};
                            
                                printf("Received UA\n");
                            }
                            else {
                                state = SET_START_STATE;
                            }
                            break;
                        default :
                            printf("Error state\n");
                            break;
                    }
                }
            }
    }
    int clstat = closeSerialPort();
    printf("Connection closed\n");

    return clstat;
}

