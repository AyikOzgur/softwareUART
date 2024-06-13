#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

#define UART_CONFIG _IOW('U', 1, UARTConfig)

/* @brief This struct will be available from userspace too.
 */
typedef struct
{
    int txPin;
    int rxPin;
    int baudRate;

    // These are gonna be implemented coming version. 
    // Current version params: 8N1
    int dataBits;
    int stopBits;
    int parity;
    char isInverted;
} UARTConfig;


int main()
{
    // Open the device file
    int fd = open("/dev/softwareUART", O_RDWR);
    if (fd < 0)
    {
        perror("Failed to open the device file");
        return errno;
    }

    // Create a UARTConfig struct
    UARTConfig uart_params;
    uart_params.txPin = 24;
    uart_params.rxPin = 114;
    uart_params.baudRate = 9600;
    uart_params.dataBits = 8;
    uart_params.stopBits = 1;
    uart_params.parity = 0;
    uart_params.isInverted = 0;

    // Write the UARTConfig struct to the device file
    if (ioctl(fd, UART_CONFIG, &uart_params) < 0)
    {
        printf("Not init\n");
    }


    // Close the device file
    //close(fd);
    while(1)
    {

        // Read from file and print it to terminal
        char buffer[32];
        int bytes = read(fd, buffer, sizeof(buffer));
        if (bytes > 0)
        {
            buffer[bytes] = 0;
            printf("Received: %s\n", buffer);
        }

    }

    return 0;
}