#include "uart.h"
#include "packet.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>

static struct termios tty;
int uart_open(const char *device, int baudrate)
{
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open uart");
        return -1;
    }

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);

    speed_t speed = B115200;
    if (baudrate == 9600) speed = B9600;
    if (baudrate == 115200) speed = B115200;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;    // no parity
    tty.c_cflag &= ~CSTOPB;    // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;        // 8 data bits

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    perror("tcsetattr");
    close(fd);
    return -1;
}

    return fd;
}

int uart_write(int fd, const packet_t *data, int len){
    return write(fd, data, len);
}


int uart_read(int fd, void *buf, int len)
{
    uint8_t *p = buf;
    int total = 0;

    while (total < len) {
        int n = read(fd, p + total, len - total);
        if (n <= 0)
            return -1;
        total += n;
    }
    return total;
}

void uart_close(int fd){
    close(fd);
}

int uart_wait_for_response(int fd, packet_t *rx_buffer, int len)
{
    int num_of_read_bytes = uart_read(fd, rx_buffer, len);

    if (num_of_read_bytes < 0) {
        perror("uart_read failed");
        return -1;
    }

    if (num_of_read_bytes != len) {
        printf("UART RX incomplete: got %d expected %d\n", num_of_read_bytes, len);
        return -1;
    }

    return 0;   // SUCCESS
}