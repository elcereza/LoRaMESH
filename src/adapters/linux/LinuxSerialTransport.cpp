#if defined(__linux__)
#include "adapters/linux/LinuxSerialTransport.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <time.h>
#include <string.h>

static speed_t getBaud(int baud)
{
    switch (baud)
    {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B9600;
    }
}

LinuxSerialTransport::LinuxSerialTransport(const char* device, int baud)
{
    fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        return;
    }

    int flags;
    ioctl(fd, TIOCMGET, &flags);
    flags |= TIOCM_DTR;
    flags |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &flags);

    usleep(200000);

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0)
    {
        close(fd);
        fd = -1;
        return;
    }

    speed_t speed = getBaud(baud);

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5; // 0.5s

    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
}

bool LinuxSerialTransport::begin()
{
    return fd >= 0;
}

size_t LinuxSerialTransport::write(const uint8_t* data, size_t len)
{
    if (fd < 0) return 0;
    return ::write(fd, data, len);
}

size_t LinuxSerialTransport::read(uint8_t* data, size_t len)
{
    if (fd < 0) return 0;
    return ::read(fd, data, len);
}

size_t LinuxSerialTransport::available()
{
    if (fd < 0) return 0;

    int bytes = 0;
    ioctl(fd, FIONREAD, &bytes);
    return bytes;
}

uint32_t LinuxSerialTransport::millis()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

#endif