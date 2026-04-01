#if defined(__linux__)
#include "adapters/linux/LinuxSerialTransport.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <time.h>

LinuxSerialTransport::LinuxSerialTransport(const char* device, int baud)
{
    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);

    struct termios tty;
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcsetattr(fd, TCSANOW, &tty);
}

bool LinuxSerialTransport::begin()
{
    return fd >= 0;
}

size_t LinuxSerialTransport::write(const uint8_t* data, size_t len)
{
    return ::write(fd, data, len);
}

size_t LinuxSerialTransport::read(uint8_t* data, size_t len)
{
    return ::read(fd, data, len);
}

size_t LinuxSerialTransport::available()
{
    int bytes;
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