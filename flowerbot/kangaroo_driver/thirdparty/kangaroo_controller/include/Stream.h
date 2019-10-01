#ifndef Stream_h
#define Stream_h

#include <string>
#include <iostream>
#include <queue>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define SERIAL_BUFFER_SIZE 1024

class Stream
{
private:
    int _port;
    std::queue<uint8_t> _rx_buffer;

    uint8_t _buffer[SERIAL_BUFFER_SIZE];
    pthread_t _read_thread;

protected:
    unsigned long _timeout; // number of milliseconds to wait for the next char before aborting timed read
public:
    int available();
    int readByte();
    void clear();
    int writeByte(uint8_t c);
    int writeBytes(uint8_t *buffer, size_t lengthOfBuffer);
    bool begin(const std::string &port_name, int timeout=1000, size_t baudrate=B9600);
    void end();
    //void readPort();
    Stream();
    ~Stream();
};




inline int Stream::writeByte(uint8_t c)
{
    return write(_port, &c, 1);
}

inline int Stream::writeBytes(uint8_t *buffer, size_t lengthOfBuffer)
{
    //std::cout<<"write: "<<buffer<<"\n";
    // for (int i=0; i<lengthOfBuffer; i++)
    // {
    //     std::cout<<"write: "<<(int)buffer[i]<<"\n";
    // }
    int rt=write(_port, buffer, lengthOfBuffer);
    
    tcdrain(_port);
    if (rt==lengthOfBuffer)
        return rt;
    else
        return -1;
}

inline int Stream::available(void)
{
    size_t bytes_available;
    ioctl(_port, FIONREAD, &bytes_available);
    return bytes_available;
}

inline void Stream::clear()
{
    tcflush(_port, TCIFLUSH);
}

inline void Stream::end()
{
    close(_port);
}
#endif