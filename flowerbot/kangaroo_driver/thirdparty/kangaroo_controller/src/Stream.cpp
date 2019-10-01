#include "Stream.h"
#include <stdexcept>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>


Stream::Stream()
{
}

Stream::~Stream()
{
    if (_port>0)
        close(_port);
}

bool Stream::begin(const std::string &port_name, int timeout, size_t baudrate)
{
    _port = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (_port < 0)
    {
        throw std::runtime_error("Can't open port");
        return false;
    }

    switch (baudrate)
    {
    case B0:
        break;
    case B50:
        break;
    case B75:
        break;
    case B110:
        break;
    case B134:
        break;
    case B150:
        break;
    case B200:
        break;
    case B300:
        break;
    case B600:
        break;
    case B1200:
        break;
    case B1800:
        break;
    case B2400:
        break;
    case B4800:
        break;
    case B9600:
        break;
    case B19200:
        break;
    case B38400:
        break;
    case B115200:
        break;
    default:
        throw std::runtime_error("Invalid baudrate.");
        return false;
    }
    struct termios SerialPortSettings; /* Create the structure                          */

    tcgetattr(_port, &SerialPortSettings); /* Get the current attributes of the Serial port */

    cfsetispeed(&SerialPortSettings, baudrate); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings, baudrate); /* Set Write Speed as 9600                       */

    SerialPortSettings.c_cflag &= ~PARENB; /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB; /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;  /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |= CS8;     /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_lflag = 0;			// no signaling chars, no echo, no canonical processing
	SerialPortSettings.c_oflag = 0;			// no remapping, no delays
	SerialPortSettings.c_cc[VMIN] = 0;			// read doesn't block
	SerialPortSettings.c_cc[VTIME] = 0;		// 0.5 seconds read timeout

	SerialPortSettings.c_cflag |= CREAD | CLOCAL;						// turn on READ & ignore ctrl lines
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);				// turn off s/w flow ctrl
	SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);		// make raw
	SerialPortSettings.c_oflag &= ~OPOST;								// make raw
    tcflush(_port, TCIFLUSH);
    tcflush(_port, TCOFLUSH);
    if ((tcsetattr(_port, TCSANOW, &SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
        throw std::runtime_error("ERROR ! in Setting attributes");
    
    //std::cout<<"Initialize prot.";
    return true;
    //_read_thread=pthread_create(&_read_thread, NULL, readPort, NULL); 
}


int Stream::readByte(void)
{
    int bytes_read;
    uint8_t byte[1];
    int a=available();
    bytes_read = read(_port, byte, 1);
    if (bytes_read <= 0)
        return -1;
    else
    {
        //std::cout<<"read:"<<(int)byte[0]<<"\n";
        return byte[0];
    }
    
}



// int Stream::readByte(void)
// {
//     if (available() == 0)
//         return -1;
//     else
//     {
//         int a = _rx_buffer.front();
//         _rx_buffer.pop();
//     }
// }


// void Stream::readPort()
// {
//     unsigned long bytes_available;
//     ioctl(_port, FIONREAD, &bytes_available);
//     int bytes_read = read(_port, _buffer, bytes_available);
//     for (int i=0; i<bytes_read; i++)
//         _rx_buffer.push(_buffer[i]);
// }
