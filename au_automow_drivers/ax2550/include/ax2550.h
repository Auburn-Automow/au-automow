#ifndef __AX2550_H
#define __AX2550_H

/* Includes */
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

/* Defines */

#define READ_BUFFER_SIZE 4
#define SERIAL_BUFFER_SIZE 1024

/* Classes */
class AX2550 {
public:
    AX2550(std::string port);
    ~AX2550();
    
    bool move(double speed, double direction);
    
private:
    void openSerialPort();
    
    boost::asio::io_service io_service;
    boost::asio::serial_port *serial_port;
    
    boost::thread * io_service_thread;
    
    char read_buffer[READ_BUFFER_SIZE];
    
    char serial_buffer[SERIAL_BUFFER_SIZE];
    
    std::string port;
};

#endif /* __AX2550_H */