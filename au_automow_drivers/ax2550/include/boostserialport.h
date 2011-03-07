#ifndef __BOOSTSERIALPORT_H
#define __BOOSTSERIALPORT_H

/* Includes */
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

/* Defines */

/* Classes */
class Serial {
public:
    Serial();
    Serial(string port, int baudrate=9600);
    ~Serial();
    
    int write(char message[]);
    
    char
    
private:
    boost::asio::io_service io_service;
};

#endif /* __BOOSTSERIALPORT_H */