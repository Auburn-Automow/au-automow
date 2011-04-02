#ifndef __AX2550_H
#define __AX2550_H

/* Includes */
#include <iostream>
#include <string>

#include "boost/thread/mutex.hpp"

#include "serial.h"

/* Defines */


/* Classes */
class AX2550 {
public:
    AX2550(std::string port);
    ~AX2550();
    
    void connect();
    
    bool move(double speed, double direction);
    
    void setInfoMsgCallback(void (*f)(std::string &msg));
    void setErrorMsgCallback(void (*f)(std::string &msg));
private:
    void sync();
    
    serial::Serial serial_port;
    std::string port;
    bool connected;
    bool synced;
    boost::mutex mc_mutex;
    void (*info)(std::string &msg);
    void (*error)(std::string &msg);
};

class ConnectionFailedException : public std::exception {
    const char * e_what;
public:
    ConnectionFailedException(const char * e_what) {this->e_what = e_what;}
    
    virtual const char* what() const throw() {
        std::stringstream ss;
        ss << "Error connecting to AX2550: " << this->e_what;
        return ss.str().c_str();
    }
};

class SynchonizationFailedException : public std::exception {
    const char * e_what;
public:
    SynchonizationFailedException(const char * e_what) {this->e_what = e_what;}
    
    virtual const char* what() const throw() {
        std::stringstream ss;
        ss << "Error synchronizing with the AX2550: " << this->e_what;
        return ss.str().c_str();
    }
};

class MovedFailedException : public std::exception {
    const char * e_what;
public:
    MovedFailedException(const char * e_what) {this->e_what = e_what;}
    
    virtual const char* what() const throw() {
        std::stringstream ss;
        ss << "Error moving with the AX2550: " << this->e_what;
        return ss.str().c_str();
    }
};

#endif /* __AX2550_H */