#ifndef __AX2550_H
#define __AX2550_H

/* Includes */
#include <iostream>
#include <string>
#include <sstream>

#include "boost/thread/mutex.hpp"

#include "serial.h"

/* Structs */
struct AX2550_RPM {
    AX2550_RPM(signed char rpm1, signed char rpm2) {
        this->rpm1 = rpm1;
        this->rpm2 = rpm2;
    };
    signed char rpm1;
    signed char rpm2;
};

/* Defines */


/* Classes */
class AX2550 {
public:
    AX2550(std::string port);
    ~AX2550();
    
    void connect();
    
    void disconnect();
    
    bool move(double speed, double direction);
    
    AX2550_RPM readRPM();
    
    // bool startReadingEncoders();
    
    void setInfoMsgCallback(void (*f)(const std::string &msg));
    void setErrorMsgCallback(void (*f)(const std::string &msg));
private:
    void sync();
    
    serial::Serial serial_port;
    std::string port;
    bool connected;
    bool synced;
    long timeout;
    boost::mutex mc_mutex;
    void (*info)(const std::string &msg);
    void (*error)(const std::string &msg);
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

class QueryFailedException : public std::exception {
    const char * e_what;
public:
    QueryFailedException(const char * e_what) {this->e_what = e_what;}
    
    virtual const char* what() const throw() {
        std::stringstream ss;
        ss << "Error querying the AX2550: " << this->e_what;
        return ss.str().c_str();
    }
};

#endif /* __AX2550_H */