#include "ax2550.h"

inline void defaultInfoMsgCallback(const std::string &msg) {
    std::cerr << "AX2550 Info: " << msg << std::endl;
}

inline void defaultErrorMsgCallback(const std::string &msg) {
    std::cerr << "AX2550 Error: " << msg << std::endl;
}

AX2550::AX2550(std::string port) : serial_port() {
    this->port = port;
    this->connected = false;
    this->synced = false;
    this->info = defaultInfoMsgCallback;
    this->error = defaultErrorMsgCallback;
    this->timeout = 250;
}

AX2550::~AX2550() {
    this->disconnect();
}

void AX2550::connect() {
    try {
        // Configure the serial port
        this->serial_port.setPort(this->port);
        this->serial_port.setBaudrate(9600);
        this->serial_port.setParity(serial::PARITY_EVEN);
        this->serial_port.setStopbits(serial::STOPBITS_ONE);
        this->serial_port.setBytesize(serial::SEVENBITS);
        this->serial_port.setTimeoutMilliseconds(this->timeout);
        
        // Open the serial port
        this->serial_port.open();
    } catch(std::exception &e) {
        throw(ConnectionFailedException(e.what()));
    }
    
    this->connected = true;
    
    // Synchronize with motor controller
    this->sync();
}

bool AX2550::isConnected() {
    return this->connected;
}

void AX2550::disconnect() {
    // Join the encoder thread
    
    // Close the serial port
    if(this->serial_port.isOpen())
        this->serial_port.close();
}

inline void printHex(char * data, int length) {
    for(int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
    }
    printf("\n");
}

void AX2550::sync() {
    if(synced)
        return;
    // Ensure the motor controller is in radio mode by reseting
    this->serial_port.write("%rrrrrr\r");
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000)); // Allow time to reset
    
    // Place the system in Serial mode
    for(int i = 0; i < 10; ++i) {
        this->serial_port.write("\r");
        boost::this_thread::sleep(boost::posix_time::milliseconds(25));
    }
    
    std::string temp = this->serial_port.read(1000);
    if(temp.length() > 0 && temp.find("OK") != std::string::npos)
        this->synced = true;
    
    if(!this->synced)
        throw(SynchonizationFailedException("Failed to get OK from motor controller."));
    this->info("Synchronized to AX2550.");
}

bool AX2550::move(double speed, double direction) {
    if(!this->connected)
        throw(MovedFailedException("Must be connected to move."));
    
    // Bounds check speed and direction
    if( (fabs(speed) > 1) || (fabs(direction) > 1) ) {
        std::stringstream ss;
        ss << "Error sending move command, speed " << speed << " or direction " << direction << " out of bounds.";
        this->error(ss.str());
        return false;
    }
    
    bool result = true;
    
    // Get the lock for reading/writing to the mc
    boost::mutex::scoped_lock lock(this->mc_mutex);
    
    // Clear out any watchdogs with a poor man's flush
    this->serial_port.setTimeoutMilliseconds(1);
    std::string temp = this->serial_port.read(1000);
    this->isRCMessage(temp);
    while(temp.length() != 0)
        temp = this->serial_port.read(1000);;
    this->serial_port.setTimeoutMilliseconds(this->timeout);
    
    char *serial_buffer = new char[5];
    
    unsigned char speed_hex, direction_hex;
    
    speed_hex = (unsigned char) (fabs(speed) * 127);
    if(speed < 0)
        sprintf(serial_buffer, "!a%.2X\r", speed_hex);
    else
        sprintf(serial_buffer, "!A%.2X\r", speed_hex);
    this->serial_port.write(serial_buffer, 5);
    
    // Read the echoed message
    temp = this->serial_port.read(5);
    this->isRCMessage(temp);
    if(temp != std::string(serial_buffer)) {
        this->error("Error sending move command, speed was not properly echoed.");
        result = false;
    }
    
    // Read the command result (+ or -)
    temp = this->serial_port.read(2);
    this->isRCMessage(temp);
    if(temp != "+\r") {
        this->error("Error sending move command, NAK received on speed.");
        result = false;
    }
    
    delete[] serial_buffer;
    serial_buffer = new char[5];
    
    direction_hex = (unsigned char) (fabs(direction) * 127);
    if(direction < 0)
        sprintf(serial_buffer, "!b%.2X\r", direction_hex);
    else
        sprintf(serial_buffer, "!B%.2X\r", direction_hex);
    
    this->serial_port.write(serial_buffer, 5);
    
    // Read the echoed message
    temp = this->serial_port.read(5);
    this->isRCMessage(temp);
    if(temp != std::string(serial_buffer)) {
        this->error("Error sending move command, direction was not properly echoed.");
        result = false;
    }
    
    // Read the command result (+ or -)
    temp = this->serial_port.read(2);
    this->isRCMessage(temp);
    if(temp != "+\r") {
        this->error("Error sending move command, NAK received on direction.");
        result = false;
    }
    
    return result;
}

bool AX2550::isRCMessage(std::string data) {
    if(data.find(":") != std::string::npos) {
        this->connected = false;
        this->error("Motor controller out of sync.");
        std::stringstream ss;
        ss << "Length " << data.length() << ": " << data;
        this->error(ss.str());
        return true;
    }
    return false;
}

AX2550_RPM AX2550::readRPM() {
    if(!this->connected)
        throw(QueryFailedException("Must be connected to query RPMs."));
    
    // Get the lock for reading/writing to the mc
    boost::mutex::scoped_lock lock(this->mc_mutex);
    
    // Clear out any watchdogs with a poor man's flush
    this->serial_port.setTimeoutMilliseconds(1);
    std::string temp = this->serial_port.read(1000);
    if(this->isRCMessage(temp))
        return AX2550_RPM(0,0);
    while(temp.length() != 0)
        temp = this->serial_port.read(1000);
    this->serial_port.setTimeoutMilliseconds(this->timeout);
    
    // Send the query
    if(this->serial_port.write("?z\r") != 3) {
        throw(QueryFailedException("Error reading RPMs, failed to write to the motor controller."));
    }
    
    // Read the echo
    temp = this->serial_port.read(3);
    this->isRCMessage(temp);
    if(temp != "?z\r") {
        throw(QueryFailedException("Error reading RPMs, failed to recieve the echo of the query."));
    }
    
    // Read the result
    std::string temp = this->serial_port.read(3);
    temp = this->serial_port.read(3);
    this->isRCMessage(temp);
    if(temp.length() != 3) {
        throw(QueryFailedException("Error reading RPMs, failed to recieve the response of the query."));
    }
    
    // Extract the data
    int rpm1 = 0;
    sscanf(temp.c_str(), "%X\r", &rpm1);
    
    // Read the second result
    temp = this->serial_port.read(3);
    this->isRCMessage(temp);
    if(temp.length() != 3) {
        throw(QueryFailedException("Error reading RPMs, failed to recieve the response of the query."));
    }
    
    // Extract the data
    int rpm2 = 0;
    sscanf(temp.c_str(), "%X\r", &rpm2);
    
    return AX2550_RPM(rpm1, -1*rpm2);
}

void AX2550::setInfoMsgCallback(void (*f)(const std::string &msg)) {
    this->info = f;
}

void AX2550::setErrorMsgCallback(void (*f)(const std::string &msg)) {
    this->error = f;
}
