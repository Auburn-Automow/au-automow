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
}

AX2550::~AX2550() {
    if(this->serial_port.isOpen())
        this->serial_port.close();
}

void AX2550::connect() {
    try {
        // Configure the serial port
        this->serial_port.setPort(this->port);
        this->serial_port.setBaudrate(9600);
        this->serial_port.setParity(serial::PARITY_EVEN);
        this->serial_port.setStopbits(serial::STOPBITS_ONE);
        this->serial_port.setBytesize(serial::SEVENBITS);
        this->serial_port.setTimeoutMilliseconds(2000);
        
        // Open the serial port
        this->serial_port.open();
    } catch(std::exception &e) {
        throw(ConnectionFailedException(e.what()));
    }
    
    this->connected = true;
    
    // Synchronize with motor controller
    this->sync();
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
    // boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    this->serial_port.write("%rrrrrr\r");
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    std::string temp;
    // temp = this->serial_port.read(1);
    // while(temp.length() != 0) {
    //     std::cout << temp << std::endl;
    //     temp = this->serial_port.read(1);
    // }
    
    // Place the system in Serial mode
    for(int i = 0; i < 10; ++i) {
        this->serial_port.write("\r");
        boost::this_thread::sleep(boost::posix_time::milliseconds(25));
    }
    
    // temp = this->serial_port.read(8000);
    // printHex(const_cast<char *>(temp.c_str()), temp.length());
    // std::cout << temp << std::endl;
    // throw(SynchonizationFailedException("Man made."));
    
    // int count = 0;
    //     // std::string temp;
    //     while(!this->synced && count != 200) {
    //         // std::cout << "> " << this->serial_port.read(1) << std::endl;
    //         // continue;
    //         temp = this->serial_port.read(1);
    //         if(temp == "O") {   
    //                 temp += this->serial_port.read(1);
    //             if(temp == "K")
    //                 this->synced = true;
    //             else
    //                 count++;
    //         } else {
    //             count++;
    //         }
    //     }
    
    temp = this->serial_port.read(1000);
    if(temp.find("OK"))
        this->synced = true;
    
    if(!this->synced)
        throw(SynchonizationFailedException("Failed to get OK from motor controller."));
    this->info("Synchronized to AX2550.");
}

bool AX2550::move(double speed, double direction) {
    if(!this->connected)
        throw(MovedFailedException("Must be connected to move."));
    
    // Get the lock for reading/writing to the mc
    boost::mutex::scoped_lock lock(this->mc_mutex);
    
    char *serial_buffer = new char[5];
    
    unsigned char speed_hex, direction_hex;
    
    speed_hex = (unsigned char) (fabs(speed) * 127);
    if(speed < 0)
        sprintf(serial_buffer, "!a%.2X\r", speed_hex);
    else
        sprintf(serial_buffer, "!A%.2X\r", speed_hex);
    this->serial_port.write(serial_buffer, 5);
    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    
    // Read the echoed message
    this->serial_port.read(5);
    
    // Read the command result (+ or -)
    this->serial_port.read(1);
    
    delete[] serial_buffer;
    serial_buffer = new char[5];
    
    direction_hex = (unsigned char) (fabs(direction) * 127);
    if(direction < 0)
        sprintf(serial_buffer, "!b%.2X\r", direction_hex);
    else
        sprintf(serial_buffer, "!B%.2X\r", direction_hex);
    
    this->serial_port.write(serial_buffer, 5);
    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    
    // Read the echoed message
    this->serial_port.read(serial_buffer, 5);
    
    // Read the command result (+ or -)
    this->serial_port.read(serial_buffer, 1);
    
    // Check for successfull command execution
    // if(char(*serial_buffer) != '+') {
    //     std::cerr << "Bad command detected (ax2550 returned '-')." << serial_buffer << std::endl;
    // }
    
    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    
    return true;
}

void AX2550::setInfoMsgCallback(void (*f)(const std::string &msg)) {
    this->info = f;
}

void AX2550::setErrorMsgCallback(void (*f)(const std::string &msg)) {
    this->error = f;
}