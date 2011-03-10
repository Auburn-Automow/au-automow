#include "ax2550.h"

AX2550::AX2550(std::string port) {
    this->port = port;
    this->openSerialPort();
}

AX2550::~AX2550() {
    if(this->serial_port->is_open())
        this->serial_port->close();
    // this->io_service_thread->join();
}

void AX2550::openSerialPort() {
    this->serial_port = new boost::asio::serial_port(this->io_service, this->port);
    
    if(not serial_port->is_open()) {
        std::cerr << "[AX2550] Failed to open serial port: " << port << std::endl;
        return;
    }
    
    try {
        this->serial_port->set_option(boost::asio::serial_port_base::baud_rate(9600));
        this->serial_port->set_option(
            boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));
        this->serial_port->set_option(
            boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::even));
        this->serial_port->set_option(
            boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
        this->serial_port->set_option(boost::asio::serial_port_base::character_size(7));
    } catch(std::exception &e) {
        std::cerr << "[AX2550] Error opening serial port: " << e.what() << std::endl;
        try {
            if(this->serial_port->is_open())
                this->serial_port->close();
        } catch(std::exception &e) {
            std::cerr << "[AX2550] Error closing serial port: " << e.what() << std::endl;
        }
        this->serial_port = NULL;
    }
}

bool AX2550::move(double speed, double direction) {
    unsigned char speed_hex, direction_hex;
    
    speed_hex = (unsigned char) (fabs(speed) * 127);
    if(speed < 0)
        sprintf(serial_buffer, "!B%.2X\r", speed_hex);
    else
        sprintf(serial_buffer, "!b%.2X\r", speed_hex);
    this->serial_port->write_some(boost::asio::buffer(serial_buffer, 5));
    // std::cout << "Speed command: " << serial_buffer << std::endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(25));
    
    // Clear the buffer
    memset(serial_buffer, NULL, SERIAL_BUFFER_SIZE);
    
    // Read the echoed message
    this->serial_port->read_some(boost::asio::buffer(serial_buffer, 5));
    
    // Clear the buffer
    memset(serial_buffer, NULL, SERIAL_BUFFER_SIZE);
    
    // Read the command result (+ or -)
    this->serial_port->read_some(boost::asio::buffer(serial_buffer, 1));
    
    // Check for successfull command execution
    // if(serial_buffer[0] != '+') {
    //     std::cerr << "Bad command detected (ax2550 returned '-')." << std::endl;
    // }
    
    direction_hex = (unsigned char) (fabs(direction) * 127);
    if(direction < 0)
        sprintf(serial_buffer, "!a%.2X\r", direction_hex);
    else
        sprintf(serial_buffer, "!A%.2X\r", direction_hex);
    
    this->serial_port->write_some(boost::asio::buffer(serial_buffer, 5));
    // std::cout << "Direction command: " << serial_buffer << std::endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(25));
    
    // Clear the buffer
    memset(serial_buffer, NULL, SERIAL_BUFFER_SIZE);
    
    // Read the echoed message
    this->serial_port->read_some(boost::asio::buffer(serial_buffer, 5));
    
    // Clear the buffer
    memset(serial_buffer, NULL, SERIAL_BUFFER_SIZE);
    
    // Read the command result (+ or -)
    this->serial_port->read_some(boost::asio::buffer(serial_buffer, 1));
    
    // Check for successfull command execution
    // if(char(*serial_buffer) != '+') {
    //     std::cerr << "Bad command detected (ax2550 returned '-')." << serial_buffer << std::endl;
    // }
    
    // Clear the buffer
    memset(serial_buffer, NULL, SERIAL_BUFFER_SIZE);
    
    boost::this_thread::sleep(boost::posix_time::milliseconds(25));
    
    return true;
}