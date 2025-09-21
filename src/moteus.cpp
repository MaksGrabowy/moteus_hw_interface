#include "moteus_hw_interface/moteus.hpp"

#include <iostream>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>      
#include <net/if.h>         
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <iomanip>

#include <chrono>
#include <thread>

moteus::moteus()
{}

void moteus::setup(u_int8_t ID, const std::string& ifname){
    can_ID = ID;
    interface = ifname;
    // Open socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        throw std::runtime_error("Error opening socket: " + std::string(strerror(errno)));
    }

    // Enable CAN FD
    int enable_canfd = 1;
    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0) {
        close(sock);
        throw std::runtime_error("Error enabling CAN FD: " + std::string(strerror(errno)));
    }

    // Locate the interface
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        close(sock);
        throw std::runtime_error("Error getting interface index: " + std::string(strerror(errno)));
    }

    // Bind
    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        throw std::runtime_error("Error binding socket: " + std::string(strerror(errno)));
    }

    // Start threads
    rxThread = std::thread(&moteus::receiveLoop, this);
    txThread = std::thread(&moteus::sendLoop, this);
    queryThread = std::thread(&moteus::queryLoop, this);
}

moteus::~moteus(){
    running = false;
    if (rxThread.joinable()) rxThread.join();
    if (txThread.joinable()) txThread.join();
    if (queryThread.joinable()) queryThread.join();
    if (sock >= 0) close(sock);
}

void moteus::deactivate(){
    running = false;
    if (rxThread.joinable()) rxThread.join();
    if (txThread.joinable()) txThread.join();
    if (queryThread.joinable()) queryThread.join();
    if (sock >= 0) close(sock);
}

void moteus::send_standard_query(){

    struct canfd_frame frame{};
    frame.can_id  = 0x8000|can_ID|1<<31;   // CAN ID + extended ID flag. has to be changed to incorporate other IDs than 1
    frame.len     = 16;       // Data length
    frame.flags   = 0;       // No special flags

    // // Example payload
    // for (int i = 0; i < frame.len; i++) {
    //     frame.data[i] = i;
    // }
    frame.data[0] = 0x11; //read one int8 register
    frame.data[1] = 0x00; //starting at MODE 0x000
    frame.data[2] = 0x1F; //read three float registers
    frame.data[3] = 0x01; //starting at POSITION 0x001
    frame.data[4] = 0x1D; //read one float register
    frame.data[5] = 0x07; //starting at TEMPERATURE 0x007
    frame.data[6] = 0x1E; //read two float registers
    frame.data[7] = 0x0D; //starting at VOLTAGE 0x00d
    frame.data[8] = 0x15; //read one int16 register
    frame.data[9] = 0x0F; //starting at FAULT 0x00F
    u_int8_t pad = 0x50;
    for(int i=0;i<6;i++)
	    frame.data[10+i] = pad; //add padding to make the frame CAN FD compliant
    
        
    int nbytes = write(sock, &frame, sizeof(struct canfd_frame));
    if (nbytes != sizeof(struct canfd_frame)) {
        std::cerr << "Write error: " << strerror(errno) << std::endl;
    } else {
        // std::cout << "Sent CAN FD frame on " << ifname << std::endl;
    }
}

void moteus::write_velocity(float velocity){
    struct canfd_frame frame{};
    frame.can_id  = 0x1000|can_ID|1<<31;   // CAN ID + extended ID flag. has to be changed to incorporate other IDs than 1
    frame.len     = 20;       // Data length
    frame.flags   = 0;       // No special flags

    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    frame.data[2] = 0x0A;
    frame.data[3] = 0x0F;
    frame.data[4] = 0x20;
    memcpy(frame.data+5,&nan,4);
    memcpy(frame.data+9,&velocity,4);
    memcpy(frame.data+13,&zero,4);
    u_int8_t pad = 0x50;
    for(int i=0;i<3;i++)
	    frame.data[17+i] = pad; //add padding to make the frame CAN FD compliant
    
    std::lock_guard<std::mutex> lock(current_frame_mutex);
    current_frame = frame;
    resend_frame = true;
}

void moteus::receiveLoop() {
    while (running) {
        struct canfd_frame frame {};
        // std::cout << "dupa" << std::endl;
        int nbytes = read(sock, &frame, sizeof(frame));
        if (nbytes < 0) {
            std::cerr << "Read error: " << strerror(errno) << std::endl;
            continue;
        }
        if (nbytes == sizeof(struct canfd_frame)) { /*means we received correct data*/
            interpret_frame(frame);
        }
    }
}

void moteus::sendLoop() {
    while (running) {

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if(resend_frame){
            std::lock_guard<std::mutex> lock(current_frame_mutex);
            int n = write(sock, &current_frame, sizeof(current_frame));
            if (n != sizeof(current_frame)) {
                std::cerr << "Failed to send CAN frame "<< std::endl;
            }
        }
    }
}

void moteus::queryLoop() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        send_standard_query();
    }
}

void moteus::interpret_frame(const struct canfd_frame& frame){
    // std::cout << "RX ID=0x" << std::hex << frame.can_id
    //             << " LEN=" << std::dec << (int)frame.len << " Data=";
    // for (int i = 0; i < frame.len; i++) {
    //     std::cout << std::hex << (int)frame.data[i] << " ";
    // }
    // std::cout << std::dec << std::endl;
    // memcpy(&this->mode,frame.data+2,1);
    std::lock_guard<std::mutex> lock(current_state_mutex);
    memcpy(&this->mode, frame.data+2,1);

    memcpy(&this->position,frame.data+5,4);
    memcpy(&this->velocity,frame.data+9,4);
    memcpy(&this->torque,frame.data+13,4);

    memcpy(&this->power,frame.data+19,4);

    memcpy(&this->voltage,frame.data+25,4);
    memcpy(&this->temperature,frame.data+29,4);

    memcpy(&this->fault,frame.data+35,2);
    // std::cout << "Mode=0x" << std::setw(2) << std::setfill('0') << std::hex << std::uppercase << (int) mode;
    // std::cout.precision(4);
    // std::cout << " Pos=" << position << " Vel=" << velocity << " Voltage=" << voltage << " Temp=" << temperature << std::endl;
}

MoteusState moteus::get_state(){
    std::lock_guard<std::mutex> lock(current_state_mutex);
    MoteusState current_state;
    current_state.mode = this->mode;
    current_state.position = this->position;
    current_state.velocity = this->velocity;
    current_state.torque = this->torque;
    current_state.power = this->power;
    current_state.voltage = this->voltage;
    current_state.board_temperature = this->temperature;
    current_state.fault = this->fault;
    return current_state;
}