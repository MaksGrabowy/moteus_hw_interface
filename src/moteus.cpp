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
#include <vector>

moteus::moteus()
{}

void moteus::setup(std::vector<uint8_t> can_ids, const std::string& ifname){
    drivers = can_ids;
    std::cout << "In moteus setup, can_ids length = "<< can_ids.size() <<std::endl;
    current_frames.resize(drivers.size());
    states_.resize(drivers.size());

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

    // let's reset the driver once everything is up
    write_stop();
    // write_brake();
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

void moteus::send_standard_query(int driver_number){

    struct canfd_frame frame{};
    frame.can_id  = 0x8000|(drivers[driver_number]&0x00ff)|1<<31;   // CAN ID + extended ID flag. has to be changed to incorporate other IDs than 1
    std::cout << "In query setup, driver id = "<< drivers[driver_number] <<std::endl;
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
    
        //this frame reads current MODE, POSITION, VELOCIY, TORQUE, VOLTAGE, BOARD TEMP, POWER
    
        
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

void moteus::write_velocity(std::vector<double> velocities){
    std::vector<canfd_frame> drivers_frames;
    drivers_frames.resize(drivers.size());
    for (auto i = 0u; i < drivers.size(); i++){
        struct canfd_frame frame{};

        frame.can_id  = 0x1000|drivers[i]|1<<31;   // CAN ID + extended ID flag. has to be changed to incorporate other IDs than 1
        frame.len     = 20;       // Data length
        frame.flags   = 0;       // No special flags

        frame.data[0] = 0x01;
        frame.data[1] = 0x00;
        frame.data[2] = 0x0A;
        frame.data[3] = 0x0F;
        frame.data[4] = 0x20;
        memcpy(frame.data+5,&nan,4);
        float vel = (float) velocities[i];
        memcpy(frame.data+9,&vel,4);
        memcpy(frame.data+13,&zero,4);
        u_int8_t pad = 0x50;
        for(int j=0;j<3;j++)
            frame.data[17+j] = pad; //add padding to make the frame CAN FD compliant
        drivers_frames[i] = frame;
    }
    std::lock_guard<std::mutex> lock(current_frame_mutex);
    current_frames = drivers_frames;
    resend_frame = true;
    // for(int driver_number = 0;driver_number<drivers.size();driver_number++)
    //     int n = write(sock, &current_frames[driver_number], sizeof(current_frames[driver_number]));
}


void moteus::write_stop(){
    std::vector<canfd_frame> drivers_frames;
    drivers_frames.resize(drivers.size());
    for (auto i = 0u; i < drivers.size(); i++){
        struct canfd_frame frame{};
        frame.can_id  = 0x1000|drivers[i]|1<<31;   // CAN ID + extended ID flag. has to be changed to incorporate other IDs than 1
        frame.len     = 8;       // Data length
        frame.flags   = 0;       // No special flags

        frame.data[0] = 0x01;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        u_int8_t pad = 0x50;
        for(int i=0;i<5;i++)
            frame.data[3+i] = pad; //add padding to make the frame CAN FD compliant

        drivers_frames[i] = frame;
    }
    std::lock_guard<std::mutex> lock(current_frame_mutex);
    current_frames = drivers_frames;
    resend_frame = true;
}

void moteus::write_brake(){
    struct canfd_frame frame{};
    frame.can_id  = 0x1000|can_ID|1<<31;   // CAN ID + extended ID flag. has to be changed to incorporate other IDs than 1
    frame.len     = 8;       // Data length
    frame.flags   = 0;       // No special flags

    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    frame.data[2] = 0x0F;
    u_int8_t pad = 0x50;
    for(int i=0;i<5;i++)
	    frame.data[3+i] = pad; //add padding to make the frame CAN FD compliant

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
    int driver_number = 0;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        if(driver_number == drivers.size())
            driver_number = 0;
        if(resend_frame){
            std::lock_guard<std::mutex> lock(current_frame_mutex);
            int n = write(sock, &current_frames[driver_number], sizeof(current_frames[driver_number]));
            if (n != sizeof(current_frames[driver_number])) {
                std::cerr << "Failed to send CAN frame "<< std::endl;
            }
            driver_number++;
        }
    }
}

void moteus::queryLoop() {
    int driver_number = 0;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if(driver_number == drivers.size())
            driver_number = 0;
        send_standard_query(driver_number);
        driver_number++;
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
    uint8_t in_id = (frame.can_id&CAN_EFF_MASK)>>8;
    int ext_id = frame.can_id&CAN_EFF_MASK;

    int driver_index = findIndex(drivers,in_id);
    if(driver_index != -1 && ext_id < 0x8000){
        std::lock_guard<std::mutex> lock(current_state_mutex);
        // memcpy(&this->states_[driver_index].mode, frame.data+2,1);

        // memcpy(&this->states_[driver_index].position,frame.data+5,4);
        // memcpy(&this->states_[driver_index].velocity,frame.data+9,4);
        // memcpy(&this->states_[driver_index].torque,frame.data+13,4);

        // memcpy(&this->states_[driver_index].power,frame.data+19,4);

        // memcpy(&this->states_[driver_index].voltage,frame.data+25,4);
        // memcpy(&this->states_[driver_index].board_temperature,frame.data+29,4);

        // memcpy(&this->states_[driver_index].fault,frame.data+35,2);
        memcpy(&this->mode, frame.data+2,1);

        memcpy(&this->position,frame.data+5,4);
        memcpy(&this->velocity,frame.data+9,4);
        memcpy(&this->torque,frame.data+13,4);

        memcpy(&this->power,frame.data+19,4);

        memcpy(&this->voltage,frame.data+25,4);
        memcpy(&this->board_temperature,frame.data+29,4);

        memcpy(&this->fault,frame.data+35,2);

        // states_[driver_index].mode = mode;

        // states_[driver_index].position = position;
        // states_[driver_index].velocity = velocity;
        // states_[driver_index].torque = torque;

        // states_[driver_index].power = power;

        // states_[driver_index].voltage = voltage;
        // states_[driver_index].board_temperature = board_temperature;

        // states_[driver_index].fault = fault;
        MoteusState temp;
        
        temp.mode = this->mode;

        temp.position = this->position;
        temp.velocity = this->velocity;
        temp.torque = this->torque;

        temp.power = this->power;

        temp.voltage = this->voltage;
        temp.board_temperature = this->board_temperature;

        temp.fault = this->fault;
        states_[driver_index] = temp;
        // std::cout << "Mode=0x" << std::setw(2) << std::setfill('0') << std::hex << std::uppercase << (int) mode;
        // std::cout.precision(4);
        // std::cout << " Pos=" << position << " Vel=" << velocity << " Voltage=" << voltage << " Temp=" << temperature << std::endl;
    }
}
std::vector<MoteusState> moteus::get_state(){
    std::lock_guard<std::mutex> lock(current_state_mutex);
    // MoteusState current_state;
    // current_state.mode = this->mode;
    // current_state.position = this->position;
    // current_state.velocity = this->velocity;
    // current_state.torque = this->torque;
    // current_state.power = this->power;
    // current_state.voltage = this->voltage;
    // current_state.board_temperature = this->temperature;
    // current_state.fault = this->fault;
    // std::vector<MoteusState> current_states = states_;
    return states_;
}

int moteus::findIndex(std::vector<uint8_t>& v, uint8_t val) {
    for (int i = 0; i < v.size(); i++) {
      
      	// When the element is found
        if (v[i] == val) {
            return i;
        }
    }

  	// When the element is not found
  	return -1;
}