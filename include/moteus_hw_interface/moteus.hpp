#ifndef INC_MOTEUS_H_
#define INC_MOTEUS_H_

#include <iostream>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>      
#include <net/if.h>         
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>

#include "state.h"

#include <functional>
#include <string>
#include <thread>


class moteus {
public:
    using Callback = std::function<void(const struct canfd_frame&)>;

    moteus();
    ~moteus();

    void setup(u_int8_t ID, const std::string& ifname);
    void deactivate();
    void send_standard_query();
    void write_velocity(float velocity);
    void write_stop();
    void write_brake();

    MoteusState get_state();

    double commanded_velocity;
    MoteusState state_;
private:
    u_int8_t can_ID;
    std::string interface;
    
    int sock{-1};
    bool running{true};
    bool resend_frame{false};

    std::thread rxThread;
    std::thread txThread;
    std::thread queryThread;

    Callback callback;

    canfd_frame current_frame;
    std::mutex current_frame_mutex;

    void receiveLoop();
    void sendLoop();
    void queryLoop();

    void interpret_frame(const struct canfd_frame& frame);

    // driver state
    std::mutex current_state_mutex;
    u_int8_t mode;
    float position;
    float velocity;
    float torque;
    float power;
    float voltage;
    float temperature;
    u_int16_t fault;

    // nan
    u_int32_t nan = 0x7fc00000;
    u_int32_t zero = 0;
};

#endif /* INC_MOTEUS_H_ */