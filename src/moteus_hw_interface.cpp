#include "moteus_hw_interface/moteus_hw_interface.hpp"

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "rclcpp/rclcpp.hpp"

// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::return_type;

namespace moteus_hw_interface
{

hardware_interface::CallbackReturn MoteusHwInterface::on_init(const hardware_interface::HardwareInfo & info){
  if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
      // RCLCPP_FATAL(rclcpp::get_logger("MoteusHW"), "stopped");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // hw_motor_temperature_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_voltage_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // ifname_ = info_.hardware_parameters["ifname"];

    // for (const hardware_interface::ComponentInfo &joint : info_.joints)
    // {
    //   can_id_.emplace_back(std::stoi(joint.parameters.at("can_id")));
    // }

    // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "Configuring...");
    // time_ = std::chrono::system_clock::now();

    ifname_ = info_.hardware_parameters["ifname"];
    can_id_ = stoi(info_.hardware_parameters["can_id"]);

    hw_states_positions_.resize(info_.joints.size(), 0.0);
    hw_states_velocities_.resize(info_.joints.size(), 0.0);

    hw_commands_velocities_.resize(info_.joints.size(), 0.0);
    // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "ifname: %c, id: %d",ifname_,can_id_);
    return CallbackReturn::SUCCESS;
  }

hardware_interface::CallbackReturn MoteusHwInterface::on_configure(const rclcpp_lifecycle::State &/*previous_state*/){
  driver.setup(can_id_,ifname_);

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteusHwInterface::on_activate(const rclcpp_lifecycle::State &/*previous_state*/){
  return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MoteusHwInterface::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/){
  driver.deactivate();
  return CallbackReturn::SUCCESS;
}

// std::vector<hardware_interface::CommandInterface> MoteusHwInterface::export_command_interfaces(){
//   // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "Configuring command ifaces");

//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//   command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[0]));

//   return command_interfaces;
// }

// std::vector<hardware_interface::CommandInterface> MoteusHwInterface::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//    for (std::size_t i = 0; i < info_.joints.size(); i++)
//   {
//     //command_interfaces.emplace_back(hardware_interface::CommandInterface(
//     //  info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
//     command_interfaces.emplace_back(hardware_interface::CommandInterface(
//       info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
//     //command_interfaces.emplace_back(hardware_interface::CommandInterface(
//     //  info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
//   }
//   return command_interfaces;
// }

// std::vector<hardware_interface::StateInterface> MoteusHwInterface::export_state_interfaces(){
//   // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "Configuring state ifaces");

//   std::vector<hardware_interface::StateInterface> state_interfaces;
//   state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[0]));
//   state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[0]));

//   return state_interfaces;
// }
// std::vector<hardware_interface::StateInterface> MoteusHwInterface::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;
//   for (std::size_t i = 0; i < info_.joints.size(); i++)
//   {
//     state_interfaces.emplace_back(hardware_interface::StateInterface(
//       info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
//     state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
//     //state_interfaces.emplace_back(hardware_interface::StateInterface(
//     //  info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
//   }
//   //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Export Vel L: %f R: %f", hw_states_velocities_[0], hw_states_velocities_[1]);

//   return state_interfaces;
// }

hardware_interface::return_type MoteusHwInterface::read(const rclcpp::Time &, const rclcpp::Duration &){
  MoteusState read_state = driver.get_state();
  // // for (std::size_t i = 0; i < info_.joints.size(); i++){
  //   hw_states_velocities_[0] = (double)read_state.velocity;
  //   hw_states_positions_[0] = (double)read_state.position;
  // // }
  for (std::size_t i = 0; i < info_.joints.size(); i++){
    const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    set_state(name_vel, read_state.velocity);
    set_state(name_pos, read_state.position);
  }

  // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "Velocity: %.2f, Position: %.2f",read_state.velocity,read_state.position);
  return return_type::OK;
}

hardware_interface::return_type MoteusHwInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
  // for (std::size_t i = 0; i < info_.joints.size(); i++){
  //   driver.write_velocity(hw_commands_velocities_[i]);
  // }
    for (const auto & [name, descr] : joint_command_interfaces_)
  {
    double target_velocity = get_command(name);
    if(target_velocity == 0.0){
      driver.write_stop();
    }else{
      driver.write_velocity(target_velocity);
    }
  }
  return return_type::OK;
}
} // namespace moteus_hw_interface


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  moteus_hw_interface::MoteusHwInterface,
  hardware_interface::SystemInterface
)