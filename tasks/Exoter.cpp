#include "Exoter.hpp"
#include <platform_driver/PlatformDriverPcan.h>
#include <platform_driver/CanDriveWhistle.h>

using namespace platform_driver;

Exoter::Exoter(std::string const& name) : ExoterBase(name) {}
Exoter::Exoter(std::string const& name, RTT::ExecutionEngine* engine) : ExoterBase(name, engine) {}
Exoter::~Exoter() {}

void Exoter::setJointCommands()
{
    if (_joints_commands.read(joints_commands_, false) == RTT::NewData)
    {
        for (size_t i = 0; i < joints_commands_.size(); ++i)
        {
            if (can_parameters_.Active[i])
            {
                base::JointState& joint(joints_commands_[i]);
                if (joint.isPosition())
                {
                    platform_driver_->nodePositionCommandRad(i, joint.position);
                }
                else if (joint.isSpeed())
                {
                    platform_driver_->nodeVelocityCommandRadS(i, joint.speed);
                }
            }
        }
    }
}

void Exoter::getJointInformation()
{
    bool error_in_motor = false;
    for (int i = 0; i < num_motors_; i++)
    {
        bool status = platform_driver_->getNodeData(i, &position_, &velocity_, &current_, &torque_);

        // Joints readings & status information
        base::JointState& joint(joints_readings_[i]);
        joint.position = position_;
        joint.speed = velocity_;
        joint.raw = current_;
        joint.effort = torque_;

        if (can_parameters_.Active[i] && status)
            joints_status_[i] = true;
        else
            joints_status_[i] = false;

        if (!status)
        {
            if (joints_resurrection_[i] < 3)
            {
                // In case of reseting a node stop the motion of the rest of the motor
                for (size_t j = 0; j < static_cast<size_t>(num_motors_); j++)
                {
                    if (can_parameters_.Active[j])
                    {
                        platform_driver_->nodeVelocityCommandRadS(j, 0.0);
                    }
                }
                LOG_DEBUG_S << "Resetting motor " << i;
                platform_driver_->resetNode(i);
                joints_resurrection_[i]++;
            }
            else
            {
                if (can_parameters_.Active[i] == ACTIVE)
                {
                    // In case of inactivating a node stop the motion of the rest of the motor
                    for (size_t j = 0; j < static_cast<size_t>(num_motors_); ++j)
                    {
                        if (can_parameters_.Active[j])
                        {
                            platform_driver_->nodeVelocityCommandRadS(j, 0.0);
                        }
                    }
                    platform_driver_->shutdownNode(i);
                    LOG_DEBUG_S << "Motor " << i << " INACTIVE";
                    can_parameters_.Active[i] = INACTIVE;
                }
            }
            _error_in_motor.write(i + 1);
            error_in_motor = true;
        }
    }
    if (!error_in_motor) _error_in_motor.write(0);

    // Get the Passive joints information
    for (const auto& passive_joint : passive_config_)
    {
        platform_driver_->getNodeAnalogInput(passive_joint.id, &analog_input_);

        base::JointState& joint(joints_readings_[passive_joint.name]);
        if (passive_joint.id == 0)
            joint.position = degToRad((-(analog_input_ - 2.5) * bogie_pitch_factor_));
        else
            joint.position = degToRad((analog_input_ - 2.5) * bogie_pitch_factor_);
    }

    // Get the Analog System information (Voltage)
    platform_driver_->getNodeAnalogInput(analog_config_[0].id, &analog_input_);
    base::JointState& joint_voltage(joints_readings_[analog_config_[0].name]);
    joint_voltage.raw = (analog_input_ - 2.5) * system_voltage_factor_;

    // Get the Analog System information (Current)
    platform_driver_->getNodeAnalogInput(analog_config_[1].id, &analog_input_);
    base::JointState& joint_current(joints_readings_[analog_config_[1].name]);
    joint_current.raw = (analog_input_ - 2.5) * system_current_factor_;
}

bool Exoter::configureHook()
{
    if (!ExoterBase::configureHook() || !Task::configureHook())
    {
        return false;
    }

    platform_driver_ = std::unique_ptr<PlatformDriverPcan>(new PlatformDriverPcan(
        _num_motors, _num_nodes, _can_dev_type, _can_dev_address, _watchdog));

    return true;
}
