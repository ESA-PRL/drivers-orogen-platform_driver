#include "Marta.hpp"
#include <platform_driver/Platform_Driver.h>

using namespace platform_driver;

Marta::Marta(std::string const& name) : MartaBase(name) {}

Marta::Marta(std::string const& name, RTT::ExecutionEngine* engine) : MartaBase(name, engine) {}

Marta::~Marta() {}

bool Marta::configureHook()
{
    if (!MartaBase::configureHook() || !Task::configureHook())
    {
        return false;
    }

    numFts = _num_fts.value();
    fts_readings.resize(numFts);

    for (int i = 0; i < numFts; ++i)
    {
        fts_readings.names[i] = canParameters.Name[numMotors + i];
    }

    return true;
}

void Marta::updateHook()
{
    MartaBase::updateHook();
    Task::updateHook();

    setJointCommands();
    getJointInformation();

    joints_readings.time = base::Time::now();
    _joints_readings.write(joints_readings);

    getFtsInformation();

    fts_readings.time = base::Time::now();
    _fts_readings.write(fts_readings);
}

void Marta::setJointCommands()
{
    if (_joints_commands.read(joints_commands, false) == RTT::NewData)
    {
        for (size_t i = 0; i < joints_commands.size(); ++i)
        {
            if (canParameters.Active[i])
            {
                base::JointState& joint(joints_commands[i]);
                if (joint.isPosition())
                {
                    m_pPlatform_Driver->nodePositionCommandRad(i, joint.position);
                }
                else if (joint.isSpeed())
                {
                    m_pPlatform_Driver->nodeVelocityCommandRadS(i, joint.speed);
                }
            }
        }
    }
}

void Marta::getJointInformation()
{
    bool error_in_motor = false;
    for (int i = 0; i < numMotors; i++)
    {
        bool status = m_pPlatform_Driver->getNodeData(
            i, &dPositionRad, &dVelocityRadS, &dCurrentAmp, &dTorqueNm);

        /** Joints readings & status information **/
        base::JointState& joint(joints_readings[i]);
        joint.position = dPositionRad;
        joint.speed = dVelocityRadS;
        joint.raw = dCurrentAmp;
        joint.effort = dTorqueNm;

        if (canParameters.Active[i] && status)
            joints_status[i] = true;
        else
            joints_status[i] = false;

        if (!status)
        {
            if (joints_resurrection[i] < 3)
            {
                //! In case of reseting a node stop the motion of the rest of the motor
                for (size_t j = 0; j < static_cast<size_t>(numMotors); j++)
                {
                    if (canParameters.Active[j])
                    {
                        m_pPlatform_Driver->nodeVelocityCommandRadS(j, 0.0);
                    }
                }
                LOG_DEBUG_S << "Resetting motor " << i;
                m_pPlatform_Driver->resetNode(i);
                joints_resurrection[i]++;
            }
            else
            {
                if (canParameters.Active[i] == ACTIVE)
                {
                    //! In case of inactivating a node stop the motion of the rest of the motor
                    for (size_t j = 0; j < static_cast<size_t>(numMotors); ++j)
                    {
                        if (canParameters.Active[j])
                        {
                            m_pPlatform_Driver->nodeVelocityCommandRadS(j, 0.0);
                        }
                    }
                    m_pPlatform_Driver->shutdownNode(i);
                    LOG_DEBUG_S << "Motor " << i << " INACTIVE";
                    canParameters.Active[i] = INACTIVE;
                }
            }
            _error_in_motor.write(i + 1);
            error_in_motor = true;
        }
    }
    if (!error_in_motor) _error_in_motor.write(0);

    /** Get the Passive joints information **/
    for (const auto& passive_joint : passiveConfig)
    {
        m_pPlatform_Driver->getNodeAnalogInput(passive_joint.id, &dAnalogInput);

        base::JointState& joint(joints_readings[passive_joint.name]);
        if (passive_joint.id == 0)
            joint.position = degToRad((-(dAnalogInput - 2.5) * bogie_pitch_factor));
        else
            joint.position = degToRad((dAnalogInput - 2.5) * bogie_pitch_factor);
    }

    /** Get the Analog System information (Voltage) **/
    m_pPlatform_Driver->getNodeAnalogInput(analogConfig[0].id, &dAnalogInput);
    base::JointState& joint_voltage(joints_readings[analogConfig[0].name]);
    joint_voltage.raw = (dAnalogInput - 2.5) * system_voltage_factor;

    /** Get the Analog System information (Current) **/
    m_pPlatform_Driver->getNodeAnalogInput(analogConfig[1].id, &dAnalogInput);
    base::JointState& joint_current(joints_readings[analogConfig[1].name]);
    joint_current.raw = (dAnalogInput - 2.5) * system_current_factor;
}

void Marta::getFtsInformation()
{
    for (int i = 0; i < numFts; i++)
    {
        double fx, fy, fz;
        double tx, ty, tz;
        
        m_pPlatform_Driver->getNodeFtsForceN(i, &fx, &fy, &fz); 
        m_pPlatform_Driver->getNodeFtsTorqueNm(i, &tx, &ty, &tz); 

        base::Wrench& wrench(fts_readings[i]);
        wrench.force = base::Vector3d(fx, fy, fz); 
        wrench.torque = base::Vector3d(tx, ty, tz); 
    }
}
