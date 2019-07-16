#include "Exoter.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace platform_driver;

Exoter::Exoter(std::string const& name)
    : ExoterBase(name)
{
}

Exoter::Exoter(std::string const& name, RTT::ExecutionEngine* engine)
    : ExoterBase(name, engine)
{
}

Exoter::~Exoter()
{
}

void Task::setJointCommands()
{

    std::vector<double> position_commands;
	double obj;

	position_commands.push_back(0);
	position_commands.push_back(0);  	//-1.570796
	position_commands.push_back(0);		//2.356194
	position_commands.push_back(0);		//-1.570796
	position_commands.push_back(0);



	if(!position_commands.empty())
	{
    base::commands::Joints joints_commands2(base::commands::Joints::Positions(position_commands));

    //if (_joints_commands.read(joints_commands, false) == RTT::NewData)
    //{
        //for (size_t i=0; i<joints_commands.size(); ++i)
        for (size_t i=0; i<joints_commands2.size(); ++i)
        {
            if (canParameters.Active[i])
            {
                //base::JointState& joint(joints_commands[i]);
                base::JointState& joint(joints_commands2[i]);
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
    //}
}

void Task::getJointInformation()
{
    bool error_in_motor=false;
    for (int i = 0; i < numMotors; i++)
    {
        bool status=m_pPlatform_Driver->getNodeData(i, &dPositionRad, &dVelocityRadS, &dCurrentAmp, &dTorqueNm);

        /** Joints readings & status information **/
        base::JointState& joint(joints_readings[i]);
        joint.position = dPositionRad;
        joint.speed = dVelocityRadS;
        joint.raw = dCurrentAmp;
        joint.effort = dTorqueNm;


        if (canParameters.Active[i] && status)
            joints_status[i]=true;
        else
            joints_status[i]=false;



        if (!status)
        {
            if (joints_resurrection[i]<3)
            {
                //! In case of reseting a node stop the motion of the rest of the motor
                for (size_t j = 0; j < static_cast<size_t>(numMotors); j++)
                {
                    if (canParameters.Active[j])
                    {
                        m_pPlatform_Driver->nodeVelocityCommandRadS(j, 0.0);
                    }
                }
                std::cout << "Resetting motor "<< i << std::endl;
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
                    std::cout << "Motor " << i << " INACTIVE" << std::endl;
                    canParameters.Active[i]=INACTIVE;
                }
            }
            _error_in_motor.write(i+1);
            error_in_motor=true;
        }

    }
    if (!error_in_motor)
        _error_in_motor.write(0);

    /** Get the Passive joints information **/
    /*for(const auto& passive_joint : passiveConfig)
    {
        m_pPlatform_Driver->getNodeAnalogInput(passive_joint.id, &dAnalogInput);

        base::JointState& joint(joints_readings[passive_joint.name]);
        if (passive_joint.id == 0)
            joint.position = (-(dAnalogInput-2.5)*bogie_pitch_factor)*D2R;
        else
            joint.position = (dAnalogInput-2.5)*bogie_pitch_factor*D2R;
    }*/


    /** Get the Analog System information (Voltage) **/
    /*m_pPlatform_Driver->getNodeAnalogInput(analogConfig[0].id, &dAnalogInput);
    base::JointState &joint_voltage(joints_readings[analogConfig[0].name]);
    joint_voltage.raw = (dAnalogInput-2.5)*system_voltage_factor;*/

    /** Get the Analog System information (Current) **/
    /*m_pPlatform_Driver->getNodeAnalogInput(analogConfig[1].id, &dAnalogInput);
    base::JointState &joint_current(joints_readings[analogConfig[1].name]);
    joint_current.raw = (dAnalogInput-2.5)*system_current_factor;*/
}
