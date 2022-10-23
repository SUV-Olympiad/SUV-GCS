#include "uav_sender.h"
#include "uav.h"
#include "logger.h"
#include "sleeper.h"
#include "vehicle.h"
#include <QDateTime>
#include <QMutexLocker>
#include <QtMath>

#define RAD2DEG		(57.0)
#define DEG2RAD     (0.0174533)

CCModelCmdSender::CCModelCmdSender(QObject* parent)    
    : QObject(parent)
{
}

CCModelCmdSender::CCModelCmdSender(CUAV* aAgent, QObject* parent)
	: QObject(parent)
{
	mTargetX = 0.0f;
	mTargetY = 0.0f;
	mTargetZ = 0.0f;
	mHead = 0.0;

    mTakeOffReady = false;
    mLandReady = false;
	mGyroCalib = false;
    mAccelCalib = false;
	mLevelCalib = false;
	mReboot = false;
    mEmbeddedScenarioMode = true;

    mAgent = aAgent;
    mComm  = mAgent->comm();

	mSysID = 255;
    mTimeOutCount = 0;

    mPrevTime = 0;
}

CCModelCmdSender::~CCModelCmdSender()
{ }

void CCModelCmdSender::lock()
{
//    ros::ServiceClient arming_client = nh_cmodelcmdsender.serviceClient<mavros_msgs::CommandLong>
//        (("/agent" + std::to_string(mAgent->id()) + "/mavros/cmd/command").c_str());
//    mavros_msgs::CommandLong arm_cmd;
//    arm_cmd.request.command = 1;
//    arming_client.call(arm_cmd);

    // mavros_msgs::CommandLongQHAC lock_cmd;
    // lock_cmd.command = 1;
    // mAgent->dataROS()->publishCommand(lock_cmd);
}

void CCModelCmdSender::unlock()
{
//    ros::ServiceClient arming_client = nh_cmodelcmdsender.serviceClient<mavros_msgs::CommandLong>
//        (("/agent" + std::to_string(mAgent->id()) + "/mavros/cmd/command").c_str());
//    mavros_msgs::CommandLong arm_cmd;
//    arm_cmd.request.command = 2;
//    arming_client.call(arm_cmd);

    // mavros_msgs::CommandLongQHAC unlock_cmd;
    // unlock_cmd.command = 2;
    // mAgent->dataROS()->publishCommand(unlock_cmd);
}

void CCModelCmdSender::takeoff(double height, double yaw)
{
    auto takeoff_cmd = px4_msgs::msg::VehicleCommand();
    takeoff_cmd.target_system = mAgent->sysID();
    takeoff_cmd.command = MAV_CMD_NAV_TAKEOFF;
    takeoff_cmd.param1 = -1.0;
    takeoff_cmd.param2 = 0.0;
    takeoff_cmd.param3 = 0.0;
    takeoff_cmd.param4 = yaw;
    takeoff_cmd.param5 = qInf();
    takeoff_cmd.param6 = qInf();
    takeoff_cmd.param7 = mAgent->data("REF_ALT").toDouble() + height;
    takeoff_cmd.confirmation = false;
    takeoff_cmd.from_external = true;
    mAgent->dataROS()->publishCommand(takeoff_cmd);
}

void CCModelCmdSender::landing()
{
    auto landing_cmd = px4_msgs::msg::VehicleCommand();
    landing_cmd.target_system = mAgent->sysID();
    landing_cmd.command = MAV_CMD_NAV_LAND;
    landing_cmd.from_external = true;
    mAgent->dataROS()->publishCommand(landing_cmd);
}

void CCModelCmdSender::reposition(double lat, double lng, double altitude, double yaw)
{
    auto reposition_cmd = px4_msgs::msg::VehicleCommand();
    reposition_cmd.target_system = mAgent->sysID();
    reposition_cmd.command = MAV_CMD_DO_REPOSITION;
    reposition_cmd.param1 = -1.0;
    reposition_cmd.param2 = 1.0;
    reposition_cmd.param3 = 0.0;
    reposition_cmd.param4 = yaw ;
    reposition_cmd.param5 = lat;
    reposition_cmd.param6 = lng;
    reposition_cmd.param7 = altitude;
    reposition_cmd.from_external = true;
    mAgent->dataROS()->publishCommand(reposition_cmd);
}

void CCModelCmdSender::planMission()
{
    auto mission_plan_cmd = px4_msgs::msg::VehicleCommand();
    mission_plan_cmd.target_system = mAgent->sysID();
    mission_plan_cmd.command = MAV_CMD_NAV_MISSION_PLAN;
    mission_plan_cmd.from_external = true;
    mAgent->dataROS()->publishCommand(mission_plan_cmd);
}

void CCModelCmdSender::startMission()
{
    auto mission_start_cmd = px4_msgs::msg::VehicleCommand();
    mission_start_cmd.target_system = mAgent->sysID();
    mission_start_cmd.command = MAV_CMD_MISSION_START;
    // mission_start_cmd.confirmation = true;
    // mission_start_cmd.from_external = true;

    mAgent->dataROS()->publishCommand(mission_start_cmd);
}

void CCModelCmdSender::arm()
{
    auto arm_cmd = px4_msgs::msg::VehicleCommand();
    arm_cmd.target_system = mAgent->sysID();
    arm_cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    arm_cmd.param1 = 1.0;
    arm_cmd.confirmation = true;
    arm_cmd.from_external = true;

    mAgent->dataROS()->publishCommand(arm_cmd);
}

void CCModelCmdSender::disarm()
{
    auto disarm_cmd = px4_msgs::msg::VehicleCommand();
    disarm_cmd.target_system = mAgent->sysID();
    disarm_cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    disarm_cmd.param1 = 0.0;
    disarm_cmd.confirmation = true;

    mAgent->dataROS()->publishCommand(disarm_cmd);
}

int CCModelCmdSender::reboot()
{
	mAgent->data()->resetAck();
	mReboot = true;

	return 0;
}

int CCModelCmdSender::procArm()
{
    auto arm_cmd = px4_msgs::msg::VehicleCommand();
    arm_cmd.target_system = mAgent->sysID();
    arm_cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    arm_cmd.param1 = 1.0;
    arm_cmd.confirmation = true;

    mAgent->dataROS()->publishCommand(arm_cmd);
    return true;
}

int CCModelCmdSender::procDisarm()
{
    auto disarm_cmd = px4_msgs::msg::VehicleCommand();
    disarm_cmd.target_system = mAgent->sysID();
    disarm_cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    disarm_cmd.param1 = 0.0;
    disarm_cmd.confirmation = true;

    mAgent->dataROS()->publishCommand(disarm_cmd);
    return true;
}

int CCModelCmdSender::procReboot()
{
    auto reboot_cmd = px4_msgs::msg::VehicleCommand();
    reboot_cmd.target_system = mAgent->sysID();
    reboot_cmd.command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    reboot_cmd.param1 = 1.0;

    reboot_cmd.confirmation = true;
    
    mAgent->dataROS()->publishCommand(reboot_cmd);

    return true;
}

int CCModelCmdSender::automode()
{
	// ros::ServiceClient set_mode_client = nh_cmodelcmdsender.serviceClient<mavros_msgs::SetMode>
	// 	(("/agent" + std::to_string(mAgent->id()) + "/mavros/set_mode").c_str());
	// mavros_msgs::SetMode offb_set_mode;
 //    offb_set_mode.request.custom_mode = "AUTO";

 //    if( set_mode_client.call(offb_set_mode) &&
 //        offb_set_mode.response.mode_sent){
 //        ROS_INFO("AUTO enabled");
 //    }

 //    mavlink_message_t message;
 //    mavlink_set_mode_t mode;


	// mode.target_system = mAgent->sysID();
 //    //mode.base_mode = 189; //MAV_MODE_MANUAL_ARMED;         //209 0xD1
 //    // 189 0xBD, 1011 1101
 //    mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
 //                     //MAV_MODE_FLAG_TEST_ENABLED |
 //                     MAV_MODE_FLAG_AUTO_ENABLED |
 //                     MAV_MODE_FLAG_GUIDED_ENABLED |
 //                     MAV_MODE_FLAG_STABILIZE_ENABLED |                
 //                     //MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
 //                     MAV_MODE_FLAG_SAFETY_ARMED ;
 //    if ( mAgent->info("mode") == QString("hils") ) {
 //        mode.base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
 //    }

 //    mode.custom_mode = 262144;

 //    // make data
 //    mavlink_msg_set_mode_encode(mSysID,50, &message, &mode);


 //    // send data
	// int len = transmit(message);

 //    return len;
    return 0;
}

int CCModelCmdSender::offboard()
{
     const int PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;

     int mode             = MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
     if ( mAgent->info("mode") == QString("hils") ) {
         mode            |= MAV_MODE_FLAG_HIL_ENABLED;
     }

     auto offboard_cmd = px4_msgs::msg::VehicleCommand();
     offboard_cmd.target_system = mAgent->sysID();
     offboard_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
     offboard_cmd.confirmation = true;
     offboard_cmd.param1 = mode;
     offboard_cmd.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;

     mAgent->dataROS()->publishCommand(offboard_cmd);
    return 0;
}

int CCModelCmdSender::automission()
{
    int mode             = MAV_MODE_FLAG_SAFETY_ARMED |
                           MAV_MODE_FLAG_AUTO_ENABLED;

     auto posctl_cmd = px4_msgs::msg::VehicleCommand();
     posctl_cmd.target_system = mAgent->sysID();
     posctl_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
     posctl_cmd.confirmation = true;
     posctl_cmd.param1 = mode;
     posctl_cmd.param2 = 3;

     mAgent->dataROS()->publishCommand(posctl_cmd);
    return 0;
}

int CCModelCmdSender::manual()
{
     const int PX4_CUSTOM_MAIN_MODE_MANUAL = 1;

     int mode             = MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
     if ( mAgent->info("mode") == QString("hils") ) {
         mode            |= MAV_MODE_FLAG_HIL_ENABLED;
     }

     auto manual_cmd = px4_msgs::msg::VehicleCommand();
     manual_cmd.target_system = mAgent->sysID();
     manual_cmd.command = MAV_CMD_DO_SET_MODE;
     manual_cmd.confirmation = true;
     manual_cmd.param1 = mode;
     manual_cmd.param2 = PX4_CUSTOM_MAIN_MODE_MANUAL;

     mAgent->dataROS()->publishCommand(manual_cmd);
     return 0;
}

int CCModelCmdSender::move(float aX, float aY, float aZ, float aHead)
{
    mAgent->data()->updateTarget(aX, aY, aZ, aHead);
	return 0;
}

float CCModelCmdSender::target(CCModelCmdSender::Target aType)
{
	QMutexLocker locker(&mMutex);	
	switch (aType ) {
	case TARGET_X:
		return mTargetX;
		break;
	case TARGET_Y:
		return mTargetY;
		break;
	case TARGET_Z:
		return mTargetZ;
		break;
	default:
		qDebug("ERROR: Invalid target %d", aType);
		break;
	}
    return 0;
}

int CCModelCmdSender::procCalibGyro()
{
    auto calibgyro_cmd = px4_msgs::msg::VehicleCommand();
    calibgyro_cmd.target_system = mAgent->sysID();
    calibgyro_cmd.command = MAV_CMD_PREFLIGHT_CALIBRATION;
    calibgyro_cmd.param1 = 1;
    calibgyro_cmd.param2 = 0;
    calibgyro_cmd.param3 = 0;
    calibgyro_cmd.param4 = 0;
    calibgyro_cmd.param5 = 0;
    calibgyro_cmd.param6 = 0;
    calibgyro_cmd.param7 = 0;

    mAgent->dataROS()->publishCommand(calibgyro_cmd);
    return true;
}

int CCModelCmdSender::procCalibLevel()
{
    auto caliblevel_cmd = px4_msgs::msg::VehicleCommand();
    caliblevel_cmd.target_system = mAgent->sysID();
    caliblevel_cmd.command = MAV_CMD_PREFLIGHT_CALIBRATION;
    caliblevel_cmd.param1 = 0;
    caliblevel_cmd.param2 = 0;
    caliblevel_cmd.param3 = 0;
    caliblevel_cmd.param4 = 0;
    caliblevel_cmd.param5 = 2;
    caliblevel_cmd.param6 = 0;
    caliblevel_cmd.param7 = 0;

    mAgent->dataROS()->publishCommand(caliblevel_cmd);
    return true;
}

int CCModelCmdSender::procCalibAccel()
{
    auto calibaccel_cmd = px4_msgs::msg::VehicleCommand();
    calibaccel_cmd.target_system = mAgent->sysID();
    calibaccel_cmd.command = MAV_CMD_PREFLIGHT_CALIBRATION;
    calibaccel_cmd.param1 = 0;
    calibaccel_cmd.param2 = 0;
    calibaccel_cmd.param3 = 0;
    calibaccel_cmd.param4 = 0;
    calibaccel_cmd.param5 = 1;
    calibaccel_cmd.param6 = 0;
    calibaccel_cmd.param7 = 0;

    mAgent->dataROS()->publishCommand(calibaccel_cmd);
    return true;
}

int CCModelCmdSender::requestParam(const QString aName)
{
    px4_msgs::msg::UavcanParameterRequest paramrequest;
    std::string param_id = aName.toStdString();
    std::copy(param_id.begin(), param_id.end(), paramrequest.param_id.data());
    paramrequest.message_type = px4_msgs::msg::UavcanParameterRequest::MESSAGE_TYPE_PARAM_REQUEST_READ;
    // mAgent->dataROS()->publishRequestParam(paramrequest);
    return true;
}

QList<QString> CCModelCmdSender::getParamRequested()
{
    return mAgent->dataROS()->getParamRequested();
}


int CCModelCmdSender::setParam(const QString aName, const QVariant aValue)
{
    px4_msgs::msg::UavcanParameterRequest paramrequest;
    std::string param_id = aName.toStdString();
    std::copy(param_id.begin(), param_id.end(), paramrequest.param_id.data());
    paramrequest.message_type = px4_msgs::msg::UavcanParameterRequest::MESSAGE_TYPE_PARAM_SET;

	switch (aValue.type()) {
	case QVariant::Char:
	{
		int value = aValue.toInt();
        paramrequest.param_type = px4_msgs::msg::UavcanParameterRequest::PARAM_TYPE_UINT8;
        paramrequest.int_value = value;
	}
		break;
	case QVariant::Int:
	{
		int value = aValue.toInt();
        paramrequest.param_type = px4_msgs::msg::UavcanParameterRequest::PARAM_TYPE_INT64;
        paramrequest.int_value = value;
	}
		break;
	case QVariant::UInt:
	{
		unsigned int value = aValue.toUInt();
        paramrequest.param_type = px4_msgs::msg::UavcanParameterRequest::PARAM_TYPE_UINT8;
        paramrequest.int_value = value;
	}
		break;
	case QVariant::Double:
        paramrequest.param_type = px4_msgs::msg::UavcanParameterRequest::PARAM_TYPE_REAL32;
        paramrequest.real_value = (float)aValue.toDouble();
		break;
    case QMetaType::Float:
        paramrequest.param_type = px4_msgs::msg::UavcanParameterRequest::PARAM_TYPE_REAL32;
        paramrequest.real_value = (float)aValue.toFloat();
        break;
	default:
        qDebug("WARN: unknown type %d", aValue.type());
        paramrequest.param_type = px4_msgs::msg::UavcanParameterRequest::PARAM_TYPE_REAL32;
        paramrequest.real_value = (float)aValue.toDouble();
		break;
	}

    // mAgent->dataROS()->publishRequestParam(paramrequest);

    return true;
}

int CCModelCmdSender::calib_aceel()
{
    mAgent->data()->resetAck();
    mAccelCalib = true;

    return 0;
}

int CCModelCmdSender::calib_gyro()
{
	mAgent->data()->resetAck();
	mGyroCalib = true;

	return 0;
}

int CCModelCmdSender::calib_level()
{
	mAgent->data()->resetAck();
	mLevelCalib = true;

	return 0;
}
void CCModelCmdSender::onTimeout()
{    
	qint64 time = QDateTime::currentMSecsSinceEpoch();
	if ( mPrevTime != 0 ) mTimerPeriod = time - mPrevTime;
    mPrevTime = time;

    if ( mTakeOffReady ) {
        mTimeOutCount++;
        this->procArm();
		if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_ACCEPTED  ) {
			mTakeOffReady = false;
			mTimeOutCount = 0;
		}
    }
	else if ( mLandReady ) {
        mTimeOutCount++;
        this->procDisarm();
		if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_ACCEPTED ) {
            mLandReady = false;
            mTimeOutCount = 0;
        }
    }
	else if ( mGyroCalib ) {
        if ( mTimeOutCount % 5 == 0 ) {this->procCalibGyro();}
        mTimeOutCount++;

        if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_PREFLIGHT_CALIBRATION) == MAV_RESULT_ACCEPTED ) {
            mGyroCalib = false;
            mTimeOutCount = 0;
        }
	}
	else if ( mLevelCalib ) {
        if ( mTimeOutCount % 5 == 0 ) {this->procCalibLevel();}
        mTimeOutCount++;

        if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_PREFLIGHT_CALIBRATION) == MAV_RESULT_ACCEPTED ) {
            mLevelCalib = false;
            mTimeOutCount = 0;
        }
    }
    else if ( mAccelCalib ) {
        if ( mTimeOutCount % 5 == 0 ) {this->procCalibAccel();}
        mTimeOutCount++;

        if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_PREFLIGHT_CALIBRATION) == MAV_RESULT_ACCEPTED ) {
            mAccelCalib = false;
            mTimeOutCount = 0;
        }

    }
	else if ( mReboot ) {
        if ( mTimeOutCount % 5 == 0 ) {this->procReboot();}
        mTimeOutCount++;

        if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) == MAV_RESULT_ACCEPTED ) {
            mReboot = false;
            mTimeOutCount = 0;
        }
	}
	else {
        if (!mEmbeddedScenarioMode) {
//            target_offboard();
        }
	}
}
