#ifndef TYPE_H
#define TYPE_H

#include <QDebug>

#define CONFIG_FILE_PATH     "/home/stmoon/Project/Scenario/Test_2017"
#define NONE_TARGET         (0xffffffff)

// For Simluation
//#define REF_LAT             (47.397324502)
//#define REF_LON             (8.5461183222)
//#define REF_ALT             (490)

// For Real environment (KARI)
//#define REF_LAT             (36.37404971793671)
//#define REF_LON             (127.35286536691916)
//#define REF_ALT             (85.4)

// For Real environment (Daejeon Drone Park)
#define REF_LAT             (36.766559)
#define REF_LON             (127.281290)
#define REF_ALT             (53.5)

#define CMDSENDER_RATE      (50)    // Hz
#define CONTROL_RATE        (50)    // Hz
#define VICON_RATE          (50)    // Hz
#define SCENARIO_PERIOD     (1000)  // msec

#define FILTER_DIST_VALUE   (1.0)       // distance (m)
#define FILTER_ROT_VALUE    (30.0)       // distance (degree)

#define EMERGENCY_LIMITATION_HEIGHT         (2.7)       // (m)

#define HALF_DIMENSION_SIZE      (30.0/2.0)


#define CARRIER_PERMIT_MAX_DISTANCE        (0.10)



////////////////////////////////////////////////
#define EMERGENCY_LIMITATION_HEIGHT         (2.7)       // (m)

#define CARRIER_PERMIT_MAX_DISTANCE        (0.10)

#define LANDING_ACCEL (0.8) // [G] // Jang changed 2015. 12. 24. from 0.9 // Jang added 2015. 11. 2.
#define XBEE_WIFI_1_OR_ZIGBEE_0 (1) // Jang added 2015. 12. 10.

#endif // TYPE_H
