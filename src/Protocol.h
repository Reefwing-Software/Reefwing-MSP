/******************************************************************
  @file       Protocol.h
  @brief      Commands, requests and response for the MultiWii Serial Protocol.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.7
  Date:        02/06/22

  1.0.0 Original Release.           22/02/22
  1.0.7 IMU ODR & offset bias added 02/06/22

  Credit - Version 2.4 of the MultiWii Protocol class.
           ref: https://github.com/xdu-aero-association/MultiWii_2_4/blob/master/MultiWii/Protocol.cpp
         - Arduino library for MSP by Fabrizio Di Vittorio
           ref: https://github.com/fdivitto/MSP
         - Version 1.44 of the Betaflight MSP Protocol
           ref: https://github.com/betaflight/betaflight/tree/master/src/main/msp

******************************************************************/

#ifndef Protocol_h
#define Protocol_h

/******************************************************************
 *
 * MultiWii Serial Protocol - 
 * 
 ******************************************************************/

// Multiwii Serial Protocol 0 
#define MSP_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1  // increment when major changes are made
#define API_VERSION_MINOR                   44 // increment after a release

/******************************************************************
 *
 * MSP Messages - 
 * 
 * To Multiwii developers/committers : do not add new MSP messages
 * without a proper argumentation/agreement on the forum.
 * 
 * Range id [50-99] won't be assigned and can therefore be used for
 * any custom multiwii fork without further MSP id conflict
 * 
 ******************************************************************/

#define MSP_API_VERSION          1    //out message
#define MSP_FC_VARIANT           2    //out message
#define MSP_FC_VERSION           3    //out message
#define MSP_BOARD_INFO           4    //out message
#define MSP_BUILD_INFO           5    //out message

#define MSP_NAME                 10   //out message          Returns user set board name - betaflight
#define MSP_SET_NAME             11   //in message           Sets board name - betaflight

/******************************************************************
 *
 *  NEXGEN SPECIFIC MSG IDS
 * 
 ******************************************************************/

#define MSP_IMU_ODR              50   //out message           Returns the IMU sample rates for gyro, acc and mag
#define MSP_IMU_BIAS             51   //out message           Returns the x,y and z bias offsets for gyro, acc and mag

#define MSP_IMU_CALIBRATION      75   //in message            no param
#define MSP_SET_ARM              76   //in message            no param - forces drone into ARMED state
#define MSP_SET_DISARM           77   //in message            no param - forces drone into DISARMED state

/******************************************************************
 *
 *  CLASSIC MSG IDS
 * 
 ******************************************************************/

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_NAV_STATUS           121   //out message         Returns navigation status
#define MSP_NAV_CONFIG           122   //out message         Returns navigation parameters

#define MSP_CELLS                130   //out message         FRSKY Battery Cell Voltages

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215   //in message          Sets nav config parameters - write to the eeprom  

#define MSP_SET_ACC_TRIM         239   //in message          set acc angle trim values
#define MSP_ACC_TRIM             240   //out message         get acc angle trim values
#define MSP_BIND                 241   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

/******************************************************************
 *
 * MSP Configuration Defines - 
 * 
 ******************************************************************/

//  unique flight controller IDENT - in accordance with Betaflight MSP guidelines
#define NEXGEN_IDENTIFIER "NXGN"
#define MULTIWII_IDENTIFIER "MWII";
#define BASEFLIGHT_IDENTIFIER "BAFL";
#define BETAFLIGHT_IDENTIFIER "BTFL"
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define INAV_IDENTIFIER "INAV"
#define RACEFLIGHT_IDENTIFIER "RCFL"

// flags for msp_status_ex_t.sensor and msp_status_t.sensor
#define MSP_STATUS_SENSOR_ACC    1
#define MSP_STATUS_SENSOR_BARO   2
#define MSP_STATUS_SENSOR_MAG    4
#define MSP_STATUS_SENSOR_GPS    8
#define MSP_STATUS_SENSOR_SONAR 16

#define MSP_MAX_SUPPORTED_SERVOS 8
#define MSP_MAX_SERVO_RULES (2 * MSP_MAX_SUPPORTED_SERVOS)
#define MSP_MAX_SUPPORTED_MOTORS 8
#define MSP_MAX_SUPPORTED_CHANNELS 16
#define MSP_QUAD_MOTORS 4

// values for msp_raw_gps_t.fixType
#define MSP_GPS_NO_FIX 0
#define MSP_GPS_FIX_2D 1
#define MSP_GPS_FIX_3D 2

// values for msp_nav_status_t.mode
#define MSP_NAV_STATUS_MODE_NONE   0
#define MSP_NAV_STATUS_MODE_HOLD   1
#define MSP_NAV_STATUS_MODE_RTH    2
#define MSP_NAV_STATUS_MODE_NAV    3
#define MSP_NAV_STATUS_MODE_EMERG 15

// values for msp_nav_status_t.state
#define MSP_NAV_STATUS_STATE_NONE                0  // None
#define MSP_NAV_STATUS_STATE_RTH_START           1  // RTH Start
#define MSP_NAV_STATUS_STATE_RTH_ENROUTE         2  // RTH Enroute
#define MSP_NAV_STATUS_STATE_HOLD_INFINIT        3  // PosHold infinit
#define MSP_NAV_STATUS_STATE_HOLD_TIMED          4  // PosHold timed
#define MSP_NAV_STATUS_STATE_WP_ENROUTE          5  // WP Enroute
#define MSP_NAV_STATUS_STATE_PROCESS_NEXT        6  // Process next
#define MSP_NAV_STATUS_STATE_DO_JUMP             7  // Jump
#define MSP_NAV_STATUS_STATE_LAND_START          8  // Start Land
#define MSP_NAV_STATUS_STATE_LAND_IN_PROGRESS    9  // Land in Progress
#define MSP_NAV_STATUS_STATE_LANDED             10  // Landed
#define MSP_NAV_STATUS_STATE_LAND_SETTLE        11  // Settling before land
#define MSP_NAV_STATUS_STATE_LAND_START_DESCENT 12  // Start descent

// values for msp_nav_status_t.activeWpAction, msp_set_wp_t.action
#define MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT 0x01
#define MSP_NAV_STATUS_WAYPOINT_ACTION_RTH      0x04

// values for msp_nav_status_t.error
#define MSP_NAV_STATUS_ERROR_NONE               0   // All systems clear
#define MSP_NAV_STATUS_ERROR_TOOFAR             1   // Next waypoint distance is more than safety distance
#define MSP_NAV_STATUS_ERROR_SPOILED_GPS        2   // GPS reception is compromised - Nav paused - copter is adrift !
#define MSP_NAV_STATUS_ERROR_WP_CRC             3   // CRC error reading WP data from EEPROM - Nav stopped
#define MSP_NAV_STATUS_ERROR_FINISH             4   // End flag detected, navigation finished
#define MSP_NAV_STATUS_ERROR_TIMEWAIT           5   // Waiting for poshold timer
#define MSP_NAV_STATUS_ERROR_INVALID_JUMP       6   // Invalid jump target detected, aborting
#define MSP_NAV_STATUS_ERROR_INVALID_DATA       7   // Invalid mission step action code, aborting, copter is adrift
#define MSP_NAV_STATUS_ERROR_WAIT_FOR_RTH_ALT   8   // Waiting to reach RTH Altitude
#define MSP_NAV_STATUS_ERROR_GPS_FIX_LOST       9   // Gps fix lost, aborting mission
#define MSP_NAV_STATUS_ERROR_DISARMED          10   // NAV engine disabled due disarm
#define MSP_NAV_STATUS_ERROR_LANDING           11   // Landing

// MSP_FEATURE mask
#define MSP_FEATURE_RX_PPM              (1 <<  0)
#define MSP_FEATURE_VBAT                (1 <<  1)
#define MSP_FEATURE_UNUSED_1            (1 <<  2)
#define MSP_FEATURE_RX_SERIAL           (1 <<  3)
#define MSP_FEATURE_MOTOR_STOP          (1 <<  4)
#define MSP_FEATURE_SERVO_TILT          (1 <<  5)
#define MSP_FEATURE_SOFTSERIAL          (1 <<  6)
#define MSP_FEATURE_GPS                 (1 <<  7)
#define MSP_FEATURE_UNUSED_3            (1 <<  8)         // was FEATURE_FAILSAFE
#define MSP_FEATURE_UNUSED_4            (1 <<  9)         // was FEATURE_SONAR
#define MSP_FEATURE_TELEMETRY           (1 << 10)
#define MSP_FEATURE_CURRENT_METER       (1 << 11)
#define MSP_FEATURE_3D                  (1 << 12)
#define MSP_FEATURE_RX_PARALLEL_PWM     (1 << 13)
#define MSP_FEATURE_RX_MSP              (1 << 14)
#define MSP_FEATURE_RSSI_ADC            (1 << 15)
#define MSP_FEATURE_LED_STRIP           (1 << 16)
#define MSP_FEATURE_DASHBOARD           (1 << 17)
#define MSP_FEATURE_UNUSED_2            (1 << 18)
#define MSP_FEATURE_BLACKBOX            (1 << 19)
#define MSP_FEATURE_CHANNEL_FORWARDING  (1 << 20)
#define MSP_FEATURE_TRANSPONDER         (1 << 21)
#define MSP_FEATURE_AIRMODE             (1 << 22)
#define MSP_FEATURE_SUPEREXPO_RATES     (1 << 23)
#define MSP_FEATURE_VTX                 (1 << 24)
#define MSP_FEATURE_RX_SPI              (1 << 25)
#define MSP_FEATURE_SOFTSPI             (1 << 26)
#define MSP_FEATURE_PWM_SERVO_DRIVER    (1 << 27)
#define MSP_FEATURE_PWM_OUTPUT_ENABLE   (1 << 28)
#define MSP_FEATURE_OSD                 (1 << 29)

// values for msp_current_meter_config_t.currentMeterType
#define MSP_CURRENT_SENSOR_NONE    0
#define MSP_CURRENT_SENSOR_ADC     1
#define MSP_CURRENT_SENSOR_VIRTUAL 2
#define MSP_CURRENT_SENSOR_MAX     CURRENT_SENSOR_VIRTUAL

// msp_rx_config_t.serialrx_provider
#define MSP_SERIALRX_SPEKTRUM1024      0
#define MSP_SERIALRX_SPEKTRUM2048      1
#define MSP_SERIALRX_SBUS              2
#define MSP_SERIALRX_SUMD              3
#define MSP_SERIALRX_SUMH              4
#define MSP_SERIALRX_XBUS_MODE_B       5
#define MSP_SERIALRX_XBUS_MODE_B_RJ01  6
#define MSP_SERIALRX_IBUS              7
#define MSP_SERIALRX_JETIEXBUS         8
#define MSP_SERIALRX_CRSF              9


// msp_rx_config_t.rx_spi_protocol values
#define MSP_SPI_PROT_NRF24RX_V202_250K 0
#define MSP_SPI_PROT_NRF24RX_V202_1M   1
#define MSP_SPI_PROT_NRF24RX_SYMA_X    2
#define MSP_SPI_PROT_NRF24RX_SYMA_X5C  3
#define MSP_SPI_PROT_NRF24RX_CX10      4
#define MSP_SPI_PROT_NRF24RX_CX10A     5
#define MSP_SPI_PROT_NRF24RX_H8_3D     6
#define MSP_SPI_PROT_NRF24RX_INAV      7

#define MSP_MAX_MAPPABLE_RX_INPUTS 8

// values for msp_sensor_alignment_t.gyro_align, acc_align, mag_align
#define MSP_SENSOR_ALIGN_CW0_DEG        1
#define MSP_SENSOR_ALIGN_CW90_DEG       2
#define MSP_SENSOR_ALIGN_CW180_DEG      3
#define MSP_SENSOR_ALIGN_CW270_DEG      4
#define MSP_SENSOR_ALIGN_CW0_DEG_FLIP   5
#define MSP_SENSOR_ALIGN_CW90_DEG_FLIP  6
#define MSP_SENSOR_ALIGN_CW180_DEG_FLIP 7
#define MSP_SENSOR_ALIGN_CW270_DEG_FLIP 8

//  multitype ID's - used in MultiWiiConf
#define TRI            1 
#define QUADP          2 
#define QUADX          3 
#define BI             4 
#define GIMBAL         5 
#define Y6             6 
#define HEX6           7 
#define FLYING_WING    8 
#define Y4             9 
#define HEX6X          10 
#define OCTOX8         11 
#define OCTOFLATX      12 
#define OCTOFLATP      13 
#define AIRPLANE       14 
#define HELI_120_CCPM  15 
#define HELI_90_DEG    16 
#define VTAIL4         17 
#define HEX6H          18 
#define PPM_TO_SERVO   19 
#define DUALCOPTER     20 
#define SINGLECOPTER   21 

/******************************************************************
 *
 * MSP Type Structures - 
 * 
 ******************************************************************/

// MSP_IDENT reply
struct msp_ident_t {
  uint8_t multiWiiVersion;
  uint8_t multiType;
  uint8_t mspVersion;
  uint32_t capability;
} __attribute__ ((packed));

// MSP_API_VERSION reply
struct msp_api_version_t {
  uint8_t protocolVersion;
  uint8_t APIMajor;
  uint8_t APIMinor;
} __attribute__ ((packed));


// MSP_FC_VARIANT reply
struct msp_fc_variant_t {
  char flightControlIdentifier[5];
} __attribute__ ((packed));


// MSP_FC_VERSION reply
struct msp_fc_version_t {
  uint8_t versionMajor;
  uint8_t versionMinor;
  uint8_t versionPatchLevel;
} __attribute__ ((packed));


// MSP_BOARD_INFO reply
struct msp_board_info_t {
  char     boardIdentifier[5];
  uint16_t hardwareRevision;
} __attribute__ ((packed));


// MSP_BUILD_INFO reply
struct msp_build_info_t {
  char buildDate[11];
  char buildTime[8];
  char shortGitRevision[7];
} __attribute__ ((packed));


// MSP_RAW_IMU reply
struct msp_raw_imu_t {
  int16_t acc[3];  // x, y, z
  int16_t gyro[3]; // x, y, z
  int16_t mag[3];  // x, y, z  
} __attribute__ ((packed));


// MSP_STATUS_EX reply
struct msp_status_ex_t {
  uint16_t cycleTime;
  uint16_t i2cErrorCounter;
  uint16_t sensor;                    // MSP_STATUS_SENSOR_...
  uint32_t flightModeFlags;           // see getActiveModes()
  uint8_t  configProfileIndex;
  uint16_t averageSystemLoadPercent;  // 0...100
  uint16_t armingFlags;
  uint8_t  accCalibrationAxisFlags;
} __attribute__ ((packed));


// MSP_STATUS
struct msp_status_t {
  uint16_t cycleTime;
  uint16_t i2cErrorCounter;
  uint16_t sensor;                    // MSP_STATUS_SENSOR_...
  uint32_t flightModeFlags;           // see getActiveModes()
  uint8_t  configProfileIndex;        // Connection Mode
} __attribute__ ((packed));


// MSP_SENSOR_STATUS reply
struct msp_sensor_status_t {
  uint8_t isHardwareHealthy;  // 0...1
  uint8_t hwGyroStatus;
  uint8_t hwAccelerometerStatus;
  uint8_t hwCompassStatus;
  uint8_t hwBarometerStatus;
  uint8_t hwGPSStatus;
  uint8_t hwRangefinderStatus;
  uint8_t hwPitotmeterStatus;
  uint8_t hwOpticalFlowStatus;  
} __attribute__ ((packed));


// MSP_SERVO reply
struct msp_servo_t {
  uint16_t servo[MSP_MAX_SUPPORTED_SERVOS];
} __attribute__ ((packed));


// MSP_SERVO_CONFIGURATIONS reply
struct msp_servo_configurations_t {
  __attribute__ ((packed)) struct {
    uint16_t min;
    uint16_t max;
    uint16_t middle;
    uint8_t rate;
    uint8_t angleAtMin;
    uint8_t angleAtMax;
    uint8_t forwardFromChannel;
    uint32_t reversedSources;
  } conf[MSP_MAX_SUPPORTED_SERVOS];
} __attribute__ ((packed));


// MSP_SERVO_MIX_RULES reply
struct msp_servo_mix_rules_t {
  __attribute__ ((packed)) struct {
    uint8_t targetChannel;
    uint8_t inputSource;
    uint8_t rate;
    uint8_t speed;
    uint8_t min;
    uint8_t max;
  } mixRule[MSP_MAX_SERVO_RULES];
} __attribute__ ((packed));


// MSP_MOTOR reply
struct msp_motor_t {
  uint16_t motor[MSP_QUAD_MOTORS];
} __attribute__ ((packed));


// MSP_RC reply
struct msp_rc_t {
  uint16_t channelValue[MSP_MAX_SUPPORTED_CHANNELS];
} __attribute__ ((packed));


// MSP_ATTITUDE reply
struct msp_attitude_t {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
} __attribute__ ((packed));


// MSP_ALTITUDE reply
struct msp_altitude_t {
  int32_t estimatedActualPosition;  // cm
  int16_t estimatedActualVelocity;  // cm/s
  int32_t baroLatestAltitude;      
} __attribute__ ((packed));


// MSP_SONAR_ALTITUDE reply
struct msp_sonar_altitude_t {
  int32_t altitude;
} __attribute__ ((packed));


// MSP_ANALOG reply
struct msp_analog_t {
  uint8_t  vbat;     // 0...255
  uint16_t mAhDrawn; // milliamp hours drawn from battery
  uint16_t rssi;     // 0..1023
  int16_t  amperage; // send amperage in 0.01 A steps, range is -320A to 320A  
} __attribute__ ((packed));


// MSP_ARMING_CONFIG reply
struct msp_arming_config_t {
  uint8_t auto_disarm_delay;
  uint8_t disarm_kill_switch;
} __attribute__ ((packed));


// MSP_LOOP_TIME reply
struct msp_loop_time_t {
  uint16_t looptime;
} __attribute__ ((packed));


// MSP_RC_TUNING reply
struct msp_rc_tuning_t {
  uint8_t  rcRate8;  // no longer used
  uint8_t  rcExpo8;
  uint8_t  rates[3]; // R,P,Y
  uint8_t  dynThrPID;
  uint8_t  thrMid8;
  uint8_t  thrExpo8;
  uint16_t tpa_breakpoint;
  uint8_t  rcYawExpo8;  
} __attribute__ ((packed));


// MSP_PID reply
struct msp_pid_t {
  uint8_t roll[3];     // 0=P, 1=I, 2=D
  uint8_t pitch[3];    // 0=P, 1=I, 2=D
  uint8_t yaw[3];      // 0=P, 1=I, 2=D
  uint8_t pos_z[3];    // 0=P, 1=I, 2=D
  uint8_t pos_xy[3];   // 0=P, 1=I, 2=D
  uint8_t vel_xy[3];   // 0=P, 1=I, 2=D
  uint8_t surface[3];  // 0=P, 1=I, 2=D
  uint8_t level[3];    // 0=P, 1=I, 2=D
  uint8_t heading[3];  // 0=P, 1=I, 2=D
  uint8_t vel_z[3];    // 0=P, 1=I, 2=D
} __attribute__ ((packed));


// MSP_MISC reply
struct msp_misc_t {
  uint16_t midrc;
  uint16_t minthrottle;
  uint16_t maxthrottle;
  uint16_t mincommand;
  uint16_t failsafe_throttle;
  uint8_t  gps_provider;
  uint8_t  gps_baudrate;
  uint8_t  gps_ubx_sbas;
  uint8_t  multiwiiCurrentMeterOutput;
  uint8_t  rssi_channel;
  uint8_t  dummy;
  uint16_t mag_declination;
  uint8_t  vbatscale;
  uint8_t  vbatmincellvoltage;
  uint8_t  vbatmaxcellvoltage;
  uint8_t  vbatwarningcellvoltage;
} __attribute__ ((packed));


// MSP_RAW_GPS reply
struct msp_raw_gps_t {
  uint8_t  fixType;       // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
  uint8_t  numSat;
  int32_t  lat;           // 1 / 10000000 deg
  int32_t  lon;           // 1 / 10000000 deg
  int16_t  alt;           // meters
  int16_t  groundSpeed;   // cm/s
  int16_t  groundCourse;  // unit: degree x 10
  uint16_t hdop;
} __attribute__ ((packed));


// MSP_COMP_GPS reply
struct msp_comp_gps_t {
  int16_t  distanceToHome;  // distance to home in meters
  int16_t  directionToHome; // direction to home in degrees
  uint8_t  heartbeat;       // toggles 0 and 1 for each change
} __attribute__ ((packed));


// MSP_NAV_STATUS reply
struct msp_nav_status_t {
  uint8_t mode;           // one of MSP_NAV_STATUS_MODE_XXX
  uint8_t state;          // one of MSP_NAV_STATUS_STATE_XXX
  uint8_t activeWpAction; // combination of MSP_NAV_STATUS_WAYPOINT_ACTION_XXX
  uint8_t activeWpNumber;
  uint8_t error;          // one of MSP_NAV_STATUS_ERROR_XXX
  int16_t magHoldHeading;
} __attribute__ ((packed));


// MSP_GPSSVINFO reply
struct msp_gpssvinfo_t {
  uint8_t dummy1;
  uint8_t dummy2;
  uint8_t dummy3;
  uint8_t dummy4;
  uint8_t HDOP;
} __attribute__ ((packed));


// MSP_GPSSTATISTICS reply
struct msp_gpsstatistics_t {
  uint16_t lastMessageDt;
  uint32_t errors;
  uint32_t timeouts;
  uint32_t packetCount;
  uint16_t hdop;
  uint16_t eph;
  uint16_t epv;
} __attribute__ ((packed));


// MSP_UID reply
struct msp_uid_t {
  uint32_t uid0;
  uint32_t uid1;
  uint32_t uid2;
} __attribute__ ((packed));


// MSP_FEATURE reply
struct msp_feature_t {
  uint32_t featureMask; // combination of MSP_FEATURE_XXX
} __attribute__ ((packed));


// MSP_BOARD_ALIGNMENT reply
struct msp_board_alignment_t {
  int16_t rollDeciDegrees;
  int16_t pitchDeciDegrees;
  int16_t yawDeciDegrees;
} __attribute__ ((packed));
 

// MSP_CURRENT_METER_CONFIG reply
struct msp_current_meter_config_t {
  int16_t currentMeterScale;
  int16_t currentMeterOffset;
  uint8_t currentMeterType; // MSP_CURRENT_SENSOR_XXX
  uint16_t batteryCapacity;  
} __attribute__ ((packed));


// MSP_RX_CONFIG reply
struct msp_rx_config_t {
  uint8_t   serialrx_provider;  // one of MSP_SERIALRX_XXX values
  uint16_t  maxcheck;
  uint16_t  midrc;
  uint16_t  mincheck;
  uint8_t   spektrum_sat_bind;
  uint16_t  rx_min_usec;
  uint16_t  rx_max_usec;
  uint8_t   dummy1;
  uint8_t   dummy2; 
  uint16_t  dummy3;
  uint8_t   rx_spi_protocol;  // one of MSP_SPI_PROT_XXX values
  uint32_t  rx_spi_id;
  uint8_t   rx_spi_rf_channel_count; 
} __attribute__ ((packed));


// MSP_RX_MAP reply
struct msp_rx_map_t {
  uint8_t rxmap[MSP_MAX_MAPPABLE_RX_INPUTS];  // [0]=roll channel, [1]=pitch channel, [2]=yaw channel, [3]=throttle channel, [3+n]=aux n channel, etc...
} __attribute__ ((packed));


// MSP_SENSOR_ALIGNMENT reply
struct msp_sensor_alignment_t {
  uint8_t gyro_align;   // one of MSP_SENSOR_ALIGN_XXX
  uint8_t acc_align;    // one of MSP_SENSOR_ALIGN_XXX
  uint8_t mag_align;    // one of MSP_SENSOR_ALIGN_XXX
} __attribute__ ((packed));


// MSP_CALIBRATION_DATA reply
struct msp_calibration_data_t {
  int16_t accZeroX;
  int16_t accZeroY;
  int16_t accZeroZ;
  int16_t accGainX;
  int16_t accGainY;
  int16_t accGainZ;
  int16_t magZeroX;
  int16_t magZeroY;
  int16_t magZeroZ;
} __attribute__ ((packed));


// MSP_SET_HEAD command
struct msp_set_head_t {
  int16_t magHoldHeading; // degrees
} __attribute__ ((packed));


// MSP_SET_RAW_RC command
struct msp_set_raw_rc_t {
  uint16_t channel[MSP_MAX_SUPPORTED_CHANNELS];
} __attribute__ ((packed));


// MSP_SET_PID command
typedef msp_pid_t msp_set_pid_t;


// MSP_SET_RAW_GPS command
struct msp_set_raw_gps_t {
  uint8_t  fixType;       // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
  uint8_t  numSat;
  int32_t  lat;           // 1 / 10000000 deg
  int32_t  lon;           // 1 / 10000000 deg
  int16_t  alt;           // meters
  int16_t  groundSpeed;   // cm/s
} __attribute__ ((packed));


// MSP_SET_WP command
// Special waypoints are 0 and 255. 0 is the RTH position, 255 is the POSHOLD position (lat, lon, alt).
struct msp_set_wp_t {
  uint8_t waypointNumber;  
  uint8_t action;   // one of MSP_NAV_STATUS_WAYPOINT_ACTION_XXX
  int32_t lat;      // decimal degrees latitude * 10000000
  int32_t lon;      // decimal degrees longitude * 10000000
  int32_t alt;      // altitude (cm)
  int16_t p1;       // speed (cm/s) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT, or "land" (value 1) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_RTH
  int16_t p2;       // not used
  int16_t p3;       // not used
  uint8_t flag;     // 0xa5 = last, otherwise set to 0
} __attribute__ ((packed));

//  Nexgen additions

// MSP_SET_MOTOR cmd
struct msp_set_motor_t {
  uint16_t motor[MSP_QUAD_MOTORS];
} __attribute__ ((packed));

// MSP_IMU_ODR reply
struct msp_imu_odr_t {
  uint16_t gyro;
  uint16_t acc;
  uint16_t mag;
} __attribute__ ((packed));

// MSP_IMU_BIAS reply
struct msp_imu_bias_t {
  char gyro_x[10];
  char gyro_y[10]; 
  char gyro_z[10];
  char acc_x[10];
  char acc_y[10];
  char acc_z[10];  
  char mag_x[10]; 
  char mag_y[10]; 
  char mag_z[10];  
} __attribute__ ((packed));

//  MSP Received Packet contents
struct msp_packet_t {
  // recvSize can be NULL
  uint8_t recvMessageID;
  uint8_t recvSizeValue;
  void * payload;
  uint8_t maxSize;
  uint8_t recvSize;
};

// MSP_DEBUG Packet contents
struct msp_debug_t {
  uint16_t debug1;
  uint16_t debug2;
  uint16_t debug3;
  uint16_t debug4;
} __attribute__ ((packed));

// MSP_DEBUGMSG Packet contents
struct msp_debug_msg_t {
  char msg[32];
} __attribute__ ((packed));

// MSP_ERROR reply
struct msp_error_t {
  uint8_t cmdID;
  char msg[32];
} __attribute__ ((packed));

//  Betaflight additions

// return positive for ACK, negative on error, zero for no reply
typedef enum {
    MSP_RESULT_ACK = 1,
    MSP_RESULT_ERROR = -1,
    MSP_RESULT_NO_REPLY = 0,
    MSP_RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
} mspResult_e;

typedef enum {
    MSP_DIRECTION_REPLY = 0,
    MSP_DIRECTION_REQUEST = 1
} mspDirection_e;

/******************************************************************
 *
 * MSP Command Lookup - 
 * 
 * Used in the viewMSPRequest.ino example sketch
 * 
 ******************************************************************/

#define BAD_ID -1

typedef struct { 
  char *key; 
  int val; 
} msp_commands_t;

static msp_commands_t lookupTable[] = {
    { "MSP_API_VERSION", MSP_API_VERSION }, { "MSP_FC_VARIANT", MSP_FC_VARIANT }, { "MSP_FC_VERSION", MSP_FC_VERSION }, { "MSP_BOARD_INFO", MSP_BOARD_INFO },
    { "MSP_BUILD_INFO", MSP_BUILD_INFO }, { "MSP_NAME", MSP_NAME }, { "MSP_SET_NAME", MSP_SET_NAME }, { "MSP_IDENT", MSP_IDENT }, { "MSP_STATUS", MSP_STATUS },
    { "MSP_RAW_IMU", MSP_RAW_IMU }, { "MSP_SERVO", MSP_SERVO }, { "MSP_MOTOR", MSP_MOTOR}, { "MSP_RC", MSP_RC }, { "MSP_ATTITUDE", MSP_ATTITUDE }, { "MSP_ALTITUDE", MSP_ALTITUDE },
    { "MSP_PID", MSP_PID }, { "MSP_BOX", MSP_BOX }, { "MSP_BOXNAMES", MSP_BOXNAMES }, { "MSP_PIDNAMES", MSP_PIDNAMES }, { "MSP_BOXIDS", MSP_BOXIDS },
    { "MSP_SET_RAW_RC", MSP_SET_RAW_RC }, { "MSP_SET_RAW_GPS", MSP_SET_RAW_GPS }, { "MSP_SET_PID", MSP_SET_PID }, { "MSP_SET_BOX", MSP_SET_BOX },
    { "MSP_SET_RC_TUNING", MSP_SET_RC_TUNING }, { "MSP_ACC_CALIBRATION", MSP_ACC_CALIBRATION }, { "MSP_MAG_CALIBRATION", MSP_MAG_CALIBRATION }, { "MSP_SET_MISC", MSP_SET_MISC },
    { "MSP_RESET_CONF", MSP_RESET_CONF }, { "MSP_SET_WP", MSP_SET_WP }, { "MSP_SELECT_SETTING", MSP_SELECT_SETTING }, { "MSP_SET_HEAD", MSP_SET_HEAD }
};

#define NKEYS (sizeof(lookupTable)/sizeof(msp_commands_t))

int idLookup(char *key);

#endif