#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
// Application dependencies
#include <GCS.h>
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_SerialManager.h>   // Serial manager library
#include <AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_NavEKF.h>
#include <AP_Mission.h>         // Mission command library
#include <AP_Rally.h>           // Rally point library
#include <AC_PID.h>             // PID library
#include <AC_PI_2D.h>           // PID library (2-axis)
#include <AC_HELI_PID.h>        // Heli specific Rate PID library
#include <AC_P.h>               // P library
#include <AC_AttitudeControl.h> // Attitude control library
#include <AC_AttitudeControl_Heli.h> // Attitude control library for traditional helicopter
#include <AC_PosControl.h>      // Position control library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>           // ArduCopter waypoint navigation library
#include <AC_Circle.h>          // circle navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence.h>           // Arducopter Fence library
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library
#include <AP_Frsky_Telem.h>
#if SPRAYER == ENABLED
#include <AC_Sprayer.h>         // crop sprayer library
#endif
#if EPM_ENABLED == ENABLED
#include <AP_EPM.h>             // EPM cargo gripper stuff
#endif
#if PARACHUTE == ENABLED
#include <AP_Parachute.h>       // Parachute release library
#endif
#include <AP_LandingGear.h>     // Landing Gear library
#include <AP_Terrain.h>
#include <LowPassFilter2p.h>
// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

# define MAIN_LOOP_RATE    400
# define MAIN_LOOP_SECONDS 0.0025f
# define MAIN_LOOP_MICROS  2500

static AP_GPS  gps;
static AP_Baro barometer;
static AP_InertialSensor ins;
static AP_SerialManager serial_manager;
static Compass compass;

// SONAR

#if CONFIG_SONAR == ENABLED
static RangeFinder sonar;
static bool sonar_enabled = true; // enable user switch for sonar
#endif
// Inertial Navigation EKF


#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs(ins, barometer, gps, sonar);
#else
AP_AHRS_DCM ahrs(ins, barometer, gps);
#endif

static AP_InertialNav_NavEKF inertial_nav(ahrs);

static LowPassFilter2pfloat fil_posz(10,0.8);
static LowPassFilter2pfloat fil_velz(10,0.3);

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
int radio_roll, radio_pitch, radio_yaw, radio_throttle, aux_1, aux_2, aux_3;
float roll, pitch, yaw, saroll, sayaw;

float baro_alt=0, alt_fil, alt_fil_a=0,velocity, vel_fil,control_altura, z_des;
uint32_t time_dt;
float gyrox, gyroy, gyroz;
Vector3f gyro;
float kp_roll, kd_roll, kp_pitch, kd_pitch, kp_yaw, kd_yaw;
float c_roll, c_pitch, c_yaw, volt, corriente_tot;
//Flags
bool flag_z=true,entre_home=true;
uint8_t flag_gps=0;
uint8_t mode_flight=1;
bool flag_aux1=true;

// the reader should verify the necessary variables

static uint8_t flag=0,flag2=1;
static Vector3f pos_gps;
static Vector3f pos;
static Vector3f ref;
static Vector3f ref_p;
static Vector3f error;
static Vector3f error_p;
static Vector3f ctrl;
static Vector3f vel;
static Vector3f off;
static Vector3f vel_gps;
float x,y,xrate,yrate, c_x,c_y;
uint8_t _buffertx[2];
uint8_t _bufferrx[2];

AP_HAL::AnalogSource* ch;
static int8_t pin;
float m1,m2,m3,m4;
uint16_t radio[6];
uint16_t ii,i,refalt=0,refi=0;
int conta=0;
float recep;
AP_HAL::DigitalSource *a_led;
AP_HAL::DigitalSource *b_led;
AP_HAL::DigitalSource *c_led;
float err_yaw;
float m1_c,m2_c,m3_c,m4_c;
char readd;

// secondary tasks defined in the scheduler

static const AP_Scheduler::Task scheduler_task[] PROGMEM = {
        {update_GPS,       8,  90}, // update GPS positions
        {update_Baro,     40,  100}, // update barometer altitude
        {Trajectory, 40,  20}, // update trajectory to follow
        {Read_radio,  4, 20}, // remote control reading
        {Flight_modes,     4,  50}, // update the flight mode
        {Read_battery, 400, 50},   // battery reading
        {Save_data, 20, 100},  // saving data into SD
};


void setup()
{

    ins.init(AP_InertialSensor::COLD_START,AP_InertialSensor::RATE_400HZ);
    serial_manager.init_console();
    serial_manager.init();
    compass.init();
    gps.init(NULL,serial_manager);
    hal.uartC->begin(57600);
    barometer.init();
    barometer.calibrate();
    scheduler.init(&scheduler_task[0],sizeof(scheduler_task)/sizeof(scheduler_task[0]));
    toshiba_led.init(); battery.set_monitoring(0,AP_BattMonitor::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT);
    battery.init();
    init_flash();



    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);
    hal.rcout->set_freq( 15, 490);

    ch = hal.analogin->channel(0);
    ch->set_pin(15);
    hal.rcout->write(0,0);
    hal.rcout->write(1,0);
    hal.rcout->write(2,0);
    hal.rcout->write(3,0);

    hal.scheduler->delay(1000);
}

void loop(void)
{
    ins.wait_for_sample(); 
    uint32_t timer =  micros();
    fast_loop();
    scheduler.tick(); 
    uint32_t time_available=(timer+MAIN_LOOP_MICROS)-micros();
    scheduler.run(time_available); 

}

static void fast_loop(void){
    ahrs.update();
    compass.read();
    gyro  = ins.get_gyro();

    c_roll  = phi_p  * ahrs.roll  + phi_d  * gyro.x;
    c_pitch = th_p * ahrs.pitch + th_d * gyro.y;
    c_yaw   = psi_p  * (ahrs.yaw-0)   + psi_d   * gyro.z;

    // writing to motors
    float m1_c, m2_c, m3_c, m4_c;
    float c_gas=radio_throttle+ctrl.z; 




    m1_c=satu((-c_roll –ctrl.x  +c_pitch –ctrl.y  +c_yaw+radio_yaw  +cgas),1700,1100);
    m2_c=satu(( c_roll +ctrl.x  -c_pitch +ctrl.y  +c_yaw+radio_yaw  +cgas),1700,1100);
    m3_c=satu(( c_roll +ctrl.x  +c_pitch –ctrl.y  -c_yaw-radio_yaw  +cgas),1700,1100);
    m4_c=satu((-c_roll –ctrl.x  -c_pitch +ctrl.y  -c_yaw-radio_yaw  +cgas),1700,1100);

// emergency stop

    if (radio_throttle>1149)
    {
        hal.rcout->write(0,m1_c);
        hal.rcout->write(1,m2_c);
        hal.rcout->write(2,m3_c);
        hal.rcout->write(3,m4_c);
    }

    else
    {
        hal.rcout->write(0,1000);
        hal.rcout->write(1,1000);
        hal.rcout->write(2,1000);
        hal.rcout->write(3,1000);
    }

}

//          auxiliar functions


// saturation function
static float satu(float nu, float ma, float mi){
    if(nu>=ma) nu=ma;
      else nu=nu;
      if(nu <= mi) nu=mi;
      else nu=nu;
    return nu;
}

static uint32_t micros(){
    return hal.scheduler->micros();
}

static void Read_battery(){
    battery.read();
    volt=battery.voltage();
    corriente_tot=battery.current_total_mah(); 
}

static void Save_data(){
    Log_Write_Pose();
    Log_Write_Control();
    Log_Write_Errors();
}



AP_HAL_MAIN();