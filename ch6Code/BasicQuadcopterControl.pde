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
#define  LOG_MSG          0x01

static AP_GPS  gps;
static AP_Baro barometer;
static AP_InertialSensor ins;
static AP_SerialManager serial_manager;
static Compass compass;
//Data
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static DataFlash_File DataFlash("/fs/microsd/APM/LOGS");
#endif

struct PACKED log_Datos{
    LOG_PACKET_HEADER;
    uint32_t  time_ms;
    float  a_roll;
    float  a_pitch;
    float  a_yaw;
    float  pos_x;
    float  pos_y;
    float  pos_z;
};

static const struct LogStructure log_structure[] PROGMEM = {
         LOG_COMMON_STRUCTURES,
         {LOG_MSG, sizeof(log_Datos),
        "1", "Iffffff", "T_MS,ROLL,PITCH,YAW,X_POS,Y_POS,Z_POS"},
};

static uint16_t log_num;   //Dataflash
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
float p_x,d_x,p_y,d_y,p_z,d_z;

float baro_alt=0, alt_fil, alt_fil_a=0,velocity, vel_fil,control_altura, z_des;
uint32_t time_dt;
uint32_t time, timemod;
float gyrox, gyroy, gyroz, velx, vely, velz;
Vector3f gyro;
float kp_roll, kd_roll, kp_pitch, kd_pitch, kp_yaw, kd_yaw;
float c_roll, c_pitch, c_yaw;
//Flags
bool flag_z=true,entre_home=true;
uint8_t flag_gps=0;
static Vector3f pos_gps;
static Vector3f vel_gps;
float x,y,xrate,yrate, c_x,c_y;

uint8_t _bufferrx[4];

AP_HAL::AnalogSource* ch;
static int8_t pin;
float m1,m2,m3,m4;
uint16_t radio[6];
uint16_t ii,refalt=0,refi=0;
int conta=0;
float recep;
AP_HAL::DigitalSource *a_led;
AP_HAL::DigitalSource *b_led;
AP_HAL::DigitalSource *c_led;
float err_yaw;
float m1_c,m2_c,m3_c,m4_c;

void setup()
{

    ins.init(AP_InertialSensor::COLD_START,AP_InertialSensor::RATE_400HZ);
    ahrs.set_compass(&compass);
    serial_manager.init_console();
    serial_manager.init();
    compass.init();
    gps.init(NULL,serial_manager);

    hal.uartC->begin(57600);

    barometer.init();
    barometer.calibrate();
    DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
       if (DataFlash.NeedErase()) {
           DataFlash.EraseAll();
          }
    log_num = DataFlash.StartNewLog();
     //0xFF  0x0F->b'00001111'
    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);
    hal.rcout->set_freq( 15, 490);

    ch = hal.analogin->channel(0);
    ch->set_pin(15);
    hal.rcout->write(0,900);
    hal.rcout->write(1,900);
    hal.rcout->write(2,900);
    hal.rcout->write(3,900);

    hal.scheduler->delay(200);
}

void loop(void)
{
// the next function obtains position and velocity values posx, posy, posz
// velx, vely, velz

    update_GPS();

// the required radio channels are read for semi-automatic operation
    radio[4] = hal.rcin->read(4);
    aux_1 = radio[4];

// this channel replaces the fact to add the term mg
    radio[2] = hal.rcin->read(2);
    radio_throttle = radio[2];
    refalt=radio_throttle;

    ahrs.update();
    barometer.update();
// angular values of orientation and speed
    roll  = ahrs.roll;
    pitch = ahrs.pitch;
    yaw   = ahrs.yaw;

    gyro  = ins.get_gyro();

    gyrox = gyro.x;
    gyroy = gyro.y;
    gyroz = gyro.z;
// gains of the controllers CAUTION, the reader is responsible for his own
// and correct tuning


    kp_roll=800;
    kd_roll=350;
    kp_pitch=800;
    kd_pitch=350;
    kp_yaw=85/2;
    kd_yaw=100/2;

// in this case, since the speed measurements were very noisy, the 
// differential values are greater than the proportional ones, that's the 
// responsibility of the user and the employed vehicles

    d_x=80;
    p_x=35;
    d_y=80;
    p_y=35;
    d_z=80;
    p_z=35;

// PD controllers

    err_yaw=yaw-0;
    c_roll  = kp_roll  * roll  + kd_roll  * gyrox;
    c_pitch = kp_pitch * pitch + kd_pitch * gyroy;
    c_yaw   = kp_yaw   * err_yaw   + kd_yaw   * gyroz;

// this reference holds the altitude while varying with the remote control
// or semiautomatic hover mode
    refx=0;
    refy=0;
    refz=posz;
    ref_px=0;
    ref_py=0;
    ref_pz=0;

    errorx=posx-refx;
    errory=posy-refy;
    errorz=posz-refz;

    error_px=velx-ref_px; 
    error_py=vely-ref_py; 
    error_pz=velz-ref_pz;

    cx=satu((p_x*(errorx)+d_x*(error_px)),50,-50);
    cy=satu((p_y*(errory)+d_y*(error_py)),50,-50);
    cz=satu((p_z*(errorz)+d_z*(error_pz)),80,-80);

// z control plus manual mg
    c_gas=refalt+cz;
    
     // BEWARE the signs of roll, X, pitch and Y, can vary according to the
     // sense of the remote control (the lever can be reversed)

// saturated propulsion matrix so that the engines never turn off
// and at the same time do not reach the maximum value of operation, this also
// is the responsibility of the reader, see also the section of the propulsion matrix

    m1_c=satu((-c_roll -cx  +c_pitch -cy  +c_yaw +cgas),1700,1100);
    m2_c=satu(( c_roll +cx  -c_pitch +cy  +c_yaw +cgas),1700,1100);
    m3_c=satu(( c_roll +cx  +c_pitch -cy  -c_yaw +cgas),1700,1100);
    m4_c=satu((-c_roll -cx  -c_pitch +cy  -c_yaw +cgas),1700,1100);

// writing to the motors if the auxiliary lever that serves as emergency stop
// is activated, otherwise, stop the motors


    if (aux_1<1500)
    {
        hal.rcout->write(0,uint16_t(m1_c));
        hal.rcout->write(1,uint16_t(m2_c));
        hal.rcout->write(2,uint16_t(m3_c));
        hal.rcout->write(3,uint16_t(m4_c));
    }

    else
    {
        hal.rcout->write(0,900);
        hal.rcout->write(1,900);
        hal.rcout->write(2,900);
        hal.rcout->write(3,900);
    }

    hal.scheduler->delay(50);


}

// saturation function

static float satu(float nu, float ma, float mi){
    if(nu>=ma) nu=ma;
      else nu=nu;
      if(nu <= mi) nu=mi;
      else nu=nu;
    return nu;
}


// update function of x, y and z via gps

static void update_GPS(void){
      static uint32_t last_msg_ms;
      gps.update();
       if (last_msg_ms != gps.last_message_time_ms())
       {
           last_msg_ms = gps.last_message_time_ms();
           const Location &loc =gps.location();
           flag = gps.status();
       }


      uint32_t currtime = hal.scheduler->millis();
      dt = (float)(currtime - last_update) / 1000.0f;
      last_update = currtime;
// a delta t is required to internally calculate velocity estimations
      inertial_nav.update(dt);     
// this part verifies that there are at least 3 satellites to operate and turn on the // led if true, also changes a variable called flag2 to update speeds
      flag= gps.num_sats();

      if(pos.x!=0 && flag >=3 && flag2==1){
           const Location &loc = gps.location();
           ahrs.set_home(loc);
           compass.set_initial_location(loc.lat, loc.lng);
           toshiba_led.set_rgb(0,LED_DIM,0);   // green 
           flag2 = 2;
        }

    pos_gps  = inertial_nav.get_position();
    vel_gps = inertial_nav.get_velocity();
// a gps of centimetric resolution is assumed
// and its value it is transformed to meters
    posx=((pos_gps.x)/100);
    posy=((pos_gps.y)/100);
    posz=((pos_gps.z)/100);
    if(flag2==2){
         velx=((vel_gps.x)/100);
         vely=((vel_gps.y)/100);
    }
    velz=((vel_gps.z)/100);
    flag2==1;

}



AP_HAL_MAIN();