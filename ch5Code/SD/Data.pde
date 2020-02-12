// register definition, see arducopter.pde code
#define LOG_POSE_MSG 0x01
#define LOG_ERR_MSG 0x0C
#define LOG_CONTROL_MSG 0x05


// DATA PACKAGE DECLARATION
// Number of packages 3, Pose, Control and Error


static uint16_t log_num;   //Dataflash


struct PACKED log_Pose{
    LOG_PACKET_HEADER;
    float    alt_barof;
    float    Roll;
    float    Pitch;
    float    Yaw;
    float    z_pos;
    float    vel_x;
    float    vel_y;
    float    vel_z;
    float    x_pos;
    float    y_pos;
    float    giroz;
    float    girox;
    float    giroy;
};



struct PACKED log_Control {
    LOG_PACKET_HEADER;
    float  time_ms;
    float  u_z;
    float  tau_theta;
    float  tau_phi;
    float  tau_psi;
    float  comodin_1;
    float  comodin_2;
    float  comodin_3; // data wildcards useful for whatever
    float  comodin_4; // you want to add
};



struct PACKED log_Errors {
    LOG_PACKET_HEADER;
    uint32_t   time_ms;
    float   error_x;
    float   error_y;
    float   error_z;
    float   voltaje;
    float   corriente;
    float   comodin_5;
    float   comodin_6;
    int   comodin_7;
    float   alt_des; 
    float   x_des;
    float   y_des;
};







//         HEADER DECLARATION

static const struct LogStructure log_structure[] PROGMEM = {
         LOG_COMMON_STRUCTURES,
       {LOG_POSE_MSG, sizeof(log_Pose),
        "1", "fffffffffffff", "a_bar,ROLL,PITCH,YAW,Z_POS,V_X,V_Y,V_Z,X_POS,Y_POS,G_Z,G_X,G_Y"},
       { LOG_CONTROL_MSG, sizeof(log_Control),
        "2", "fffffffff", "T_MS,UZ,T_TH,T_PHI,T_PSI,TAUX,TAUY,S_PHI,S_PSI"},
       { LOG_ERR_MSG, sizeof(log_Errors),
        "3", "IfffffffIfff", "T_MS,E_X,E_Y,E_Z,VOLT,AMP,nav_z,nav_zp,con_alt,ZDES,XDES,YDES"},
};



//         INICIALIZACION

static void init_flash() {
    DataFlash.Init(log_structure,sizeof(log_structure)/sizeof(log_structure[0]));
    if (DataFlash.NeedErase()) {
        DataFlash.EraseAll();
    }
    log_num=DataFlash.StartNewLog();
}

//    SAVING SEQUENCE DATA, BY PACKAGE, Pose, Control, Errors
// DATA TO THE RIGHT is assumed previously defined in the main cycle or auxiliary
// functions, if not just change to 0 and add your own values for testing

static void Log_Write_Pose()
{
    struct log_Pose pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POSE_MSG),
        alt_barof    : baro_alt, 
        Roll         : ahrs.roll,
        Pitch        : ahrs.pitch,
        Yaw          : ahrs.yaw,
        z_pos        : pos.z,
        vel_x        : vel.x,
        vel_y        : vel.y,
        vel_z        : vel.z,
        x_pos        : pos.x,
        y_pos        : pos.y,
        giroz        : gyro.z,
        girox        : gyro.x,
        giroy        : gyro.y,
    };	
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}


static void Log_Write_Control(){
    struct log_Control pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_MSG),
        time_ms     : (float)(hal.scheduler->millis()/1000),
        u_z         : ctrl.z,
        tau_theta   : (ctrl.x+c_pitch),
        tau_phi     : (ctrl.y+c_roll),
        tau_psi     : c_yaw,
        comodin_1       : 0,
        comodin_2       : 0,
        comodin_3    : 0,
        comodin_4    : 0,
    };	
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}


static void Log_Write_Errors(){
    struct log_Errors pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERR_MSG),
        time_ms        : (hal.scheduler->millis()/100),
        error_x        : error.x, 
        error_y        : error.y,
        error_z        : error.z,
        voltaje        : volt,
        corriente      : corriente_tot,
        comodin_5         : 0,
        comodin_6        : 0,
        comodin_7        : radio_throttle,
        alt_des        :  ref.z,
        x_des          :  ref.x,
        y_des          :  ref.y,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
