# define LED_DIM 0x11 // LED variable definition
static float dt; // delta of time

static uint32_t last_update;    // inertial navigation variable

///////              GPS
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
      inertial_nav.update(dt);
   

      if(pos.x!=0 && flag >=3 && flag2==1){

           const Location &loc = gps.location();
           ahrs.set_home(loc);

           compass.set_initial_location(loc.lat, loc.lng);
           toshiba_led.set_rgb(0,LED_DIM,0);  
           flag2 = 2;

        }

    pos_gps  = inertial_nav.get_position();
    vel_gps = inertial_nav.get_velocity();

    pos.x=((pos_gps.x)/100)-off.x;
    pos.y=((pos_gps.y)/100)-off.y;
    pos.z=((pos_gps.z)/100)-off.z;

    if(flag2==2){
         vel.x=((vel_gps.x)/100);
         vel.y=((vel_gps.y)/100);
    }
    vel.z=((vel_gps.z)/100);

}

static void update_Baro() {
    barometer.update();
    baro_alt=barometer.get_altitude();
}

static void Trajectory(){
    if(mode_flight==3 && s_time<=360){
    //posic
    ref.x=(-3*cos(s_time*(3.1416/180))+3)+off.x; // starts where it is currently placed
    ref.y=(-3*sin(s_time*(3.1416/180)))+off.y; 
    ref.z=pos.z;
    //veloc

    ref_p.x=0; // remember the soft-flight mode but you can paste here the time          
               // derivative
    ref_p.y=0;
    ref_p.z=0;
    }
}
