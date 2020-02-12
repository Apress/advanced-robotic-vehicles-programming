static void Read_radio(){
    for (uint8_t i=0;i<=6; i++)
    {radio[i]=hal.rcin->read(i);}
    radio_roll=(radio[0]-1500)/3;
    radio_pitch=(radio[1]-1500)/3;
    radio_throttle=radio[2];
    if(radio_throttle>=1149 && radio_throttle<1152){
        off.z=pos.z;
    }

    radio_yaw=(radio[3]-1500)/2;
    aux_1=radio[4];
    aux_2=radio[5];
}
