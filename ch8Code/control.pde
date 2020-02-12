static void Flight_modes(){

if(aux_1<1600){ 
    if(flag_aux1){
        ref.x=pos.x;
        off.x=pos.x;  
        ref.y=pos.y;
        off.y=pos.y;
        ref.z=pos.z;
        mode_flight =2;
    }

    if(aux_2>=1600) {
        mode_flight =3;
    } else { mode_flight =2;}

}

else{
    mode_flight =1;
    flag_aux1=false;
}




switch(mode_flight) {
case 1: // just attitude
    ctrl.x=0;
    ctrl.y=0;
    ctrl.z=0; 
    ref_p.x=0;
    ref_p.y=0;
    ref_p.z=0;
    break;
case 2: // hover
    error.x=pos.x-ref.x; 
    error.y=pos.y-ref.y; // check remote control levers to coincide with the signs
    error.z=ref.z-pos.z;
    error_p.x=vel.x-ref_p.x; 
    error_p.y=vel.y-ref_p.y; 
    error_p.z=ref_p.z-vel.z;
    ctrl.x=satu((p_x*(error.x)+d_x*(error_p.x)),50,-50);
    ctrl.y=satu((p_y*(error.y)+d_y*(error_p.y)),50,-50);
    ctrl.z=satu((alt_p*(error.z)+alt_d*(error_p.z)),80,-80);
    break;
case 3: // trajectory following
    error.x=pos.x-ref.x; 
    error.y=pos.y-ref.y;
    error.z=ref.z-pos.z;
    error_p.x=vel.x-ref_p.x; 
    error_p.y=vel.y-ref_p.y; 
    error_p.z=ref_p.z-vel.z;
    ctrl.x=satu((p_x*(error.x)+d_x*(error_p.x)),50,-50);
    ctrl.y=satu((p_y*(error.y)+d_y*(error_p.y)),50,-50);
    ctrl.z=satu((alt_p*(error.z)+alt_d*(error_p.z)),80,-80);
    break;
default:
    break;
}
} 
