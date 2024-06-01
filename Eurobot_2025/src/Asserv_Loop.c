#include "main.h"
#include "lib_asserv/lib_asserv.h"


int Last_Timer_Asserv = 0;
int Asserv_State = 0;
int Asserv_Odo_Count = 0;
int QEId, QEIg = 0;

void Asserv_Loop(void){
    if(Asserv_State == 0){
        //-----------------------------------
        // QEI step
        //-----------------------------------
        if((Timer_ms1 - Last_Timer_Asserv) > ODO_EVERY_MS){
            Last_Timer_Asserv += ODO_EVERY_MS;
            // read_qei();
            Asserv_State ++;
        }
    }else if(Asserv_State == 1){
        //-----------------------------------
        // ODO step
        //-----------------------------------
        QEId = qei_0;
        QEIg = qei_1;

        // Compute the distance and angle
        // odo_position_step(QEId, QEIg);
        Asserv_Odo_Count++;
        if (Asserv_Odo_Count >= ASSERV_EVERY){
        	Asserv_Odo_Count = 0;
            Asserv_State ++;
        } else {
            Asserv_State = 0;
        }
    }else if(Asserv_State == 2){
        //-----------------------------------
        // ODO speed
        //-----------------------------------
        // odo_speed_step(ASSERV_EVERY*ODO_EVERY_MS);
        Asserv_State ++;
    }else if(Asserv_State == 3){
        //-----------------------------------
        // Asserv
        //-----------------------------------
        motor1_current_order = 0;
        motor2_current_order= 0;
        Asserv_State = 0;
    }
}


