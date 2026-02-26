#include "../main.h" 

uint8_t Send_FEETECH_Cmd(void){
    uint32_t val32;
    uint8_t Id;
    uint8_t Reg;
    uint16_t Consigne;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Id = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Reg = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Consigne = val32;

    PutFEETECH(Id, Reg, Consigne);
    return 0;
}

uint8_t Get_FEETECH_Cmd(void){
    uint32_t val32;
    uint8_t Id;
    uint8_t Reg;

    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Id = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Reg = val32;
    xil_printf("Get FEETECH Id=%d Reg=%d\r\n", Id, Reg);
    xil_printf("  Value=%d\r\n", GetFEETECH_Wait(Id, Reg));
    return 0;
}

uint8_t Send_FEETECH_SCS_Cmd(void){
    uint32_t val32;
    uint8_t Id;
    uint8_t Reg;
    uint16_t Consigne;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Id = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Reg = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Consigne = val32;

    PutFEETECH_SCS(Id, Reg, Consigne);
    return 0;
}

uint8_t Get_FEETECH_SCS_Cmd(void){
    uint32_t val32;
    uint8_t Id;
    uint8_t Reg;

    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Id = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Reg = val32;
    xil_printf("Get FEETECH Id=%d Reg=%d\r\n", Id, Reg);
    xil_printf("  Value=%d\r\n", GetFEETECH_Wait_SCS(Id, Reg));
    return 0;
}

//-------------------------------------------------------------------------------
// Fonctions Search ID return all id detected
//-------------------------------------------------------------------------------

uint8_t start_ID=0;
uint8_t etat_ID=0;
uint8_t done_ID=0;
uint16_t ID_return=0;
uint16_t ID_test=0;
uint8_t ID_Search_Status=0; 

void FEETECH_Search_ID_Loop(void)
{
    static uint32_t t0 = 0;

    switch (etat_ID)
    {
        case 0:
            if (start_ID == 1) {
                start_ID = 0;
                ID_test = 0;
                etat_ID = 1;
            }
            break;

        case 1:
            if(FEETECH_All_Cmd_Done()){
                if (ID_test <= 253) {
                    done_ID = 0;
                    ID_Search_Status = 0;

                    GetFEETECH_Ext_Done_With_Status(ID_test, FEETECH_PRESENT_POSITION_L, &ID_return, &done_ID, &ID_Search_Status);
                    printf("Testing ID: %d\n", ID_test);
                    etat_ID = 2;
                } else {
                    printf("ID Search Complete\n");
                    etat_ID = 0;
                }
            }
            break;
        case 2:
            // wait for reply
            if (done_ID) {
                if(ID_Search_Status == FEETECH_STATUS_OK){
                    printf("  --> Found FEETECH ID: %d\n", ID_test);
                }
                ID_test++;
                etat_ID = 1;
            }
            break;
    }
}


uint8_t Test_ID_FEETECH_Cmd(void){
    //launch search id
    start_ID=1;
    return 0;
}



uint8_t feetech_action_state=0;
uint32_t feetech_action_timer=0;
uint8_t feetech_action_done=0;
uint32_t feetech_action_position=0;
uint32_t feetech_test_ctnr=0;

void FEETECH_action_loop(void){
    if(Timer_ms1 - feetech_action_timer >= 1000){
        feetech_action_timer = Timer_ms1;
        feetech_test_ctnr++;
        printf("FEETECH action loop %d\r\n", feetech_test_ctnr);
        switch(feetech_action_state){
            case 0:
                PutFEETECH(10, PUMP_CMD_2, 255);
                feetech_action_state = 1;
                break;
            case 1:
                PutFEETECH(10, PUMP_CMD_2, 0);
                feetech_action_state = 0;
                break;
        }
    }
}

uint8_t pince_action_step = 0;
uint8_t pince_action_done = 0;
int pince_action_timer = 0;

uint32_t pince_action_position = 0;

void pince_loop(void){
    for (int i = 0; i < NBR_PINCES; i++) {
        pince_action_loop(&robot_pinces[i]);
    }
}

void pince_action_loop(Pince_t *pince){
    switch(pince->action_step){
        case 0:
            break;

        /* ---------------------------------------------------- */
        /* ------------- RAMASSER_OBJETS -----------------------*/
        /* ---------------------------------------------------- */

        case 10: // baisser la pince
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.ramasser_pos, &pince->action_done);
            pince->gros_pos.cmd_timer = Timer_ms1;
            pince->action_step++;
            break;

        case 11: // allumer les pompes
            if(pince->action_done){
                pince->action_done = 0; // reset done flag
                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_ON);
                PutFEETECH_Ext_Done_SCS(pince->id_pump, PUMP_CMD_2, PUMP_ON, &pince->action_done);
                pince->pump_right.cmd_timer = Timer_ms1;
                pince->pump_left.cmd_timer = Timer_ms1;
                pince->action_step++;
            }
            break;

        case 12: // check pump are on
            if(pince->action_done){
                if (Timer_ms1 - pince->pump_right.cmd_timer >= 50){ // wait a bit for current to stabilize
                    pince->action_done = 0; // reset done flag
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current, &pince->action_done);
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
                    pince->action_step++;
                }
            }
            break;

        case 13: 
            if(pince->action_done){
                if(pince->pump_right.pump_current > 100 && pince->pump_left.pump_current > 100){ // if current is above threshold, we assume the pump are on
                    printf("Pump on\n");
                    pince->action_done = 0;
                    pince->action_step++; // keep checking current
                } else {
                    printf("Pump not ok\n");
                    pince->action_step = 11; // pump not on, try again
                }
            }

        case 14: // wait for pince to reach position
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
            pince->action_step++;
            break;    

        case 15:
            if(pince->action_done){
                if((pince->gros_pos.ramasser_pos - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.ramasser_pos + 100)){
                    printf("Pince action done at position %d\n", pince->gros_pos.current_position);
                    pince->action_timer = Timer_ms1; // timer pour laisser un peu de temps pour aspirer une fois qu'on est en position
                    pince->action_done = 0;
                    pince->action_step++;
                } else if (Timer_ms1 - pince->gros_pos.cmd_timer >= 3000){
                    printf("Pince action timeout at position %d\n", pince->gros_pos.current_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 14; // keep waiting
                }
            }
            break;
 
        case 16: // verifier si on aspire un objet
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current, &pince->action_done);
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
            pince->action_step++;
            break;

        case 17:
            if(pince->action_done){
                pince->action_done = 0;
                if(pince->pump_right.pump_current > 150 && pince->pump_left.pump_current > 150){ // if current is above threshold, we assume we have an object
                    printf("Object ramassé\n");
                    pince->action_step++; 
                } else {
                    printf("No object detected\n");
                    if (Timer_ms1 - pince->action_timer >= 2000){ // if after 2s we don't detect an object, we assume there's no object and we stop trying
                        printf("Pince action timeout, no object detected\n");
                        pince->action_step = 0;
                    } else {
                        pince->action_step = 16; // keep checking for object
                    }
                }
            }
            break;
        
        case 18: //remonter la pince
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 19;
            break;

        case 19:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
                pince->action_step = 20;
            }
            break;

        case 20:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    printf("Pince action done at position %d\n", pince->action_position);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout at position %d\n", pince->action_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 19; // keep waiting
                }
            }
            break;






        
        // case 20: // monter la pince
        //     PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.retrait_pos, &pince->action_done);
        //     pince->action_timer = Timer_ms1;
        //     pince->action_step = 21;
        //     break;
        // case 21:
        //     if(pince->action_done){
        //         pince->action_done = 0;
        //         GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
        //         pince->action_step = 22;
        //     }
        //     break;
        // case 22:
        //     if(pince->action_done){
        //         if((pince->gros_pos.retrait_pos - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.retrait_pos + 100)){
        //             printf("Pince action done at position %d\n", pince->action_position);
        //             pince->action_done = 0;
        //             pince->action_step = 0;
        //         } else if (Timer_ms1 - pince->action_timer >= 3000){
        //             printf("Pince action timeout at position %d\n", pince->action_position);
        //             pince->action_step = 0;
        //         } else {
        //             pince->action_step = 21; // keep waiting
        //         }
        //     }
        //     break;

        // case 30: //alumer les pompes 
        //     PutFEETECH(pince->id_pump, PUMP_CMD_1, 255);
        //     PutFEETECH(pince->id_pump, PUMP_CMD_2, 255);
        //     pince->action_timer = Timer_ms1;
        //     pince->action_step = 0;
        //     break;
            
        // case 35: // eteindre les pompes
        //     PutFEETECH(pince->id_pump, PUMP_CMD_1, 0);
        //     PutFEETECH(pince->id_pump, PUMP_CMD_2, 0);
        //     pince->action_timer = Timer_ms1;
        //     pince->action_step = 0;
        //     break;

        // case 40: //activate valves
        //     PutFEETECH(pince->id_pump, VALVE_CMD_1, 1);
        //     PutFEETECH(pince->id_pump, VALVE_CMD_2, 1);
        //     pince->action_step = 0;
        //     break;

        // case 50: //sortir clapet
        //     PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.sortie_pos, &pince->action_done);
        //     PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.sortie_pos, &pince->action_done);
        //     pince->action_step = 51;
        //     break;
        // case 51:
        //     if(pince->action_done){
        //         printf("Clapet sortie done\n");
        //         pince->action_done = 0;
        //         pince->action_step = 0;
        //     }
        //     break;

        // case 60: //ranger clapet
        //     PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.retrait_pos, &pince->action_done);
        //     PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.retrait_pos, &pince->action_done);
        //     pince->action_step = 61;
        //     break;
        // case 61:
        //     if(pince->action_done){
        //         printf("Clapet rentré done\n");
        //         pince->action_done = 0;
        //         pince->action_step = 0;
        //     }
        //     break;
    }
}

uint8_t Monter_pince_cmd(void){
    pince_action_step = 20;
    return 0;
}

uint8_t Baisser_pince_cmd(void){
    pince_action_step = 10;
    return 0;
}

uint8_t Allumer_pompes_cmd(void){
    pince_action_step = 30;
    return 0;
}

uint8_t Eteindre_pompes_cmd(void){
    pince_action_step = 35;
    return 0;
}

uint8_t Activate_Valves_cmd(void){
    pince_action_step = 40;
    return 0;
}

uint8_t Ouvrir_clapet_cmd(void){
    pince_action_step = 50;
    return 0;
}

uint8_t start_pince = 0;
int Test_pince_action_step = 0;
int Test_pince_action_timer = 0;

void Test_pince_action_loop(void){
    switch(Test_pince_action_step){
        case 0:
            if(start_pince){
                start_pince = 0;
                pince_action_step = 10; //baisser la pince
                Test_pince_action_timer = Timer_ms1;
                Test_pince_action_step = 1;
            }
            break;
        case 1: // aspirer
            if(Timer_ms1 - Test_pince_action_timer >= 1000){
                pince_action_step = 30;
                Test_pince_action_timer = Timer_ms1;
                Test_pince_action_step = 2;
            }
            break;
        case 2: // monter la pince
            if(Timer_ms1 - Test_pince_action_timer >= 500){
                pince_action_step = 20; 
                Test_pince_action_timer = Timer_ms1;
                Test_pince_action_step = 3;
            }
            break;
        case 3: // sortir les clapets   
            if(Timer_ms1 - Test_pince_action_timer >= 1000){
                pince_action_step = 50; 
                Test_pince_action_timer = Timer_ms1;
                Test_pince_action_step = 4;
            }
            break;

        case 4: // ouvrir les valves
            if(Timer_ms1 - Test_pince_action_timer >= 1000){
                pince_action_step = 40;
                Test_pince_action_timer = Timer_ms1;
                Test_pince_action_step = 5;
            }
            break;
        case 5: // etteindre les pompes
            if(Timer_ms1 - Test_pince_action_timer >= 200){
                pince_action_step = 35;
                Test_pince_action_timer = Timer_ms1;
                Test_pince_action_step = 6;
            }
            break;
        case 6: // ranger les clapets
            if(Timer_ms1 - Test_pince_action_timer >= 500){
                pince_action_step = 60;
                Test_pince_action_timer = Timer_ms1;
                Test_pince_action_step = 0;
            }
            break;
        default:
            break;
                
    }
}


uint8_t Test_pince_cmd(void){
    start_pince = 1;
    Test_pince_action_step = 0;
    return 0;
}


//// ACTION 2026 ////
Pince_t robot_pinces[NBR_PINCES];

uint8_t init_pince_state = 0;
uint32_t init_pince_timer = 0;

void Init_Pinces_Loop(void){

    switch(init_pince_state){
        case 0:
            if (init_pince_state == 0){
                init_pince_state = 1;
            }
            break;
        case 1:
            for (uint8_t i = 0; i < NBR_PINCES; i++){
                robot_pinces[i].id_gros     = (i + 1)*10 + 1; 
                robot_pinces[i].id_droite   = (i + 1)*10 + 3;
                robot_pinces[i].id_gauche   = (i + 1)*10 + 2;
                robot_pinces[i].id_pump     = (i + 1)*10;

                robot_pinces[i].action_step = 0;
                robot_pinces[i].action_done = 0;
                robot_pinces[i].action_timer = 0;
                robot_pinces[i].action_position = 0;

                robot_pinces[i].pump_right.pump_current = 0;
                robot_pinces[i].pump_left.pump_current = 0;

                robot_pinces[i].current_command = CMD_IDLE;  
            }
            // PINCE_1
            robot_pinces[0].gros_pos.idle_position = PINCE_1_GROS_IDLE_POS;
            robot_pinces[0].gros_pos.ramasser_pos = PINCE_1_GROS_RAMASSER_POS;
            robot_pinces[0].gros_pos.lacher_pos = PINCE_1_GROS_LACHER_POS;
            robot_pinces[0].petit_droite_pos.sortie_pos = PINCE_1_DROITE_SORTIE_POS;
            robot_pinces[0].petit_droite_pos.retrait_pos = PINCE_1_DROITE_RETRAIT_POS;
            robot_pinces[0].petit_gauche_pos.sortie_pos = PINCE_1_GAUCHE_SORTIE_POS;
            robot_pinces[0].petit_gauche_pos.retrait_pos = PINCE_1_GAUCHE_RETRAIT_POS;

            // PINCE_2
            robot_pinces[1].gros_pos.idle_position = PINCE_2_GROS_IDLE_POS;
            robot_pinces[1].gros_pos.ramasser_pos = PINCE_2_GROS_RAMASSER_POS;
            robot_pinces[1].gros_pos.lacher_pos = PINCE_2_GROS_LACHER_POS;
            robot_pinces[1].petit_droite_pos.sortie_pos = PINCE_2_DROITE_SORTIE_POS;
            robot_pinces[1].petit_droite_pos.retrait_pos = PINCE_2_DROITE_RETRAIT_POS;
            robot_pinces[1].petit_gauche_pos.sortie_pos = PINCE_2_GAUCHE_SORTIE_POS;
            robot_pinces[1].petit_gauche_pos.retrait_pos = PINCE_2_GAUCHE_RETRAIT_POS;

            init_pince_state = 2;
            break;
        case 2:
            for (uint8_t i = 0; i < NBR_PINCES; i++){
                // move gros servo to IDLE position
                PutFEETECH_Ext_Done(robot_pinces[i].id_gros, FEETECH_GOAL_POSITION_L, robot_pinces[i].gros_pos.idle_position, &robot_pinces[i].action_done);
                // move petit servos to RETRAIT position
                PutFEETECH_Ext_Done_SCS(robot_pinces[i].id_droite, FEETECH_GOAL_POSITION_L, robot_pinces[i].petit_droite_pos.retrait_pos, &robot_pinces[i].action_done);
                PutFEETECH_Ext_Done_SCS(robot_pinces[i].id_gauche, FEETECH_GOAL_POSITION_L, robot_pinces[i].petit_gauche_pos.retrait_pos, &robot_pinces[i].action_done);
            }
            init_pince_timer = Timer_ms1;
            init_pince_state = 3;
            break;
        case 3:
            // wait for all servos to reach position
            if (Timer_ms1 - init_pince_timer >= 3000){
                for (uint8_t i = 0; i < NBR_PINCES; i++){
                    robot_pinces[i].action_done = 0;
                }
                init_pince_state = 4;
            }
            break;
        case 4:
            printf("Pinces initialized\n");
            init_pince_state = 0;
            break;
        default:
            break;

    }
}

