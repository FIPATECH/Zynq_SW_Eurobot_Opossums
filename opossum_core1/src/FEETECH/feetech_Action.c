#include "../main.h" 

//// ACTION 2026 ////
Pince_t robot_pinces[NBR_PINCES];


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
                if (Timer_ms1 - pince->pump_right.cmd_timer >= 250){ // wait a bit for current to stabilize
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
            break;

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
                    pince->action_step = 500; // on a un problème, on coupe les pompes et on abandonne
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
                
                // On vérifie si chaque pompe a bien fait le vide
                printf("Pump currents: Right=%d, Left=%d\n", pince->pump_right.pump_current, pince->pump_left.pump_current);
                uint8_t right_ok = (pince->pump_right.pump_current < 1650);
                uint8_t left_ok = (pince->pump_left.pump_current < 1650);

                if(right_ok && left_ok){ 
                    // Cas idéal : les 2 objets sont attrapés rapidement
                    printf("Deux objets ramassés\n");
                    pince->action_step = 18; 
                } else {
                    if (Timer_ms1 - pince->action_timer >= 2000){ 
                        // Le délai de 2 secondes est écoulé, on prend une décision finale
                        if (right_ok || left_ok) {
                            printf("Timeout: 1 seul objet ramassé (G:%d, D:%d)\n", left_ok, right_ok);
                            
                            // On coupe uniquement la pompe qui tourne dans le vide
                            if (!right_ok) PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                            if (!left_ok)  PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                            
                            pince->action_step = 18; // On remonte quand même le bras avec 1 seul palet
                        } else {
                            printf("Timeout: Aucun objet ramassé\n");
                            
                            // Aucun objet, on coupe les deux pompes et on abandonne
                            PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                            PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                            
                            pince->action_step = 500; // Abandon
                        }
                    } else {
                        // Pas encore de timeout et les 2 ne sont pas OK, on relance une lecture
                        pince->action_step = 16; 
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
                    pince->action_step = 500; // on a un problème, on coupe les pompes et on abandonne
                } else {
                    pince->action_step = 19; // keep waiting
                }
            }
            break;

        
        /* ---------------------------------------------------- */
        /* ------------- MONTER_PINCE --------------------------*/
        /* ---------------------------------------------------- */

        case 100:
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 101;
            break;
        case 101:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 102;
            }
            break;
        case 102:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.idle_position + 100)){
                    printf("Pince action done at position %d\n", pince->gros_pos.current_position);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout at position %d\n", pince->gros_pos.current_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 101; // keep waiting
                }
            }
            break;

        
        /* ---------------------------------------------------- */
        /* ------------- RETOURNER PALETS ----------------------*/
        /* ---------------------------------------------------- */

        case 200:
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.lacher_pos, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 201;
            break;
        case 201:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 202;
            }
            break;
        case 202:
            if(pince->action_done){
                if((pince->gros_pos.lacher_pos - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.lacher_pos + 100)){
                    printf("Pince action done at position %d\n", pince->gros_pos.current_position);
                    pince->action_done = 0;
                    pince->action_step++;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout at position %d\n", pince->gros_pos.current_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 201; // keep waiting
                }
            }
            break;
        case 203: // ouvrir les clapets des palets à lacher
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.sortie_pos, &pince->action_done);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.sortie_pos, &pince->action_done);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 204;
            break;
        case 204: // attendre que les clapets soient ouverts
            if(pince->action_done){
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_PRESENT_POSITION_L, &pince->petit_gauche_pos.current_position, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_PRESENT_POSITION_L, &pince->petit_droite_pos.current_position, &pince->action_done);
                }
                pince->action_step = 205;
                pince->action_done = 0;
            }
            break;
        case 205:
            if(pince->action_done){
                uint8_t gauche_ok = 1;
                uint8_t droite_ok = 1;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(!( (pince->petit_gauche_pos.sortie_pos - 100) <= pince->petit_gauche_pos.current_position && pince->petit_gauche_pos.current_position <= (pince->petit_gauche_pos.sortie_pos + 100) )){
                        gauche_ok = 0;
                    }
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(!( (pince->petit_droite_pos.sortie_pos - 100) <= pince->petit_droite_pos.current_position && pince->petit_droite_pos.current_position <= (pince->petit_droite_pos.sortie_pos + 100) )){
                        droite_ok = 0;
                    }
                }

                if(gauche_ok && droite_ok){
                    printf("Pince action done at position gauche: %d droite: %d\n", pince->petit_gauche_pos.current_position, pince->petit_droite_pos.current_position);
                    pince->action_done = 0;
                    pince->action_step++;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout at position gauche: %d droite: %d\n", pince->petit_gauche_pos.current_position, pince->petit_droite_pos.current_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 204; // keep waiting
                }
            }
            break;
        case 206: // allumer les electrovannes pour lacher les objets
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 207;
            break;
        case 207: // attendre un peu avant de ranger les clapets
            if(Timer_ms1 - pince->action_timer >= 1000){
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.retrait_pos, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.retrait_pos, &pince->action_done);
                }
                pince->action_step = 208;
                pince->action_timer = Timer_ms1;
            }
            break;
        case 208: // attendre que les clapets soient rentrés
            if(pince->action_done){
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_PRESENT_POSITION_L, &pince->petit_gauche_pos.current_position, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_PRESENT_POSITION_L, &pince->petit_droite_pos.current_position, &pince->action_done);
                }
                pince->action_step = 209;
                pince->action_done = 0;
            }
            break;
        case 209:
            if(pince->action_done){ 
                uint8_t gauche_ok = 1;
                uint8_t droite_ok = 1;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(!( (pince->petit_gauche_pos.retrait_pos - 100) <= pince->petit_gauche_pos.current_position && pince->petit_gauche_pos.current_position <= (pince->petit_gauche_pos.retrait_pos + 100) )){
                        gauche_ok = 0;
                    }
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(!( (pince->petit_droite_pos.retrait_pos - 100) <= pince->petit_droite_pos.current_position && pince->petit_droite_pos.current_position <= (pince->petit_droite_pos.retrait_pos + 100) )){
                        droite_ok = 0;
                    }
                }

                if(gauche_ok && droite_ok){
                    printf("Pince action done at position gauche: %d droite: %d\n", pince->petit_gauche_pos.current_position, pince->petit_droite_pos.current_position);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout at position gauche: %d droite: %d\n", pince->petit_gauche_pos.current_position, pince->petit_droite_pos.current_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 208; // keep waiting
                }
            }
            break;
        
        /* ---------------------------------------------------- */
        /* ------------- RETOURNER PALETS ----------------------*/
        /* ---------------------------------------------------- */
        case 300:
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.ramasser_pos, &pince->action_done); // même position pour lacher sans retourner que pour ramasser
            pince->action_timer = Timer_ms1;
            pince->action_step = 301;
            break;
        case 301:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 302;
            }
            break;
        case 302:
            if(pince->action_done){
                if((pince->gros_pos.ramasser_pos - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.ramasser_pos + 100)){
                    printf("Pince action done at position %d\n", pince->gros_pos.current_position);
                    pince->action_done = 0;
                    pince->action_step = 303;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout at position %d\n", pince->gros_pos.current_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 301; // keep waiting
                }
            }
            break;
        case 303: // on allume les valves et éteind les pompes des objets à déposer
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
                PutFEETECH_Ext_Done(pince->id_pump, PUMP_CMD_1, PUMP_OFF, &pince->action_done);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
                PutFEETECH_Ext_Done(pince->id_pump, PUMP_CMD_2, PUMP_OFF, &pince->action_done);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 304;
            break;
        case 304: // attendre que les commandes soient prises en compte
            if(pince->action_done){
                pince->action_done = 0;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_left.pump_current, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_right.pump_current, &pince->action_done);
                }
                pince->action_step = 305;
            }
            break;
        case 305:
            if(pince->action_done){
                uint8_t pump_ok = 1;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_left.pump_current > 100){ // if current is below threshold, we assume the pump is off
                        printf("Valve gauche not on or pump not off\n");
                        pump_ok = 0;
                    }
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_right.pump_current > 100){ // if current is below threshold, we assume the pump is off
                        printf("Valve droite not on or pump not off\n");
                        pump_ok = 0;
                    }
                }

                if(pump_ok){
                    printf("Pince action done, object released\n");
                    pince->action_done = 0;
                    pince->action_step = 306; // put pince in idle pos
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout, valve not on or pump not off\n");
                    pince->action_step = 0;
                } else {
                    pince->action_step = 304; // keep waiting
                }
            }
            break;
        case 306:
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 307;
            break;
        case 307:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
                pince->action_step = 308;
            }
            break;
        case 308:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    printf("Pince action done at position %d\n", pince->action_position);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout at position %d\n", pince->action_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 307; // keep waiting
                }
            }
            break;

        /* ---------------------------------------------------- */
        /* ------------- SEQUENCE D'ABANDON / ERREUR -----------*/
        /* ---------------------------------------------------- */
        case 500:
            printf("Pince Abort ! Extinction pompes et retour IDLE\n");
            // 1. Eteindre les pompes
            PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            // 2. ranger les clapets pour éviter de les casser en cas de collision au retour
            PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.retrait_pos, &pince->action_done);
            PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.retrait_pos, &pince->action_done);
            // 3. Ordonner la remontée de sécurité
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 501;
            break;
            
        case 501:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
                pince->action_step = 502;
            }
            break;
            
        case 502:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    printf("Pince action annulée et retour au repos ok\n");
                    pince->action_done = 0;
                    pince->action_step = 0; // Prêt pour une nouvelle commande
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Timeout critique : impossible de remonter la pince.\n");
                    pince->action_step = 0; // On force à 0 pour ne pas geler tout le robot
                } else {
                    pince->action_step = 501; // Continue de vérifier la position
                }
            }
            break;

        default:
            break;
        
    }
}

uint8_t pince_action_cmd(void){
    uint32_t ID_pince;
    if (Get_Param_u32(&ID_pince))
        return PARAM_ERROR_CODE;

    uint32_t command;
    if (Get_Param_u32(&command))
        return PARAM_ERROR_CODE;
    
    uint32_t param;
    if (Get_Param_u32(&param))
        return PARAM_ERROR_CODE;

    if(ID_pince < 0 || ID_pince > NBR_PINCES-1){
        printf("Invalid pince ID\n");
        return PARAM_ERROR_CODE;
    }

    Pince_t *pince = &robot_pinces[ID_pince];
    if(pince->action_step != 0){
        printf("Pince is busy\n");
        return PARAM_ERROR_CODE;
    }
    switch (command){
        case 1:
            pince->current_command = CMD_RAMASSER;
            pince->action_step = 10;
            break;
        case 2:
            pince->current_command = CMD_LACHER_G;
            pince->action_step = 200;
            break;
        case 3:
            pince->current_command = CMD_LACHER_D;
            pince->action_step = 200;
            break;
        case 4:
            pince->current_command = CMD_LACHER_ALL;
            pince->action_step = 200;
            break;
        case 5:
            pince->current_command = CMD_MONTER;
            pince->action_step = 100;
            break;
    }
    return 0;
}



uint8_t init_pince_state = 0;
uint32_t init_pince_timer = 0;
uint8_t pinces_initialized = 0;

void AU_pinces(void){
    pinces_initialized = 0;
}

void Init_Pinces_Loop(void){

    switch(init_pince_state){
        case 0:
            if (pinces_initialized == 0){
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
            pinces_initialized = 1;
            init_pince_state = 0;
            break;
        default:
            break;

    }
}

