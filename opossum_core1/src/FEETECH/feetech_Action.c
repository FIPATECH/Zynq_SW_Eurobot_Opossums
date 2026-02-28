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
            pince->retry_count = 0; 
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.ramasser_pos, &pince->action_done);
            pince->gros_pos.cmd_timer = Timer_ms1;
            pince->action_step++;
            break;

        case 11: // allumer les pompes
            if(pince->action_done){
                pince->action_done = 0;
                if (pince->retry_count > 0) {
                    printf("Tentative allumage pompes %d/3\n", pince->retry_count + 1);
                }
                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_ON);
                PutFEETECH_Ext_Done_SCS(pince->id_pump, PUMP_CMD_2, PUMP_ON, &pince->action_done);
                pince->pump_right.cmd_timer = Timer_ms1;
                pince->pump_left.cmd_timer = Timer_ms1;
                pince->action_step++;
            }
            break;

        case 12: // check pump are on
            if(pince->action_done){
                if (Timer_ms1 - pince->pump_right.cmd_timer >= 250){ 
                    pince->action_done = 0; 
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current, &pince->action_done);
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
                    pince->action_step++;
                }
            }
            break;

        case 13: 
            if(pince->action_done){
                if(pince->pump_right.pump_current > CURRENT_THRESHOLD_ON_ALLUMAGE && pince->pump_left.pump_current > CURRENT_THRESHOLD_ON_ALLUMAGE){ 
                    printf("Pump on. Lancement calcul du baseline à vide...\n");
                    pince->retry_count = 0; 
                    pince->action_done = 0;
                    
                    // Préparation pour l'échantillonnage de base (baseline)
                    pince->pump_right.sum_current = 0;
                    pince->pump_left.sum_current = 0;
                    pince->sample_count = 0;
                    pince->action_step = 14; // On passe à la sous-étape d'échantillonnage
                } else {
                    pince->retry_count++;
                    if (pince->retry_count >= 3) {
                        printf("ERREUR CRITIQUE: Impossible d'allumer les pompes après 3 essais. Abandon.\n");
                        pince->retry_count = 0; 
                        pince->action_step = 500; 
                    } else {
                        printf("Pump not ok (D:%d, G:%d), retry...\n", pince->pump_right.pump_current, pince->pump_left.pump_current);
                        pince->action_step = 11; 
                    }
                }
            }
            break;

        // --- ECHANTILLONNAGE BASELINE (A VIDE) ---
        case 14:
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current, &pince->action_done);
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
            pince->action_step = 15;
            break;

        case 15:
            if(pince->action_done){
                pince->action_done = 0;
                pince->pump_right.sum_current += pince->pump_right.pump_current;
                pince->pump_left.sum_current  += pince->pump_left.pump_current;
                pince->sample_count++;
                
                if (pince->sample_count >= NBR_VALUES_FOR_MEAN) { // On fait la moyenne sur 4 valeurs
                    pince->pump_right.baseline_current = pince->pump_right.sum_current / NBR_VALUES_FOR_MEAN;
                    pince->pump_left.baseline_current  = pince->pump_left.sum_current / NBR_VALUES_FOR_MEAN;
                    printf("Baseline trouvé - D:%d, G:%d\n", pince->pump_right.baseline_current, pince->pump_left.baseline_current);
                    pince->action_step = 16; // On reprend le flux normal
                } else {
                    pince->action_step = 14; // On boucle pour le prochain échantillon
                }
            }
            break;
        // -------------------------------------------------------------

        case 16: // wait for pince to reach position
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
            pince->action_step++;
            break;    

        case 17:
            if(pince->action_done){
                if((pince->gros_pos.ramasser_pos - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.ramasser_pos + 100)){
                    printf("Pince en position, debut de l'aspiration...\n");
                    pince->action_timer = Timer_ms1; 
                    pince->action_done = 0;
                    
                    // Préparation pour l'échantillonnage de prise (catch)
                    pince->pump_right.sum_current = 0;
                    pince->pump_left.sum_current = 0;
                    pince->sample_count = 0;
                    pince->action_step = 18;
                } else if (Timer_ms1 - pince->gros_pos.cmd_timer >= 3000){
                    printf("Pince action timeout at position %d\n", pince->gros_pos.current_position);
                    pince->action_step = 500; 
                } else {
                    pince->action_step = 16; 
                }
            }
            break;
 
        case 18: // Demande de mesure
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current, &pince->action_done);
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
            pince->action_step = 19;
            break;

        case 19: // Lecture, Moyenne et Décision
            if(pince->action_done){
                pince->action_done = 0;
                
                // Accumulation
                pince->pump_right.sum_current += pince->pump_right.pump_current;
                pince->pump_left.sum_current  += pince->pump_left.pump_current;
                pince->sample_count++;

                // On attend d'avoir 3 échantillons pour lisser les pics aléatoires
                if (pince->sample_count >= NBR_VALUES_FOR_MEAN) {
                    uint16_t avg_right = pince->pump_right.sum_current / NBR_VALUES_FOR_MEAN;
                    uint16_t avg_left  = pince->pump_left.sum_current / NBR_VALUES_FOR_MEAN;

                    printf("Moyennes actuelles après %d échantillons: D=%d, G=%d\n", NBR_VALUES_FOR_MEAN, avg_right, avg_left);
                    
                    // On compare la moyenne actuelle avec le baseline (à vide)
                    uint8_t right_ok = (ABS_DIFF(pince->pump_right.baseline_current, avg_right) > CURRENT_VARIATION_CATCH);
                    uint8_t left_ok  = (ABS_DIFF(pince->pump_left.baseline_current, avg_left) > CURRENT_VARIATION_CATCH);

                    if(right_ok && left_ok){ 
                        printf("Deux objets ramassés (Variations > %d)\n", CURRENT_VARIATION_CATCH);
                        pince->action_step = 20; // On passe à la remontée
                    } else {
                        if (Timer_ms1 - pince->action_timer >= 2000){ 
                            // TIMEOUT 2 SECONDES
                            if (right_ok || left_ok) {
                                printf("Timeout: 1 seul objet ramassé (G:%d, D:%d)\n", left_ok, right_ok);
                                if (!right_ok) PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                                if (!left_ok)  PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                                pince->action_step = 20; 
                            } else {
                                printf("Timeout: Aucun objet ramassé. Moyennes: D=%d (Base %d), G=%d (Base %d)\n", 
                                       avg_right, pince->pump_right.baseline_current, avg_left, pince->pump_left.baseline_current);
                                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                                PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                                pince->action_step = 500; 
                            }
                        } else {
                            // --- LA CORRECTION EST ICI ---
                            // On n'a pas encore timeout, on relance juste l'échantillonnage de courant
                            pince->pump_right.sum_current = 0;
                            pince->pump_left.sum_current = 0;
                            pince->sample_count = 0;
                            pince->action_step = 18; // Retour au 18 (Demande de mesure) et non au 16 !
                        }
                    }
                } else {
                    // On n'a pas encore nos 3 échantillons, on boucle sur la mesure
                    pince->action_step = 18;
                }
            }
            break;
        
        case 20: //remonter la pince
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 21;
            break;

        case 21:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
                pince->action_step = 22;
            }
            break;

        case 22:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    printf("Pince action done at position %d\n", pince->action_position);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Pince action timeout at position %d\n", pince->action_position);
                    pince->action_step = 500; // on a un problème, on coupe les pompes et on abandonne
                } else {
                    pince->action_step = 21; // keep waiting
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
            printf("Pince Abort ! Extinction pompes (Essai %d/3)...\n", pince->retry_count + 1);
            // 1. Eteindre les pompes
            PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            
            pince->action_timer = Timer_ms1;
            pince->action_step = 501;
            break;

        case 501:
            // On laisse un court instant (ex: 250ms) pour que l'inertie du moteur 
            // tombe et que le courant redescende réellement.
            if(Timer_ms1 - pince->action_timer >= 250){
                GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current, &pince->action_done);
                GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
                pince->action_step = 502;
            }
            break;

        case 502:
            if(pince->action_done){
                pince->action_done = 0;
                
                // On vérifie si les courants sont bien retombés (j'utilise < 100 comme seuil d'extinction)
                if(pince->pump_right.pump_current < CURRENT_THRESHOLD_ON_EXTINCTION && pince->pump_left.pump_current < CURRENT_THRESHOLD_ON_EXTINCTION){
                    printf("Pompes confirmées éteintes.\n");
                    pince->retry_count = 0; // On reset pour la prochaine fois
                    pince->action_step = 503; // On passe à la suite de la mise en sécurité
                } else {
                    pince->retry_count++;
                    if(pince->retry_count >= 3){
                        printf("ERREUR CRITIQUE: Impossible d'éteindre les pompes après 3 essais. Poursuite de l'abandon.\n");
                        pince->retry_count = 0; // On reset
                        pince->action_step = 503; // On force la suite pour ne pas bloquer le robot
                    } else {
                        printf("Echec extinction pompes (D:%d, G:%d), nouvel essai...\n", pince->pump_right.pump_current, pince->pump_left.pump_current);
                        pince->action_step = 500; // On boucle pour renvoyer la commande
                    }
                }
            }
            break;

        case 503:
            // 2. On ouvre les valves au cas où on avait en main un objet pour éviter de le garder 
            // en succion et de risquer de le faire tomber au mauvais moment
            PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
            
            // 3. ranger les clapets pour éviter de les casser en cas de collision au retour
            PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.retrait_pos, &pince->action_done);
            PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.retrait_pos, &pince->action_done);
            
            // 4. Ordonner la remontée de sécurité
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            
            pince->action_timer = Timer_ms1;
            pince->action_step = 504;
            break;
            
        case 504:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
                pince->action_step = 505;
            }
            break;
            
        case 505:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    printf("Pince action annulée et retour au repos ok\n");
                    pince->action_done = 0;
                    pince->action_step = 0; // Prêt pour une nouvelle commande
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Timeout critique : impossible de remonter la pince.\n");
                    pince->action_step = 0; // On force à 0 pour ne pas geler tout le robot
                } else {
                    pince->action_step = 504; // Continue de vérifier la position
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
    init_pince_state = 0;

    for (int i = 0; i < NBR_PINCES; i++) {
        robot_pinces[i].action_step = 0;
        robot_pinces[i].current_command = CMD_IDLE;
    }
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

                robot_pinces[i].retry_count = 0;

                robot_pinces[i].sample_count = 0;

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


// Tu peux changer cet ID pour tester les autres pinces (10, 20, 30...)
#define CALIB_PUMP_ID 20 
#define CALIB_WINDOW 50
// Variables globales pour la calibration
uint8_t calib_step = 0;
uint32_t calib_timer = 0;
uint8_t calib_done = 0;
uint16_t calib_current_right = 0;
uint16_t calib_current_left = 0;

// Variables dédiées à la moyenne glissante
uint16_t samples_right[CALIB_WINDOW] = {0};
uint16_t samples_left[CALIB_WINDOW]  = {0};
uint32_t sum_right = 0;
uint32_t sum_left = 0;
uint8_t sample_idx = 0;
uint8_t buffer_full = 0;
uint8_t print_counter = 0;


void Pump_Calibration_Loop(void) {
    switch(calib_step) {
        case 0:
            printf("\n=== DEMARRAGE CALIBRATION POMPE %d ===\n", CALIB_PUMP_ID);
            
            // 1. Réinitialisation complète du buffer circulaire
            sum_right = 0;
            sum_left = 0;
            sample_idx = 0;
            buffer_full = 0;
            print_counter = 0;
            for(int i = 0; i < CALIB_WINDOW; i++){
                samples_right[i] = 0;
                samples_left[i] = 0;
            }
            
            // Allumage des deux pompes
            // INFO : J'utilise un PutFEETECH "normal" pour la 1ère commande.
            // Utiliser deux fois Ext_Done de suite avec le même &calib_done risque d'écraser le flag.
            PutFEETECH(CALIB_PUMP_ID, PUMP_CMD_1, PUMP_ON);
            PutFEETECH_Ext_Done(CALIB_PUMP_ID, PUMP_CMD_2, PUMP_ON, &calib_done);
            calib_timer = Timer_ms1;
            calib_step = 1;
            break;
            
        case 1:
            // Attendre 500ms que les pompes soient bien lancées
            if (calib_done) {
                if (Timer_ms1 - calib_timer >= 500) { 
                    calib_done = 0;
                    calib_step = 2;
                }
            }
            break;
            
        case 2:
            // Demander le courant Droit (1)
            GetFEETECH_Ext_Done(CALIB_PUMP_ID, ADDR_CURRENT_1_L, &calib_current_right, &calib_done);
            calib_step = 3;
            break;
            
        case 3:
            if(calib_done) {
                calib_done = 0;
                // Demander le courant Gauche (2)
                GetFEETECH_Ext_Done(CALIB_PUMP_ID, ADDR_CURRENT_2_L, &calib_current_left, &calib_done);
                calib_step = 4;
            }
            break;
            
        case 4:
            if(calib_done) {
                calib_done = 0;
                
                // --- ALGORITHME DE MOYENNE GLISSANTE ---
                // a) Retirer la plus vieille valeur de la somme
                sum_right -= samples_right[sample_idx];
                sum_left  -= samples_left[sample_idx];
                
                // b) Ajouter la nouvelle valeur au tableau et à la somme
                samples_right[sample_idx] = calib_current_right;
                samples_left[sample_idx]  = calib_current_left;
                sum_right += calib_current_right;
                sum_left  += calib_current_left;
                
                // c) Avancer le curseur du buffer circulaire
                sample_idx++;
                if(sample_idx >= CALIB_WINDOW) {
                    sample_idx = 0;
                    buffer_full = 1; // Le tableau est entièrement rempli
                }
                
                // d) Calculer la moyenne
                uint16_t avg_right, avg_left;
                if(buffer_full) {
                    avg_right = sum_right / CALIB_WINDOW;
                    avg_left  = sum_left  / CALIB_WINDOW;
                } else {
                    // Si on vient de démarrer, on divise seulement par les valeurs acquises
                    avg_right = sum_right / sample_idx;
                    avg_left  = sum_left  / sample_idx;
                }
                
                // --- AFFICHAGE REDUIT ---
                print_counter++;
                if (print_counter >= 10) { // N'affiche qu'1 fois sur 10
                    printf("CALIB - Brut[D:%4d G:%4d] | MOYENNE LISSEE[D:%4d G:%4d]\n", 
                           calib_current_right, calib_current_left, avg_right, avg_left);
                    print_counter = 0;
                }
                
                calib_timer = Timer_ms1;
                calib_step = 5;
            }
            break;
            
        case 5:
            // On tourne très vite ! 20ms = 50 mesures par seconde
            if (Timer_ms1 - calib_timer >= 20) { 
                calib_step = 2; // On boucle
            }
            break;
    }
}