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
            pince->current_command = CMD_IDLE;
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
                    printf("[pince : %d] : Tentative allumage pompes %d/3\n", pince->id, pince->retry_count + 1);
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
                if (Timer_ms1 - pince->pump_right.cmd_timer >= 500){ 
                    pince->action_done = 0; 
                    GetFEETECH(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current);
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
                    pince->action_step++;
                }
            }
            break;

        case 13: 
            if(pince->action_done){
                if(pince->pump_right.pump_current > CURRENT_THRESHOLD_ON_ALLUMAGE && pince->pump_left.pump_current > CURRENT_THRESHOLD_ON_ALLUMAGE){ 
                    // printf("Pump on. Lancement calcul du baseline à vide...\n");
                    pince->retry_count = 0; 
                    pince->action_done = 0;
                    
                    // Préparation pour l'échantillonnage de base (baseline)
                    pince->pump_right.sum_current = 0;
                    pince->pump_left.sum_current = 0;
                    pince->sample_idx = 0;
                    pince->action_step = 14; 
                } else {
                    pince->retry_count++;
                    if (pince->retry_count >= 3) {
                        printf("[pince : %d] : ERREUR CRITIQUE: Impossible d'allumer les pompes après 3 essais. Abandon.\n", pince->id);
                        pince->retry_count = 0; 
                        pince->action_step = 500; 
                    } else {
                        // printf("Pump not ok (D:%d, G:%d), retry...\n", pince->pump_right.pump_current, pince->pump_left.pump_current);
                        pince->action_step = 11; 
                    }
                }
            }
            break;

        // --- ECHANTILLONNAGE BASELINE (A VIDE) ---
        case 14:
            GetFEETECH(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current);
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
            pince->action_step = 15;
            break;

        case 15:
            if(pince->action_done){
                pince->action_done = 0;
                
                // Ici une simple moyenne classique suffit pour la base
                pince->pump_right.sum_current += pince->pump_right.pump_current;
                pince->pump_left.sum_current  += pince->pump_left.pump_current;
                pince->sample_idx++;
                
                if (pince->sample_idx >= NBR_VALUES_FOR_MEAN) { 
                    pince->pump_right.baseline_current = pince->pump_right.sum_current / NBR_VALUES_FOR_MEAN;
                    pince->pump_left.baseline_current  = pince->pump_left.sum_current / NBR_VALUES_FOR_MEAN;
                    printf("[pince : %d] : Baseline trouvé - D:%d, G:%d\n", pince->id, pince->pump_right.baseline_current, pince->pump_left.baseline_current);
                    pince->action_step = 16; // On reprend le flux normal
                } else {
                    pince->action_step = 14; 
                }
            }
            break;

        // -------------------------------------------------------------

        case 16: // wait for pince to reach position
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
            pince->action_step++;
            break;    

        case 17: // check position and stabilize
            if(pince->action_done){
                // Vérification de la position avec tolérance
                if((pince->gros_pos.ramasser_pos - 20) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.ramasser_pos + 20)){
                    
                    // On ne passe pas au step 18 tant qu'on n'a pas attendu 150ms EN POSITION
                    if (pince->action_timer == 0) {
                        pince->action_timer = Timer_ms1; // On lance le chrono au premier passage
                        printf("[pince : %d] : Position atteinte, stabilisation et mise en pression (150ms)...\n", pince->id);
                    }

                    if (Timer_ms1 - pince->action_timer >= 150) {
                        printf("[pince : %d] : Stabilisation finie, début de l'analyse.\n", pince->id);
                        pince->action_timer = 0; // Reset pour usage futur
                        pince->action_done = 0;
                        
                        // --- INITIALISATION DU BUFFER DE MOYENNE GLISSANTE ---
                        pince->pump_right.sum_current = 0;
                        pince->pump_left.sum_current = 0;
                        pince->sample_idx = 0;
                        pince->buffer_full = 0;
                        for(int i = 0; i < NBR_VALUES_FOR_MEAN; i++) {
                            pince->pump_right.samples[i] = 0;
                            pince->pump_left.samples[i]  = 0;
                        }
                        pince->gros_pos.current_position = 0;
                        pince->action_step = 18; // On passe enfin à la mesure
                    }
                } else if (Timer_ms1 - pince->gros_pos.cmd_timer >= 3000){
                    printf("[pince : %d] : Pince action timeout at position %d\n", pince->id, pince->gros_pos.current_position);
                    pince->action_step = 500; 
                } else {
                    pince->action_timer = 0; // On reset le timer si on sort de la zone de tolérance
                    pince->action_step = 16; 
                }
            }
            break;
 
        case 18: // Demande de mesure continue
            GetFEETECH(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current);
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
            pince->action_step = 19;
            break;

        case 19: // Moyenne Glissante et Décision
            if(pince->action_done){
                pince->action_done = 0;
                
                // 1. Soustraire l'ancienne valeur du buffer (Seulement si le buffer est plein)
                if (pince->buffer_full) {
                    pince->pump_right.sum_current -= pince->pump_right.samples[pince->sample_idx];
                    pince->pump_left.sum_current  -= pince->pump_left.samples[pince->sample_idx];
                }

                // 2. Insérer la nouvelle valeur
                pince->pump_right.samples[pince->sample_idx] = pince->pump_right.pump_current;
                pince->pump_left.samples[pince->sample_idx]  = pince->pump_left.pump_current;
                
                // 3. Ajouter à la somme totale
                pince->pump_right.sum_current += pince->pump_right.pump_current;
                pince->pump_left.sum_current  += pince->pump_left.pump_current;
                
                // 4. Avancer l'index tournant
                pince->sample_idx++;
                if (pince->sample_idx >= NBR_VALUES_FOR_MEAN) {
                    pince->sample_idx = 0;
                    pince->buffer_full = 1; // Le tableau a fait un tour complet !
                }

                // 5. Décision (On commence à évaluer dès 5 échantillons pour aller vite si on a tout)
                uint8_t current_sample_count = pince->buffer_full ? NBR_VALUES_FOR_MEAN : pince->sample_idx;

                if (current_sample_count >= 5) { 
                    uint16_t avg_right = pince->pump_right.sum_current / current_sample_count;
                    uint16_t avg_left  = pince->pump_left.sum_current / current_sample_count;
                    printf("[pince : %d] : Moyenne glissante - D:%d, G:%d (Baseline D:%d, G:%d)\n", pince->id, avg_right, avg_left, pince->pump_right.baseline_current, pince->pump_left.baseline_current);

                    // On compare la moyenne glissante avec le baseline
                    uint8_t right_ok = (ABS_DIFF(pince->pump_right.baseline_current, avg_right) > CURRENT_VARIATION_CATCH);
                    uint8_t left_ok  = (ABS_DIFF(pince->pump_left.baseline_current, avg_left) > CURRENT_VARIATION_CATCH);

                    if(right_ok && left_ok){ 
                        printf("[pince : %d] : Deux objets ramassés (Variations > %d)\n", pince->id, CURRENT_VARIATION_CATCH);
                        pince->action_step = 20; // On passe à la remontée INSTANTANEMENT
                    } else {
                        // --- NOUVELLE LOGIQUE D'ABANDON BASÉE SUR LE BUFFER ---
                        if (pince->buffer_full) { 
                            // Le buffer a fait un tour complet : l'analyse est définitive
                            if (right_ok || left_ok) {
                                printf("[pince : %d] : Analyse terminée: 1 seul objet ramassé (G:%d, D:%d)\n", pince->id, left_ok, right_ok);
                                if (!right_ok) PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                                if (!left_ok)  PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                                pince->action_step = 20; 
                            } else {
                                printf("[pince : %d] : Analyse terminée: Aucun objet ramassé.\n", pince->id);
                                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                                PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                                pince->action_step = 500; 
                            }
                        } else {
                            // Le buffer n'a pas encore fini son premier tour, on continue d'échantillonner
                            pince->action_step = 18; 
                        }
                    }
                } else {
                    // On n'a pas encore nos 5 premiers échantillons, on relance direct au 18
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
                    printf("[pince : %d] : Pince action done at position %d\n", pince->id, pince->action_position);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("[pince : %d] : Pince action timeout at position %d\n", pince->id, pince->action_position);
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
                    printf("[pince : %d] : Pince action done at position %d\n", pince->id, pince->gros_pos.current_position);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("[pince : %d] : Pince action timeout at position %d\n", pince->id, pince->gros_pos.current_position);
                    pince->action_step = 0;
                } else {
                    pince->action_step = 101; // keep waiting
                }
            }
            break;

        
        /* ---------------------------------------------------- */
        /* ------------- RETOURNER PALETS ----------------------*/
        /* ---------------------------------------------------- */

        case 200: // Ordonner la descente du bras
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.lacher_pos, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 201;
            break;

        case 201: // Demander la position
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 202;
            }
            break;

        case 202: // Vérifier l'arrivée en position
            if(pince->action_done){
                if((pince->gros_pos.lacher_pos - 50) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.lacher_pos + 50)){
                    printf("[pince : %d] : Pince en position de lacher (%d)\n", pince->id, pince->gros_pos.current_position);
                    pince->action_done = 0;
                    pince->action_step = 203;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("[pince : %d] : Pince action timeout at position %d\n", pince->id, pince->gros_pos.current_position);
                    pince->action_step = 500; // Sas de sécurité
                } else {
                    pince->action_step = 201; // Continue de vérifier
                }
            }
            break;

        case 203: // Ouvrir les clapets (les sortir)
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                printf("[pince : %d] : Ouverture clapet gauche a la position %d\n", pince->id, pince->petit_gauche_pos.sortie_pos);
                PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.sortie_pos, &pince->action_done);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                printf("[pince : %d] : Ouverture clapet droite a la position %d\n", pince->id, pince->petit_droite_pos.sortie_pos);
                PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.sortie_pos, &pince->action_done);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 204;
            break;

        case 204: // Demander la position des clapets
            if(pince->action_done){
                pince->action_done = 0;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_PRESENT_POSITION_L, &pince->petit_gauche_pos.current_position, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_PRESENT_POSITION_L, &pince->petit_droite_pos.current_position, &pince->action_done);
                }
                pince->action_step = 205;
            }
            break;

        case 205: // Vérifier l'ouverture des clapets
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
                    pince->action_done = 0;
                    pince->retry_count = 0;   // <-- INITIALISATION DU COMPTEUR DE RETRY
                    pince->action_step = 206; // Clapets ouverts, on passe aux pompes
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("Timeout ouverture clapets\n");
                    pince->action_step = 500; // Sas de sécurité
                } else {
                    pince->action_step = 204; // Continue de vérifier
                }
            }
            break;

        case 206: // Éteindre les pompes
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 207;
            break;

        case 207: // Demander les courants (après une courte pause de 100ms pour l'inertie)
            if(Timer_ms1 - pince->action_timer >= 100){
                pince->action_done = 0;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_left.pump_current, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_right.pump_current, &pince->action_done);
                }
                pince->action_step = 208;
            }
            break;

        case 208: // Vérifier la chute de courant (Pompes bien éteintes)
            if(pince->action_done){
                pince->action_done = 0;
                uint8_t pump_ok = 1;
                
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_left.pump_current > CURRENT_THRESHOLD_ON_EXTINCTION) pump_ok = 0;
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_right.pump_current > CURRENT_THRESHOLD_ON_EXTINCTION) pump_ok = 0;
                }

                if(pump_ok){
                    printf("[pince : %d] : Pompes coupees. Activation valves.\n", pince->id);
                    pince->retry_count = 0;
                    pince->action_step = 209;
                } else {
                    pince->retry_count++;
                    if(pince->retry_count >= 3){ 
                        printf("[pince : %d] : ERREUR CRITIQUE: Le courant pompe ne chute pas apres 3 tentatives. Abandon.\n", pince->id);
                        pince->action_step = 500; // Sas de sécurité
                    } else {
                        printf("[pince : %d] : Echec extinction pompes, tentative %d/3...\n", pince->id, pince->retry_count + 1);
                        pince->action_step = 206; // <-- RETOUR À L'ÉTAPE 206 POUR RENVOYER LA COMMANDE OFF
                    }
                }
            }
            break;

        case 209: // Allumer les électrovannes pour casser le vide et lâcher l'objet
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 210;
            break;

        case 210: // Attendre 1s que ça tombe, puis retirer les clapets
            if(Timer_ms1 - pince->action_timer >= 1000){
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.retrait_pos, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.retrait_pos, &pince->action_done);
                }
                pince->action_timer = Timer_ms1;
                pince->action_step = 211;
            }
            break;

        case 211: // Demander la position de retrait
            if(pince->action_done){
                pince->action_done = 0;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_PRESENT_POSITION_L, &pince->petit_gauche_pos.current_position, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_PRESENT_POSITION_L, &pince->petit_droite_pos.current_position, &pince->action_done);
                }
                pince->action_step = 212;
            }
            break;

        case 212: // Vérifier que les clapets sont bien rentrés
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
                    printf("[pince : %d] : Clapets rentres, depot termine avec succes.\n", pince->id);
                    pince->action_done = 0;
                    pince->action_step = 213; // remettre la pince en position haute pour éviter les accrochages
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("[pince : %d] : Timeout retrait clapets\n", pince->id);
                    pince->action_step = 500; // Sas de sécurité
                } else {
                    pince->action_step = 211; // Continue de vérifier
                }
            }
            break;

        case 213: // Remonter la pince après le dépôt
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 214;
            break;

        case 214: // Vérifier la remontée
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 215;
            }
            break;

        case 215:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.idle_position + 100)){
                    printf("[pince : %d] : Pince remontee en position idle (%d)\n", pince->id, pince->gros_pos.current_position);
                    pince->action_done = 0;
                    pince->action_step = 0; // On a fini le cycle de dépôt, on reset l'état pour la prochaine commande
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("[pince : %d] : Timeout remontée pince après dépôt (pos: %d)\n", pince->id, pince->gros_pos.current_position);
                    pince->action_step = 500; // Sas de sécurité
                } else {
                    pince->action_step = 214; // Continue de vérifier
                }
            }
            break;
        
        /* ---------------------------------------------------- */
        /* ------------- DEPOSER PALETS ----------------------*/
        /* ---------------------------------------------------- */
        
        case 300: // 1. Ordre de baisser la pince
            pince->retry_count = 0; 
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.ramasser_pos, &pince->action_done); 
            pince->gros_pos.cmd_timer = Timer_ms1;
            pince->action_step = 301;
            break;
            
        case 301: // 2. Attendre que l'ordre de descente soit bien transmis
            if(pince->action_done){
                pince->action_done = 0;
                // On met une valeur "impossible" dans current_position pour forcer une vraie lecture
                pince->gros_pos.current_position = 0xFFFF; 
                pince->action_step = 302;
            }
            break;
            
        case 302: // 3. Demander la position actuelle
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
            pince->action_step = 303;
            break;
            
        case 303: // 4. Attendre la réponse et vérifier la position
            if(pince->action_done){
                pince->action_done = 0; 
                
                // On vérifie si on est bien arrivé en position basse
                if((pince->gros_pos.ramasser_pos - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.ramasser_pos + 100)){
                    printf("[pince : %d] : Pince en position de depot (%d)\n", pince->id, pince->gros_pos.current_position);
                    pince->action_step = 304; // ---> PINCE EN BAS : ON PEUT LACHER
                } else if (Timer_ms1 - pince->gros_pos.cmd_timer >= 3000){
                    printf("[pince : %d] : Timeout descente pince depot (pos: %d)\n", pince->id, pince->gros_pos.current_position);
                    pince->action_step = 500; // ---> ABANDON DE SECURITE
                } else {
                    pince->action_step = 302; // Pas encore en bas : on redemande la position
                }
            }
            break;
            
        case 304: // 5. COUPER LES POMPES EN PREMIER
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            }
            
            pince->action_timer = Timer_ms1;
            pince->action_step = 305;
            break;
            
        case 305: // 6. LAISSER UN MINI TEMPS DE 100ms
            if(Timer_ms1 - pince->action_timer >= 100){
                pince->action_step = 306;
            }
            break;

        case 306: // 7. ACTIVER LES ELECTROVANNES POUR CASSER LE VIDE
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
            }
            pince->action_step = 307;
            break;
            
        case 307: // 8. DEMANDER LES COURANTS POUR VERIFIER LA CHUTE
            pince->action_done = 0;
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_left.pump_current, &pince->action_done);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_right.pump_current, &pince->action_done);
            }
            pince->action_step = 308;
            break;
            
        case 308: // 9. VERIFIER LA CHUTE DE COURANT
            if(pince->action_done){
                pince->action_done = 0;
                uint8_t pump_ok = 1;
                
                // On utilise CURRENT_THRESHOLD_ON_EXTINCTION (par exemple 300 ou 400)
                // pour s'assurer que le moteur est bien en train de s'arrêter.
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_left.pump_current > CURRENT_THRESHOLD_ON_EXTINCTION){ 
                        pump_ok = 0;
                    }
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_right.pump_current > CURRENT_THRESHOLD_ON_EXTINCTION){ 
                        pump_ok = 0;
                    }
                }

                if(pump_ok){
                    printf("[pince : %d] : Courant bas, pompes coupees, palet relache.\n", pince->id);
                    pince->retry_count = 0;
                    pince->action_step = 309; // Le palet est tombé, on remonte
                } else {
                    pince->retry_count++;
                    if(pince->retry_count >= 50){ 
                        // Au bout de 50 essais (environ 1 seconde de boucle), on abandonne
                        printf("[pince : %d] : ERREUR CRITIQUE: Le courant ne chute pas (%d / %d). Abandon.\n", pince->id, pince->pump_left.pump_current, pince->pump_right.pump_current);
                        pince->action_step = 500; 
                    } else {
                        // Le rotor tourne encore un peu, on refait une mesure
                        pince->action_step = 307; 
                    }
                }
            }
            break;

        case 309: // 10. Ordre de remonter la pince
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 310;
            break;

        case 310: // 11. Attendre que l'ordre de remontée parte
            if(pince->action_done){
                pince->action_done = 0;
                pince->action_step = 311;
            }
            break;
            
        case 311: // 12. Demander position actuelle
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
            pince->action_step = 312;
            break;
            
        case 312: // 13. Verifier si on est en haut
            if(pince->action_done){
                pince->action_done = 0;
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    printf("Pince remontee at position %d\n", pince->action_position);
                    pince->action_step = 0; // FIN DU CYCLE
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("[pince : %d] : Timeout remontee pince (pos: %d)\n", pince->id, pince->action_position);
                    pince->action_step = 500; 
                } else {
                    pince->action_step = 311; // keep waiting
                }
            }
            break;

       /* ---------------------------------------------------- */
        /* ------------- SEQUENCE D'ABANDON / ERREUR -----------*/
        /* ---------------------------------------------------- */
        case 500: // Sas d'initialisation de la mise en sécurité
            pince->retry_count = 0;
            pince->action_step = 501;
            break;
        case 501:
            printf( "\n\n====================\n" );
            printf("[pince : %d] : Pince Abort ! Extinction pompes (Essai %d/3)...\n", pince->id, pince->retry_count + 1);
            // 1. Eteindre les pompes
            PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            
            pince->action_timer = Timer_ms1;
            pince->action_step = 502;
            break;

        case 502:
            // On laisse un court instant (ex: 250ms) pour que l'inertie du moteur 
            // tombe et que le courant redescende réellement.
            if(Timer_ms1 - pince->action_timer >= 250){
                GetFEETECH(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current);
                GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
                pince->action_step = 503;
            }
            break;

        case 503:
            if(pince->action_done){
                pince->action_done = 0;
                
                // On vérifie si les courants sont bien retombés (j'utilise < 100 comme seuil d'extinction)
                if(pince->pump_right.pump_current < CURRENT_THRESHOLD_ON_EXTINCTION && pince->pump_left.pump_current < CURRENT_THRESHOLD_ON_EXTINCTION){
                    printf("Pompes confirmées éteintes.\n");
                    pince->retry_count = 0; // On reset pour la prochaine fois
                    pince->action_step = 504; // On passe à la suite de la mise en sécurité
                } else {
                    pince->retry_count++;
                    if(pince->retry_count >= 3){
                        printf("[pince : %d] : ERREUR CRITIQUE: Impossible d'éteindre les pompes après 3 essais. Poursuite de l'abandon.\n", pince->id);
                        pince->retry_count = 0; // On reset
                        pince->action_step = 504; // On force la suite pour ne pas bloquer le robot
                    } else {
                        printf("[pince : %d] : Echec extinction pompes (D:%d, G:%d), nouvel essai...\n", pince->id, pince->pump_right.pump_current, pince->pump_left.pump_current);
                        pince->action_step = 501; // On boucle pour renvoyer la commande
                    }
                }
            }
            break;

        case 504:
            // 2. On ouvre les valves au cas où on avait en main un objet pour éviter de le garder 
            // en succion et de risquer de le faire tomber au mauvais moment
            PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
            
            // 3. ranger les clapets pour éviter de les casser en cas de collision au retour
            PutFEETECH(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.retrait_pos);
            PutFEETECH(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.retrait_pos);
            
            // 4. Ordonner la remontée de sécurité
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            
            pince->action_timer = Timer_ms1;
            pince->action_step = 505;
            break;
            
        case 505:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
                pince->action_step = 506;
            }
            break;
            
        case 506:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    printf("[pince : %d] : Pince action annulée et retour au repos ok\n", pince->id);
                    pince->action_done = 0;
                    pince->action_step = 0; // Prêt pour une nouvelle commande
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    printf("[pince : %d] : Timeout critique : impossible de remonter la pince.\n", pince->id);
                    pince->action_step = 0; // On force à 0 pour ne pas geler tout le robot
                } else {
                    pince->action_step = 505; // Continue de vérifier la position
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
            
            if(param == 0){
                pince->current_command = CMD_LACHER_G;
            } else if (param == 1){
                pince->current_command = CMD_LACHER_D;
            } else if (param == 2){
                pince->current_command = CMD_LACHER_ALL;
            } else {
                printf("Invalid parameter for LACHER command\n");
                return PARAM_ERROR_CODE;
            }
            pince->action_step = 300;
            break;
        case 3:
            if(param == 0){
                pince->current_command = CMD_LACHER_G;
            } else if (param == 1){
                pince->current_command = CMD_LACHER_D;
            } else if (param == 2){
                pince->current_command = CMD_LACHER_ALL;
            } else {
                printf("Invalid parameter for LACHER command\n");
                return PARAM_ERROR_CODE;
            }
            pince->action_step = 200;
            break;
        case 4:
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
                robot_pinces[i].id = i; 

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

                robot_pinces[i].sample_idx = 0;
                robot_pinces[i].buffer_full = 0;

                robot_pinces[i].current_command = CMD_IDLE;  
            }
            // PINCE_0
            robot_pinces[0].gros_pos.idle_position = PINCE_1_GROS_IDLE_POS;
            robot_pinces[0].gros_pos.ramasser_pos = PINCE_1_GROS_RAMASSER_POS;
            robot_pinces[0].gros_pos.lacher_pos = PINCE_1_GROS_LACHER_POS;
            robot_pinces[0].petit_droite_pos.sortie_pos = PINCE_1_DROITE_SORTIE_POS;
            robot_pinces[0].petit_droite_pos.retrait_pos = PINCE_1_DROITE_RETRAIT_POS;
            robot_pinces[0].petit_gauche_pos.sortie_pos = PINCE_1_GAUCHE_SORTIE_POS;
            robot_pinces[0].petit_gauche_pos.retrait_pos = PINCE_1_GAUCHE_RETRAIT_POS;

            // PINCE_1
            robot_pinces[1].gros_pos.idle_position = PINCE_2_GROS_IDLE_POS;
            robot_pinces[1].gros_pos.ramasser_pos = PINCE_2_GROS_RAMASSER_POS;
            robot_pinces[1].gros_pos.lacher_pos = PINCE_2_GROS_LACHER_POS;
            robot_pinces[1].petit_droite_pos.sortie_pos = PINCE_2_DROITE_SORTIE_POS;
            robot_pinces[1].petit_droite_pos.retrait_pos = PINCE_2_DROITE_RETRAIT_POS;
            robot_pinces[1].petit_gauche_pos.sortie_pos = PINCE_2_GAUCHE_SORTIE_POS;
            robot_pinces[1].petit_gauche_pos.retrait_pos = PINCE_2_GAUCHE_RETRAIT_POS;

            // PINCE_2
            robot_pinces[2].gros_pos.idle_position = PINCE_3_GROS_IDLE_POS;
            robot_pinces[2].gros_pos.ramasser_pos = PINCE_3_GROS_RAMASSER_POS;
            robot_pinces[2].gros_pos.lacher_pos = PINCE_3_GROS_LACHER_POS;
            robot_pinces[2].petit_droite_pos.sortie_pos = PINCE_3_DROITE_SORTIE_POS;
            robot_pinces[2].petit_droite_pos.retrait_pos = PINCE_3_DROITE_RETRAIT_POS;
            robot_pinces[2].petit_gauche_pos.sortie_pos = PINCE_3_GAUCHE_SORTIE_POS;
            robot_pinces[2].petit_gauche_pos.retrait_pos = PINCE_3_GAUCHE_RETRAIT_POS;

            // PINCE_3
            robot_pinces[3].gros_pos.idle_position = PINCE_4_GROS_IDLE_POS;
            robot_pinces[3].gros_pos.ramasser_pos = PINCE_4_GROS_RAMASSER_POS;
            robot_pinces[3].gros_pos.lacher_pos = PINCE_4_GROS_LACHER_POS;
            robot_pinces[3].petit_droite_pos.sortie_pos = PINCE_4_DROITE_SORTIE_POS;
            robot_pinces[3].petit_droite_pos.retrait_pos = PINCE_4_DROITE_RETRAIT_POS;
            robot_pinces[3].petit_gauche_pos.sortie_pos = PINCE_4_GAUCHE_SORTIE_POS;
            robot_pinces[3].petit_gauche_pos.retrait_pos = PINCE_4_GAUCHE_RETRAIT_POS;

            // PINCE_4
            robot_pinces[4].gros_pos.idle_position = PINCE_5_GROS_IDLE_POS;
            robot_pinces[4].gros_pos.ramasser_pos = PINCE_5_GROS_RAMASSER_POS;
            robot_pinces[4].gros_pos.lacher_pos = PINCE_5_GROS_LACHER_POS;
            robot_pinces[4].petit_droite_pos.sortie_pos = PINCE_5_DROITE_SORTIE_POS;
            robot_pinces[4].petit_droite_pos.retrait_pos = PINCE_5_DROITE_RETRAIT_POS;
            robot_pinces[4].petit_gauche_pos.sortie_pos = PINCE_5_GAUCHE_SORTIE_POS;
            robot_pinces[4].petit_gauche_pos.retrait_pos = PINCE_5_GAUCHE_RETRAIT_POS;

            // PINCE_5
            robot_pinces[5].gros_pos.idle_position = PINCE_6_GROS_IDLE_POS;
            robot_pinces[5].gros_pos.ramasser_pos = PINCE_6_GROS_RAMASSER_POS;
            robot_pinces[5].gros_pos.lacher_pos = PINCE_6_GROS_LACHER_POS;
            robot_pinces[5].petit_droite_pos.sortie_pos = PINCE_6_DROITE_SORTIE_POS;
            robot_pinces[5].petit_droite_pos.retrait_pos = PINCE_6_DROITE_RETRAIT_POS;
            robot_pinces[5].petit_gauche_pos.sortie_pos = PINCE_6_GAUCHE_SORTIE_POS;
            robot_pinces[5].petit_gauche_pos.retrait_pos = PINCE_6_GAUCHE_RETRAIT_POS;

            // PINCE_6
            robot_pinces[6].gros_pos.idle_position = PINCE_7_GROS_IDLE_POS;
            robot_pinces[6].gros_pos.ramasser_pos = PINCE_7_GROS_RAMASSER_POS;
            robot_pinces[6].gros_pos.lacher_pos = PINCE_7_GROS_LACHER_POS;
            robot_pinces[6].petit_droite_pos.sortie_pos = PINCE_7_DROITE_SORTIE_POS;
            robot_pinces[6].petit_droite_pos.retrait_pos = PINCE_7_DROITE_RETRAIT_POS;
            robot_pinces[6].petit_gauche_pos.sortie_pos = PINCE_7_GAUCHE_SORTIE_POS;
            robot_pinces[6].petit_gauche_pos.retrait_pos = PINCE_7_GAUCHE_RETRAIT_POS;

            // PINCE_7
            robot_pinces[7].gros_pos.idle_position = PINCE_8_GROS_IDLE_POS;
            robot_pinces[7].gros_pos.ramasser_pos = PINCE_8_GROS_RAMASSER_POS;
            robot_pinces[7].gros_pos.lacher_pos = PINCE_8_GROS_LACHER_POS;
            robot_pinces[7].petit_droite_pos.sortie_pos = PINCE_8_DROITE_SORTIE_POS;
            robot_pinces[7].petit_droite_pos.retrait_pos = PINCE_8_DROITE_RETRAIT_POS;
            robot_pinces[7].petit_gauche_pos.sortie_pos = PINCE_8_GAUCHE_SORTIE_POS;
            robot_pinces[7].petit_gauche_pos.retrait_pos = PINCE_8_GAUCHE_RETRAIT_POS;

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

uint8_t setup_pince_set_pos_cmd(void){
    uint32_t ID_pince;
    if (Get_Param_u32(&ID_pince))
        return PARAM_ERROR_CODE;

    if(ID_pince < 0 || ID_pince > NBR_PINCES-1){
        printf("Invalid pince ID\n");
        return PARAM_ERROR_CODE;
    }

    uint32_t servo_type; 
    if (Get_Param_u32(&servo_type))
        return PARAM_ERROR_CODE;
    
    if (servo_type > 2){
        printf("Invalid servo type\n");
        return PARAM_ERROR_CODE;
    }
    
    uint32_t param; // type de position à update 
    if (Get_Param_u32(&param))
        return PARAM_ERROR_CODE;

    uint32_t val;
    if (Get_Param_u32(&val))
        return PARAM_ERROR_CODE;

    Pince_t *pince = &robot_pinces[ID_pince];
    
    switch (servo_type){
        case 0:
            if(param == 0){
                pince->gros_pos.idle_position =  val;
            } else if (param == 1){
                pince->gros_pos.ramasser_pos = val;
            } else if (param == 2){
                pince->gros_pos.lacher_pos = val;
            } else {
                printf("Invalid parameter for GROS servo\n");
                return PARAM_ERROR_CODE;
            }
            printf ("Updated gros servo position: idle=%d, ramasser=%d, lacher=%d\n", 
                    pince->gros_pos.idle_position, pince->gros_pos.ramasser_pos, pince->gros_pos.lacher_pos);
            break;
        case 1:
            if(param == 0){
                pince->petit_droite_pos.sortie_pos = val;
            } else if (param == 1){
                pince->petit_droite_pos.retrait_pos = val;
            } else {
                printf("Invalid parameter for PETIT DROITE servo\n");
                return PARAM_ERROR_CODE;
            }
            printf ("Updated petit_droite servo position: sortie=%d, retrait=%d\n", 
                    pince->petit_droite_pos.sortie_pos, pince->petit_droite_pos.retrait_pos);
            break;
        case 2:
            if(param == 0){
                pince->petit_gauche_pos.sortie_pos = val;
            } else if (param == 1){
                pince->petit_gauche_pos.retrait_pos = val;
            } else {
                printf("Invalid parameter for PETIT GAUCHE servo\n");
                return PARAM_ERROR_CODE;
            }
            printf ("Updated petit_gauche servo position: sortie=%d, retrait=%d\n", 
                    pince->petit_gauche_pos.sortie_pos, pince->petit_gauche_pos.retrait_pos);
            break;
    }
    return 0;    
}