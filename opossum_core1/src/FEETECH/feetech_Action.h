#ifndef FEETECH_ACTION_H
#define FEETECH_ACTION_H

#define NBR_PINCES 8

#define ABS_DIFF(a, b) ((a) > (b) ? ((a) - (b)) : ((b) - (a)))

// -------------------------------------------------- //
// -------------- GLOBAL USE  ----------------------- //
#define NBR_VALUES_FOR_MEAN 40

// ------------------------------------------ //
// -------------- defines pompes ------------ //
#define PUMP_ON 255
#define PUMP_OFF 0

#define CURRENT_THRESHOLD_ON_ALLUMAGE 100 // if current is above this threshold, we assume the pump is on wen starting it
#define CURRENT_THRESHOLD_ON_EXTINCTION 1600 // if current is above this threshold, we assume the pump is on when trying to turn it off (we put a higher threshold here because it's possible that the pump is still stopping and consuming more current than usual, but if it's above this threshold, we can assume that the pump is not turning off correctly)

#define CURRENT_THRESHOLD_CATCH 2050 // if current is above this threshold, we assume an object is catched

#define CURRENT_VARIATION_CATCH 100 //62

#define VALVE_ON 1

// ------------------------------------------ //
// ---------- define des positions ---------- //
// ------------------------------------------ //

// ---------------- PINCE 0 ----------------- //
#define PINCE_1_GROS_IDLE_POS 2450
#define PINCE_1_GROS_RAMASSER_POS 3615
#define PINCE_1_GROS_LACHER_POS 2200

#define PINCE_1_DROITE_SORTIE_POS 620
#define PINCE_1_DROITE_RETRAIT_POS 512

#define PINCE_1_GAUCHE_SORTIE_POS 380
#define PINCE_1_GAUCHE_RETRAIT_POS 512

// ---------------- PINCE 1 ----------------- //
#define PINCE_2_GROS_IDLE_POS 3580
#define PINCE_2_GROS_RAMASSER_POS 2395
#define PINCE_2_GROS_LACHER_POS 3800

#define PINCE_2_DROITE_SORTIE_POS 620 //700
#define PINCE_2_DROITE_RETRAIT_POS 512

#define PINCE_2_GAUCHE_SORTIE_POS 380 //300
#define PINCE_2_GAUCHE_RETRAIT_POS 512

// ---------------- PINCE 2 ----------------- //
#define PINCE_3_GROS_IDLE_POS 2300
#define PINCE_3_GROS_RAMASSER_POS 3450
#define PINCE_3_GROS_LACHER_POS 2000

#define PINCE_3_DROITE_SORTIE_POS 620
#define PINCE_3_DROITE_RETRAIT_POS 512

#define PINCE_3_GAUCHE_SORTIE_POS 380
#define PINCE_3_GAUCHE_RETRAIT_POS 512

// ---------------- PINCE 3 ----------------- //
#define PINCE_4_GROS_IDLE_POS 3400
#define PINCE_4_GROS_RAMASSER_POS 2300
#define PINCE_4_GROS_LACHER_POS 3700

#define PINCE_4_DROITE_SORTIE_POS 620 //700
#define PINCE_4_DROITE_RETRAIT_POS 512

#define PINCE_4_GAUCHE_SORTIE_POS 380 //300
#define PINCE_4_GAUCHE_RETRAIT_POS 512

// ---------------- PINCE 4 ----------------- //
#define PINCE_5_GROS_IDLE_POS 2500
#define PINCE_5_GROS_RAMASSER_POS 3630
#define PINCE_5_GROS_LACHER_POS 2200

#define PINCE_5_DROITE_SORTIE_POS 620
#define PINCE_5_DROITE_RETRAIT_POS 512

#define PINCE_5_GAUCHE_SORTIE_POS 380
#define PINCE_5_GAUCHE_RETRAIT_POS 512

// ---------------- PINCE 5 ----------------- //
#define PINCE_6_GROS_IDLE_POS 3650
#define PINCE_6_GROS_RAMASSER_POS 2490
#define PINCE_6_GROS_LACHER_POS 3900

#define PINCE_6_DROITE_SORTIE_POS 620 //700
#define PINCE_6_DROITE_RETRAIT_POS 512

#define PINCE_6_GAUCHE_SORTIE_POS 380 //300
#define PINCE_6_GAUCHE_RETRAIT_POS 512

// ---------------- PINCE 6 ----------------- //
#define PINCE_7_GROS_IDLE_POS 2450
#define PINCE_7_GROS_RAMASSER_POS 3570
#define PINCE_7_GROS_LACHER_POS 2200

#define PINCE_7_DROITE_SORTIE_POS 620
#define PINCE_7_DROITE_RETRAIT_POS 512

#define PINCE_7_GAUCHE_SORTIE_POS 380
#define PINCE_7_GAUCHE_RETRAIT_POS 512

// ---------------- PINCE 7 ----------------- //
#define PINCE_8_GROS_IDLE_POS 3580
#define PINCE_8_GROS_RAMASSER_POS 2395
#define PINCE_8_GROS_LACHER_POS 3800

#define PINCE_8_DROITE_SORTIE_POS 620 //700
#define PINCE_8_DROITE_RETRAIT_POS 512

#define PINCE_8_GAUCHE_SORTIE_POS 380 //300
#define PINCE_8_GAUCHE_RETRAIT_POS 512


typedef enum {
    CMD_IDLE = 0,
    CMD_RAMASSER,
    CMD_LACHER_G,
    CMD_LACHER_D,
    CMD_LACHER_ALL,
    CMD_MONTER
} Pince_Command_t;


typedef struct {
    uint16_t idle_position;
    uint16_t ramasser_pos;
    uint16_t lacher_pos;

    uint16_t current_position;
    uint32_t cmd_timer;
} Gros_Servo_Pos_t;

typedef struct {
    uint16_t sortie_pos;
    uint16_t retrait_pos;

    uint16_t current_position;
    uint32_t cmd_timer;
} Petit_Servo_Pos_t;

typedef struct {
    uint16_t pump_current;
    uint32_t cmd_timer;

    uint16_t baseline_current; 
    uint32_t sum_current;

    uint16_t samples[NBR_VALUES_FOR_MEAN];
} Pump_t;


typedef struct {
    // -- ID matériel -- //
    uint8_t id_gros;
    uint8_t id_droite;
    uint8_t id_gauche;
    uint8_t id_pump;

    // -- variables d'états fsm -- //
    uint16_t action_step;
    uint8_t action_done;
    uint32_t action_timer;
    uint32_t action_position;

    // -- positions -- //
    Gros_Servo_Pos_t gros_pos;
    Petit_Servo_Pos_t petit_droite_pos;
    Petit_Servo_Pos_t petit_gauche_pos;

    Pump_t pump_right;
    Pump_t pump_left;

    uint8_t retry_count;

    uint8_t sample_count;

    uint8_t sample_idx;
    uint8_t buffer_full;

    // -- consignes -- //
    Pince_Command_t current_command;
} Pince_t;



uint8_t Send_FEETECH_Cmd(void);
uint8_t Get_FEETECH_Cmd(void);

uint8_t Send_FEETECH_SCS_Cmd(void);
uint8_t Get_FEETECH_SCS_Cmd(void);

void FEETECH_action_loop(void);

void pince_action_loop(Pince_t *pince);
uint8_t pince_action_cmd(void);

void pince_loop(void);
void Init_Pinces_Loop(void);
void AU_pinces(void);

void Pump_Calibration_Loop(void);

void setup_pince_set_pos_cmd(void);
#endif // FEETECH_ACTION_H