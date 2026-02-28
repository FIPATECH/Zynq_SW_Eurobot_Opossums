#ifndef FEETECH_ACTION_H
#define FEETECH_ACTION_H

#define NBR_PINCES 2

#define ABS_DIFF(a, b) ((a) > (b) ? ((a) - (b)) : ((b) - (a)))

// -------------------------------------------------- //
// -------------- GLOBAL USE  ----------------------- //
#define NBR_VALUES_FOR_MEAN 30

// ------------------------------------------ //
// -------------- defines pompes ------------ //
#define PUMP_ON 255
#define PUMP_OFF 0

#define CURRENT_THRESHOLD_ON_ALLUMAGE 100 // if current is above this threshold, we assume the pump is on wen starting it
#define CURRENT_THRESHOLD_ON_EXTINCTION 500 // if current is above this threshold, we assume the pump is on when trying to turn it off (we put a higher threshold here because it's possible that the pump is still stopping and consuming more current than usual, but if it's above this threshold, we can assume that the pump is not turning off correctly)

#define CURRENT_THRESHOLD_CATCH 2050 // if current is above this threshold, we assume an object is catched

#define CURRENT_VARIATION_CATCH 150

#define VALVE_ON 1

// ------------------------------------------ //
// ---------- define des positions ---------- //
// ------------------------------------------ //

// ---------------- PINCE 1 ----------------- //
#define PINCE_1_GROS_IDLE_POS 2600
#define PINCE_1_GROS_RAMASSER_POS 3500
#define PINCE_1_GROS_LACHER_POS 2300

#define PINCE_1_DROITE_SORTIE_POS 700
#define PINCE_1_DROITE_RETRAIT_POS 512

#define PINCE_1_GAUCHE_SORTIE_POS 300
#define PINCE_1_GAUCHE_RETRAIT_POS 512

// ---------------- PINCE 2 ----------------- //
#define PINCE_2_GROS_IDLE_POS 3580
#define PINCE_2_GROS_RAMASSER_POS 2400
#define PINCE_2_GROS_LACHER_POS 3700

#define PINCE_2_DROITE_SORTIE_POS 700
#define PINCE_2_DROITE_RETRAIT_POS 512

#define PINCE_2_GAUCHE_SORTIE_POS 300
#define PINCE_2_GAUCHE_RETRAIT_POS 512


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
#endif // FEETECH_ACTION_H