#ifndef FEETECH_ACTION_H
#define FEETECH_ACTION_H

#define NBR_PINCES 2

typedef enum {
    CMD_IDLE = 0,
    CMD_RAMASSER,
    CMD_LACHER
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
} Pump_t;


typedef struct {
    // -- ID matériel -- //
    uint8_t id_gros;
    uint8_t id_droite;
    uint8_t id_gauche;
    uint8_t id_pump;

    // -- variables d'états fsm -- //
    uint8_t action_step;
    uint8_t action_done;
    uint32_t action_timer;
    uint32_t action_position;

    // -- positions -- //
    Gros_Servo_Pos_t gros_pos;
    Petit_Servo_Pos_t petit_droite_pos;
    Petit_Servo_Pos_t petit_gauche_pos;

    Pump_t pump_right;
    Pump_t pump_left;

    // -- consignes -- //
    Pince_Command_t current_command;
} Pince_t;

// ------------------------------------------ //
// -------------- defines pompes ------------ //
#define PUMP_ON 255
#define PUMP_OFF 0


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
#define PINCE_2_GROS_IDLE_POS 3200
#define PINCE_2_GROS_RAMASSER_POS 2300
#define PINCE_2_GROS_LACHER_POS 3500

#define PINCE_2_DROITE_SORTIE_POS 700
#define PINCE_2_DROITE_RETRAIT_POS 512

#define PINCE_2_GAUCHE_SORTIE_POS 300
#define PINCE_2_GAUCHE_RETRAIT_POS 512



uint8_t Send_FEETECH_Cmd(void);
uint8_t Get_FEETECH_Cmd(void);

uint8_t Send_FEETECH_SCS_Cmd(void);
uint8_t Get_FEETECH_SCS_Cmd(void);

void FEETECH_Search_ID_Loop(void);
uint8_t Test_ID_FEETECH_Cmd(void);

void FEETECH_action_loop(void);

void pince_action_loop(Pince_t *pince);
uint8_t Monter_pince_cmd(void);
uint8_t Baisser_pince_cmd(void);
uint8_t Allumer_pompes_cmd(void);
uint8_t Eteindre_pompes_cmd(void);
uint8_t Activate_Valves_cmd(void);
uint8_t Ouvrir_clapet_cmd(void);

void Test_pince_action_loop(void);
uint8_t Test_pince_cmd(void);

void pince_loop(void);
void Init_Pinces_Loop(void);
#endif // FEETECH_ACTION_H