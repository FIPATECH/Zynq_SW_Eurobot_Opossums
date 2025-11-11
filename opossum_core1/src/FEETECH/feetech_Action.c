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

//-------------------------------------------------------------------------------
// Fonctions Search ID return all id detected
//-------------------------------------------------------------------------------

uint8_t start_ID=0;
uint8_t etat_ID=0;
uint8_t done_ID=0;
uint16_t ID_return=0;
uint16_t ID_test=0;

/* Example baudrate table for FEETECH servos (you can adjust this list) */
static const uint32_t FEETECH_Baudrates[] = {
    38400, 57600, 76800, 115200, 128000, 250000, 50000, 1000000
};
#define FEETECH_BAUD_COUNT (sizeof(FEETECH_Baudrates)/sizeof(FEETECH_Baudrates[0]))

void FEETECH_Search_ID_Loop(void) {
    static uint8_t baud_index = 0; // index in baudrate table

    switch (etat_ID) {

        case 0:
            if (start_ID == 1) {
                start_ID = 0;
                ID_test = 0;
                ID_return = 0;
                baud_index = 0;

                /* set first baudrate */
                XUartPs_SetBaudRate(&Uart1_Instance, FEETECH_Baudrates[baud_index]);
                xil_printf("Start FEETECH ID scan at %lu bps\n", FEETECH_Baudrates[baud_index]);
                etat_ID = 1;
            }
            break;

        case 1:
            if (ID_test < 255) {
                /* send read command to test current ID at current baudrate */
                GetFEETECH_Ext_Done(ID_test, FEETECH_ID, &ID_return, &done_ID);
                xil_printf("Testing ID=%d at %lu bps\n", ID_test, FEETECH_Baudrates[baud_index]);
                etat_ID = 2;
            } else {
                /* Finished all IDs at current baudrate, move to next */
                baud_index++;
                if (baud_index < FEETECH_BAUD_COUNT) {
                    XUartPs_SetBaudRate(&Uart1_Instance, FEETECH_Baudrates[baud_index]);
                    xil_printf("Switch to baudrate %lu bps\n", FEETECH_Baudrates[baud_index]);
                    ID_test = 0;
                    etat_ID = 1;
                } else {
                    xil_printf("FEETECH search finished.\n");
                    baud_index = 0;
                    ID_test = 0;
                    ID_return = 0;
                    etat_ID = 0;
                }
            }
            break;

        case 2:
            if (done_ID) {
                done_ID = 0;
                if (ID_return != 0) {
                    xil_printf("Found servo: ID=%d at %lu bps\n",
                               ID_return, FEETECH_Baudrates[baud_index]);
                }
                ID_test++;
                ID_return = 0;
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

