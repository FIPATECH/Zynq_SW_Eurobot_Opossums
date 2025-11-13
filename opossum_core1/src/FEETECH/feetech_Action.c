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
            if (ID_test < 255) {
                GetFEETECH_Ext_Done(ID_test, FEETECH_ID, &ID_return, &done_ID);
                printf("testing ID %d\n", ID_test);
                t0 = Timer_ms1;          // remember start time
                etat_ID = 2;             // wait for reply or timeout
            } else {
                printf("done research\n");
                etat_ID = 0;
            }
            break;

        case 2:
            // wait for reply
            if (done_ID) {
                done_ID = 0;
                if (ID_return != 0) {
                    printf("ID return: %d\n", ID_return);
                }
                ID_return = 0;
                ID_test++;
                etat_ID = 1;
            }
            // or timeout (no reply)
            else if ((Timer_ms1 - t0) > 10) { // 10 ms timeout per ID
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

