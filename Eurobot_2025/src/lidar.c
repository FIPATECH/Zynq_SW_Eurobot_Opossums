#include "main.h"


XAxiDma AxiDma;                  // Instance DMA
u8 RxBuf[NUM_BUFFERS][FRAME_BYTES] __attribute__ ((aligned(DMA_ALIGN)));


int DMA_Timer = 0;


// ---------------------
// Init DMA pour réception Lidar
// ---------------------
int init_dma(void) {
    XAxiDma_Config *CfgPtr;
    int Status;

    // Récupérer la config DMA
    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!CfgPtr) {
        xil_printf("Erreur: Impossible de trouver la config DMA\r\n");
        return XST_FAILURE;
    }

    // Initialiser DMA
    Status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
    if (Status != XST_SUCCESS) {
        xil_printf("Erreur: Init DMA échouée\r\n");
        return XST_FAILURE;
    }

    // Vérifier que le DMA est bien en mode Scatter-Gather désactivé
    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("Erreur: DMA configuré en mode SG, attendu mode simple\r\n");
        return XST_FAILURE;
    }

    // Reset du DMA
    XAxiDma_Reset(&AxiDma);
    while (!XAxiDma_ResetIsDone(&AxiDma));

    xil_printf("DMA initialisé avec succès\r\n");
    return XST_SUCCESS;
}

int dma_recv_frame_blocking(u8 *dst, u32 len_bytes)
{
    int status;

    // Invalider le cache de la zone destination avant que le DMA écrive
    Xil_DCacheInvalidateRange((UINTPTR)dst, len_bytes);

    // Lancer la réception S2MM
    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)dst, len_bytes, XAXIDMA_DEVICE_TO_DMA);
    if (status != XST_SUCCESS) {
        // xil_printf("S2MM SimpleTransfer failed \r\n");
        return status;
    }

    DMA_Timer = Timer_ms1; // Enregistrer le temps de début
    // Attendre que S2MM termine (poll sur Idle)
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        if (Timer_ms1 - DMA_Timer > DMA_TIMEOUT) {
            xil_printf("DMA receive timeout\r\n");
            return XST_FAILURE;
        }
    }

    // À ce stade, dst contient len_bytes de données de la trame
    // Invalider à nouveau par prudence si le cache a été préchargé
    Xil_DCacheInvalidateRange((UINTPTR)dst, len_bytes);

    return XST_SUCCESS;
}


void dump_frame(const u8 *buf, u32 len)
{
    const LidarPoint *p = (const LidarPoint *)buf;
    const int n = len / BYTES_PER_POINT;

    xil_printf("Frame %d points\r\n", n);
    for (int i = 0; i < n; i++) {
        xil_printf("  #%02d  dist=%4u mm  ang8=%3u  I=%3u\r\n",
            i, (unsigned)p[i].dist_mm, (unsigned)p[i].angle_deg, (unsigned)p[i].intensity);
    }
}

int lidar_dma_recv(void *dst, u32 len_bytes) {
    int status;

    // Démarre transfert S2MM
    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)dst, len_bytes, XAXIDMA_DEVICE_TO_DMA);
    if (status != XST_SUCCESS) {
        xil_printf("Erreur : echec transfert S2MM\r\n");
        return XST_FAILURE;
    }

    // Attendre fin transfert
    DMA_Timer = Timer_ms1; // Enregistrer le temps de début
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        if (Timer_ms1 - DMA_Timer > DMA_TIMEOUT) {
            xil_printf("DMA receive timeout\r\n");
            return XST_FAILURE;
        }
    }

    return XST_SUCCESS;
}