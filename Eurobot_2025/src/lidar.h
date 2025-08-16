// ---------------------
// buffer and dma parameters
// ---------------------
#define DMA_DEV_ID      XPAR_AXIDMA_0_DEVICE_ID
#define RX_BUFFER_SIZE  48 // Taille en bytes (ex : 12 points * 4 bytes)

#define NUM_BUFFERS        2 // 
#define DMA_ALIGN          32

#define DMA_TIMEOUT        100 // Timeout en ms


// ---------- Format des points ----------
#define BYTES_PER_POINT    4    // [31:16]=dist, [15:8]=angle8, [7:0]=intensité
#define POINTS_PER_FRAME   12   // 12 points per frame
#define FRAME_BYTES        (POINTS_PER_FRAME * BYTES_PER_POINT)

extern u8 RxBuf[NUM_BUFFERS][FRAME_BYTES] __attribute__ ((aligned(DMA_ALIGN)));

typedef struct {
    uint16_t dist_mm; // [31:16]
    uint8_t angle_deg; // [15:8]
    uint8_t intensity; // [7:0]
} LidarPoint;

int init_dma(void);

int dma_recv_frame_blocking(u8 *dst, u32 len_bytes);


void dump_frame(const u8 *buf, u32 len);

int lidar_dma_recv(void *dst, u32 len_bytes);