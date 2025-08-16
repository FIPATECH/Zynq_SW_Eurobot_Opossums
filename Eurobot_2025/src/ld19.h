#ifndef LD19_H
#define LD19_H

#define LD19_HEADER 0x54
#define LD19_VER_SIZE 0x2C

#define LD19_PTS_PER_PACKETS 12
#define LD19_PACKET_SIZE 47 // 1(header) + 1(ver_len) + 2(speed) + 2(start_angle) + 12*3(point) + 2(end_angle) + 2(timestamp) + 1(crc8)
#define LD19_DATA_SIZE 3

#define LD19_MAX_PTS_SCAN 1200 

#define LD19_ANGLE_STEP_MAX 5

typedef struct { 
    uint16_t distance; 
    uint8_t intensity; 
} LidarPointStructDef; 

typedef struct { 
    uint8_t  header; 
    uint8_t  ver_len; 
    uint16_t speed; 
    uint16_t startAngle; 
    LidarPointStructDef point[LD19_PTS_PER_PACKETS]; 
    float angles[LD19_PTS_PER_PACKETS];
    uint16_t lastAngle; 
    uint16_t timestamp; 
    uint8_t crc8; 
    uint8_t bytes[LD19_PACKET_SIZE]
} LD19Packet;

typedef struct {
    uint16_t distance; //mm
    float angle; //int16_t
    int x;
    int y;
    uint8_t intensity; 
} DataPoint; 

typedef struct  {
  DataPoint points[LD19_MAX_PTS_SCAN];
  uint16_t index;
} LD19DataPointHandler;

typedef struct {
    LD19Packet packet;
    uint8_t computedCRC;
    uint8_t index;

    uint8_t checksumFailCount; // Count of checksum failures
} LD19PacketHandler;


typedef struct {
    uint16_t min_distance;
    uint16_t max_distance;
    float min_angle;
    float max_angle;
    uint8_t intensity_threshold;
} LD19Filter;

typedef struct {
    LD19PacketHandler receivedData;
    LD19Filter filter;
    LD19DataPointHandler dataHandler;
    uint8_t en_crc_check; // Enable CRC check
    uint8_t en_full_scan; // return true only when a full 360° scan is available
    uint8_t en_filtering; // Enable filtering
    uint8_t set_upside_down; // Set upside down (1: upside down)

    LD19DataPointHandler scanA;
    LD19DataPointHandler scanB;
    LD19DataPointHandler *currentScan;
    LD19DataPointHandler *previousScan;
    uint8_t currentBuffer; // 0: scanA, 1: scanB

    int xPosition; // X position of the sensor in the robot 
    int yPosition; // Y position of the sensor in the robot 
    int xOffset; // X offset of the sensor in the robot
    int yOffset; // Y offset of the sensor in the robot
    float angularPosition; // Angular position of the sensor in the robot 
    float angularOffset; // Angular offset of the sensor in the robot 
}LD19Instance;

uint8_t CalCRC8(uint8_t *p, uint8_t len);

void initLD19Instance(LD19Instance *instance);
void initLiDARFrame(LD19PacketHandler *frame);
void initLiDARFilter(LD19Filter *filter);

void setIntensityThreshold(LD19Filter *filter, uint8_t threshold);
void setDistanceRange(LD19Filter *filter, uint16_t min_distance, uint16_t max_distance);
void setMinDistance(LD19Filter *filter, uint16_t min_distance);
void setMaxDistance(LD19Filter *filter, uint16_t max_distance);
void setAngleRange(LD19Filter *filter, float min_angle, float max_angle);
void setMinAngle(LD19Filter *filter, float min_angle);
void setMaxAngle(LD19Filter *filter, float max_angle);

void enableCRCCheck(LD19Instance *instance);
void disableCRCCheck(LD19Instance *instance);
void enableFullScan(LD19Instance *instance);
void disableFullScan(LD19Instance *instance);
void enableFiltering(LD19Instance *instance);
void disableFiltering(LD19Instance *instance);
void turnUpsideDown(LD19Instance *instance);
void turnRightSideUp(LD19Instance *instance);


uint8_t ld19_readData(LD19Instance *instance, XUartLite *UartLite);
uint8_t ld19_readDataCRC(LD19Instance *instance, XUartLite *UartLite);
uint8_t ld19_readDataNoCRC(LD19Instance *instance, XUartLite *UartLite);

void ld19_readscan(LD19Instance *instance, XUartLite *UartLite);
float ld19_getAngleStep(LD19Instance *instance);

void ld19_swapBuffers(LD19Instance *instance);

#endif // LD19_H
