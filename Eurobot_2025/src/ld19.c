#include "main.h"


uint8_t CalCRC8(uint8_t *p, uint8_t len){ 
    uint8_t crc = 0; 
    uint16_t i; 
    for (i = 0; i < len; i++){ 
        crc = CrcTable[(crc ^ *p++) & 0xff]; 
    } 
    return crc;
}

void initLD19Instance(LD19Instance *instance) {
    initLiDARFrame(&instance->frame);
    initLiDARFilter(&instance->filter);
    instance->data_handler.index = 0;

    instance->en_crc_check = 1;
    instance->en_full_scan = 1;
    instance->en_filtering = 1;
    instance->set_upside_down = 0;
}

void initLiDARFrame(LD19PacketHandler *frame) {
    frame->packet.header = LD19_HEADER;
    frame->packet.ver_len = sizeof(LD19PacketHandler);
    frame->packet.speed = 0;
    frame->packet.start_angle = 0;
    for (int i = 0; i < LD19_PACKET_SIZE; i++) {
        frame->packet.point[i].distance = 0;
        frame->packet.point[i].intensity = 0;
    }
    frame->packet.end_angle = 0;
    frame->packet.timestamp = 0;
    frame->packet.crc8 = 0;
}

void initLiDARFilter(LD19Filter *filter) {
    filter->min_distance = 0;
    filter->max_distance = 12000; // 12m
    filter->min_angle = 0.0f;
    filter->max_angle = 360.0f;
    filter->intensity_threshold = 0;
}

void setIntensityThreshold(LD19Filter *filter, uint8_t threshold) {
    filter->intensity_threshold = threshold;
}

void setDistanceRange(LD19Filter *filter, uint16_t min_distance, uint16_t max_distance) {
    filter->min_distance = min_distance;
    filter->max_distance = max_distance;
}

void setMinDistance(LD19Filter *filter, uint16_t min_distance) {
    filter->min_distance = min_distance;
}

void setMaxDistance(LD19Filter *filter, uint16_t max_distance) {
    filter->max_distance = max_distance;
}

void setMinAngle(LD19Filter *filter, float min_angle) {
    filter->min_angle = min_angle;
}

void setMaxAngle(LD19Filter *filter, float max_angle) {
    filter->max_angle = max_angle;
}

void setAngleRange(LD19Filter *filter, float min_angle, float max_angle) {
    filter->min_angle = min_angle;
    filter->max_angle = max_angle;
}

void enableCRCCheck(LD19Instance *instance) {
    instance->en_crc_check = 1;
}

void disableCRCCheck(LD19Instance *instance) {
    instance->en_crc_check = 0;
}

void enableFullScan(LD19Instance *instance) {
    instance->en_full_scan = 1;
}

void disableFullScan(LD19Instance *instance) {
    instance->en_full_scan = 0;
}

void enableFiltering(LD19Instance *instance) {
    instance->en_filtering = 1;
}

void disableFiltering(LD19Instance *instance) {
    instance->en_filtering = 0;
}

void turnUpsideDown(LD19Instance *instance) {
    instance->set_upside_down = 1;
}

void turnRightSideUp(LD19Instance *instance) {
    instance->set_upside_down = 0;
}


uint8_t ld19_readData(LD19Instance *instance, XUartLite *UartLite) {
    return instance->en_crc_check ? ld19_readDataCRC(instance, UartLite) : ld19_readDataNoCRC(instance, UartLite);
}

uint8_t ld19_readDataCRC(LD19Instance *instance, XUartLite *UartLite) {
    // Read data from the LiDAR sensor and process it
    uint8_t current, result = 0;
    if (XUartLite_Recv(UartLite, &current, 1)) {
        // Process the received data
        if((instance->receivedData.index > 1) || 
                    (instance->receivedData.index == 0 && current == LD19_HEADER) ||
                    (instance->receivedData.index == 1 && current == LD19_VER_SIZE)) {
                        
            instance->receivedData.packet.bytes[instance->receivedData.index] = current;
            if(instance->receivedData.index < LD19_PACKET_SIZE - 1) { 
                instance->receivedData.computedCRC = CrcTable[instance->receivedData.computedCRC ^ current];
                instance->receivedData.index++;
            }else{
                if(instance->receivedData.computedCRC == current) {
                    ld19_computeData(instance);
                    result = 1;
                }else{
                    instance->receivedData.checksumFailCount++;
                }
                instance->receivedData.index = 0;
                instance->receivedData.computedCRC = 0;
            }
        }
    }
    return result;
}

uint8_t ld19_readDataNoCRC(LD19Instance *instance, XUartLite *UartLite) {
    // Read data from the LiDAR sensor without CRC check
    uint8_t current, result = 0;
    if (XUartLite_Recv(UartLite, &current, 1)) {
        // Process the received data
        if((instance->receivedData.index > 1) || 
                    (instance->receivedData.index == 0 && current == LD19_HEADER) ||
                    (instance->receivedData.index == 1 && current == LD19_VER_SIZE)) {

            instance->receivedData.packet.bytes[instance->receivedData.index] = current;
            if(instance->receivedData.index < LD19_PACKET_SIZE - 1) {
                instance->receivedData.index++;
            }else{
                ld19_computeData(instance);
                result = 1;
                instance->receivedData.index = 0;
            }
        }
    }
    return result;
}

void ld19_readscan(LD19Instance *instance, XUartLite *UartLite) {
    // TODO write function to build a full scan from the received packets
}

void ld19_computeData(LD19Instance *instance) {
}
