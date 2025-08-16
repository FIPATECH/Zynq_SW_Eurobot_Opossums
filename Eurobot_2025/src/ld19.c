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
}

void initLiDARFrame(LiDARFrameTypeDef *frame) {
    frame->header = HEADER;
    frame->ver_len = sizeof(LiDARFrameTypeDef);
    frame->speed = 0;
    frame->start_angle = 0;
    for (int i = 0; i < LD19_POINT_PER_PACK; i++) {
        frame->point[i].distance = 0;
        frame->point[i].intensity = 0;
    }
    frame->end_angle = 0;
    frame->timestamp = 0;
    frame->crc8 = 0;
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



