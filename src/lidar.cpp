#include "lidar.h"

#include <stdio.h>

#include "uart.h"

#define START 0xFA
#define FRAME_LENGTH 22

void *handleSerial(void* args) {
  Lidar *obj = (Lidar*)args;
  int32_t fd = uart_init();
  while (obj->running) {
    uint8_t buf;
    while (read(fd, &buf, 1) > 0) {
      obj->processByte(buf);
    }
    usleep(10000);
  }
  return nullptr;
}

namespace {

uint16_t computeChecksum(uint16_t *buf) {
  uint32_t checksum = 0;
  uint8_t i = 0;
  for (i = 0; i < FRAME_LENGTH/2 - 1; i++) {
    checksum = (checksum << 1) + buf[i];
  }
  checksum = (checksum & 0x7FFF) + (checksum >> 15);
  checksum = checksum & 0x7FFF;
  return checksum;
}

}  // anonymous namespace

Lidar::lidarState_t Lidar::lookingForStart(uint8_t b) {
  if (b == START && frameIdx == 0) {
    frameBuf[frameIdx++] = b;
  } else if (frameIdx > 0) {
    frameBuf[frameIdx++] = b;
  }

  if (frameIdx == FRAME_LENGTH) {
    uint16_t checksum = computeChecksum((uint16_t *)frameBuf);
    uint16_t rxChecksum = (frameBuf[20] & 0xFF) | ((frameBuf[21] & 0xFF) << 8);
    
    printf("checksum: %0.4x\r\n", checksum);
    printf("rx checksum: %0.4x\r\n", rxChecksum);
    
    frameIdx = 0;
    if (checksum == rxChecksum) {
      return Lidar::lidarState_receivingFrames;
      printf("Found Start!!");
    }
  }
  return Lidar::lidarState_lookingForStart;
}

void Lidar::processFrame(uint8_t *buf) {
 lidar_frame_t *frame = (lidar_frame_t *)buf;
 processLidarFrame(frame);
}

Lidar::lidarState_t Lidar::receiveFrames(uint8_t b) {
  frameBuf[frameIdx++] = b;
  if (frameIdx == FRAME_LENGTH) {
    uint16_t checksum = computeChecksum((uint16_t *)frameBuf);
    uint16_t rxChecksum = (frameBuf[FRAME_LENGTH - 2] & 0xFF) | 
                          ((frameBuf[FRAME_LENGTH - 1] & 0xFF) << 8);
    frameIdx = 0;
    if (checksum != rxChecksum) {
      return Lidar::lidarState_lookingForStart;
    }
    processFrame(frameBuf);
  }
  return Lidar::lidarState_receivingFrames;
}

void Lidar::processByte(uint8_t b) {
  switch (state) {
    case lidarState_lookingForStart:
      state = lookingForStart(b);
      break;
    case lidarState_receivingFrames:
      state = receiveFrames(b);     
      break;
  }
}


void Lidar::processLidarFrame(lidar_frame_t *frame) {
  uint16_t degreeStart = (frame->index - 0xA0) * 4;
  lidar_reading_t tmp;
  for (uint8_t i = 0; i < 4; i++) {
    tmp = frame->readings[i];
    uint32_t *bucket = &(lidarSamples[degreeStart + i]);
    if (tmp.invalidData) {
      *bucket = 0xFFFFFFFF;
    } else {
      *bucket = tmp.distance_mm;
    }
  }

  if (frame->index == 0xF9) {
    printf("*****START*****\r\n");
    for (uint16_t i = 0; i < 360; i++) {
        printf("%4d ", lidarSamples[i]);
        if (i % 20 == 19) {
          printf("\r\n");
        }
    }
    printf("*****END*****\r\n");
  }
}
