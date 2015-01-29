#ifndef LIDAR_H
#define LIDAR_H

#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "error.h"

typedef struct {
  uint16_t distance_mm:14;
  uint8_t strengthWarning:1;
  uint8_t invalidData:1;
  uint16_t strength:16;
} lidar_reading_t;

typedef struct {
  uint8_t startByte;
  uint8_t index;
  uint16_t rpm_10q6;
  lidar_reading_t readings[4];
  uint16_t checksum;
} lidar_frame_t;

void *handleSerial(void* args);

class Lidar {
 public:
  Lidar() {
    pthread_create(&serialHandler, NULL, handleSerial, this);
  }
  ~Lidar() {
    running = false;
    pthread_join(serialHandler, NULL);
  }

  uint32_t getSample(int i) {
    rassert(0 <= i && i < 360);
    return lidarSamples[i];
  }

  void processByte(uint8_t byte);

  bool running = true;

  enum lidarState_t {
    lidarState_lookingForStart,
    lidarState_receivingFrames
  };

 private:
  uint32_t lidarSamples[360] = {0xFFFFFFFF};

  uint8_t frameBuf[22] = {0};
  uint8_t frameIdx = 0; 

  lidarState_t state = lidarState_lookingForStart;

  pthread_t serialHandler;

  lidarState_t lookingForStart(uint8_t b);
  lidarState_t receiveFrames(uint8_t b);
  void processFrame(uint8_t *buf);
  void processLidarFrame(lidar_frame_t *frame);
};

#endif

