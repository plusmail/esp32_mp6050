#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;
#define OUTPUT_TEAPOT

#define INTERRUPT_PIN 2
#define LED_PIN 2

// DMP 변수
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// 자세 변수
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Teapot 패킷 (Processing 전송용)
uint8_t teapotPacket[14] = {
    '$', 0x02,
    0, 0, // q0
    0, 0, // q1
    0, 0, // q2
    0, 0, // q3
    0x00, 0x00, '\r', '\n'};

volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady()
{
  mpuInterrupt = true;
}
void setup() {
    Wire.begin(21, 22);
    Wire.setClock(400000);
    Serial.begin(115200);
    delay(2000);   // 부팅 안정화 대기

    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 연결 실패!");
        while (1);
    }
    Serial.println("MPU6050 연결 성공");

    // ↓ 이 부분 전체 삭제!
    // Serial.println("아무 키나 입력하세요...");
    // while (!Serial.available());
    // while (Serial.available() && Serial.read());

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN),
                       dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("DMP 준비 완료!");
    }
}

void loop()
{
  if (!dmpReady)
    return;

  // 인터럽트 대기
  while (!mpuInterrupt && fifoCount < packetSize)
    ;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // FIFO 오버플로우 처리
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    return;
  }

  if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Teapot 패킷으로 Processing에 전송
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++;

    // YPR도 디버그용으로 출력 (선택)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}