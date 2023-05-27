#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Arduino.h>
// #define CAN_TX_IDX TWAI_TX_IDX
// #define CAN_RX_IDX TWAI_RX_IDX


CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 100;          // interval at which send CAN Messages (milliseconds)
// const int rx_queue_size = 10;       // Receive Queue size


void setup() {
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");
  CAN_cfg.speed = CAN_SPEED_1000KBPS;//1000000
  CAN_cfg.tx_pin_id = GPIO_NUM_32;
  // CAN_cfg.rx_pin_id = GPIO_NUM_33;
  // CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
}

void loop() {

  // CAN_frame_t rx_frame;

  // unsigned long currentMillis = millis();

  // // Receive next CAN frame from queue
  // if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

  //   if (rx_frame.FIR.B.FF == CAN_frame_std) {
  //     printf("New standard frame");
  //   }
  //   else {
  //     printf("New extended frame");
  //   }

  //   if (rx_frame.FIR.B.RTR == CAN_RTR) {
  //     printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
  //   }
  //   else {
  //     printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
  //     for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
  //       printf("0x%02X ", rx_frame.data.u8[i]);
  //     }
  //     printf("\n");
  //   }
  // }
  // Send CAN Message

  Serial.printf("Init\n");
  // previousMillis = currentMillis;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x01;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0xFF;
  tx_frame.data.u8[1] = 0xFF;
  tx_frame.data.u8[2] = 0xFF;
  tx_frame.data.u8[3] = 0xFF;
  tx_frame.data.u8[4] = 0xFF;
  tx_frame.data.u8[5] = 0xFF;
  tx_frame.data.u8[6] = 0xFF;
  tx_frame.data.u8[7] = 0xFC;
  // 7F FF FF F0 00 33 37 FF
  ESP32Can.CANWriteFrame(&tx_frame);
  Serial.printf("InitSent!\n");
  delay(1000);

  while(0)
  {
    Serial.printf("Writing\n");
    previousMillis = currentMillis;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x001;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 0x7F;
    tx_frame.data.u8[1] = 0xFF;
    tx_frame.data.u8[2] = 0xFF;
    tx_frame.data.u8[3] = 0xF0;
    tx_frame.data.u8[4] = 0x00;
    tx_frame.data.u8[5] = 0x33;
    tx_frame.data.u8[6] = 0x37;
    tx_frame.data.u8[7] = 0xFF;
    // 7F FF FF F0 00 33 37 FF
    ESP32Can.CANWriteFrame(&tx_frame);
    Serial.printf("Sent!\n");
    delay(1000);
  }
}