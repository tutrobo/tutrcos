#include "main.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#include "tutrcos/peripheral/fdcan.hpp"

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t) {
  static FDCAN_RxHeaderTypeDef rx_header;
  static tutrcos::peripheral::CANMessage msg;

  auto itr = tutrcos::peripheral::FDCAN::get_instances().find(hfdcan);
  if (itr != tutrcos::peripheral::FDCAN::get_instances().end()) {
    auto fdcan = itr->second;
    for (size_t i = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0); i > 0;
         --i) {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header,
                                 msg.data.data()) != HAL_OK) {
        return;
      }

      msg.id = rx_header.Identifier;
      switch (rx_header.IdType) {
      case FDCAN_STANDARD_ID:
        msg.id_type = tutrcos::peripheral::CANIDType::STANDARD;
        break;
      case FDCAN_EXTENDED_ID:
        msg.id_type = tutrcos::peripheral::CANIDType::EXTENDED;
        break;
      }
      switch (rx_header.DataLength) {
      case FDCAN_DLC_BYTES_0:
        msg.dlc = 0;
        break;
      case FDCAN_DLC_BYTES_1:
        msg.dlc = 1;
        break;
      case FDCAN_DLC_BYTES_2:
        msg.dlc = 2;
        break;
      case FDCAN_DLC_BYTES_3:
        msg.dlc = 3;
        break;
      case FDCAN_DLC_BYTES_4:
        msg.dlc = 4;
        break;
      case FDCAN_DLC_BYTES_5:
        msg.dlc = 5;
        break;
      case FDCAN_DLC_BYTES_6:
        msg.dlc = 6;
        break;
      case FDCAN_DLC_BYTES_7:
        msg.dlc = 7;
        break;
      case FDCAN_DLC_BYTES_8:
        msg.dlc = 8;
        break;
      }

      fdcan->rx_queue_.push(msg, 0);
    }
  }
}

#endif
