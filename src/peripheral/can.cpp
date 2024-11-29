#include "main.h"

#ifdef HAL_CAN_MODULE_ENABLED

#include "tutrcos/peripheral/can.hpp"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  static CAN_RxHeaderTypeDef rx_header;
  static tutrcos::peripheral::CANBase::Message msg;

  auto itr = tutrcos::peripheral::CAN::get_instances().find(hcan);
  if (itr != tutrcos::peripheral::CAN::get_instances().end()) {
    auto can = itr->second;
    for (size_t i = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0); i > 0;
         --i) {
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
                               msg.data.data()) != HAL_OK) {
        return;
      }

      switch (rx_header.IDE) {
      case CAN_ID_STD:
        msg.id_type = tutrcos::peripheral::CANBase::IDType::STANDARD;
        msg.id = rx_header.StdId;
        break;
      case CAN_ID_EXT:
        msg.id_type = tutrcos::peripheral::CANBase::IDType::EXTENDED;
        msg.id = rx_header.ExtId;
        break;
      }
      msg.dlc = rx_header.DLC;

      can->rx_queue_.push(msg, 0);
    }
  }
}

#endif
