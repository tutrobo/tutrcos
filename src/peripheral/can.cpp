#include "main.h"

#ifdef HAL_CAN_MODULE_ENABLED

#include "tutrcos/peripheral/can.hpp"

using tutrcos::peripheral::CAN;
using tutrcos::peripheral::CANBase;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  static CAN_RxHeaderTypeDef rx_header{};
  static CANBase::Message msg{};

  for (CAN *can : CAN::get_instances()) {
    if (can->hcan_ == hcan) {
      for (size_t i = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0); i > 0;
           --i) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
                                 msg.data.data()) != HAL_OK) {
          break;
        }

        switch (rx_header.IDE) {
        case CAN_ID_STD:
          msg.id_type = CANBase::IDType::STANDARD;
          msg.id = rx_header.StdId;
          break;
        case CAN_ID_EXT:
          msg.id_type = CANBase::IDType::EXTENDED;
          msg.id = rx_header.ExtId;
          break;
        }
        msg.dlc = rx_header.DLC;

        can->rx_queue_.push(msg, 0);
      }
      break;
    }
  }
}

#endif
