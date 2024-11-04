#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>
#include <map>

#include "tutrcos/core.hpp"

#include "can_base.hpp"

namespace tutrcos {
namespace peripheral {

class CAN : public CANBase {
public:
  CAN(CAN_HandleTypeDef *hcan, size_t rx_queue_size = 64)
      : hcan_{hcan}, rx_queue_{rx_queue_size} {
    get_instances()[hcan_] = this;

    CAN_FilterTypeDef filter{};
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank = 0;
#ifdef CAN2
    if (hcan_->Instance == CAN2) {
      filter.FilterBank = 14;
    }
#endif
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan_, &filter) != HAL_OK) {
      Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) !=
        HAL_OK) {
      Error_Handler();
    }

    if (HAL_CAN_Start(hcan_) != HAL_OK) {
      Error_Handler();
    }
  }

  ~CAN() { HAL_CAN_Stop(hcan_); }

  bool transmit(const CANMessage &msg) override {
    CAN_TxHeaderTypeDef tx_header{};
    switch (msg.id_type) {
    case CANIDType::STANDARD:
      tx_header.StdId = msg.id;
      tx_header.IDE = CAN_ID_STD;
      break;
    case CANIDType::EXTENDED:
      tx_header.ExtId = msg.id;
      tx_header.IDE = CAN_ID_EXT;
      break;
    }
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = msg.dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox;

    return HAL_CAN_AddTxMessage(hcan_, &tx_header, msg.data.data(),
                                &tx_mailbox) == HAL_OK;
  }

  bool receive(CANMessage &msg, uint32_t timeout) override {
    return rx_queue_.pop(msg, timeout);
  }

  size_t available() override { return rx_queue_.size(); }

private:
  CAN_HandleTypeDef *hcan_;
  core::Queue<CANMessage> rx_queue_;

  static inline std::map<CAN_HandleTypeDef *, CAN *> &get_instances() {
    static std::map<CAN_HandleTypeDef *, CAN *> instances;
    return instances;
  }

  friend void ::HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
};

} // namespace peripheral
} // namespace tutrcos