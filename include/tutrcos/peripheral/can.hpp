#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>

#include "tutrcos/core.hpp"
#include "tutrcos/utility.hpp"

#include "can_base.hpp"
#include "instance_table.hpp"

namespace tutrcos {
namespace peripheral {

class CAN : public CANBase {
public:
  CAN(CAN_HandleTypeDef *hcan, size_t rx_queue_size = 64)
      : hcan_{hcan}, rx_queue_{rx_queue_size} {
    get_instances().set(hcan_, this);

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

    TUTRCOS_VERIFY(HAL_CAN_ConfigFilter(hcan_, &filter) == HAL_OK);
    TUTRCOS_VERIFY(HAL_CAN_ActivateNotification(
                       hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
    TUTRCOS_VERIFY(HAL_CAN_Start(hcan_) == HAL_OK);
  }

  ~CAN() {
    TUTRCOS_VERIFY(HAL_CAN_Stop(hcan_) == HAL_OK);
    get_instances().erase(hcan_);
  }

  bool transmit(const Message &msg, uint32_t timeout) override {
    CAN_TxHeaderTypeDef tx_header{};
    switch (msg.id_type) {
    case IDType::STANDARD:
      tx_header.StdId = msg.id;
      tx_header.IDE = CAN_ID_STD;
      break;
    case IDType::EXTENDED:
      tx_header.ExtId = msg.id;
      tx_header.IDE = CAN_ID_EXT;
      break;
    }
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = msg.dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox;

    uint32_t start = core::Kernel::get_ticks();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }
    return HAL_CAN_AddTxMessage(hcan_, &tx_header, msg.data.data(),
                                &tx_mailbox) == HAL_OK;
  }

  bool receive(Message &msg, uint32_t timeout) override {
    return rx_queue_.pop(msg, timeout);
  }

  CAN_HandleTypeDef *get_hal_handle() { return hcan_; }

private:
  CAN_HandleTypeDef *hcan_;
  core::Queue<Message> rx_queue_;

  static inline InstanceTable<CAN_HandleTypeDef *, CAN, 32> &get_instances() {
    static InstanceTable<CAN_HandleTypeDef *, CAN, 32> instances;
    return instances;
  }

  friend void ::HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
};

} // namespace peripheral
} // namespace tutrcos
