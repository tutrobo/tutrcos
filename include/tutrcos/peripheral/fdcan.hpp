#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>
#include <map>

#include "tutrcos/core.hpp"
#include "tutrcos/utility.hpp"

#include "can_base.hpp"

namespace tutrcos {
namespace peripheral {

class FDCAN : public CANBase {
public:
  FDCAN(FDCAN_HandleTypeDef *hfdcan, size_t rx_queue_size = 64)
      : hfdcan_{hfdcan}, rx_queue_{rx_queue_size} {
    get_instances()[hfdcan_] = this;

    TUTRCOS_ASSERT(HAL_FDCAN_ActivateNotification(
                       hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK);
    TUTRCOS_ASSERT(HAL_FDCAN_Start(hfdcan_) == HAL_OK);
  }

  ~FDCAN() {
    get_instances().erase(hfdcan_);
    HAL_FDCAN_Stop(hfdcan_);
  }

  bool transmit(const CANMessage &msg, uint32_t timeout) override {
    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.Identifier = msg.id;
    switch (msg.id_type) {
    case CANIDType::STANDARD:
      tx_header.IdType = FDCAN_STANDARD_ID;
      break;
    case CANIDType::EXTENDED:
      tx_header.IdType = FDCAN_EXTENDED_ID;
      break;
    }
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    switch (msg.dlc) {
    case 0:
      tx_header.DataLength = FDCAN_DLC_BYTES_0;
      break;
    case 1:
      tx_header.DataLength = FDCAN_DLC_BYTES_1;
      break;
    case 2:
      tx_header.DataLength = FDCAN_DLC_BYTES_2;
      break;
    case 3:
      tx_header.DataLength = FDCAN_DLC_BYTES_3;
      break;
    case 4:
      tx_header.DataLength = FDCAN_DLC_BYTES_4;
      break;
    case 5:
      tx_header.DataLength = FDCAN_DLC_BYTES_5;
      break;
    case 6:
      tx_header.DataLength = FDCAN_DLC_BYTES_6;
      break;
    case 7:
      tx_header.DataLength = FDCAN_DLC_BYTES_7;
      break;
    case 8:
      tx_header.DataLength = FDCAN_DLC_BYTES_8;
      break;
    }
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    uint32_t start = core::Kernel::get_ticks();
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan_) == 0) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header,
                                         msg.data.data()) == HAL_OK;
  }

  bool receive(CANMessage &msg, uint32_t timeout) override {
    return rx_queue_.pop(msg, timeout);
  }

private:
  FDCAN_HandleTypeDef *hfdcan_;
  core::Queue<CANMessage> rx_queue_;

  static inline std::map<FDCAN_HandleTypeDef *, FDCAN *> &get_instances() {
    static std::map<FDCAN_HandleTypeDef *, FDCAN *> instances;
    return instances;
  }

  friend void ::HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs);
};

} // namespace peripheral
} // namespace tutrcos
