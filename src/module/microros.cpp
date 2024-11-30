#if __has_include(<rmw_microxrcedds_c/config.h>)
#include <rmw_microxrcedds_c/config.h>
#endif

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

#include <uxr/client/transport.h>

#include "tutrcos/module/microros.hpp"

bool tutrcos_module_MicroROS_open_cb(uxrCustomTransport *) {
  // 何もしない
  return true;
}

bool tutrcos_module_MicroROS_close_cb(uxrCustomTransport *) {
  // 何もしない
  return true;
}

size_t tutrcos_module_MicroROS_write_cb(uxrCustomTransport *transport,
                                        const uint8_t *buffer, size_t length,
                                        uint8_t *) {
  auto uart = reinterpret_cast<tutrcos::peripheral::UART *>(transport->args);
  if (!uart->transmit(buffer, length, tutrcos::core::Kernel::MAX_DELAY)) {
    return 0;
  }
  return length;
}

size_t tutrcos_module_MicroROS_read_cb(uxrCustomTransport *transport,
                                       uint8_t *buffer, size_t length,
                                       int timeout, uint8_t *) {
  auto uart = reinterpret_cast<tutrcos::peripheral::UART *>(transport->args);
  if (!uart->receive(buffer, length, timeout)) {
    return 0;
  }
  return length;
}

#endif
