#pragma once

#include <cstddef>

#include <rcl/rcl.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <uxr/client/transport.h>

#include "tutrcos/peripheral/uart.hpp"

extern "C" {

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void *state);
}

namespace tutrcos {
namespace module {

class MicroROS {
public:
  static inline bool init(peripheral::UART &uart) {
    if (rmw_uros_set_custom_transport(true, &uart, open_cb, close_cb, write_cb,
                                      read_cb) != RMW_RET_OK) {
      return false;
    }

    rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate;
    allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;

    return rcutils_set_default_allocator(&allocator);
  };

private:
  static inline bool open_cb(uxrCustomTransport *) {
    // 何もしない
    return true;
  }

  static inline bool close_cb(uxrCustomTransport *) {
    // 何もしない
    return true;
  }

  static inline size_t write_cb(uxrCustomTransport *transport,
                                const uint8_t *buffer, size_t length,
                                uint8_t *) {
    auto uart = reinterpret_cast<tutrcos::peripheral::UART *>(transport->args);
    if (!uart->transmit(buffer, length, tutrcos::core::Kernel::MAX_DELAY)) {
      return 0;
    }
    return length;
  }

  static inline size_t read_cb(uxrCustomTransport *transport, uint8_t *buffer,
                               size_t length, int timeout, uint8_t *) {
    auto uart = reinterpret_cast<tutrcos::peripheral::UART *>(transport->args);
    if (!uart->receive(buffer, length, timeout)) {
      return 0;
    }
    return length;
  }
};

} // namespace module
} // namespace tutrcos
