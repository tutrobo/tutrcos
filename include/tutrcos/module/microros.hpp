#pragma once

#include <cstddef>

#include <rcl/rcl.h>
#include <rmw_microros/rmw_microros.h>

#include "tutrcos/peripheral/uart.hpp"

extern "C" {

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void *state);

bool tutrcos_module_MicroROS_open_cb(uxrCustomTransport *transport);
bool tutrcos_module_MicroROS_close_cb(uxrCustomTransport *transport);
size_t tutrcos_module_MicroROS_write_cb(uxrCustomTransport *transport,
                                        const uint8_t *buffer, size_t length,
                                        uint8_t *error_code);
size_t tutrcos_module_MicroROS_read_cb(uxrCustomTransport *transport,
                                       uint8_t *buffer, size_t length,
                                       int timeout, uint8_t *error_code);
}

namespace tutrcos {
namespace module {

class MicroROS {
public:
  static inline bool init(peripheral::UART &uart) {
    if (rmw_uros_set_custom_transport(
            true, &uart, tutrcos_module_MicroROS_open_cb,
            tutrcos_module_MicroROS_close_cb, tutrcos_module_MicroROS_write_cb,
            tutrcos_module_MicroROS_read_cb) != RMW_RET_OK) {
      return false;
    }

    rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate;
    allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;

    return rcutils_set_default_allocator(&allocator);
  };

  friend bool ::tutrcos_module_MicroROS_open_cb(uxrCustomTransport *transport);
  friend bool ::tutrcos_module_MicroROS_close_cb(uxrCustomTransport *transport);
  friend size_t ::tutrcos_module_MicroROS_write_cb(
      uxrCustomTransport *transport, const uint8_t *buffer, size_t length,
      uint8_t *error_code);
  friend size_t ::tutrcos_module_MicroROS_read_cb(uxrCustomTransport *transport,
                                                  uint8_t *buffer,
                                                  size_t length, int timeout,
                                                  uint8_t *error_code);
};

} // namespace module
} // namespace tutrcos
