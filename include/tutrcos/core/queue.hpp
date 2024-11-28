#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>

#include "cmsis_os2.h"

namespace tutrcos {
namespace core {

template <class T> class Queue {
private:
  struct Deleter {
    void operator()(osMessageQueueId_t queue_id) {
      osMessageQueueDelete(queue_id);
    }
  };

  using QueueId =
      std::unique_ptr<std::remove_pointer_t<osMessageQueueId_t>, Deleter>;

public:
  Queue(size_t capacity) {
    queue_id_ = QueueId{osMessageQueueNew(capacity, sizeof(T), nullptr)};
  }

  bool push(const T &value, uint32_t timeout) {
    return osMessageQueuePut(queue_id_.get(), &value, 0, timeout) == osOK;
  }

  bool pop(T &value, uint32_t timeout) {
    return osMessageQueueGet(queue_id_.get(), &value, nullptr, timeout) == osOK;
  }

  void clear() { osMessageQueueReset(queue_id_.get()); }

  size_t size() { return osMessageQueueGetCount(queue_id_.get()); }

private:
  QueueId queue_id_;
};

} // namespace core
} // namespace tutrcos
