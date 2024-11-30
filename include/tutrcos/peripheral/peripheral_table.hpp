#pragma once

#include <array>
#include <cstddef>
#include <functional>

namespace tutrcos {
namespace peripheral {

template <class K, class V, size_t N> class PeripheralTable {
private:
  struct Entry {
    enum class State {
      EMPTY,
      OCCUPIED,
      ERASED,
    };

    K key;
    V *value;
    State state = State::EMPTY;
  };

public:
  V *get(const K &key) {
    for (size_t i = 0; i < N; ++i) {
      Entry &entry = entries_[hash(key, i)];
      if (entry.state == Entry::State::EMPTY) {
        return nullptr;
      }
      if (entry.state == Entry::State::OCCUPIED && entry.key == key) {
        return entry.value;
      }
    }
    return nullptr;
  }

  bool set(const K &key, V *value) {
    for (size_t i = 0; i < N; ++i) {
      Entry &entry = entries_[hash(key, i)];
      if (entry.state != Entry::State::OCCUPIED) {
        entry.key = key;
        entry.value = value;
        entry.state = Entry::State::OCCUPIED;
        return true;
      }
      if (entry.key == key) {
        entry.value = value;
        return true;
      }
    }
    return false;
  }

  bool erase(const K &key) {
    for (size_t i = 0; i < N; ++i) {
      Entry &entry = entries_[hash(key, i)];
      if (entry.state == Entry::State::EMPTY) {
        return false;
      }
      if (entry.state == Entry::State::OCCUPIED && entry.key == key) {
        entry.state = Entry::State::ERASED;
        return true;
      }
    }
    return false;
  }

private:
  std::array<Entry, N> entries_{};

  size_t hash(const K &key, size_t i) const {
    return (std::hash<K>{}(key) + i) % N;
  }
};

} // namespace peripheral
} // namespace tutrcos
