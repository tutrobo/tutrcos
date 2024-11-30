#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <optional>

namespace tutrcos {
namespace core {

template <class K, class V, size_t N> class FixedHashMap {
private:
  struct Entry {
    enum class State {
      EMPTY,
      OCCUPIED,
      ERASED,
    };

    std::optional<K> key;
    std::optional<V> value;
    State state = State::EMPTY;
  };

public:
  std::optional<V> get(const K &key) {
    for (size_t i = 0; i < N; ++i) {
      Entry &entry = entries_[hash(key, i)];
      if (entry.state == Entry::State::EMPTY) {
        return std::nullopt;
      }
      if (entry.state == Entry::State::OCCUPIED && entry.key == key) {
        return entry.value;
      }
    }
    return std::nullopt;
  }

  bool set(const K &key, const V &value) {
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
        entry.key = std::nullopt;
        entry.value = std::nullopt;
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

} // namespace core
} // namespace tutrcos
