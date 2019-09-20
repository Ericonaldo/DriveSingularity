#pragma once

#include <iostream>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <json/json.h>

namespace ds {
struct EventFlag {
  using Type = std::uint64_t;
  static constexpr std::size_t Number = 7;

  // clang-format off
  static constexpr Type None             = 0b0000;
  static constexpr Type Follow           = 0b0001;    // keep straight: 1
  static constexpr Type Collision        = 0b0010;    // vehicle collision: 2
  static constexpr Type OverSolidLine    = 0b0100;    // over solid line: 4
  static constexpr Type IllegalDirection = 0b1000;    // reverse direction: 8
  static constexpr Type MaxSpeed         = 0b10000;   // higher speed, better for you: 16
  static constexpr Type SwitchLane       = 0b100000;  // switch lane: 32
  // clang-format on

  static std::size_t toIndex(Type flag) {
//      std::cout << "flag and flag - 1: " << flag << " " << flag - 1 << std::endl;
    assert(flag);
    assert((flag & (flag - 1)) == 0);
    auto idx = ffsll(flag);
    assert(0 < idx && idx <= Number);
    return idx;
  }
};

class EventContainer {
public:
  void addEvent(EventFlag::Type event, Json::Value value) {
    events.at(EventFlag::toIndex(event)) = std::move(value);
    flags |= event;
  }

  const Json::Value &operator[](EventFlag::Type event) const {
//      std::cout << "[DEBUG] Events::operator[] event evaluate value: " << event << std::endl;
    assert(event & flags);
    return events.at(EventFlag::toIndex(event));
  }

private:
  EventFlag::Type flags = EventFlag::None;
  std::array<Json::Value, EventFlag::Number> events;
};

} // namespace ds
