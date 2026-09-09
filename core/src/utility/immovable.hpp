#pragma once

namespace libhcs::core::utility {

class Immovable {
public:
    Immovable() = default;
    Immovable(const Immovable&) = delete;
    Immovable& operator=(const Immovable&) = delete;
    Immovable(Immovable&&) = delete;
    Immovable& operator=(Immovable&&) = delete;
    ~Immovable() = default;
};

} // namespace libhcs::core::utility
