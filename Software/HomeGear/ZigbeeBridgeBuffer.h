#ifndef HOMEGEAR_ZIGBEE_BRIDGE_BUFFER_H
#define HOMEGEAR_ZIGBEE_BRIDGE_BUFFER_H

#include <cstddef>
#include <cstdint>

namespace homegear {
namespace zigbee {

// Single-owner byte queue. A short/blocked transfer consumes only the bytes
// actually transferred; the next call resumes at the same byte, including wrap.
template <std::size_t Capacity>
class ByteRing {
 public:
  static_assert(Capacity > 0, "A bridge queue must have storage");

  std::size_t size() const { return size_; }
  std::size_t free() const { return Capacity - size_; }
  void clear() { head_ = size_ = 0; }

  template <typename Reader>
  std::ptrdiff_t readFrom(std::size_t budget, Reader reader) {
    const std::size_t tail = (head_ + size_) % Capacity;
    const std::size_t amount = minimum(budget, minimum(free(), Capacity - tail));
    if (amount == 0) return 0;
    const std::ptrdiff_t result = reader(bytes_ + tail, amount);
    if (result > static_cast<std::ptrdiff_t>(amount)) return -1;
    if (result > 0) size_ += static_cast<std::size_t>(result);
    return result;
  }

  template <typename Writer>
  std::ptrdiff_t writeTo(std::size_t budget, Writer writer) {
    const std::size_t amount = minimum(budget, minimum(size_, Capacity - head_));
    if (amount == 0) return 0;
    const std::ptrdiff_t result = writer(bytes_ + head_, amount);
    if (result > static_cast<std::ptrdiff_t>(amount)) return -1;
    if (result > 0) {
      head_ = (head_ + static_cast<std::size_t>(result)) % Capacity;
      size_ -= static_cast<std::size_t>(result);
    }
    return result;
  }

 private:
  static std::size_t minimum(std::size_t a, std::size_t b) {
    return a < b ? a : b;
  }

  std::uint8_t bytes_[Capacity] = {};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};

}  // namespace zigbee
}  // namespace homegear

#endif
