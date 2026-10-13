#pragma once
#include <cstdint>
namespace homegear {
// Pure timing policy; no SDK calls or credential state.
class WifiPortalPolicy {
 public:
  static constexpr std::uint32_t kStableMs = 30000;
  static constexpr std::uint32_t kSaveGuardMs = 45000;
  void protectSave(std::uint32_t now) { savePending_ = true; saveAt_ = now; }
  bool saveGuardActive(std::uint32_t now) {
    if (savePending_ && static_cast<std::uint32_t>(now - saveAt_) >= kSaveGuardMs) savePending_ = false;
    return savePending_;
  }
  void disconnected() { stable_ = false; }
  bool canReclaim(std::uint32_t now, bool connected, bool apEnabled, unsigned clients) {
    const bool guarded = saveGuardActive(now);
    if (!connected) { disconnected(); return false; }
    if (!stable_) { stable_ = true; connectedAt_ = now; }
    return apEnabled && clients == 0 && !guarded &&
        static_cast<std::uint32_t>(now - connectedAt_) >= kStableMs;
  }
 private:
  bool stable_ = false;
  bool savePending_ = false;
  std::uint32_t connectedAt_ = 0;
  std::uint32_t saveAt_ = 0;
};
}
