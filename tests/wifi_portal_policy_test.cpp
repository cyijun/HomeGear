#include <cassert>
#include <cstdint>
#include <iostream>
#include "../Software/HomeGear/WifiPortalPolicy.h"
using homegear::WifiPortalPolicy;
int main() {
  WifiPortalPolicy policy;
  assert(!policy.canReclaim(0, false, true, 0));
  assert(!policy.canReclaim(1000, true, true, 0));
  assert(!policy.canReclaim(30999, true, true, 0));
  assert(!policy.canReclaim(31000, true, true, 1));
  assert(!policy.canReclaim(31000, true, false, 0));
  assert(policy.canReclaim(31000, true, true, 0));
  policy.protectSave(31000);
  assert(policy.saveGuardActive(75999));
  assert(!policy.canReclaim(75999, true, true, 0));
  assert(!policy.saveGuardActive(76000));
  assert(policy.canReclaim(76000, true, true, 0));
  assert(!policy.canReclaim(77000, false, true, 0));
  assert(!policy.canReclaim(78000, true, true, 0));
  assert(!policy.canReclaim(107999, true, true, 0));
  assert(policy.canReclaim(108000, true, true, 0));
  policy.disconnected(); // A queued loss/recovery between ticks restarts stability.
  assert(!policy.canReclaim(109000, true, true, 0));
  assert(policy.canReclaim(139000, true, true, 0));
  WifiPortalPolicy wrap;
  const std::uint32_t origin = UINT32_MAX - 15000U;
  assert(!wrap.canReclaim(origin, true, true, 0));
  assert(!wrap.canReclaim(origin + 29999U, true, true, 0));
  assert(wrap.canReclaim(origin + 30000U, true, true, 0));
  wrap.protectSave(origin + 30000U);
  assert(!wrap.canReclaim(origin + 74999U, true, true, 0));
  assert(wrap.canReclaim(origin + 75000U, true, true, 0));
  WifiPortalPolicy saveBeforeConnection;
  saveBeforeConnection.protectSave(0);
  assert(!saveBeforeConnection.canReclaim(1000, true, true, 0));
  assert(!saveBeforeConnection.canReclaim(31000, true, true, 0));
  assert(saveBeforeConnection.canReclaim(45000, true, true, 0));
  std::cout << "WiFi portal stability, client/save guard, loss and wrap tests passed\n";
}
