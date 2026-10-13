#include "../Software/HomeGear/SensorSnapshot.h"
#include <cassert>
#include <cstdint>
#include <limits>
#include <iostream>
#include <vector>

using namespace homegear::sensors;

int main() {
  SnapshotPublisher publisher;
  publisher.request();
  assert(!publisher.shouldPublish(0));
  assert(!publisher.update(Field::ThTemperature, -100.0f));
  assert(!publisher.update(Field::ThHumidity, 101.0f));
  assert(!publisher.update(Field::IaqPressure, 0.0f));
  assert(!publisher.update(Field::ThTemperature, std::numeric_limits<float>::quiet_NaN()));
  assert(!publisher.update(Field::IaqVoc, std::numeric_limits<float>::infinity()));
  assert(publisher.update(Field::ThTemperature, 0.0f));  // Zero is a real measurement.
  assert(publisher.shouldPublish(0));
  publisher.committed(0, publisher.snapshot());
  assert(!publisher.dirty());
  assert(publisher.update(Field::ThTemperature, 20.0f));
  assert(!publisher.shouldPublish(14999));
  assert(publisher.shouldPublish(15000));

  // Coalesce to the newest reading and include all previously known fields.
  assert(publisher.update(Field::ThTemperature, 21.0f));
  assert(publisher.update(Field::ThHumidity, 50.0f));
  const Snapshot full = publisher.snapshot();
  std::vector<Field> fields;
  assert(forEachKnown(full, [&](Field field, float) { fields.push_back(field); }) == 2);
  assert(full.values[static_cast<size_t>(Field::ThTemperature)] == 21.0f);
  publisher.committed(15000, full);
  assert(!publisher.shouldPublish(15001));
  assert(publisher.update(Field::ThTemperature, 21.1f));
  assert(!publisher.dirty());
  assert(!publisher.shouldPublish(30000));
  assert(publisher.shouldPublish(315000));  // Heartbeat includes unchanged known humidity.
  publisher.request();
  assert(publisher.shouldPublish(15001));  // Forced pub bypasses normal 15 s gate.

  // A failure cannot advance the baseline or consume a pending forced request.
  publisher.failed(15001);
  assert(publisher.lastPublished() == 15000);
  assert(!publisher.shouldPublish(16000));
  assert(publisher.shouldPublish(16001));
  assert(publisher.update(Field::ThTemperature, 25.0f));
  assert(publisher.dirty());
  publisher.failed(16001);
  assert(publisher.dirty());
  assert(publisher.lastPublished() == 15000);
  publisher.committed(17001, publisher.snapshot());
  assert(!publisher.dirty());
  assert(!publisher.shouldPublish(17002));

  SnapshotPublisher wrapped;
  const uint32_t nearWrap = UINT32_MAX - 1000;
  wrapped.update(Field::IaqTemperature, 25.0f);
  wrapped.committed(nearWrap, wrapped.snapshot());
  wrapped.update(Field::IaqTemperature, 27.0f);
  assert(!wrapped.shouldPublish(nearWrap + 14999u));
  assert(wrapped.shouldPublish(nearWrap + 15000u));
  wrapped.request();
  wrapped.failed(UINT32_MAX - 100);
  assert(!wrapped.shouldPublish(898));
  assert(wrapped.shouldPublish(899));
  wrapped.committed(nearWrap, wrapped.snapshot());
  assert(!wrapped.shouldPublish(nearWrap + 299999u));
  assert(wrapped.shouldPublish(nearWrap + 300000u));
  assert(elapsed(10, UINT32_MAX - 1000, 1000));

  // Invalid later samples do not replace a good value or create bogus JSON fields.
  SnapshotPublisher validity;
  validity.update(Field::ThTemperature, 23.0f);
  validity.committed(1, validity.snapshot());
  assert(!validity.update(Field::ThTemperature, -100.0f));
  assert(validity.snapshot().values[0] == 23.0f);
  assert(forEachKnown(validity.snapshot(), [](Field, float) {}) == 1);
  validity.update(Field::IaqPressure, 101325.0f);
  assert(forEachKnown(validity.snapshot(), [](Field, float) {}) == 2);

  std::cout << "sensor_snapshot_test: PASS\n";
}
