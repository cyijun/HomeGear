#!/bin/sh
set -eu
test_source_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$test_source_dir/.." && pwd)
library_dir=${HOMEGEAR_TEST_LIBRARIES:-"$repo_dir/../HomeGear-build-env/libraries"}
compiler=${CXX:-c++}
test_output_dir=$(mktemp -d "${TMPDIR:-/tmp}/homegear-tests.XXXXXX")
trap 'rm -rf "$test_output_dir"' EXIT HUP INT TERM

for test_source in hitachi_ac3_control_test serial_config_test sensor_snapshot_test zigbee_bridge_test wifi_portal_policy_test; do
  "$compiler" -std=c++11 -Wall -Wextra -Werror -pedantic \
    "$test_source_dir/$test_source.cpp" -o "$test_output_dir/$test_source"
  "$test_output_dir/$test_source"
done

"$compiler" -std=c++11 -Wall -Wextra -Werror -pedantic \
  -I "$test_source_dir/power_policy_stubs" \
  "$test_source_dir/power_policy_test.cpp" -o "$test_output_dir/power_policy_test"
"$test_output_dir/power_policy_test"

json_dir="$library_dir/ArduinoJson-6.21.5/src"
bsec_dir="$library_dir/BSEC_Software_Library-1.8.1492/src"
if [ ! -d "$json_dir" ] || [ ! -d "$bsec_dir" ]; then
  echo "Set HOMEGEAR_TEST_LIBRARIES to the pinned library directory for the SensorService integration test." >&2
  exit 1
fi
"$compiler" -std=c++11 -Wall -Wextra -Werror -pedantic \
  -I "$test_source_dir/sensor_service_stubs" -I "$json_dir" -I "$bsec_dir" \
  "$test_source_dir/sensor_service_mock_test.cpp" -o "$test_output_dir/sensor_service_mock_test"
"$test_output_dir/sensor_service_mock_test"
