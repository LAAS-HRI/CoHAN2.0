# Behavior Tree Debug Logging

## Overview
The HATEB local planner includes optional debug logging for behavior tree operations. This can be enabled at compile time using a CMake option.

## Usage

### Default Behavior (Debug Logging OFF)
By default, BT debug logging is **disabled**. Build normally:
```bash
colcon build --packages-select hateb_local_planner
```

### Enable Debug Logging
To enable BT debug logging, build with the CMake option:
```bash
colcon build --packages-select hateb_local_planner --cmake-args -DENABLE_BT_DEBUG=ON
```

### What Gets Logged
When enabled, the behavior tree nodes will output:
- **BT_INFO**: Regular informational messages (white text)
- **BT_WARN**: Warning messages (yellow text)
- **BT_ERROR**: Error messages (red text)

