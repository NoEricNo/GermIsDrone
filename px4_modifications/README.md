# PX4 Modifications for Hormone System

## Files Modified:
- `msg/hormone_broadcast.msg` - Custom uORB message
- `src/modules/mavlink/mavlink_messages.cpp` - Hormone heartbeat stream
- `src/modules/hormone_source/` - Hormone publisher module
- `boards/px4/sitl/default.px4board` - Module registration

## Changes Made:
1. Extended HEARTBEAT message with 20-byte hormone field
2. Created hormone decay and injection logic
3. Integrated with MAVLink telemetry system

