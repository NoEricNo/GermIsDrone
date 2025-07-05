# 🚁 GermIsDrone - Bio-Inspired Swarm Coordination

## 🧬 Hormone-Based Multi-Drone Coordination System

A working implementation of bio-inspired swarm intelligence using hormone-like signaling between autonomous drones.

### ✅ Features Implemented & Tested:
- **Multi-drone hormone communication** - Real-time coordination between PX4 instances
- **Exponential hormone decay** (λ = 0.9 s⁻¹) - Bio-inspired signal degradation
- **Multi-channel signaling** - Stress, coordination, and environmental hormones
- **Emergency response coordination** - Swarm-level fault tolerance
- **Real-time adaptation** - Dynamic behavior based on hormone levels

### 🚀 Quick Start:
```bash
# 1. Setup PX4 SITL
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && make px4_sitl_default none

# 2. Start second drone (new terminal)
PX4_SYS_AUTOSTART=4001 ./build/px4_sitl_default/bin/px4 -i 1 -d

# 3. Test swarm coordination
python3 real_multi_drone_test.py