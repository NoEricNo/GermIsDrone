#!/usr/bin/env python3
"""
Advanced single drone test simulating multi-drone hormone interactions
This simulates what would happen in a real swarm by creating virtual "neighbor" drones
"""

import asyncio
import numpy as np
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class VirtualSwarmNode:
    """Simulates a drone in the swarm with hormone state"""
    
    def __init__(self, drone_id, decay_rate=0.9):
        self.drone_id = drone_id
        self.decay_rate = decay_rate
        self.hormone_state = np.zeros(20, dtype=np.uint8)
        self.last_update = time.perf_counter()
        self.is_connected = False
        
    def update_hormones(self, dt):
        """Apply decay and return current state"""
        decay_factor = self.decay_rate ** dt
        self.hormone_state = np.clip(
            self.hormone_state * decay_factor, 0, 255
        ).astype(np.uint8)
        return self.hormone_state
    
    def inject_hormone(self, hormone_type, amount):
        """Inject hormone of specific type"""
        if 0 <= hormone_type < 20:
            self.hormone_state[hormone_type] = min(255, self.hormone_state[hormone_type] + amount)
            return True
        return False
    
    def receive_broadcast(self, neighbor_hormones):
        """Receive hormone broadcast from neighbor and respond"""
        # Simple propagation: add fraction of neighbor's hormones
        propagation_factor = 0.3
        for i in range(min(len(neighbor_hormones), 20)):
            propagated_amount = int(neighbor_hormones[i] * propagation_factor)
            self.hormone_state[i] = min(255, self.hormone_state[i] + propagated_amount)

class SwarmSimulator:
    """Simulates multi-drone swarm using one real PX4 connection"""
    
    def __init__(self, num_virtual_drones=3):
        self.real_drone = None
        self.virtual_drones = []
        self.num_virtual_drones = num_virtual_drones
        
        # Create virtual drone nodes
        for i in range(num_virtual_drones):
            drone = VirtualSwarmNode(drone_id=i+1)  # Real drone is ID 0
            self.virtual_drones.append(drone)
        
        # Swarm state
        self.swarm_hormone_state = np.zeros(20, dtype=np.uint8)
        self.last_swarm_update = time.perf_counter()
        
    async def connect_real_drone(self):
        """Connect to the real PX4 drone"""
        try:
            from mavsdk import System
            
            self.real_drone = System()
            await self.real_drone.connect(system_address="udp://:14540")
            
            # Wait for connection
            async for state in self.real_drone.core.connection_state():
                if state.is_connected:
                    logger.info("✓ Real drone connected!")
                    break
                    
        except Exception as e:
            logger.error(f"Failed to connect real drone: {e}")
            raise
    
    async def simulate_swarm(self, duration_minutes=5):
        """Run swarm simulation"""
        logger.info(f"🚁 Starting swarm simulation with 1 real + {self.num_virtual_drones} virtual drones")
        
        start_time = time.perf_counter()
        end_time = start_time + (duration_minutes * 60)
        cycle_count = 0
        
        try:
            async for flight_mode in self.real_drone.telemetry.flight_mode():
                cycle_count += 1
                now = time.perf_counter()
                dt = now - self.last_swarm_update
                
                # Update all drone hormone states
                await self.update_all_drones(dt, flight_mode)
                
                # Simulate swarm interactions
                self.simulate_swarm_communication()
                
                # Check for swarm-level responses
                await self.check_swarm_responses()
                
                # Log swarm state
                if cycle_count % 15 == 0:  # Every 15 cycles
                    self.log_swarm_state(cycle_count)
                
                # Inject random swarm events
                self.simulate_swarm_events()
                
                self.last_swarm_update = now
                
                # Stop after specified duration
                if now > end_time:
                    logger.info(f"🏁 Swarm simulation complete after {duration_minutes} minutes")
                    break
                    
        except Exception as e:
            logger.error(f"Swarm simulation error: {e}")
    
    async def update_all_drones(self, dt, flight_mode):
        """Update hormone states for all drones"""
        
        # Update virtual drones
        for drone in self.virtual_drones:
            drone.update_hormones(dt)
        
        # Update swarm-level hormone state (represents real drone)
        decay_factor = 0.9 ** dt
        self.swarm_hormone_state = np.clip(
            self.swarm_hormone_state * decay_factor, 0, 255
        ).astype(np.uint8)
        
        # Real drone hormone injection based on flight mode and health
        try:
            health_task = asyncio.create_task(self.get_health_status())
            health = await asyncio.wait_for(health_task, timeout=0.1)
            self.inject_real_drone_hormones(flight_mode, health)
        except asyncio.TimeoutError:
            self.inject_simple_hormones(flight_mode)
    
    async def get_health_status(self):
        """Get health status from real drone"""
        async for health in self.real_drone.telemetry.health():
            return health
    
    def inject_real_drone_hormones(self, flight_mode, health):
        """Inject hormones based on real drone status"""
        
        # Stress hormones from health issues
        if not health.is_armable:
            self.swarm_hormone_state[0] = min(255, self.swarm_hormone_state[0] + 60)
            logger.warning("💉 Real drone: Armability stress hormone")
            
        if not health.is_global_position_ok:
            self.swarm_hormone_state[0] = min(255, self.swarm_hormone_state[0] + 40)
            logger.warning("💉 Real drone: GPS stress hormone")
        
        # Coordination hormones from flight mode
        if "HOLD" in str(flight_mode):
            self.swarm_hormone_state[1] = min(255, self.swarm_hormone_state[1] + 25)
            logger.info("💉 Real drone: Hold pattern coordination hormone")
    
    def inject_simple_hormones(self, flight_mode):
        """Simple hormone injection when health unavailable"""
        if np.random.random() < 0.2:  # 20% chance
            hormone_type = np.random.randint(0, 3)
            amount = np.random.randint(30, 80)
            self.swarm_hormone_state[hormone_type] = min(255, self.swarm_hormone_state[hormone_type] + amount)
            logger.info(f"💉 Real drone: Random hormone[{hormone_type}] = {amount}")
    
    def simulate_swarm_communication(self):
        """Simulate hormone communication between drones"""
        
        # Real drone broadcasts to virtual drones
        for drone in self.virtual_drones:
            drone.receive_broadcast(self.swarm_hormone_state)
        
        # Virtual drones broadcast to each other and back to real drone
        all_virtual_hormones = []
        for drone in self.virtual_drones:
            all_virtual_hormones.append(drone.hormone_state)
        
        # Average virtual drone hormones affect real drone
        if all_virtual_hormones:
            avg_virtual = np.mean(all_virtual_hormones, axis=0)
            propagation = (avg_virtual * 0.2).astype(np.uint8)  # 20% propagation
            self.swarm_hormone_state = np.clip(
                self.swarm_hormone_state + propagation, 0, 255
            ).astype(np.uint8)
    
    def simulate_swarm_events(self):
        """Simulate random swarm events"""
        
        # Virtual drone fault
        if np.random.random() < 0.05:  # 5% chance
            faulty_drone = np.random.choice(self.virtual_drones)
            faulty_drone.inject_hormone(0, 150)  # High stress
            logger.error(f"🚨 Virtual drone {faulty_drone.drone_id} FAULT! Injecting stress hormones")
        
        # Virtual drone discovery
        if np.random.random() < 0.08:  # 8% chance
            discovering_drone = np.random.choice(self.virtual_drones)
            discovering_drone.inject_hormone(2, 100)  # Environmental discovery
            logger.info(f"🔍 Virtual drone {discovering_drone.drone_id} discovered target!")
        
        # Coordination event
        if np.random.random() < 0.12:  # 12% chance
            coordinating_drone = np.random.choice(self.virtual_drones)
            coordinating_drone.inject_hormone(1, 80)  # Coordination signal
            logger.info(f"🤝 Virtual drone {coordinating_drone.drone_id} requesting coordination")
    
    async def check_swarm_responses(self):
        """Check for swarm-level hormone responses"""
        
        # Check real drone hormone levels
        real_mean = np.mean(self.swarm_hormone_state)
        real_max = np.max(self.swarm_hormone_state)
        
        # Check virtual drone hormone levels
        virtual_means = [np.mean(drone.hormone_state) for drone in self.virtual_drones]
        swarm_mean = np.mean([real_mean] + virtual_means)
        
        # Swarm-level responses
        if swarm_mean > 75:
            logger.error(f"🚨🚨 SWARM EMERGENCY! Average hormone: {swarm_mean:.1f}")
            await self.emergency_response()
        elif swarm_mean > 50:
            logger.warning(f"⚠️ Swarm coordination needed. Average hormone: {swarm_mean:.1f}")
            await self.coordination_response()
    
    async def emergency_response(self):
        """Swarm emergency response"""
        # In real implementation, this would trigger emergency actions
        logger.error("🚁 All drones switching to emergency mode!")
        
        # Reduce all hormone levels (emergency protocol executed)
        self.swarm_hormone_state = np.clip(self.swarm_hormone_state * 0.5, 0, 255).astype(np.uint8)
        for drone in self.virtual_drones:
            drone.hormone_state = np.clip(drone.hormone_state * 0.5, 0, 255).astype(np.uint8)
    
    async def coordination_response(self):
        """Swarm coordination response"""
        logger.info("🤝 Executing swarm coordination maneuver")
        
        # Moderate hormone reduction (coordination executed)
        self.swarm_hormone_state = np.clip(self.swarm_hormone_state * 0.7, 0, 255).astype(np.uint8)
    
    def log_swarm_state(self, cycle):
        """Log current swarm state"""
        real_mean = np.mean(self.swarm_hormone_state)
        real_max = np.max(self.swarm_hormone_state)
        
        virtual_stats = []
        for i, drone in enumerate(self.virtual_drones):
            mean_h = np.mean(drone.hormone_state)
            max_h = np.max(drone.hormone_state)
            virtual_stats.append(f"D{i+1}:{mean_h:.1f}/{max_h}")
        
        logger.info(f"🚁 Cycle #{cycle} | Real Drone: {real_mean:.1f}/{real_max} | Virtual: {' '.join(virtual_stats)}")

async def main():
    """Run the advanced swarm simulation"""
    
    simulator = SwarmSimulator(num_virtual_drones=3)
    
    logger.info("Connecting to real drone...")
    await simulator.connect_real_drone()
    
    logger.info("Starting swarm simulation...")
    await simulator.simulate_swarm(duration_minutes=3)  # 3 minute test

if __name__ == "__main__":
    asyncio.run(main())