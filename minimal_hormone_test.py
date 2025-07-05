#!/usr/bin/env python3
"""
Minimal hormone receiver test for pure SITL (no Gazebo)
This focuses on the core algorithm without simulation overhead
"""

import asyncio
import numpy as np
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MinimalHormoneReceiver:
    def __init__(self, decay_rate=0.9, drone_id=0):
        self.decay_rate = decay_rate
        self.drone_id = drone_id
        self.hormone_state = np.zeros(20, dtype=np.uint8)
        self.last_update = time.perf_counter()
        self.heartbeat_count = 0
        
    async def connect_and_monitor(self, port=14540):
        """Connect to PX4 SITL and monitor hormone state"""
        try:
            from mavsdk import System
            
            drone = System()
            system_address = f"udp://:{port}"
            
            logger.info(f"Connecting to PX4 SITL at {system_address}")
            await drone.connect(system_address=system_address)
            
            # Wait for connection (Python 3.10 compatible)
            logger.info("Waiting for connection...")
            connection_timeout = 15.0  # 15 seconds should be enough
            start_time = time.time()
            
            async for state in drone.core.connection_state():
                if state.is_connected:
                    logger.info("✓ Connected to PX4 SITL!")
                    break
                    
                # Check for timeout
                if time.time() - start_time > connection_timeout:
                    raise asyncio.TimeoutError("Connection timeout")
            
            # Start monitoring
            await self.monitor_hormone_loop(drone)
            
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            raise
    
    async def monitor_hormone_loop(self, drone):
        """Main hormone monitoring loop"""
        logger.info("Starting hormone monitoring...")
        
        try:
            # Use flight_mode as our telemetry source (equivalent to heartbeat timing)
            async for flight_mode in drone.telemetry.flight_mode():
                self.heartbeat_count += 1
                now = time.perf_counter()
                dt = now - self.last_update
                
                # Apply hormone decay
                if self.last_update > 0:
                    decay_factor = self.decay_rate ** dt
                    self.hormone_state = np.clip(
                        self.hormone_state * decay_factor, 0, 255
                    ).astype(np.uint8)
                
                # Get system health for hormone injection logic
                try:
                    # Get health status (non-blocking)
                    health_task = asyncio.create_task(self.get_health_status(drone))
                    health = await asyncio.wait_for(health_task, timeout=0.1)
                    self.simulate_hormone_injection_from_health(flight_mode, health)
                except asyncio.TimeoutError:
                    # If health check times out, just use flight mode
                    self.simulate_hormone_injection_simple(flight_mode)
                
                # Check for hormone response triggers
                mean_hormone = np.mean(self.hormone_state)
                max_hormone = np.max(self.hormone_state)
                
                if mean_hormone > 50:
                    logger.warning(f"🚨 High hormone detected! Mean: {mean_hormone:.1f}, Max: {max_hormone}")
                    await self.respond_to_hormone(drone)
                
                # Log status every 10 cycles (more frequent for testing)
                if self.heartbeat_count % 10 == 0:
                    logger.info(f"Cycle #{self.heartbeat_count}: Mode={flight_mode}, "
                              f"Hormone Mean: {mean_hormone:.1f}, Max: {max_hormone}")
                
                self.last_update = now
                
        except Exception as e:
            logger.error(f"Monitoring loop error: {e}")
    
    async def get_health_status(self, drone):
        """Get health status (with timeout)"""
        async for health in drone.telemetry.health():
            return health
    
    def simulate_hormone_injection_from_health(self, flight_mode, health):
        """Simulate hormone injection based on health and flight mode"""
        
        # Inject stress hormone based on health issues
        if not health.is_global_position_ok:
            self.hormone_state[0] = min(255, self.hormone_state[0] + 50)
            logger.warning("💉 Injecting GPS stress hormone")
            
        if not health.is_armable:
            self.hormone_state[0] = min(255, self.hormone_state[0] + 30)
            logger.warning("💉 Injecting armability stress hormone")
        
        # Inject coordination hormone based on flight mode
        if "HOLD" in str(flight_mode) or "LOITER" in str(flight_mode):
            self.hormone_state[1] = min(255, self.hormone_state[1] + 20)
            logger.info("💉 Injecting coordination hormone (holding pattern)")
        
        # Random coordination hormone (simulate swarm communication)
        if np.random.random() < 0.1:  # 10% chance
            coord_level = np.random.randint(15, 60)
            self.hormone_state[1] = min(255, self.hormone_state[1] + coord_level)
            logger.info(f"💉 Random coordination hormone: {coord_level}")
    
    def simulate_hormone_injection_simple(self, flight_mode):
        """Simpler hormone injection when health data unavailable"""
        
        # Inject based on flight mode changes
        if "MANUAL" in str(flight_mode):
            self.hormone_state[0] = min(255, self.hormone_state[0] + 40)
            logger.info("💉 Manual mode stress hormone")
        
        # Random hormone injection (simulate various triggers)
        if np.random.random() < 0.15:  # 15% chance
            hormone_type = np.random.randint(0, 3)  # 3 hormone types
            injection_amount = np.random.randint(20, 80)
            self.hormone_state[hormone_type] = min(255, self.hormone_state[hormone_type] + injection_amount)
            logger.info(f"💉 Injecting hormone[{hormone_type}]: {injection_amount}")
    
    def simulate_hormone_injection(self, heartbeat):
        """Legacy method - replaced by health-based injection"""
        # This method is no longer used but kept for compatibility
        pass
    
    async def respond_to_hormone(self, drone):
        """Respond to high hormone levels"""
        try:
            # Get current flight mode
            logger.info("Responding to hormone trigger...")
            
            # In real implementation, this would trigger swarm coordination
            # For now, just log the response
            logger.info("🤖 Hormone response: Coordinating with swarm...")
            
            # Simulate response decay
            self.hormone_state = np.clip(self.hormone_state * 0.8, 0, 255).astype(np.uint8)
            
        except Exception as e:
            logger.error(f"Hormone response failed: {e}")

class SwarmSimulator:
    """Simulate multiple drones in the swarm"""
    
    def __init__(self, num_drones=3):
        self.num_drones = num_drones
        self.drones = []
        
    async def start_swarm(self):
        """Start multiple drone instances"""
        logger.info(f"Starting swarm simulation with {self.num_drones} drones")
        
        # Create hormone receivers for each drone
        for i in range(self.num_drones):
            drone = MinimalHormoneReceiver(decay_rate=0.9, drone_id=i)
            self.drones.append(drone)
        
        # Start monitoring tasks
        tasks = []
        for i, drone in enumerate(self.drones):
            port = 14540 + i  # Assume multiple PX4 instances on different ports
            task = asyncio.create_task(self.monitor_drone(drone, port, i))
            tasks.append(task)
        
        # Run all drones concurrently
        try:
            await asyncio.gather(*tasks)
        except Exception as e:
            logger.error(f"Swarm simulation error: {e}")
    
    async def monitor_drone(self, drone_receiver, port, drone_id):
        """Monitor individual drone with error handling"""
        try:
            await drone_receiver.connect_and_monitor(port)
        except Exception as e:
            logger.error(f"Drone {drone_id} (port {port}) failed: {e}")
            # Continue with other drones

async def test_single_drone():
    """Test single drone hormone system"""
    logger.info("=== Testing Single Drone Hormone System ===")
    
    receiver = MinimalHormoneReceiver(decay_rate=0.9, drone_id=0)
    await receiver.connect_and_monitor(14540)

async def test_swarm():
    """Test swarm hormone system (requires multiple PX4 instances)"""
    logger.info("=== Testing Swarm Hormone System ===")
    
    swarm = SwarmSimulator(num_drones=3)
    await swarm.start_swarm()

def main():
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "swarm":
        logger.info("Running swarm test")
        asyncio.run(test_swarm())
    else:
        logger.info("Running single drone test")
        asyncio.run(test_single_drone())

if __name__ == "__main__":
    main()