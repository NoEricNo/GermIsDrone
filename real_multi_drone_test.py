#!/usr/bin/env python3
"""
Real multi-drone hormone system test
Tests hormone propagation between actual PX4 instances
"""

import asyncio
import numpy as np
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - [%(name)s] - %(levelname)s - %(message)s')

class DroneHormoneNode:
    """Individual drone node with hormone system"""
    
    def __init__(self, drone_id, port, decay_rate=0.9):
        self.drone_id = drone_id
        self.port = port
        self.decay_rate = decay_rate
        self.hormone_state = np.zeros(20, dtype=np.uint8)
        self.last_update = time.perf_counter()
        self.drone = None
        self.logger = logging.getLogger(f"Drone{drone_id}")
        self.cycle_count = 0
        self.is_connected = False
        
    async def connect(self):
        """Connect to PX4 instance"""
        try:
            from mavsdk import System
            
            self.drone = System()
            system_address = f"udp://:{self.port}"
            
            self.logger.info(f"Connecting to port {self.port}...")
            await self.drone.connect(system_address=system_address)
            
            # Wait for connection with timeout
            connection_timeout = 10.0
            start_time = time.time()
            
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.is_connected = True
                    self.logger.info(f"✓ Connected to PX4 on port {self.port}")
                    return True
                    
                if time.time() - start_time > connection_timeout:
                    raise asyncio.TimeoutError(f"Connection timeout for drone {self.drone_id}")
            
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            return False
    
    async def start_monitoring(self, swarm_coordinator):
        """Start hormone monitoring loop"""
        if not self.is_connected:
            self.logger.error("Cannot start monitoring - not connected")
            return
            
        self.logger.info("Starting hormone monitoring...")
        
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                self.cycle_count += 1
                now = time.perf_counter()
                dt = now - self.last_update
                
                # Apply hormone decay
                if self.last_update > 0:
                    decay_factor = self.decay_rate ** dt
                    self.hormone_state = np.clip(
                        self.hormone_state * decay_factor, 0, 255
                    ).astype(np.uint8)
                
                # Get health and inject hormones
                await self.update_hormones(flight_mode)
                
                # Share hormone state with swarm
                await swarm_coordinator.receive_hormone_broadcast(
                    self.drone_id, self.hormone_state.copy()
                )
                
                # Receive hormones from other drones
                neighbor_hormones = await swarm_coordinator.get_neighbor_hormones(self.drone_id)
                self.process_neighbor_hormones(neighbor_hormones)
                
                # Check for responses
                await self.check_hormone_responses()
                
                # Log status
                if self.cycle_count % 20 == 0:
                    self.log_status(flight_mode)
                
                self.last_update = now
                
        except Exception as e:
            self.logger.error(f"Monitoring error: {e}")
    
    async def update_hormones(self, flight_mode):
        """Update hormone state based on drone status"""
        
        try:
            # Try to get health status
            health_task = asyncio.create_task(self.get_health_status())
            health = await asyncio.wait_for(health_task, timeout=0.1)
            
            # Health-based hormone injection
            if not health.is_armable:
                self.inject_hormone(0, 40, "Armability stress")
                
            if not health.is_global_position_ok:
                self.inject_hormone(0, 30, "GPS stress")
                
        except asyncio.TimeoutError:
            # If health unavailable, use simple injection
            pass
        
        # Flight mode based hormones
        if "HOLD" in str(flight_mode):
            self.inject_hormone(1, 20, "Hold coordination")
        
        # Random events (simulate real-world triggers)
        if np.random.random() < 0.1:  # 10% chance
            event_type = np.random.randint(0, 3)
            amount = np.random.randint(25, 75)
            self.inject_hormone(event_type, amount, f"Random event {event_type}")
    
    async def get_health_status(self):
        """Get health status from drone"""
        async for health in self.drone.telemetry.health():
            return health
    
    def inject_hormone(self, hormone_type, amount, reason):
        """Inject specific hormone type"""
        if 0 <= hormone_type < 20:
            old_value = self.hormone_state[hormone_type]
            self.hormone_state[hormone_type] = min(255, old_value + amount)
            self.logger.info(f"💉 {reason}: hormone[{hormone_type}] {old_value} → {self.hormone_state[hormone_type]}")
    
    def process_neighbor_hormones(self, neighbor_hormones):
        """Process hormone broadcasts from neighbor drones"""
        if not neighbor_hormones:
            return
            
        # Simple propagation model: add 30% of neighbor's hormones
        propagation_factor = 0.3
        total_neighbors = len(neighbor_hormones)
        
        for neighbor_id, hormones in neighbor_hormones.items():
            for i in range(min(len(hormones), 20)):
                propagated = int(hormones[i] * propagation_factor / total_neighbors)
                if propagated > 0:
                    old_value = self.hormone_state[i]
                    self.hormone_state[i] = min(255, old_value + propagated)
                    
        if total_neighbors > 0:
            self.logger.debug(f"🔄 Received hormones from {total_neighbors} neighbors")
    
    async def check_hormone_responses(self):
        """Check for hormone response triggers"""
        mean_hormone = np.mean(self.hormone_state)
        max_hormone = np.max(self.hormone_state)
        
        if mean_hormone > 80:
            self.logger.error(f"🚨 EMERGENCY! Mean hormone: {mean_hormone:.1f}")
            await self.emergency_response()
        elif mean_hormone > 50:
            self.logger.warning(f"⚠️ High hormone detected: {mean_hormone:.1f}")
            await self.coordination_response()
    
    async def emergency_response(self):
        """Emergency response to high hormone levels"""
        self.logger.error("🚁 Emergency protocol activated!")
        # Reduce hormone levels (response executed)
        self.hormone_state = np.clip(self.hormone_state * 0.4, 0, 255).astype(np.uint8)
    
    async def coordination_response(self):
        """Coordination response to moderate hormone levels"""
        self.logger.info("🤝 Coordination protocol activated")
        # Moderate hormone reduction
        self.hormone_state = np.clip(self.hormone_state * 0.7, 0, 255).astype(np.uint8)
    
    def log_status(self, flight_mode):
        """Log current status"""
        mean_h = np.mean(self.hormone_state)
        max_h = np.max(self.hormone_state)
        stress = self.hormone_state[0]
        coord = self.hormone_state[1]
        env = self.hormone_state[2]
        
        self.logger.info(f"📊 Cycle #{self.cycle_count} | Mode: {flight_mode} | "
                        f"Hormones - Mean: {mean_h:.1f}, Max: {max_h} | "
                        f"Stress: {stress}, Coord: {coord}, Env: {env}")

class SwarmCoordinator:
    """Coordinates hormone sharing between drones"""
    
    def __init__(self):
        self.hormone_broadcasts = {}  # {drone_id: (timestamp, hormone_state)}
        self.logger = logging.getLogger("Swarm")
        
    async def receive_hormone_broadcast(self, drone_id, hormone_state):
        """Receive hormone broadcast from a drone"""
        timestamp = time.perf_counter()
        self.hormone_broadcasts[drone_id] = (timestamp, hormone_state)
        
        # Clean old broadcasts (older than 5 seconds)
        current_time = time.perf_counter()
        to_remove = []
        for did, (ts, _) in self.hormone_broadcasts.items():
            if current_time - ts > 5.0:
                to_remove.append(did)
        
        for did in to_remove:
            del self.hormone_broadcasts[did]
    
    async def get_neighbor_hormones(self, requesting_drone_id):
        """Get hormone states from neighbor drones"""
        neighbors = {}
        current_time = time.perf_counter()
        
        for drone_id, (timestamp, hormones) in self.hormone_broadcasts.items():
            if drone_id != requesting_drone_id and current_time - timestamp < 3.0:
                neighbors[drone_id] = hormones
                
        return neighbors
    
    def log_swarm_state(self):
        """Log overall swarm state"""
        if not self.hormone_broadcasts:
            return
            
        all_means = []
        status_parts = []
        
        for drone_id, (_, hormones) in self.hormone_broadcasts.items():
            mean_h = np.mean(hormones)
            all_means.append(mean_h)
            status_parts.append(f"D{drone_id}:{mean_h:.1f}")
        
        swarm_mean = np.mean(all_means) if all_means else 0
        self.logger.info(f"🌐 Swarm State | Overall: {swarm_mean:.1f} | Individual: {' '.join(status_parts)}")

async def test_multi_drone_swarm():
    """Test real multi-drone hormone system"""
    
    logger = logging.getLogger("Main")
    logger.info("🚁 Starting Real Multi-Drone Hormone Test")
    
    # Configure drones (add more as you start more PX4 instances)
    drone_configs = [
        {"id": 0, "port": 14540},  # Original drone
        {"id": 1, "port": 14541},  # Drone started with -i 1
        # {"id": 2, "port": 14542},  # Uncomment when you start drone 2
    ]
    
    # Create swarm coordinator
    coordinator = SwarmCoordinator()
    
    # Create and connect drones
    drones = []
    for config in drone_configs:
        drone = DroneHormoneNode(
            drone_id=config["id"], 
            port=config["port"], 
            decay_rate=0.9
        )
        
        if await drone.connect():
            drones.append(drone)
            logger.info(f"✓ Drone {config['id']} ready")
        else:
            logger.error(f"✗ Failed to connect drone {config['id']}")
    
    if len(drones) < 2:
        logger.error("Need at least 2 drones connected for swarm test")
        return
    
    logger.info(f"🚁 Starting swarm with {len(drones)} drones")
    
    # Start monitoring tasks
    tasks = []
    for drone in drones:
        task = asyncio.create_task(drone.start_monitoring(coordinator))
        tasks.append(task)
    
    # Swarm state logging task
    async def log_swarm_periodically():
        while True:
            await asyncio.sleep(30)  # Log every 30 seconds
            coordinator.log_swarm_state()
    
    swarm_log_task = asyncio.create_task(log_swarm_periodically())
    tasks.append(swarm_log_task)
    
    # Run for 5 minutes
    try:
        logger.info("🚀 Swarm monitoring active for 5 minutes...")
        await asyncio.wait_for(asyncio.gather(*tasks), timeout=300)  # 5 minutes
    except asyncio.TimeoutError:
        logger.info("⏰ Test completed after 5 minutes")
    
    # Cancel tasks
    for task in tasks:
        task.cancel()
    
    logger.info("🏁 Multi-drone hormone test complete!")

if __name__ == "__main__":
    asyncio.run(test_multi_drone_swarm())