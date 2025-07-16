#!/usr/bin/env python3
"""
Integrated Fault Injector - Connects directly to your hormone system
This version integrates with your real_multi_drone_test.py
"""

import asyncio
import time
import logging
import argparse
import socket
import json
from mavsdk import System

logging.basicConfig(level=logging.INFO, format='%(asctime)s - [%(name)s] - %(levelname)s - %(message)s')

class IntegratedFaultInjector:
    """Fault injector that communicates with your hormone system via UDP"""
    
    def __init__(self, drone_id, port):
        self.drone_id = drone_id
        self.port = port
        self.drone = System()
        self.logger = logging.getLogger(f"IntegratedFaultInjector-{drone_id}")
        self.is_connected = False
        
        # UDP socket for hormone injection
        self.hormone_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.hormone_port = 9999  # Communication port with hormone system
        
    async def connect(self):
        """Connect to PX4 instance"""
        try:
            system_address = f"udp://:{self.port}"
            self.logger.info(f"Connecting to drone {self.drone_id} on port {self.port}...")
            
            await self.drone.connect(system_address=system_address)
            
            # Wait for connection
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.is_connected = True
                    self.logger.info(f"✓ Connected to drone {self.drone_id}")
                    return True
                    
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            return False
    
    def send_hormone_injection(self, hormone_type, amount, reason):
        """Send hormone injection command to your hormone system"""
        try:
            message = {
                'command': 'inject_hormone',
                'drone_id': self.drone_id,
                'hormone_type': hormone_type,
                'amount': amount,
                'reason': reason,
                'timestamp': time.time()
            }
            
            data = json.dumps(message).encode()
            self.hormone_socket.sendto(data, ('127.0.0.1', self.hormone_port))
            self.logger.info(f"📡 Sent hormone injection command: {reason}")
            
        except Exception as e:
            self.logger.error(f"Failed to send hormone injection: {e}")
    
    async def inject_gps_fault_simulation(self, duration=10):
        """Simulate GPS fault with hormone injection"""
        if not self.is_connected:
            self.logger.error("Not connected to drone")
            return False
            
        try:
            self.logger.warning(f"🛰️ SIMULATING GPS FAULT for {duration}s")
            
            # Send hormone injection to your system
            self.send_hormone_injection(
                hormone_type=0,  # Stress hormone
                amount=200,      # High stress
                reason=f"GPS fault simulation on drone {self.drone_id}"
            )
            
            await asyncio.sleep(duration)
            
            self.logger.info(f"✓ GPS fault simulation complete for drone {self.drone_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"GPS fault simulation failed: {e}")
            return False
    
    async def inject_motor_fault_simulation(self, duration=8):
        """Simulate motor fault with hormone injection"""
        try:
            self.logger.warning(f"⚙️ SIMULATING MOTOR FAULT for {duration}s")
            
            # Send coordination hormone injection
            self.send_hormone_injection(
                hormone_type=1,  # Coordination hormone
                amount=150,      # High coordination stress
                reason=f"Motor fault simulation on drone {self.drone_id}"
            )
            
            await asyncio.sleep(duration)
            
            self.logger.info(f"✓ Motor fault simulation complete for drone {self.drone_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Motor fault simulation failed: {e}")
            return False
    
    async def inject_critical_stress(self, level=255, duration=5):
        """Inject critical stress hormone"""
        try:
            self.logger.warning(f"🚨 INJECTING CRITICAL STRESS (level {level}) for {duration}s")
            
            # Send critical stress hormone
            self.send_hormone_injection(
                hormone_type=0,  # Stress hormone
                amount=level,    # Critical level
                reason=f"Critical stress injection on drone {self.drone_id}"
            )
            
            await asyncio.sleep(duration)
            
            self.logger.info(f"✓ Critical stress injection complete for drone {self.drone_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Critical stress injection failed: {e}")
            return False

class IntegratedScenarioRunner:
    """Runs integrated fault scenarios"""
    
    def __init__(self):
        self.logger = logging.getLogger("IntegratedScenarios")
        
    async def scenario_sequential_gps_faults(self, drone_configs, fault_duration=10):
        """Scenario: Sequential GPS faults across drones"""
        self.logger.info("🎬 SCENARIO: Sequential GPS Faults")
        
        for i, config in enumerate(drone_configs):
            self.logger.info(f"🎯 Targeting drone {config['id']} for GPS fault")
            
            injector = IntegratedFaultInjector(config["id"], config["port"])
            
            if await injector.connect():
                await injector.inject_gps_fault_simulation(duration=fault_duration)
                
                # Wait before next fault
                if i < len(drone_configs) - 1:
                    await asyncio.sleep(3)  # 3 second delay between faults
        
        self.logger.info("✅ Sequential GPS fault scenario complete")
    
    async def scenario_simultaneous_motor_faults(self, drone_configs):
        """Scenario: Simultaneous motor faults"""
        self.logger.info("🎬 SCENARIO: Simultaneous Motor Faults")
        
        tasks = []
        for config in drone_configs:
            async def motor_fault(drone_config):
                injector = IntegratedFaultInjector(drone_config["id"], drone_config["port"])
                if await injector.connect():
                    await injector.inject_motor_fault_simulation(duration=8)
            
            task = asyncio.create_task(motor_fault(config))
            tasks.append(task)
        
        await asyncio.gather(*tasks)
        self.logger.info("✅ Simultaneous motor fault scenario complete")
    
    async def scenario_escalating_stress(self, drone_configs):
        """Scenario: Escalating stress levels"""
        self.logger.info("🎬 SCENARIO: Escalating Stress Levels")
        
        stress_levels = [100, 150, 200, 255]
        
        for i, config in enumerate(drone_configs):
            stress_level = stress_levels[i % len(stress_levels)]
            
            self.logger.info(f"🎯 Drone {config['id']} - Stress level {stress_level}")
            
            injector = IntegratedFaultInjector(config["id"], config["port"])
            if await injector.connect():
                await injector.inject_critical_stress(level=stress_level, duration=6)
                await asyncio.sleep(2)  # Brief delay between injections
        
        self.logger.info("✅ Escalating stress scenario complete")

async def main():
    """Main integrated fault injection test"""
    parser = argparse.ArgumentParser(description='Integrated Fault Injector for Hormone Swarm')
    parser.add_argument('--scenario', 
                       choices=['sequential_gps', 'simultaneous_motor', 'escalating_stress'], 
                       default='sequential_gps', 
                       help='Fault scenario to run')
    parser.add_argument('--drones', type=int, default=2, help='Number of drones')
    
    args = parser.parse_args()
    
    # Configure drone ports (match your running instances)
    drone_configs = [
        {"id": 0, "port": 14540},
        {"id": 1, "port": 14541},
    ][:args.drones]
    
    # Create scenario runner
    runner = IntegratedScenarioRunner()
    
    logger = logging.getLogger("Main")
    logger.info(f"🚀 Starting integrated fault injection scenario: {args.scenario}")
    logger.info(f"🎯 Target drones: {[c['id'] for c in drone_configs]}")
    logger.info("💡 Make sure your hormone system (real_multi_drone_test.py) is running!")
    
    try:
        if args.scenario == 'sequential_gps':
            await runner.scenario_sequential_gps_faults(drone_configs)
        elif args.scenario == 'simultaneous_motor':
            await runner.scenario_simultaneous_motor_faults(drone_configs)
        elif args.scenario == 'escalating_stress':
            await runner.scenario_escalating_stress(drone_configs)
            
    except Exception as e:
        logger.error(f"Scenario failed: {e}")
    
    logger.info("🏁 Integrated fault injection complete!")

if __name__ == "__main__":
    asyncio.run(main())