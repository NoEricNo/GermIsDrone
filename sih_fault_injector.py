#!/usr/bin/env python3
"""
SIH-Compatible Fault Injector for Hormone Swarm System
Modified to work with SIH simulator (no GPS blocking available)
"""

import asyncio
import time
import logging
import argparse
from mavsdk import System

logging.basicConfig(level=logging.INFO, format='%(asctime)s - [%(name)s] - %(levelname)s - %(message)s')

class SIHFaultInjector:
    """Fault injector compatible with SIH simulator"""
    
    def __init__(self, drone_id, port):
        self.drone_id = drone_id
        self.port = port
        self.drone = System()
        self.logger = logging.getLogger(f"SIHFaultInjector-{drone_id}")
        self.is_connected = False
        
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
    
    async def inject_simulated_gps_fault(self, duration=10, hormone_trigger_callback=None):
        """
        Simulate GPS fault without actually blocking GPS
        (SIH doesn't support SIM_GPS_BLOCK parameter)
        """
        if not self.is_connected:
            self.logger.error("Not connected to drone")
            return False
            
        try:
            self.logger.warning(f"🛰️ SIMULATING GPS FAULT for {duration}s")
            
            # Trigger hormone response immediately (simulated fault)
            if hormone_trigger_callback:
                await hormone_trigger_callback(
                    drone_id=self.drone_id,
                    fault_type="gps_loss_simulated",
                    hormone_type=0,  # Stress hormone
                    hormone_amount=200,  # High stress
                    reason="Simulated GPS signal lost"
                )
            
            # Wait for fault duration (just simulation time)
            await asyncio.sleep(duration)
            
            self.logger.info(f"✓ Simulated GPS fault cleared for drone {self.drone_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Simulated GPS fault injection failed: {e}")
            return False
    
    async def inject_flight_mode_change(self, target_mode="HOLD", duration=8, hormone_trigger_callback=None):
        """
        Force flight mode change to simulate coordination stress
        """
        if not self.is_connected:
            self.logger.error("Not connected to drone")
            return False
            
        try:
            self.logger.warning(f"✈️ FORCING FLIGHT MODE to {target_mode} for {duration}s")
            
            # Force HOLD mode (this works in SIH)
            await self.drone.action.hold()
            
            # Trigger hormone response
            if hormone_trigger_callback:
                await hormone_trigger_callback(
                    drone_id=self.drone_id,
                    fault_type="forced_mode_change",
                    hormone_type=1,  # Coordination hormone
                    hormone_amount=150,
                    reason=f"Forced mode change to {target_mode}"
                )
            
            await asyncio.sleep(duration)
            
            self.logger.info(f"✓ Flight mode fault cleared for drone {self.drone_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Flight mode fault injection failed: {e}")
            return False
    
    async def inject_simulated_battery_critical(self, duration=6, hormone_trigger_callback=None):
        """
        Simulate battery critical condition
        """
        try:
            self.logger.warning(f"🔋 SIMULATING BATTERY CRITICAL for {duration}s")
            
            # Trigger hormone response
            if hormone_trigger_callback:
                await hormone_trigger_callback(
                    drone_id=self.drone_id,
                    fault_type="battery_critical_simulated",
                    hormone_type=0,  # Stress hormone
                    hormone_amount=180,
                    reason="Simulated battery critical: 15%"
                )
            
            await asyncio.sleep(duration)
            
            self.logger.info(f"✓ Simulated battery fault cleared for drone {self.drone_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Simulated battery fault injection failed: {e}")
            return False
    
    async def inject_high_stress_hormone(self, hormone_level=255, duration=5, hormone_trigger_callback=None):
        """
        Direct hormone injection for testing
        """
        try:
            self.logger.warning(f"💉 INJECTING HIGH STRESS HORMONE (level {hormone_level}) for {duration}s")
            
            # Trigger hormone response
            if hormone_trigger_callback:
                await hormone_trigger_callback(
                    drone_id=self.drone_id,
                    fault_type="direct_hormone_injection",
                    hormone_type=0,  # Stress hormone
                    hormone_amount=hormone_level,
                    reason=f"Direct stress hormone injection: {hormone_level}"
                )
            
            await asyncio.sleep(duration)
            
            self.logger.info(f"✓ Direct hormone injection complete for drone {self.drone_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Direct hormone injection failed: {e}")
            return False

class SIHFaultScenarioRunner:
    """Runs SIH-compatible fault scenarios"""
    
    def __init__(self, hormone_system_callback=None):
        self.hormone_callback = hormone_system_callback
        self.logger = logging.getLogger("SIHFaultScenarios")
        
    async def scenario_simulated_gps_fault(self, drone_configs, fault_duration=10):
        """Scenario: Simulated GPS fault"""
        self.logger.info("🎬 SCENARIO: Simulated GPS Fault")
        
        # Pick first drone for fault
        target_drone = drone_configs[0]
        injector = SIHFaultInjector(target_drone["id"], target_drone["port"])
        
        if await injector.connect():
            await injector.inject_simulated_gps_fault(
                duration=fault_duration,
                hormone_trigger_callback=self.hormone_callback
            )
        
        self.logger.info("✅ Simulated GPS fault scenario complete")
    
    async def scenario_cascading_stress(self, drone_configs, delay_between=3):
        """Scenario: Cascading stress hormone injection"""
        self.logger.info("🎬 SCENARIO: Cascading Stress Hormones")
        
        tasks = []
        for i, config in enumerate(drone_configs):
            # Stagger the stress injections
            delay = i * delay_between
            stress_level = 200 + (i * 25)  # Increasing stress levels
            
            async def delayed_stress_injection(drone_config, fault_delay, stress):
                await asyncio.sleep(fault_delay)
                injector = SIHFaultInjector(drone_config["id"], drone_config["port"])
                if await injector.connect():
                    await injector.inject_high_stress_hormone(
                        hormone_level=stress,
                        duration=8,
                        hormone_trigger_callback=self.hormone_callback
                    )
            
            task = asyncio.create_task(delayed_stress_injection(config, delay, stress_level))
            tasks.append(task)
        
        await asyncio.gather(*tasks)
        self.logger.info("✅ Cascading stress scenario complete")
    
    async def scenario_coordinated_mode_changes(self, drone_configs):
        """Scenario: Force all drones to HOLD mode"""
        self.logger.info("🎬 SCENARIO: Coordinated Mode Changes")
        
        tasks = []
        for config in drone_configs:
            async def force_mode_change(drone_config):
                injector = SIHFaultInjector(drone_config["id"], drone_config["port"])
                if await injector.connect():
                    await injector.inject_flight_mode_change(
                        target_mode="HOLD",
                        duration=10,
                        hormone_trigger_callback=self.hormone_callback
                    )
            
            task = asyncio.create_task(force_mode_change(config))
            tasks.append(task)
        
        await asyncio.gather(*tasks)
        self.logger.info("✅ Coordinated mode change scenario complete")

# Global variable to store swarm coordinator reference
swarm_coordinator_ref = None

async def hormone_injection_callback(drone_id, fault_type, hormone_type, hormone_amount, reason):
    """
    Callback function to inject hormones into your existing hormone system
    
    This needs to be connected to your actual DroneHormoneNode
    """
    logger = logging.getLogger("HormoneInjection")
    logger.critical(f"🩸 HORMONE INJECTION: Drone {drone_id} | {fault_type} | "
                   f"Type: {hormone_type}, Amount: {hormone_amount} | Reason: {reason}")
    
    # TODO: Connect this to your actual SwarmCoordinator
    # You'll need to modify real_multi_drone_test.py to expose the coordinator
    if swarm_coordinator_ref:
        # This would call your DroneHormoneNode.inject_hormone() method
        logger.info(f"📡 Forwarding hormone injection to swarm coordinator")
        # await swarm_coordinator_ref.inject_external_hormone(drone_id, hormone_type, hormone_amount, reason)
    else:
        logger.warning("⚠️ No swarm coordinator reference - hormone injection simulated only")

async def main():
    """Main fault injection test"""
    parser = argparse.ArgumentParser(description='SIH-Compatible Fault Injector for Hormone Swarm')
    parser.add_argument('--scenario', choices=['simulated_gps', 'cascading_stress', 'coordinated_modes'], 
                       default='simulated_gps', help='Fault scenario to run')
    parser.add_argument('--drones', type=int, default=1, help='Number of drones (adjust based on running instances)')
    
    args = parser.parse_args()
    
    # Configure drone ports (adjust based on your running PX4 instances)
    drone_configs = [
        {"id": 0, "port": 14540},  # Your running PX4 instance
        # {"id": 1, "port": 14541},  # Uncomment if you start second instance
    ][:args.drones]
    
    # Create scenario runner
    runner = SIHFaultScenarioRunner(hormone_system_callback=hormone_injection_callback)
    
    logger = logging.getLogger("Main")
    logger.info(f"🚀 Starting SIH fault injection scenario: {args.scenario}")
    logger.info(f"🎯 Target drones: {[c['id'] for c in drone_configs]}")
    
    try:
        if args.scenario == 'simulated_gps':
            await runner.scenario_simulated_gps_fault(drone_configs)
        elif args.scenario == 'cascading_stress':
            await runner.scenario_cascading_stress(drone_configs)
        elif args.scenario == 'coordinated_modes':
            await runner.scenario_coordinated_mode_changes(drone_configs)
            
    except Exception as e:
        logger.error(f"Scenario failed: {e}")
    
    logger.info("🏁 SIH fault injection complete!")

if __name__ == "__main__":
    asyncio.run(main())