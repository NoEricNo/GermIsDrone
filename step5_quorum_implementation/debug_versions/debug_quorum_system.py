#!/usr/bin/env python3
"""
DEBUG VERSION: Bio-Inspired Swarm Coordination - Step 5: Quorum Rules
This version has extensive debugging to find where the coordination loop fails
"""

import asyncio
import numpy as np
import time
import socket
import json
import logging
from dataclasses import dataclass
from typing import Dict, List, Optional

# Configure detailed logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class QuorumDecision:
    """Represents a collective decision made by the swarm"""
    decision_type: str
    confidence: float
    participating_drones: List[int]
    timestamp: float

class DebugHormoneReceiver:
    """Simplified drone for debugging"""
    
    def __init__(self, drone_id: int):
        self.drone_id = drone_id
        self.hormone_state = np.zeros(20, dtype=np.float32)
        logger.debug(f"[Drone{drone_id}] Initialized with zero hormone state")
        
    def get_stress_level(self) -> float:
        """Return current stress level (0-255)"""
        stress = float(np.mean(self.hormone_state))
        logger.debug(f"[Drone{self.drone_id}] Current stress level: {stress:.1f}")
        return stress
    
    def is_stressed(self, threshold=80.0) -> bool:
        """Am I currently stressed?"""
        stress = self.get_stress_level()
        stressed = stress > threshold
        logger.debug(f"[Drone{self.drone_id}] Stressed check (>{threshold}): {stressed}")
        return stressed

class DebugSwarmCoordinator:
    """Simplified coordinator with extensive debugging"""
    
    def __init__(self, udp_port=9999):
        self.drones: Dict[int, DebugHormoneReceiver] = {}
        self.udp_port = udp_port
        self.running = False
        
        # Simplified quorum rules for testing
        self.formation_threshold = 80.0
        self.emergency_threshold = 150.0
        self.formation_quorum = 0.4  # 40% of drones
        self.emergency_quorum = 0.6  # 60% of drones
        
        logger.info(f"🧠 DebugSwarmCoordinator initialized")
        logger.info(f"   Formation rule: {self.formation_quorum:.0%} of drones > {self.formation_threshold}")
        logger.info(f"   Emergency rule: {self.emergency_quorum:.0%} of drones > {self.emergency_threshold}")
        
    def add_drone(self, drone: DebugHormoneReceiver):
        """Add a drone to the swarm"""
        self.drones[drone.drone_id] = drone
        logger.info(f"🤝 Added Drone{drone.drone_id} to swarm (total: {len(self.drones)})")
        
    async def check_quorum_rules(self):
        """Check if any quorum thresholds are met"""
        logger.debug("🔍 Starting quorum rule check...")
        
        if not self.drones:
            logger.debug("❌ No drones in swarm, skipping quorum check")
            return
            
        total_drones = len(self.drones)
        stress_levels = {}
        
        # Collect stress levels
        logger.debug(f"📊 Collecting stress levels from {total_drones} drones...")
        for drone_id, drone in self.drones.items():
            stress = drone.get_stress_level()
            stress_levels[drone_id] = stress
            logger.debug(f"   Drone{drone_id}: {stress:.1f}")
        
        # Check formation hold rule
        formation_count = sum(1 for stress in stress_levels.values() if stress > self.formation_threshold)
        formation_percentage = formation_count / total_drones
        formation_needed = int(total_drones * self.formation_quorum)
        
        logger.debug(f"🤝 Formation check: {formation_count}/{total_drones} stressed (need {formation_needed})")
        
        if formation_count >= formation_needed:
            logger.warning(f"🗳️ FORMATION QUORUM REACHED!")
            logger.warning(f"   📊 {formation_count}/{total_drones} drones above {self.formation_threshold}")
            logger.warning(f"   🎯 Percentage: {formation_percentage:.1%} (need {self.formation_quorum:.1%})")
            await self.execute_formation_hold()
        
        # Check emergency land rule
        emergency_count = sum(1 for stress in stress_levels.values() if stress > self.emergency_threshold)
        emergency_percentage = emergency_count / total_drones
        emergency_needed = int(total_drones * self.emergency_quorum)
        
        logger.debug(f"🚨 Emergency check: {emergency_count}/{total_drones} stressed (need {emergency_needed})")
        
        if emergency_count >= emergency_needed:
            logger.error(f"🗳️ EMERGENCY QUORUM REACHED!")
            logger.error(f"   📊 {emergency_count}/{total_drones} drones above {self.emergency_threshold}")
            logger.error(f"   🎯 Percentage: {emergency_percentage:.1%} (need {self.emergency_quorum:.1%})")
            await self.execute_emergency_land()
            
        logger.debug("✅ Quorum rule check complete")
        
    async def execute_formation_hold(self):
        """Execute formation hold"""
        logger.warning(f"🤝 FORMATION HOLD PROTOCOL ACTIVATED")
        logger.warning(f"   🛑 All {len(self.drones)} drones should hold position")
        
    async def execute_emergency_land(self):
        """Execute emergency landing"""
        logger.error(f"🚨 EMERGENCY LAND PROTOCOL ACTIVATED")
        logger.error(f"   ✈️ All {len(self.drones)} drones should land immediately")
    
    async def udp_hormone_injector(self):
        """UDP injection system with debugging"""
        logger.info(f"🩸 Starting UDP hormone injector on port {self.udp_port}")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('localhost', self.udp_port))
        sock.settimeout(0.1)
        
        logger.info(f"🩸 UDP hormone injector listening...")
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                message = json.loads(data.decode())
                await self.process_external_injection(message)
            except socket.timeout:
                # This is normal - just continue
                pass
            except Exception as e:
                logger.error(f"UDP injection error: {e}")
                
        sock.close()
        logger.info("🩸 UDP hormone injector stopped")
        
    async def process_external_injection(self, message):
        """Process external hormone injection with debugging"""
        drone_id = message.get('drone_id')
        hormone_type = message.get('hormone_type', 0)
        amount = message.get('amount', 100)
        
        logger.debug(f"📨 Received injection: Drone{drone_id}, Type{hormone_type}, Amount{amount}")
        
        if drone_id in self.drones:
            drone = self.drones[drone_id]
            old_value = drone.hormone_state[hormone_type]
            drone.hormone_state[hormone_type] = min(255, amount)
            new_stress = drone.get_stress_level()
            
            logger.info(f"🩸 EXTERNAL HORMONE INJECTION: Drone{drone_id} | Type: {hormone_type}, Amount: {amount}")
            logger.debug(f"   Hormone[{hormone_type}]: {old_value} → {amount}")
            logger.debug(f"   New mean stress: {new_stress:.1f}")
        else:
            logger.warning(f"❌ Unknown drone ID: {drone_id}")
        
    async def display_swarm_state(self):
        """Display current swarm state"""
        if not self.drones:
            logger.debug("📊 No drones to display")
            return
            
        stress_levels = [drone.get_stress_level() for drone in self.drones.values()]
        overall_stress = np.mean(stress_levels)
        
        high_stress_count = sum(1 for s in stress_levels if s > self.emergency_threshold)
        med_stress_count = sum(1 for s in stress_levels if s > self.formation_threshold)
        total_drones = len(self.drones)
        
        drone_states = " ".join([
            f"D{drone_id}:{stress:.1f}" 
            for drone_id, stress in zip(self.drones.keys(), stress_levels)
        ])
        
        logger.info(f"🌐 Swarm State | Overall: {overall_stress:.1f} | Individual: {drone_states}")
        logger.info(f"📊 Quorum Status | High Stress: {high_stress_count}/{total_drones} | Med Stress: {med_stress_count}/{total_drones}")
        
    async def coordination_loop(self):
        """Main coordination loop with extensive debugging"""
        logger.info("🧠 Starting coordination loop...")
        self.running = True
        
        # Start UDP listener in background
        logger.debug("🔧 Starting UDP listener task...")
        udp_task = asyncio.create_task(self.udp_hormone_injector())
        logger.debug("✅ UDP listener task started")
        
        loop_count = 0
        
        while self.running:
            try:
                logger.debug(f"🔄 Coordination loop iteration {loop_count}")
                
                # Display current state
                logger.debug("📊 Updating swarm display...")
                await self.display_swarm_state()
                
                # Check quorum rules
                logger.debug("🗳️ Checking quorum rules...")
                await self.check_quorum_rules()
                
                # Sleep before next iteration
                logger.debug("💤 Sleeping 1 second...")
                await asyncio.sleep(1.0)  # Slower for easier debugging
                
                loop_count += 1
                
            except Exception as e:
                logger.error(f"❌ Coordination loop error: {e}")
                logger.exception("Full exception details:")
                break
                
        logger.info("🛑 Coordination loop stopped")
        udp_task.cancel()

# Main execution
async def run_debug_system():
    """Run the debug version"""
    logger.info("🧬 Starting DEBUG Bio-Inspired Swarm Coordination")
    
    coordinator = DebugSwarmCoordinator()
    
    # Create 3 test drones
    for i in range(3):
        drone = DebugHormoneReceiver(drone_id=i)
        coordinator.add_drone(drone)
    
    logger.info("🚀 Starting coordination system...")
    
    try:
        await coordinator.coordination_loop()
    except KeyboardInterrupt:
        logger.info("🛑 Received Ctrl+C, shutting down...")
        coordinator.running = False
    except Exception as e:
        logger.error(f"❌ System error: {e}")
        logger.exception("Full exception details:")

if __name__ == "__main__":
    print("🔬 DEBUG VERSION - Bio-Inspired Swarm Coordination")
    print("This version has extensive logging to diagnose issues")
    print("💡 Press Ctrl+C to stop")
    print()
    
    asyncio.run(run_debug_system())