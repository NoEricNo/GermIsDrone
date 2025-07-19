#!/usr/bin/env python3
"""
FIXED VERSION: Bio-Inspired Swarm Coordination - Step 5: Quorum Rules
This version fixes the coordination loop async issue
"""

import asyncio
import numpy as np
import time
import socket
import json
import logging
from dataclasses import dataclass
from typing import Dict, List, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
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

class WorkingHormoneReceiver:
    """Simplified drone for working system"""
    
    def __init__(self, drone_id: int):
        self.drone_id = drone_id
        self.hormone_state = np.zeros(20, dtype=np.float32)
        
    def get_stress_level(self) -> float:
        """Return current stress level (0-255)"""
        return float(np.mean(self.hormone_state))
    
    def is_stressed(self, threshold=80.0) -> bool:
        """Am I currently stressed?"""
        return self.get_stress_level() > threshold

class WorkingSwarmCoordinator:
    """Working coordinator with proper async handling"""
    
    def __init__(self, udp_port=9999):
        self.drones: Dict[int, WorkingHormoneReceiver] = {}
        self.udp_port = udp_port
        self.running = False
        
        # Quorum rules
        self.formation_threshold = 80.0
        self.emergency_threshold = 150.0
        self.formation_quorum = 0.4  # 40% of drones
        self.emergency_quorum = 0.6  # 60% of drones
        
        logger.info(f"🧠 WorkingSwarmCoordinator initialized")
        logger.info(f"   Formation rule: {self.formation_quorum:.0%} of drones > {self.formation_threshold}")
        logger.info(f"   Emergency rule: {self.emergency_quorum:.0%} of drones > {self.emergency_threshold}")
        
    def add_drone(self, drone: WorkingHormoneReceiver):
        """Add a drone to the swarm"""
        self.drones[drone.drone_id] = drone
        logger.info(f"🤝 Added Drone{drone.drone_id} to swarm (total: {len(self.drones)})")
        
    async def check_quorum_rules(self):
        """Check if any quorum thresholds are met"""
        if not self.drones:
            return
            
        total_drones = len(self.drones)
        stress_levels = {drone_id: drone.get_stress_level() for drone_id, drone in self.drones.items()}
        
        # Check formation hold rule
        formation_count = sum(1 for stress in stress_levels.values() if stress > self.formation_threshold)
        formation_needed = max(1, int(total_drones * self.formation_quorum))
        
        if formation_count >= formation_needed:
            formation_percentage = formation_count / total_drones
            logger.warning(f"🗳️ FORMATION QUORUM REACHED!")
            logger.warning(f"   📊 {formation_count}/{total_drones} drones above {self.formation_threshold}")
            logger.warning(f"   🎯 Percentage: {formation_percentage:.1%} (need {self.formation_quorum:.1%})")
            await self.execute_formation_hold()
        
        # Check emergency land rule
        emergency_count = sum(1 for stress in stress_levels.values() if stress > self.emergency_threshold)
        emergency_needed = max(1, int(total_drones * self.emergency_quorum))
        
        if emergency_count >= emergency_needed:
            emergency_percentage = emergency_count / total_drones
            logger.error(f"🗳️ EMERGENCY QUORUM REACHED!")
            logger.error(f"   📊 {emergency_count}/{total_drones} drones above {self.emergency_threshold}")
            logger.error(f"   🎯 Percentage: {emergency_percentage:.1%} (need {self.emergency_quorum:.1%})")
            await self.execute_emergency_land()
            
    async def execute_formation_hold(self):
        """Execute formation hold"""
        logger.warning(f"🤝 FORMATION HOLD PROTOCOL ACTIVATED")
        logger.warning(f"   🛑 All {len(self.drones)} drones should hold position")
        
    async def execute_emergency_land(self):
        """Execute emergency landing"""
        logger.error(f"🚨 EMERGENCY LAND PROTOCOL ACTIVATED")
        logger.error(f"   ✈️ All {len(self.drones)} drones should land immediately")
    
    async def udp_hormone_injector(self):
        """UDP injection system - RUNS IN BACKGROUND"""
        logger.info(f"🩸 Starting UDP hormone injector on port {self.udp_port}")
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('localhost', self.udp_port))
            sock.settimeout(0.1)  # Non-blocking with short timeout
            
            logger.info(f"🩸 UDP hormone injector listening on port {self.udp_port}...")
            
            while self.running:
                try:
                    data, addr = sock.recvfrom(1024)
                    message = json.loads(data.decode())
                    await self.process_external_injection(message)
                except socket.timeout:
                    # This is normal - continue to allow other async tasks
                    await asyncio.sleep(0.01)  # Small yield to other tasks
                    continue
                except json.JSONDecodeError:
                    logger.warning("Invalid JSON received via UDP")
                except Exception as e:
                    logger.error(f"UDP injection error: {e}")
                    
            sock.close()
            logger.info("🩸 UDP hormone injector stopped (normal shutdown)")
            
        except Exception as e:
            logger.error(f"❌ UDP injector failed to start: {e}")
            logger.exception("UDP startup error details:")
            raise
        
    async def process_external_injection(self, message):
        """Process external hormone injection"""
        drone_id = message.get('drone_id')
        hormone_type = message.get('hormone_type', 0)
        amount = message.get('amount', 100)
        
        if drone_id in self.drones:
            drone = self.drones[drone_id]
            drone.hormone_state[hormone_type] = min(255, amount)
            new_stress = drone.get_stress_level()
            
            logger.info(f"🩸 EXTERNAL HORMONE INJECTION: Drone{drone_id} | Type: {hormone_type}, Amount: {amount}")
            logger.info(f"   💊 New mean stress: {new_stress:.1f}")
        else:
            logger.warning(f"❌ Unknown drone ID: {drone_id}")
        
    async def display_swarm_state(self):
        """Display current swarm state"""
        if not self.drones:
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
        """Main coordination loop - PROPERLY ASYNC"""
        logger.info("🧠 Starting coordination loop...")
        self.running = True
        
        loop_count = 0
        
        while self.running:
            try:
                logger.info(f"🔄 Coordination loop iteration {loop_count}")
                
                # Display current state
                await self.display_swarm_state()
                
                # Check quorum rules
                await self.check_quorum_rules()
                
                # Sleep before next iteration
                await asyncio.sleep(2.0)  # 2 second intervals
                
                loop_count += 1
                
            except Exception as e:
                logger.error(f"❌ Coordination loop error: {e}")
                break
                
        logger.info("🛑 Coordination loop stopped")

# Main execution with SIMPLIFIED async structure
async def run_working_system():
    """Run the working version with proper async"""
    logger.info("🧬 Starting WORKING Bio-Inspired Swarm Coordination")
    
    coordinator = WorkingSwarmCoordinator()
    
    # Create 3 test drones
    for i in range(3):
        drone = WorkingHormoneReceiver(drone_id=i)
        coordinator.add_drone(drone)
    
    logger.info("🚀 Starting coordination system...")
    
    try:
        # Start UDP listener in background
        udp_task = asyncio.create_task(coordinator.udp_hormone_injector())
        
        # Run coordination loop in main thread (this keeps the program alive)
        await coordinator.coordination_loop()
        
    except KeyboardInterrupt:
        logger.info("🛑 Received Ctrl+C, shutting down...")
        coordinator.running = False
        
    except Exception as e:
        logger.error(f"❌ System error: {e}")
        logger.exception("Full exception details:")
        coordinator.running = False
    
    # Clean up
    if 'udp_task' in locals():
        udp_task.cancel()
        try:
            await udp_task
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    print("🎯 WORKING VERSION - Bio-Inspired Swarm Coordination")
    print("This version fixes the coordination loop async issue")
    print("💡 Press Ctrl+C to stop")
    print()
    
    asyncio.run(run_working_system())