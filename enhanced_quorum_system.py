#!/usr/bin/env python3
"""
Bio-Inspired Swarm Coordination - Step 5: Quorum Rules Extension
Enhanced version of your existing hormone system with collective decision-making
"""

import asyncio
import numpy as np
import time
import socket
import json
import logging
from dataclasses import dataclass
from typing import Dict, List, Optional
from mavsdk import System

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class QuorumDecision:
    """Represents a collective decision made by the swarm"""
    decision_type: str  # 'emergency_land', 'hold_formation', 'return_home'
    confidence: float   # 0.0 to 1.0
    participating_drones: List[int]
    timestamp: float

class EnhancedHormoneReceiver:
    """Individual drone with hormone processing + quorum awareness"""
    
    def __init__(self, drone_id: int, decay_rate=0.9):
        self.drone_id = drone_id
        self.decay_rate = decay_rate
        self.hormone_state = np.zeros(20, dtype=np.float32)
        self.last_update = time.time()
        self.drone = System()
        
        # NEW: Quorum-related state
        self.stress_threshold = 80.0  # When I consider myself "stressed"
        self.last_quorum_check = time.time()
        self.recent_decisions = []  # Track recent quorum decisions
        
    async def connect(self, port=14540):
        """Connect to drone"""
        await self.drone.connect(f"udp://:{port}")
        logger.info(f"[Drone{self.drone_id}] Connected to port {port}")
        
    def is_stressed(self) -> bool:
        """Am I currently stressed? (Simple individual assessment)"""
        mean_hormone = np.mean(self.hormone_state)
        return mean_hormone > self.stress_threshold
    
    def get_stress_level(self) -> float:
        """Return current stress level (0-255)"""
        return np.mean(self.hormone_state)
    
    async def hormone_listener(self):
        """Main hormone processing loop (your existing logic + quorum checks)"""
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                await self.update_hormones()
                # Your existing hormone logic continues to work...
                
        except Exception as e:
            logger.error(f"[Drone{self.drone_id}] Hormone listener error: {e}")
            
    async def update_hormones(self):
        """Apply decay and update hormone state (your existing logic)"""
        now = time.time()
        dt = now - self.last_update
        
        # Exponential decay (your existing algorithm)
        decay_factor = self.decay_rate ** dt
        self.hormone_state *= decay_factor
        
        self.last_update = now

class SwarmQuorumCoordinator:
    """Enhanced coordinator with collective decision-making capabilities"""
    
    def __init__(self, udp_port=9999):
        self.drones: Dict[int, EnhancedHormoneReceiver] = {}
        self.udp_port = udp_port
        self.running = False
        
        # NEW: Quorum rule parameters
        self.quorum_rules = {
            'emergency_land': {
                'stress_threshold': 150.0,  # High stress level
                'min_percentage': 0.6,      # 60% of drones must agree
                'min_absolute': 2,          # At least 2 drones
                'action': self.execute_emergency_land
            },
            'formation_hold': {
                'stress_threshold': 80.0,   # Medium stress level  
                'min_percentage': 0.4,      # 40% of drones must agree
                'min_absolute': 1,          # At least 1 drone
                'action': self.execute_formation_hold
            }
        }
        
        self.last_quorum_check = time.time()
        self.quorum_check_interval = 2.0  # Check every 2 seconds
        
    def add_drone(self, drone: EnhancedHormoneReceiver):
        """Add a drone to the swarm"""
        self.drones[drone.drone_id] = drone
        logger.info(f"🤝 Added Drone{drone.drone_id} to swarm (total: {len(self.drones)})")
        
    async def check_quorum_rules(self):
        """NEW: Check if any quorum thresholds are met"""
        if not self.drones:
            return
            
        now = time.time()
        if now - self.last_quorum_check < self.quorum_check_interval:
            return
            
        total_drones = len(self.drones)
        
        # Collect current stress levels from all drones
        stress_levels = {}
        for drone_id, drone in self.drones.items():
            stress_levels[drone_id] = drone.get_stress_level()
            
        # Check each quorum rule
        for rule_name, rule_config in self.quorum_rules.items():
            await self._evaluate_quorum_rule(rule_name, rule_config, stress_levels, total_drones)
            
        self.last_quorum_check = now
        
    async def _evaluate_quorum_rule(self, rule_name: str, rule_config: dict, 
                                   stress_levels: dict, total_drones: int):
        """Evaluate a specific quorum rule"""
        threshold = rule_config['stress_threshold']
        min_percentage = rule_config['min_percentage']
        min_absolute = rule_config['min_absolute']
        
        # Count drones meeting the stress threshold
        stressed_drones = [
            drone_id for drone_id, stress in stress_levels.items() 
            if stress > threshold
        ]
        
        stressed_count = len(stressed_drones)
        stressed_percentage = stressed_count / total_drones if total_drones > 0 else 0
        
        # Check if quorum is met
        quorum_met = (
            stressed_count >= min_absolute and 
            stressed_percentage >= min_percentage
        )
        
        if quorum_met:
            confidence = min(1.0, stressed_percentage / min_percentage)
            
            decision = QuorumDecision(
                decision_type=rule_name,
                confidence=confidence,
                participating_drones=stressed_drones,
                timestamp=time.time()
            )
            
            logger.warning(f"🗳️ QUORUM REACHED: {rule_name}")
            logger.warning(f"   📊 {stressed_count}/{total_drones} drones stressed ({stressed_percentage:.1%})")
            logger.warning(f"   🎯 Confidence: {confidence:.2f}")
            logger.warning(f"   🤖 Participating: {stressed_drones}")
            
            # Execute the collective action
            await rule_config['action'](decision)
            
    async def execute_emergency_land(self, decision: QuorumDecision):
        """Execute emergency landing for all drones"""
        logger.error(f"🚨 EMERGENCY LAND PROTOCOL ACTIVATED")
        logger.error(f"   Reason: {len(decision.participating_drones)} drones critically stressed")
        
        # Command all drones to land
        for drone_id, drone in self.drones.items():
            try:
                await drone.drone.action.land()
                logger.error(f"   ✈️ Drone{drone_id}: Emergency land command sent")
            except Exception as e:
                logger.error(f"   ❌ Drone{drone_id}: Land command failed - {e}")
                
    async def execute_formation_hold(self, decision: QuorumDecision):
        """Execute formation hold for all drones"""
        logger.warning(f"🤝 FORMATION HOLD PROTOCOL ACTIVATED")
        logger.warning(f"   Reason: {len(decision.participating_drones)} drones stressed")
        
        # Command all drones to hold position
        for drone_id, drone in self.drones.items():
            try:
                await drone.drone.action.hold()
                logger.warning(f"   🛑 Drone{drone_id}: Hold position command sent")
            except Exception as e:
                logger.warning(f"   ❌ Drone{drone_id}: Hold command failed - {e}")
    
    async def udp_hormone_injector(self):
        """Your existing UDP injection system (unchanged)"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('localhost', self.udp_port))
        sock.settimeout(0.1)
        
        logger.info(f"🩸 UDP hormone injector listening on port {self.udp_port}")
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                message = json.loads(data.decode())
                await self.process_external_injection(message)
            except socket.timeout:
                pass
            except Exception as e:
                logger.error(f"UDP injection error: {e}")
                
        sock.close()
        
    async def process_external_injection(self, message):
        """Process external hormone injection (your existing logic)"""
        drone_id = message.get('drone_id')
        hormone_type = message.get('hormone_type', 0)
        amount = message.get('amount', 100)
        
        if drone_id in self.drones:
            drone = self.drones[drone_id]
            drone.hormone_state[hormone_type] = min(255, amount)
            logger.info(f"🩸 EXTERNAL HORMONE INJECTION: Drone{drone_id} | Type: {hormone_type}, Amount: {amount}")
        
    async def coordination_loop(self):
        """Main coordination loop with quorum checking"""
        logger.info("🧠 SwarmQuorumCoordinator started")
        self.running = True
        
        # Start UDP listener
        udp_task = asyncio.create_task(self.udp_hormone_injector())
        
        while self.running:
            try:
                # Your existing hormone propagation logic...
                await self.propagate_hormones()
                
                # NEW: Check quorum rules
                await self.check_quorum_rules()
                
                # Update display
                await self.display_swarm_state()
                
                await asyncio.sleep(0.5)
                
            except Exception as e:
                logger.error(f"Coordination loop error: {e}")
                
        udp_task.cancel()
        
    async def propagate_hormones(self):
        """Your existing hormone propagation (unchanged)"""
        if len(self.drones) < 2:
            return
            
        # Apply neighbor influence (30% factor)
        influence_factor = 0.3
        
        for drone_id, drone in self.drones.items():
            neighbors = [d for d_id, d in self.drones.items() if d_id != drone_id]
            
            if neighbors:
                neighbor_avg = np.mean([d.hormone_state for d in neighbors], axis=0)
                drone.hormone_state = np.clip(
                    drone.hormone_state + (neighbor_avg * influence_factor),
                    0, 255
                ).astype(np.float32)
                
    async def display_swarm_state(self):
        """Enhanced display with quorum information"""
        if not self.drones:
            return
            
        stress_levels = [drone.get_stress_level() for drone in self.drones.values()]
        overall_stress = np.mean(stress_levels)
        
        # Count stressed drones for each threshold
        high_stress_count = sum(1 for s in stress_levels if s > 150)
        med_stress_count = sum(1 for s in stress_levels if s > 80)
        total_drones = len(self.drones)
        
        drone_states = " ".join([
            f"D{drone_id}:{stress:.1f}" 
            for drone_id, stress in zip(self.drones.keys(), stress_levels)
        ])
        
        logger.info(f"🌐 Swarm State | Overall: {overall_stress:.1f} | Individual: {drone_states}")
        logger.info(f"📊 Quorum Status | High Stress: {high_stress_count}/{total_drones} | Med Stress: {med_stress_count}/{total_drones}")

# Main execution function  
async def run_real_quorum_system():
    """Run the actual quorum system for real testing"""
    coordinator = SwarmQuorumCoordinator()
    
    # Create 3 test drones
    for i in range(3):
        drone = EnhancedHormoneReceiver(drone_id=i, decay_rate=0.9)
        
        # Mock MAVSDK connection for testing (real implementation would connect to actual drones)
        drone.drone = None  
        
        coordinator.add_drone(drone)
    
    # Start coordination loop and let it run indefinitely
    try:
        await coordinator.coordination_loop()
    except KeyboardInterrupt:
        logger.info("🛑 Shutting down quorum system...")
        coordinator.running = False

if __name__ == "__main__":
    print("🧬 Bio-Inspired Swarm Coordination - Step 5: Quorum Rules")
    print("🔬 Testing enhanced hormone system with collective decision-making")
    print("💡 Press Ctrl+C to stop")
    
    asyncio.run(run_real_quorum_system())