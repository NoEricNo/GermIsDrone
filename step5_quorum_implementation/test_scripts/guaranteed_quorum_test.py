#!/usr/bin/env python3
"""
Guaranteed quorum trigger test - will definitely trigger both quorum rules
"""

import socket
import json
import time

def send_hormone_injection(drone_id, hormone_type, amount):
    """Send hormone injection via UDP"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    message = {
        'drone_id': drone_id,
        'hormone_type': hormone_type, 
        'amount': amount
    }
    
    data = json.dumps(message).encode()
    sock.sendto(data, ('localhost', 9999))
    sock.close()
    
    print(f"💉 Injected: Drone{drone_id} hormone[{hormone_type}] = {amount}")

def max_stress_injection():
    """Inject maximum stress to ALL hormone types for guaranteed trigger"""
    print("🚨 MAXIMUM STRESS INJECTION - Guaranteed Quorum Trigger!")
    print("   Injecting 255 to ALL 20 hormone types for ALL drones")
    print("   This will create mean stress of 255 for each drone")
    
    # Inject to ALL 20 hormone types with maximum value
    for drone_id in [0, 1, 2]:
        print(f"\n🤖 Injecting maximum stress to Drone {drone_id}...")
        for hormone_type in range(20):  # All 20 hormone types
            send_hormone_injection(drone_id, hormone_type, 255)
            time.sleep(0.05)  # Small delay to avoid overwhelming
        
        print(f"✅ Drone {drone_id}: Mean stress will be 255")
    
    print(f"\n📊 Expected Results:")
    print(f"   - All drones: Mean stress = 255")
    print(f"   - Formation hold threshold (>80): ✅ ALL DRONES")  
    print(f"   - Emergency land threshold (>150): ✅ ALL DRONES")
    print(f"   - Should trigger BOTH quorum decisions!")

if __name__ == "__main__":
    print("🧬 GUARANTEED Quorum Test")
    print("This will definitely trigger quorum decisions!")
    print("Keep watching Terminal 3 for quorum responses...\n")
    
    # Give user time to prepare
    for i in range(3, 0, -1):
        print(f"Starting in {i}...")
        time.sleep(1)
    
    max_stress_injection()
    
    print(f"\n⏳ Waiting 15 seconds for system to process and respond...")
    print(f"Watch Terminal 3 for:")
    print(f"   🗳️ QUORUM REACHED: formation_hold")
    print(f"   🗳️ QUORUM REACHED: emergency_land")
    print(f"   🤝 FORMATION HOLD PROTOCOL ACTIVATED") 
    print(f"   🚨 EMERGENCY LAND PROTOCOL ACTIVATED")
    
    time.sleep(15)
    
    print("\n✅ Maximum stress injection complete!")
    print("If you don't see quorum decisions, there might be a loop issue in the enhanced system.")