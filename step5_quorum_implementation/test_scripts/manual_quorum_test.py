#!/usr/bin/env python3
"""
Manual test to trigger quorum decisions
Run this while your enhanced_quorum_system.py is running
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

def test_formation_hold_quorum():
    """Test formation hold quorum (lower threshold)"""
    print("🧪 Testing Formation Hold Quorum")
    print("   Rule: 40% of drones with mean stress > 80")
    
    # Inject moderate stress to multiple hormone types for higher mean
    for drone_id in [0, 1]:
        for hormone_type in range(5):  # Inject to first 5 hormone types
            send_hormone_injection(drone_id, hormone_type, 200)
            time.sleep(0.1)
    
    print("   Expected: 2/3 drones (66.7%) with high mean stress")
    print("   Should trigger FORMATION HOLD quorum!")

def test_emergency_land_quorum():
    """Test emergency land quorum (higher threshold)"""
    print("\n🚨 Testing Emergency Land Quorum")
    print("   Rule: 60% of drones with mean stress > 150")
    
    # Inject very high stress across many hormone types
    for drone_id in [0, 1, 2]:  # All 3 drones
        for hormone_type in range(10):  # Inject to first 10 hormone types
            send_hormone_injection(drone_id, hormone_type, 255)
            time.sleep(0.1)
    
    print("   Expected: 3/3 drones (100%) with very high mean stress")
    print("   Should trigger EMERGENCY LAND quorum!")

if __name__ == "__main__":
    print("🧬 Manual Quorum Testing")
    print("Make sure enhanced_quorum_system.py is running in another terminal!")
    print()
    
    # Test 1: Formation hold (easier to trigger)
    test_formation_hold_quorum()
    
    print("\nWaiting 10 seconds to observe formation hold response...")
    time.sleep(10)
    
    # Test 2: Emergency land (harder to trigger)  
    test_emergency_land_quorum()
    
    print("\nWaiting 10 seconds to observe emergency land response...")
    time.sleep(10)
    
    print("\n✅ Manual quorum testing complete!")
    print("Check your enhanced_quorum_system.py terminal for quorum decisions!")