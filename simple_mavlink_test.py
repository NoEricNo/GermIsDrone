#!/usr/bin/env python3
"""
Simple MAVLink connection test for Codespaces
This will help us debug the connection issue
"""

import socket
import time
import sys

def test_port_connection(host='127.0.0.1', port=14540, timeout=5):
    """Test if we can connect to a port"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        
        # For UDP, we send a simple message and see if port is open
        sock.sendto(b'test', (host, port))
        print(f"✓ Port {port} appears to be reachable")
        sock.close()
        return True
        
    except Exception as e:
        print(f"✗ Port {port} connection failed: {e}")
        return False

def test_mavsdk_simple():
    """Test MAVSDK with minimal setup"""
    try:
        from mavsdk import System
        print("✓ MAVSDK imported successfully")
        
        # Create system but don't connect yet
        drone = System()
        print("✓ MAVSDK System created")
        return True
        
    except Exception as e:
        print(f"✗ MAVSDK test failed: {e}")
        return False

def check_running_processes():
    """Check what PX4-related processes are running"""
    import subprocess
    
    try:
        # Check for PX4 processes
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        
        px4_processes = [line for line in lines if 'px4' in line.lower()]
        gazebo_processes = [line for line in lines if 'gazebo' in line.lower()]
        
        print(f"PX4 processes found: {len(px4_processes)}")
        for proc in px4_processes:
            print(f"  {proc}")
            
        print(f"Gazebo processes found: {len(gazebo_processes)}")
        for proc in gazebo_processes:
            print(f"  {proc}")
            
    except Exception as e:
        print(f"Process check failed: {e}")

def check_listening_ports():
    """Check what ports are listening"""
    import subprocess
    
    try:
        result = subprocess.run(['netstat', '-ln'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        
        # Look for MAVLink ports (14540-14550)
        mavlink_ports = [line for line in lines if any(f':{port}' in line for port in range(14540, 14551))]
        
        print("MAVLink-related listening ports:")
        if mavlink_ports:
            for port in mavlink_ports:
                print(f"  {port}")
        else:
            print("  None found")
            
    except Exception as e:
        print(f"Port check failed: {e}")

async def test_mavsdk_connection():
    """Test actual MAVSDK connection"""
    try:
        from mavsdk import System
        
        drone = System()
        print("Attempting to connect to udp://:14540...")
        
        # Set a shorter timeout for Codespaces
        await drone.connect(system_address="udp://:14540")
        
        print("Connection initiated, waiting for heartbeat...")
        
        # Wait for connection with timeout (Python 3.10 compatible)
        import asyncio
        
        try:
            # Python 3.10 compatible timeout
            async def connection_with_timeout():
                async for state in drone.core.connection_state():
                    print(f"Connection state: {state}")
                    if state.is_connected:
                        print("✓ Successfully connected to PX4!")
                        
                        # Get one heartbeat to verify
                        async for heartbeat in drone.telemetry.raw_heartbeat():
                            print(f"✓ Received heartbeat: System {heartbeat.system_id}, Type: {heartbeat.type}")
                            break
                            
                        return True
            
            result = await asyncio.wait_for(connection_with_timeout(), timeout=10.0)
            return result
                        
        except asyncio.TimeoutError:
            print("✗ Connection timeout after 10 seconds")
            return False
            
    except Exception as e:
        print(f"✗ MAVSDK connection test failed: {e}")
        return False

def main():
    print("=== MAVLink Connection Diagnostic ===\n")
    
    print("1. Checking running processes...")
    check_running_processes()
    print()
    
    print("2. Checking listening ports...")
    check_listening_ports()
    print()
    
    print("3. Testing port connectivity...")
    test_port_connection()
    print()
    
    print("4. Testing MAVSDK import...")
    if test_mavsdk_simple():
        print()
        print("5. Testing MAVSDK connection...")
        import asyncio
        asyncio.run(test_mavsdk_connection())
    
    print("\n=== Diagnostic Complete ===")

if __name__ == "__main__":
    main()