#!/usr/bin/env python3
"""
Fixed MAVSDK test using correct telemetry methods
"""

import asyncio
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_available_telemetry():
    """Test what telemetry methods are actually available"""
    try:
        from mavsdk import System
        
        drone = System()
        await drone.connect(system_address="udp://:14540")
        
        # Wait for connection
        async for state in drone.core.connection_state():
            if state.is_connected:
                logger.info("✓ Connected to PX4!")
                break
        
        # Check what telemetry methods are available
        telemetry_methods = [method for method in dir(drone.telemetry) if not method.startswith('_')]
        logger.info(f"Available telemetry methods: {telemetry_methods}")
        
        # Try different heartbeat/status methods
        logger.info("Testing flight_mode...")
        try:
            async for flight_mode in drone.telemetry.flight_mode():
                logger.info(f"✓ Flight mode: {flight_mode}")
                break
        except Exception as e:
            logger.error(f"flight_mode failed: {e}")
        
        logger.info("Testing armed status...")
        try:
            async for armed in drone.telemetry.armed():
                logger.info(f"✓ Armed status: {armed}")
                break
        except Exception as e:
            logger.error(f"armed failed: {e}")
            
        logger.info("Testing health...")
        try:
            async for health in drone.telemetry.health():
                logger.info(f"✓ Health: {health}")
                break
        except Exception as e:
            logger.error(f"health failed: {e}")
            
        # Try attitude (should work)
        logger.info("Testing attitude...")
        try:
            async for attitude in drone.telemetry.attitude_euler():
                logger.info(f"✓ Attitude: roll={attitude.roll_deg:.1f}, pitch={attitude.pitch_deg:.1f}, yaw={attitude.yaw_deg:.1f}")
                break
        except Exception as e:
            logger.error(f"attitude failed: {e}")
            
        return True
        
    except Exception as e:
        logger.error(f"Test failed: {e}")
        return False

async def test_hormone_simulation():
    """Test hormone simulation using available telemetry"""
    try:
        from mavsdk import System
        import numpy as np
        
        drone = System()
        await drone.connect(system_address="udp://:14540")
        
        # Wait for connection
        async for state in drone.core.connection_state():
            if state.is_connected:
                logger.info("✓ Connected for hormone test!")
                break
        
        # Initialize hormone state
        hormone_state = np.zeros(20, dtype=np.uint8)
        last_update = time.perf_counter()
        decay_rate = 0.9
        count = 0
        
        logger.info("Starting hormone simulation using flight_mode telemetry...")
        
        # Use flight_mode as our "heartbeat" equivalent
        async for flight_mode in drone.telemetry.flight_mode():
            count += 1
            now = time.perf_counter()
            dt = now - last_update
            
            # Apply hormone decay
            if last_update > 0:
                decay_factor = decay_rate ** dt
                hormone_state = np.clip(
                    hormone_state * decay_factor, 0, 255
                ).astype(np.uint8)
            
            # Simulate hormone injection randomly
            if np.random.random() < 0.3:  # 30% chance
                injection_amount = np.random.randint(20, 100)
                hormone_state[0] = min(255, hormone_state[0] + injection_amount)
                logger.info(f"💉 Hormone injection: {injection_amount}")
            
            # Check hormone levels
            mean_hormone = np.mean(hormone_state)
            max_hormone = np.max(hormone_state)
            
            logger.info(f"Cycle #{count}: Mode={flight_mode}, Hormone Mean={mean_hormone:.1f}, Max={max_hormone}")
            
            if mean_hormone > 50:
                logger.warning(f"🚨 High hormone detected! Mean: {mean_hormone:.1f}")
            
            last_update = now
            
            # Run for 10 cycles then stop
            if count >= 10:
                logger.info("Hormone simulation test complete!")
                break
                
        return True
        
    except Exception as e:
        logger.error(f"Hormone simulation failed: {e}")
        return False

async def main():
    logger.info("=== Fixed MAVSDK Telemetry Test ===\n")
    
    logger.info("1. Testing available telemetry methods...")
    await test_available_telemetry()
    
    logger.info("\n2. Testing hormone simulation...")
    await test_hormone_simulation()
    
    logger.info("\n=== Test Complete ===")

if __name__ == "__main__":
    asyncio.run(main())