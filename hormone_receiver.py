#!/usr/bin/env python3
"""
MAVSDK-Python Hormone Receiver Implementation
Step 3 of the Hormone Broadcast Implementation Guide
"""

from mavsdk import System
import numpy as np
import asyncio
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class HormoneReceiver:
    def __init__(self, decay_rate=0.9, system_id=1):
        """
        Initialize hormone receiver
        
        Args:
            decay_rate: Rate at which hormone levels decay (0.9 s⁻¹)
            system_id: System ID for this drone
        """
        self.decay_rate = decay_rate
        self.system_id = system_id
        self.hormone_state = np.zeros(20, dtype=np.uint8)
        self.last_update = time.perf_counter()
        self.drone = System()
        self.is_connected = False
        
    async def connect(self, system_address="udp://:14540"):
        """Connect to the drone"""
        try:
            logger.info(f"Attempting to connect to drone at {system_address}")
            await self.drone.connect(system_address=system_address)
            
            # Wait for connection to establish
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    logger.info(f"✓ Connected to drone at {system_address}")
                    self.is_connected = True
                    break
                    
        except Exception as e:
            logger.error(f"Failed to connect to drone: {e}")
            raise
            
    async def hormone_listener(self):
        """Listen for hormone broadcasts and apply decay"""
        if not self.is_connected:
            logger.error("Not connected to drone. Call connect() first.")
            return
            
        logger.info("Starting hormone listener...")
        
        try:
            # Listen for heartbeat messages
            async for heartbeat in self.drone.telemetry.raw_heartbeat():
                await self._process_heartbeat(heartbeat)
                
        except Exception as e:
            logger.error(f"Error in hormone listener: {e}")
            
    async def _process_heartbeat(self, heartbeat):
        """Process received heartbeat and update hormone state"""
        now = time.perf_counter()
        dt = now - self.last_update
        
        # Apply decay to current hormone state
        decay_factor = self.decay_rate ** dt
        self.hormone_state = np.clip(
            self.hormone_state * decay_factor, 0, 255
        ).astype(np.uint8)
        
        # Check if heartbeat has hormone field
        received_hormone = self._extract_hormone_from_heartbeat(heartbeat)
        
        if received_hormone is not None:
            # Add received hormone (taking maximum value)
            self.hormone_state = np.maximum(self.hormone_state, received_hormone)
            
            # Log reception
            mean_hormone = np.mean(received_hormone)
            if mean_hormone > 0:
                logger.info(f"Received hormone: mean={mean_hormone:.1f}, max={np.max(received_hormone)}")
        
        self.last_update = now
        
        # Check for threshold triggers
        current_mean = np.mean(self.hormone_state)
        if current_mean > 50:  # Threshold for response
            logger.warning(f"HIGH HORMONE DETECTED: {current_mean:.1f}")
            await self.respond_to_hormone(current_mean)
            
        # Log current state periodically (every 5 seconds)
        if int(now) % 5 == 0:
            logger.info(f"Current hormone state - mean: {current_mean:.1f}, max: {np.max(self.hormone_state)}")
            
    def _extract_hormone_from_heartbeat(self, heartbeat):
        """Extract hormone data from heartbeat message"""
        try:
            # Check if heartbeat has hormone field (custom MAVLink)
            if hasattr(heartbeat, 'hormone') and heartbeat.hormone:
                return np.frombuffer(heartbeat.hormone, dtype=np.uint8)
            else:
                # Fallback: look for custom_mode encoding
                if hasattr(heartbeat, 'custom_mode'):
                    # Use custom_mode as simple hormone indicator
                    # This is a fallback for when custom MAVLink isn't available
                    hormone_level = (heartbeat.custom_mode & 0xFF)
                    if hormone_level > 0:
                        result = np.zeros(20, dtype=np.uint8)
                        result[0] = hormone_level
                        return result
                        
        except Exception as e:
            logger.debug(f"Could not extract hormone from heartbeat: {e}")
            
        return None
        
    async def respond_to_hormone(self, hormone_level):
        """Respond to high hormone levels"""
        try:
            if hormone_level > 200:  # Critical level
                logger.critical("CRITICAL HORMONE LEVEL - Emergency response")
                await self.drone.action.kill()
                
            elif hormone_level > 100:  # High level
                logger.warning("High hormone level - Switching to HOLD mode")
                await self.drone.action.hold()
                
            elif hormone_level > 50:  # Medium level
                logger.info("Medium hormone level - Reducing speed")
                # Could implement speed reduction here
                
        except Exception as e:
            logger.error(f"Failed to respond to hormone: {e}")
            
    async def inject_stress_hormone(self, level=255):
        """Inject stress hormone (for testing)"""
        logger.warning(f"Injecting stress hormone at level {level}")
        self.hormone_state[0] = level
        
    def get_hormone_summary(self):
        """Get current hormone state summary"""
        return {
            'mean': float(np.mean(self.hormone_state)),
            'max': int(np.max(self.hormone_state)),
            'min': int(np.min(self.hormone_state)),
            'std': float(np.std(self.hormone_state)),
            'raw_state': self.hormone_state.tolist()
        }

# Alternative implementation using NAMED_VALUE_FLOAT (fallback)
class HormoneReceiverLegacy:
    """Legacy implementation using NAMED_VALUE_FLOAT messages"""
    
    def __init__(self, decay_rate=0.9):
        self.decay_rate = decay_rate
        self.hormone_state = np.zeros(5, dtype=np.float32)  # 5 floats = 20 bytes
        self.last_update = time.perf_counter()
        self.drone = System()
        
    async def connect(self, system_address="udp://:14540"):
        await self.drone.connect(system_address=system_address)
        logger.info(f"Legacy receiver connected to {system_address}")
        
    async def hormone_listener_legacy(self):
        """Listen using NAMED_VALUE_FLOAT messages"""
        logger.info("Starting legacy hormone listener (NAMED_VALUE_FLOAT)...")
        
        try:
            async for named_value in self.drone.telemetry.named_value_float():
                if named_value.name.startswith("HORMONE_"):
                    await self._process_named_value(named_value)
                    
        except Exception as e:
            logger.error(f"Error in legacy hormone listener: {e}")
            
    async def _process_named_value(self, named_value):
        """Process NAMED_VALUE_FLOAT hormone message"""
        try:
            index = int(named_value.name.split("_")[1])
            if 0 <= index < 5:
                now = time.perf_counter()
                dt = now - self.last_update
                
                # Apply decay
                decay_factor = self.decay_rate ** dt
                self.hormone_state *= decay_factor
                
                # Update specific hormone component
                self.hormone_state[index] = max(
                    self.hormone_state[index], named_value.value
                )
                
                self.last_update = now
                
                # Log significant changes
                if named_value.value > 10:
                    logger.info(f"Legacy hormone update: {named_value.name} = {named_value.value}")
                    
        except (ValueError, IndexError) as e:
            logger.debug(f"Invalid hormone message format: {e}")

async def test_single_drone():
    """Test hormone receiver with single drone"""
    logger.info("Starting single drone test...")
    
    # Create hormone receiver
    receiver = HormoneReceiver(decay_rate=0.9, system_id=1)
    
    # Connect to drone
    await receiver.connect("udp://:14540")
    
    # Create test task
    async def test_injection():
        """Inject test hormone after 10 seconds"""
        await asyncio.sleep(10)
        await receiver.inject_stress_hormone(128)
        logger.info("Test hormone injected")
    
    # Run both tasks concurrently
    await asyncio.gather(
        receiver.hormone_listener(),
        test_injection()
    )

async def main():
    """Main execution function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Hormone Receiver')
    parser.add_argument('--port', type=int, default=14540, help='UDP port (default: 14540)')
    parser.add_argument('--system-id', type=int, default=1, help='System ID (default: 1)')
    parser.add_argument('--decay-rate', type=float, default=0.9, help='Decay rate (default: 0.9)')
    parser.add_argument('--legacy', action='store_true', help='Use legacy NAMED_VALUE_FLOAT mode')
    parser.add_argument('--test', action='store_true', help='Run test mode with injection')
    
    args = parser.parse_args()
    
    system_address = f"udp://:{args.port}"
    
    if args.legacy:
        # Use legacy implementation
        receiver = HormoneReceiverLegacy(decay_rate=args.decay_rate)
        await receiver.connect(system_address)
        await receiver.hormone_listener_legacy()
    else:
        # Use main implementation
        if args.test:
            await test_single_drone()
        else:
            receiver = HormoneReceiver(decay_rate=args.decay_rate, system_id=args.system_id)
            await receiver.connect(system_address)
            await receiver.hormone_listener()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Hormone receiver stopped by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        raise
