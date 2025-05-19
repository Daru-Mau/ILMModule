#!/usr/bin/env python3
"""
Debug Movement Test Script

This script includes extra logging to debug the UART connection.
"""

from uart_communication import UARTCommunicator
import time
import sys

def test_movement():
    # 1. Enable debug logging
    import logging
    logging.basicConfig(level=logging.DEBUG, 
                       format='%(asctime)s [%(levelname)s] %(message)s')
    logger = logging.getLogger()
    
    # 2. Create communicator - try both automatic and manual port selection
    print("Attempting auto-detection...")
    comm = UARTCommunicator(debug=True)
    
    if not comm.connected:
        print("Auto-detection failed, trying manual port...")
        comm = UARTCommunicator(port='/dev/ttyACM0', debug=True)
    
    if not comm.connected:
        print("ERROR: Could not connect to Arduino.")
        print("Make sure the Arduino is connected and has the correct sketch uploaded.")
        return False
    
    print(f"SUCCESS: Connected to Arduino on {comm.port}")
    
    try:
        # 3. Test ping
        print("\nTesting PING command...")
        ping_result = comm.ping()
        print(f"Ping result: {ping_result}")
        
        # 4. Get initial stats
        print("\nInitial connection stats:")
        stats = comm.get_connection_stats()
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
        # 5. Test basic movement
        direction_names = ["STOP", "FORWARD", "BACKWARD", "LEFT", "RIGHT"]
        
        for direction in range(5):
            print(f"\nSending {direction_names[direction]} command...")
            result = comm.send_movement(direction=direction, speed=255)  # Maximum speed
            print(f"Command result: {result}")
            # Don't wait for STOP command
            if direction > 0:
                time.sleep(10)  # Move for 10 seconds with higher speed
            
            # Always end with a stop after each direction
            if direction > 0:
                print("\nSending STOP command...")
                comm.send_movement(direction=0)
        
        # 6. Get final stats
        print("\nFinal connection stats:")
        stats = comm.get_connection_stats()
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Always stop motors and disconnect
        print("\nSending final STOP command...")
        comm.send_movement(direction=0)
        
        print("\nDisconnecting from Arduino...")
        comm.disconnect()
        print("Disconnected")
    
    return True

if __name__ == "__main__":
    print("=== Debug Movement Test ===")
    test_movement()
    print("Test completed")
