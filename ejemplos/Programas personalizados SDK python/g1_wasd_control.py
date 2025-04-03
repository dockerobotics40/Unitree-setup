#!/usr/bin/env python3
"""
Unitree G1 Robot Control Script
This script provides keyboard-based control for the Unitree G1 robot using WASD keys.
"""

import sys
import time
import tty
import termios
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# Configuration constants
FORWARD_SPEED = 0.4     # Forward/backward speed in m/s
LATERAL_SPEED = 0.3     # Left/right speed in m/s
ROTATION_SPEED = 0.5    # Rotation speed in rad/s
STARTUP_DELAY = 3.0     # Delay after standing up in seconds
INIT_DELAY = 1.0        # Delay after initialization in seconds
COMMAND_DELAY = 2.0     # Delay after starting in seconds

def getch():
    """
    Get a single character from the user without requiring Enter key.
    Returns:
        str: Single character input from the user
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def print_controls():
    """Display available control commands to the user."""
    print("\nUnitree G1 Robot Controls:")
    print("-------------------------")
    print("Movement:")
    print("  W: Move Forward")
    print("  S: Move Backward")
    print("  A: Move Left")
    print("  D: Move Right")
    print("  Q: Rotate Left")
    print("  E: Rotate Right")
    print("\nOther Commands:")
    print("  Space: Stop")
    print("  Esc: Quit")
    print("\nCurrent Status: Robot Ready")

def initialize_robot(network_interface):
    """
    Initialize the robot and get it ready for movement.
    Args:
        network_interface (str): Network interface to use for robot communication
    Returns:
        LocoClient: Initialized robot client
    """
    # Initialize SDK
    ChannelFactoryInitialize(0, network_interface)
    
    # Create and initialize client
    client = LocoClient()
    client.SetTimeout(10.0)
    client.Init()
    
    # Initialize robot state
    print("Setting initial state...")
    client.Damp()
    time.sleep(INIT_DELAY)
    
    print("Standing up...")
    client.StandUp()
    time.sleep(STARTUP_DELAY)
    
    print("Setting robot in balance stand mode...")
    client.BalanceStand(0)
    time.sleep(2) 
    
    print("Starting robot...")
    client.Start()
    time.sleep(COMMAND_DELAY)
    
    print("Robot started")
    
    
    return client

def handle_movement(key, client):
    """
    Handle movement commands based on key press.
    Args:
        key (str): The pressed key
        client (LocoClient): Robot client instance
    Returns:
        bool: True if should continue, False if should exit
    """
    x_vel = 0.0
    y_vel = 0.0
    yaw_vel = 0.0
    status = "Stopped          "

    if key == 'w':
        x_vel = FORWARD_SPEED
        status = "Moving Forward    "
    elif key == 's':
        x_vel = -FORWARD_SPEED
        status = "Moving Backward   "
    elif key == 'a':
        y_vel = LATERAL_SPEED
        status = "Moving Left       "
    elif key == 'd':
        y_vel = -LATERAL_SPEED
        status = "Moving Right      "
    elif key == 'q':
        yaw_vel = ROTATION_SPEED
        status = "Rotating Left     "
    elif key == 'e':
        yaw_vel = -ROTATION_SPEED
        status = "Rotating Right    "
    elif key == ' ':
        status = "Stopped          "
    elif ord(key) == 27:  # Esc key
        print("\nExiting...")
        return False

    print(f"\rCurrent Status: {status}", end='')
    client.Move(x_vel, y_vel, yaw_vel)
    sys.stdout.flush()
    return True

def main():
    """Main function to run the robot control program."""
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(1)

    print("\nWARNING: Please ensure there are no obstacles around the robot.")
    input("Press Enter when the area is clear...")

    try:
        # Initialize robot
        client = initialize_robot(sys.argv[1])
        print_controls()

        # Main control loop
        while True:
            key = getch().lower()
            if not handle_movement(key, client):
                break

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError occurred: {str(e)}")
    finally:
        # Ensure robot stops safely
        try:
            client.Move(0, 0, 0)
            print("\nRobot stopped safely")
        except Exception as e:
            print(f"\nError stopping robot: {str(e)}")

if __name__ == "__main__":
    main()
