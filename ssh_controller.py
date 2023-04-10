import sys
import select
import tty
import termios
import time
from roomba_controller import RoombaController


def is_key_pressed():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def get_key():
    return sys.stdin.read(1)


def set_terminal_raw_mode():
    return termios.tcgetattr(sys.stdin.fileno())


def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)


# Set up the RoombaController
roomba = RoombaController("/dev/ttyUSB0")  # Replace with your serial port address

print("Roomba Text-based Controller")
print(
    "Press 'w' to move forward, 's' to move backward, 'a' to spin left, 'd' to spin right, 'q' to quit"
)

# Set terminal to raw mode
old_settings = set_terminal_raw_mode()
tty.setraw(sys.stdin.fileno())

# Main control loop
running = True
while running:
    try:
        if is_key_pressed():
            command = get_key()

            # Control the Roomba using AWSD keys
            if command == "a":
                roomba.spin_left(100)
            elif command == "d":
                roomba.spin_right(100)
            elif command == "w":
                roomba.forward(100)
            elif command == "s":
                roomba.backward(100)
            elif command == "q":
                running = False
            else:
                roomba.stop()

        # Sleep for a short period to avoid high CPU usage
        time.sleep(0.1)
    except KeyboardInterrupt:
        running = False

# Restore terminal settings before exiting
restore_terminal_settings(old_settings)

# Clean up before exiting
roomba.turn_off()
roomba.close()
