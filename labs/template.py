"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################

import drone_core
# import drone_utils  # uncomment when you need helpers from drone_utils

########################################################################################
# Global variables
########################################################################################

drone = drone_core.create_drone()

# Declare any global variables here


########################################################################################
# Functions
########################################################################################

# [FUNCTION] start() is run once when the simulation begins
def start():
    pass  # Remove 'pass' and write your source code for start() here


# [FUNCTION] update() is called once every frame (~60 fps)
def update():
    pass  # Remove 'pass' and write your source code for update() here


# [FUNCTION] update_slow() is called once per second — useful for debug prints
def update_slow():
    pass  # Remove 'pass' and write your source code for update_slow() here


########################################################################################
# DO NOT MODIFY: Register callbacks and begin execution
########################################################################################

if __name__ == "__main__":
    drone.set_start_update(start, update, update_slow)
    drone.go()
