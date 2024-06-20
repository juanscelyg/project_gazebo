"""
Mision utils
"""
from as2_msgs.msg import YawMode
from time import sleep
from as2_msgs.msg._behavior_status import BehaviorStatus
import rclpy
from as2_python_api.drone_interface import DroneInterface

sleep_time = 2.0


def drone_land(drone_interface: DroneInterface, land: bool) -> bool:

    if land:
        """ Land the drone """

        print("Landing")
        drone_interface.land(speed=0.5)
        print("Land done")

        drone_interface.disarm()
        return


def go_home(drone_interface: DroneInterface, home: list, speed: float, go_home: bool) -> bool:
    futures = []
    if go_home:
        print("Go to home")
        drone_interface.go_to(*home, speed=speed, wait=True, yaw_mode=YawMode.PATH_FACING)
        while drone_interface.go_to.is_running():
            print('GOING HOME')
        if drone_interface.go_to.wait_to_result():
            print("Go to home done")
            sleep(sleep_time)
            futures = {'go_home': False, 'land': True}

    return futures
