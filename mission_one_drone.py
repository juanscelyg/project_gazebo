"""
mission_one_drone.py
"""
import mission_utils
from as2_msgs.msg import YawMode
from time import sleep
from as2_msgs.msg._behavior_status import BehaviorStatus
import rclpy
from as2_knowledge_graph_integration.read_graph import ReadMyGraph
from as2_python_api.drone_interface import DroneInterface


# def drone_land(drone_interface: DroneInterface):
#     """ Land the drone """

#     print("Landing")
#     drone_interface.land(speed=0.5)
#     print("Land done")

#     drone_interface.disarm()


def drone_run(drone_interface: DroneInterface, graph_reader: ReadMyGraph):
    """ Run the mission """

    speed = 0.5
    takeoff_height = 1.0
    height = 1.0

    sleep_time = 2.0

    dim = 5.0
    ascend = 1.0
    path = [
        [dim, 0, height],
        [-dim, ascend, height],
        [dim, ascend + 1, height],
        [-dim, ascend + 2, height],
        [dim, ascend + 3, height],
        [-dim, ascend + 4, height],
    ]
    home = [0, 0, height]   # home position
    command = {'go_home': False, 'land': False, 'continue_path': True}

    print("Start mission")

    ##### ARM OFFBOARD #####
    print("Arm")
    drone_interface.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_interface.arm()
    sleep(sleep_time)

    ##### TAKE OFF #####
    if graph_reader.check_status_of_edge('Dron', 'Battery', 'high'):
        print('Batery is high')
        print("Take Off")
        drone_interface.takeoff(takeoff_height, speed=1.0)
        print("Take Off done")
        sleep(sleep_time)

        ##### GO TO #####
        for goal in path:
            if command['continue_path']:
                print(f"Go to with path facing {goal}")
                drone_interface.go_to(*goal, speed=speed, wait=False, yaw_mode=YawMode.PATH_FACING)
                sleep(1)
                print(drone_interface.go_to.is_running())
                while drone_interface.go_to.is_running():
                    # while drone_interface.go_to.is_running() == False:
                    print('RUNNING')
                    if (graph_reader.check_status_of_edge('Dron', 'Battery', 'low')):
                        print('Battery is low')
                        print('Emergency landing')
                        command['land'] = True
                        command['continue_path'] = False
                        drone_interface.go_to.stop()    # Stop the drone
                        break

                    if (graph_reader.check_status_of_edge('Dron', 'Person', 'seeing')):
                        print('The person has been located')
                        command['continue_path'] = False
                        command['go_home'] = True
                        drone_interface.go_to.stop()    # Stop the drone
                        break

                if drone_interface.go_to.wait_to_result():
                    print("Go to done")
            sleep(sleep_time)

        ##### GO TO HOME #####
        futures = mission_utils.go_home(
            drone_interface=drone_interface, home=home, speed=speed, go_home=command['go_home'])
        command['go_home'] = futures['go_home']
        command['land'] = futures['land']

        # ##### LAND #####
        command['land'] = mission_utils.drone_land(
            drone_interface=drone_interface, land=command['land'])

    else:
        print('Battery is low')


if __name__ == '__main__':
    rclpy.init()

    uav = DroneInterface("drone0", verbose=False, use_sim_time=True)
    graph_reader = ReadMyGraph()

    drone_run(uav, graph_reader=graph_reader)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
