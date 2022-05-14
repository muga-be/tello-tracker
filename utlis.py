from djitellopy import Tello
import cv2


def initilizeTello():
    tux_drone = Tello()
    tux_drone.connect()
    tux_drone.for_back_velocity = 0
    tux_drone.left_right_velocity = 0
    tux_drone.up_down_velocity = 0
    tux_drone.yaw_velocity = 0
    tux_drone.speed = 0
    print(tux_drone.get_battery())
    tux_drone.streamoff()
    tux_drone.streamon()
    return tux_drone
