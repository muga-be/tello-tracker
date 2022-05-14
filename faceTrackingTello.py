from djitellopy import Tello
import cv2
import numpy as np
from time import sleep

def initilize_tello():
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


def tello_get_frame(drone, w=360, h=240):
    tux_frame = drone.get_frame_read()
    tux_frame = tux_frame.frame
    img = cv2.resize(tux_frame, (w, h))
    return img


def find_face(img):
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(img_gray, 1.2, 4) #check how this function works, how can we improve face detection

    # save all the faces that have been detected
    # store the centerpoint of all faces that we have detected

    faces_list_c = []
    faces_list_area = []

    for(x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h),(0,0,255), 2)
        center_x = x + w//2
        center_y = y + h//2
        area = w*h
        faces_list_c.append([center_x, center_y])
        faces_list_area.append(area)

        #check if the face is present or not
        #find the index of the largest area
        #this is the face closest to the camera
        #and return those x-y values
    if len(faces_list_area) != 0:
        i = faces_list_area.index(max(faces_list_area))
        return img, [faces_list_c[i], faces_list_area[i]]
    else:
        return img, [[0, 0], 0]


def track_face(drone, info, width, pid, p_error):

    #PID controller
    error = info[0][0] - width//2
    speed = pid[0]*error + pid[1] * (error - p_error)
    speed = int(np.clip(speed, -100, 100))
    print(speed)
    if info[0][0] != 0:
        drone.yaw_velocity = speed
    else:
        drone.for_back_velocity = 0
        drone.left_right_velocity = 0
        drone.up_down_velocity = 0
        drone.yaw_velocity = 0
        error = 0
    if drone.send_rc_control:
        drone.send_rc_control(drone.left_right_velocity,
                              drone.for_back_velocity,
                              drone.up_down_velocity,
                              drone.yaw_velocity)

    return error


def go_to_start_position(drone, pos_x):
    print("move up")
    result = drone.move_up(100)
    sleep(2)
    print(drone.get_height())
    result = drone.move_up(150)
    sleep(2)
    drone.send_rc_control(0,
                          0,
                          30,
                          0)
    sleep(1)
    print(drone.get_height())

    return result
