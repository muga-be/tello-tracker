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
    faces = face_cascade.detectMultiScale(img_gray, scaleFactor=1.05, minNeighbors=10, minSize=(40, 40))

    # save all the faces that have been detected
    # store the centerpoint of all faces that we have detected

    faces_list_c = []
    faces_list_area = []
    p_area = 0

    for (x, y, w, h) in faces:
        area = w * h
        if area > p_area:
            p_area = area
            # center_x = x + w // 2
            # center_y = y + h // 2
            center_x = x
            center_y = y
            width = w
            height = h
    if p_area:
        cv2.rectangle(img, (center_x, center_y), (center_x + width, center_y + height), (0, 0, 255), 2)
        return img, center_x, center_y, width, height
    else:
        return img, 0, 0, 0, 0


def track_face(img, drone, info, frame, p_measurement):
    x = info[0]
    y = info[1]
    w = info[2]
    h = info[3]
    x_velocity = 0
    y_velocity = 0
    z_velocity = 0
    target_w = 100
    target_h = 100
    previous_x_velocity, previous_y_velocity, previous_z_velocity = p_measurement

    center_x = x + w // 2
    center_y = y + h // 2

    target_x = int(frame[0] / 2 - target_w / 2)
    target_y = int(frame[1] / 2 - target_h / 2)

    if center_x:
        x_velocity = (target_x - center_x) / 5 # better to devide in the PID module
    if center_y:
        y_velocity = (target_y - center_y) / 5
    if w:
        if w == 0:
            z_velocity = 0
        else:
            z_velocity = (w-target_w)/5


    # print(y_velocity)
    # print(z_velocity)


    #draw target
    cv2.rectangle(img, (target_x, target_y), (target_x + target_w, target_y + target_h), (0, 255, 0), 2)

    # if x_velocity != 0 :
    #     print(x_velocity)

    # if y_velocity != 0 :
    #     print(y_velocity)

    drone.for_back_velocity = 0
    drone.left_right_velocity = 0
    drone.up_down_velocity = 0
    drone.yaw_velocity = 0

    if x_velocity != 0:
        x_speed, x_P, x_I, x_D = pid(0, x_velocity, previous_x_velocity, 1, 1)
        drone.yaw_velocity = int(np.clip(x_speed, -100, 100))
    if y_velocity != 0:
        y_speed, y_P, y_I, y_D = pid(0, y_velocity, previous_y_velocity, 1, 1)
        drone.up_down_velocity = int(np.clip(-y_speed, -100, 100))

    if z_velocity != 0:
        z_speed, z_P, z_I, z_D = pid(0, z_velocity, previous_z_velocity, 1, 1)

        drone.for_back_velocity = int(np.clip(z_speed, -100, 100))

        if(z_speed < 0):
            print("forward")
            print(z_speed)
        else :
            print("backward")
            print(z_speed)

    if drone.send_rc_control:
        drone.send_rc_control(drone.left_right_velocity,
                              drone.for_back_velocity,
                              drone.up_down_velocity,
                              drone.yaw_velocity)


    # # print(info[0][0])
    # ## step 3
    # previous_error = track_face(tux_drone, info, frame, pid, previous_error)
    #
    # #PID controller
    # error = info[0][0] - width//2
    # speed = pid[0]*error + pid[1] * (error - p_error)
    # speed = int(np.clip(speed, -100, 100))
    # print(speed)
    # if info[0][0] != 0:
    #     drone.yaw_velocity = speed
    # else:
    #     drone.for_back_velocity = 0
    #     drone.left_right_velocity = 0
    #     drone.up_down_velocity = 0
    #     drone.yaw_velocity = 0
    #     error = 0
    # if drone.send_rc_control:
    #     drone.send_rc_control(drone.left_right_velocity,
    #                           drone.for_back_velocity,
    #                           drone.up_down_velocity,
    #                           drone.yaw_velocity)

    return img, [x_velocity, y_velocity, z_velocity]


def go_to_start_position(drone, pos_x):
    print("move up")
    result = 0
    drone.send_rc_control(0,
                          0,
                          30,
                          0)
    sleep(1)
    print(drone.get_height())

    return result

# -----------------------------
# Input Kc,tauI,tauD
# -----------------------------
Kc = 0.5
tauI = 0.5
tauD =  0.8

#------------------------
# PID Controller Function
#------------------------
# sp = setpoint
# pv = current temperature
# pv_last = prior temperature
# ierr = integral error
# dt = time increment between measurements
# outputs ---------------
# op = output of the PID controller
# P = proportional contribution
# I = integral contribution
# D = derivative contribution

def pid(set_point,current_measurement,previous_measurement,ierr,dt):
    # Parameters in terms of PID coefficients
    KP = Kc
    KI = Kc/tauI
    KD = Kc*tauD

    # upper and lower bounds on heater level
    speed_hi = 100
    speed_lo = -100
    # calculate the error
    error = set_point-current_measurement
    # calculate the integral error
    ierr = ierr + KI * error * dt
    # calculate the measurement derivative
    dpv = (current_measurement - previous_measurement) / dt
    # calculate the PID output
    P = KP * error
    I = ierr
    D = -KD * dpv
    speed = P + I + D
    # implement anti-reset windup
    if speed < speed_lo or speed > speed_hi:
        I = I - KI * error * dt
        # clip output
        speed = max(speed_lo, min(speed_hi, speed))
    # return the controller output and PID terms
    return [speed, P, I, D]
