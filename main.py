from utilis import *
import cv2

frame_width, frame_height = 360*2, 240*2
pid = [0.5, 0.5, 0]
previous_error = [0,0,0]

tux_drone = initilize_tello()



#myDrone = intializeTello()

take_off = 1 # put to zero if you dont want the drone to take off

if take_off:
    sleep(2)
    tux_drone.takeoff()
    go_to_start_position(tux_drone, 50)



while True:

    ##
    ## Step 1
    img = tello_get_frame(tux_drone, frame_width, frame_height)
    ## Step 2
    img, center_x, center_y, width, height = find_face(img)
    info = [center_x, center_y, width, height]
    frame = [frame_width, frame_height]

    # print(info[0][0])
    ## step 3
    img, previous_error = track_face(img,tux_drone, info, frame, previous_error)

    cv2.namedWindow('Drone View', cv2.WINDOW_NORMAL)
    cv2.imshow('Drone View', img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        print(tux_drone.get_battery())
        tux_drone.land()
        break

#cv2.destroyAllWindows()
