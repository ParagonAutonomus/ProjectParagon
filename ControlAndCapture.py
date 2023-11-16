from djitellopy import tello
import KeyPressModule as kp
import cv2
import time

kp.init()
me = tello.Tello()
me.connect()
batteryLvl = me.get_battery()
print("Battery Level : ", batteryLvl)

me.streamon()


def getKeyboardInput():
    lr, fb, up, yv = 0, 0, 0, 0
    speed = 50

    if kp.getKey("e"):
        me.takeoff()

    if kp.getKey("LEFT"):
        lr = -speed
    elif kp.getKey("RIGHT"):
        lr = speed

    if kp.getKey("UP"):
        fb = speed
    elif kp.getKey("DOWN"):
        fb = -speed

    if kp.getKey("w"):
        up = speed
    elif kp.getKey("s"):
        up = -speed

    if kp.getKey("a"):
        yv = -speed
    elif kp.getKey("d"):
        yv = speed

    if kp.getKey("q"):
        me.land()

    if kp.getKey("c"):
        cv2.imwrite(f'/Users/ilhombek/Downloads/DroneIMG/{time.time()}.jpg', img)

    return [lr, fb, up, yv]


while True:
    vals = getKeyboardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    img = me.get_frame_read().frame
    img = cv2.resize(img, (360,240))
    cv2.imshow("Image", img)
    cv2.waitKey(1)
