from djitellopy import tello
import cv2
import KeyPressModule as kp
import time
from time import sleep


me = tello.Tello()
me.connect()
batteryLvl = me.get_battery()
print("Battery Level : ", batteryLvl)

me.streamon()

#need to define some basic movement, if the module is run
#it should have the ability to work with the camera
def getKeyboardInput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 50
    if kp.getKey("LEFT"): lr = -speed
    elif kp.getKey("RIGHT"): lr = speed

    if kp.getKey("UP"): fb = speed
    elif kp.getKey("DOWN"): fb = -speed 

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed 

    if kp.getKey("a"): yv = -speed
    elif kp.getKey("d"): yv = speed 

    if kp.getKey("q"): me.land(); time.sleep(3)

    if kp.getKey("e"): me.takeoff()

    #press z to take a picture and store it on the drone path

    if kp.getKey("z"):
        cv2.imwrite(f'Resources/Images/{time.time()}.jpg', img)

    return[lr,fb,ud,yv]

me.takeoff()



while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (360,240))
    cv2.imshow("Image", img)
    cv2.waitKey(1)
    vals = getKeyboardInput()
    me.send_rc_control(vals[0],vals[1],vals[2],vals[3])
    sleep(0.05)
