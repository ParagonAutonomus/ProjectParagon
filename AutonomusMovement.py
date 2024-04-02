#python -m pip install {package_name} CLUTCH ASF
from djitellopy import tello
from time import sleep
import KeyPressModule as kp
import numpy as np
import cv2
import math
#import matplotlib.pyplot as plt (might need for plots and vectors)

#parameters for the movement of the drone

fSpeed = 117/10 #Forward speed in cm / seconds
aSpeed = 360/10 #Angular Speed in degreees/s
interval = 0.25

dInterval = fSpeed*interval
aInterval = aSpeed*interval



kp.init()
me = tello.Tello()
#me.connect()
#print(me.get_battery())
#me.takeoff()

#this is the array for the displayed point values
#initializers for global values

points = [(0,0),(0,0)]
x,y = 500,500
a = 0
yaw = 0
yv = 0




def waypoint_click(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        #params are place, spot 1, spot 2, color, thickness
        #so we got x and y there, so one should 
        #cv2.line(img, (a,b), (a,y), (0,0,0), 3)
        #cv2.line(img, (a,y), (x,y), (255,0,0), 3)
        centerX = 500
        centerY = 245
        goalX = 616
        goalY = 355
        cv2.line(img, (centerX,centerY), (x,y), (0,0,255), 3)

        #so this twas a good idea, attempt 2 below
        # c += float(math.sin(math.radians(x)))
        # d += float(math.cos(math.radians(y)))
        # c^2 = a^2 + b^2 - 2ab cos(theta), need distance between the
        #500,245 (drone), 616,355 (sim tar), x,y (curr face) 
        #only param is that c is generated between click and 616
        #this WORKS WAHOOO
        a = math.sqrt(np.square(centerX-x)+np.square(centerY-y))
        b = math.sqrt(np.square(centerX-goalX)+np.square(centerY-goalY))
        c = math.sqrt(np.square(goalX-x)+np.square(goalY-y))
        theta = (c*c - a*a - b*b)/(-2*a*b)
        finalRad = math.acos(theta)
        finalDeg = (180*finalRad)/np.pi

        #turn amount in degrees

        turnAmount = yaw - finalDeg

        #turn duration at 36 deg/second
        #potential to devide by aInterval as well
        #to get turn incriments needed, which could
        #be measuered by a variable

        turnTime = turnAmount/aSpeed
        turnCount = turnAmount/aInterval

        print(f'({x},{y},{finalDeg})')
        print("theta" + f'({theta})')
        cv2.putText(img, f'({x},{y})', (x,y),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), 2)
        #drawing a circle on the image
        cv2.circle(img, (x,y), 3, (0,0,0), -1)
        cv2.putText(img, f'({goalX},{goalY})', (goalX,goalY),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), 2)
        cv2.circle(img, (goalX,goalY), 5, (255,0,0), -1)
    #going to try and draw from one place to another, need two lines
    
    cv2.circle(img, (centerX,centerY), 7, (255, 0, 255), -1 )

    #make the drone face the right direction

    if turnAmount > 0:
            #turn the drone until it is facing the same way
            # turnAmount -= turnTime
            # yv = -aSpeed
            # yaw -= aInterval
            sleep(interval)
    else:
            #turn the drone until it is facing the same way, just the other direction
            # turnAmount += turnTime
            # yv = aSpeed
            # yaw += aInterval
            sleep(interval)
    autoMoveToSpot(x,y)

#should move the drone forward until the values are the same

def autoMoveToSpot(finalX, finalY):
    while (x != finalX or x+1 != finalX) and (y != finalY or y+1 != finalY):
        kp.getKey("UP")



def getKeyboardInput():
    lr, fb, ud,yv = 0,0,0,0
    speed = 15
    aSpeed = 50
    global x,y,yaw,a
    d = 0
    #for this I need to get the click to work and then also to make a straight line to it,
    #and then use math to basically change the drone to face (hard part) and go that direction until the 
    #position matches that, and honestly I would use a +-10 pixel window for the case that
    #the motors aren't perfectly accurate like I'm playing ultimate frisbee
    #rotate based on numpy until the angle is zero based on a factor thatll
    #tell me which is shorter

    #how do I know where i'm facing??
    #you can tell by if(go forward){how does x and y change}
    if kp.getKey("LEFT"): 
        lr = -speed
        d = dInterval
        a = -180

    elif kp.getKey("RIGHT"): 
        lr = speed 
        d = -dInterval
        a = 180

    if kp.getKey("UP"): 
        fb = speed
        d = dInterval
        a = 270

    elif kp.getKey("DOWN"): 
        fb = -speed 
        d = -dInterval
        a = -90

    if kp.getKey("w"):
        ud = speed

    elif kp.getKey("s"): 
        ud = -speed 

    if kp.getKey("a"): 
        yv = -aSpeed
        yaw -= aInterval


    elif kp.getKey("d"): 
        yv = aSpeed 
        yaw += aInterval

    if kp.getKey("q"): me.land()

    if kp.getKey("e"): me.takeoff()

    #the below code copies the movement of the drone on the map
    #that is generated
    #ok so the idea is to draw a point, then
    #use the difference in position to calculate
    #movement first in the x direction then y direction
    #yaw variable 

    sleep(interval)
    a += yaw
    x += int(d*math.cos(math.radians(a)))
    y += int(d*math.sin(math.radians(a)))
    return[lr,fb,ud,yv,x,y]





def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5,(0,0,255), cv2.FILLED)
        cv2.circle(img, points[-1], 7,(0,255,255), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]-500)/100},{(points[-1][1]-500)/100})m', 
                (points[-1][0]+10,points[-1][1]+30),cv2.FONT_HERSHEY_PLAIN,1,(255,0,255),1)
    
while True:
    vals = getKeyboardInput()
    me.send_rc_control(vals[0],vals[1],vals[2],vals[3])
    img = np.zeros((1000,1000,3), np.uint8)
    if (points[-1][0] != vals[4] or points[-1][1] != vals[5]):
        points.append((vals[4],vals[5]))
    drawPoints(img, points)
    cv2.imshow("Output",img)
    cv2.waitKey(1)
    #waypoint_click()
    #just to quit to test for when drone isn't working and even if it is
    if cv2.waitKey(1) & 0xFF == 27: 
        break
cv2.destroyAllWindows()
