from djitellopy import tello
from time import sleep

me = tello.Tello()
me.connect()
batteryLvl = me.get_battery()
print("Battery Level : ", batteryLvl)

me.takeoff()
me.send_rc_control(0, 50, 0, 0)
sleep(4)
me.send_rc_control(-75, 0, 0, 0)
sleep(4)
me.send_rc_control(0, 0, 0, 0)
me.land()