import uptech
from up_controller import UpController

controller = UpController()
up = uptech.UpTech()
up.CDS_Open()

# =============================
servo_speed = 0   # 0=MaxSpeed.
# 80

temp = 300        #300=90du,allreset

servo_r = 500 + temp
servo_l = 485 - temp


# =============================
# stop motor #1(left), #2(right).
up.CDS_SetSpeed(1, 0)
up.CDS_SetSpeed(2, 0)


controller.uptech.CDS_SetAngle(4,servo_r, servo_speed)
controller.uptech.CDS_SetAngle(5,servo_l, servo_speed)

controller.uptech.CDS_SetAngle(6, servo_r , servo_speed)
controller.uptech.CDS_SetAngle(7, servo_l, servo_speed)    # ok


# =====================================
# reset duoji #4,#5,#6,#7.
# controller.up.CDS_SetAngle(4, servo_r, servo_speed)
# controller.up.CDS_SetAngle(5, servo_l, servo_speed)
# controller.up.CDS_SetAngle(6, servo_r, servo_speed)
# controller.up.CDS_SetAngle(7, servo_l, servo_speed)


up.CDS_SetSpeed(1, 0)
up.CDS_SetSpeed(2, 0)
