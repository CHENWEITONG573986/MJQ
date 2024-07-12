'''
===2024.07.02===
main.py
作者:马思奇
功能:实现智能小车控制：上台,寻台,台上漫步,台上扫码,推箱子
Debuging on 2024.07.12 06:58
=================
'''
from up_controller import UpController  # 引入控制小车的类
import time                             # 引入时间库（标准库）
import random
'''
注意:红外传感器的id从1开始,存储数据的io_data索引从0开始
    舵机编号从1开始
    红外光电传感器值为0表示接收到反光
'''

car=UpController()      # 建立控制小车的对象
car.set_chassis_mode(2) #设置CDS模式为电机模式2,1是舵机模式

car.open_get_ADC()#启动灰度，红外数值获取，判断台上台下线程，io_data[],adc_data[]
car.open_edge_detect()  #边缘检测及控制线程
car.open_scan_pull_thread()  #扫描二维码及控制线程


#小车软启动（0表示被遮挡/有反光）
#if car.io_data[4]==0 and car.io_data[5]==0:#两侧传感器遮挡启动
    #car.move_up(200,200)

#小车直接启动
car.move_up(200,200)
pre=None
while True:#主循环
    try:
        #台下车辆倾覆自救
#         if car.adc_data[3]>2500 and car.flag_going_on_platform==False and car.io_data[1]==0 and car.io_data[3]==0:
#             print("main:车辆倾覆自救_前部朝上")
#             car.after_servo_degree(0,car.servo_speed)
#             time.sleep(0.8)
#             car.after_servo_degree(90,car.servo_speed)    
#         if car.adc_data[3]>2500 and car.flag_going_on_platform==False and car.io_data[0]==0 and car.io_data[2]==0:
#             print("main:车辆倾覆自救_尾部朝上")
#             car.front_servo_degree(0,car.servo_speed)
#             time.sleep(0.8)
#             car.front_servo_degree(90,car.servo_speed)
        if car.is_on_platform==False:   #在台下
            car.flag_going_on_platform==None    #True表示正在上台
            car.front_servo_degree(90,car.servo_speed)#冲下台以后舵机复位
            car.after_servo_degree(90,car.servo_speed)
            car.move_up(200,200)
            #已经找到台
            if car.io_data[6]==0 and car.io_data[7]==1:#前下传感器有障碍，前上无障碍
                #满足安全上台条件
                if car.io_data[0]==0 and car.io_data[2]==0:
                    car.flag_going_on_platform=True
                    car.move_up(200,200)    
                    car.go_up_platform()    #该函数只有单纯上台操作
                    time.sleep(0.5)
                    car.flag_going_on_platform=False
                    car.front_servo_degree(-24,car.servo_speed)#前爪触底
                    car.after_servo_degree(90,car.servo_speed)#后抓垂直
                    continue

                #不满足安全上台条件
                if car.io_data[0]==1 and car.io_data[2]==0:#右边缘悬空
                    car.move_back()
                    time.sleep(0.5)
                    car.make_a_turn(-1,0.55)
                    car.move_up()
                    time.sleep(0.4)
                    car.make_a_turn(1,0.55)
                    car.move_up()
                    continue

                if car.io_data[0]==0 and car.io_data[2]==1:#左边缘悬空
                    car.move_back()
                    time.sleep(0.5)
                    car.make_a_turn(1,0.55)
                    car.move_up()
                    time.sleep(0.4)
                    car.make_a_turn(-1,0.55)
                    car.move_up()
                    continue
            #台下寻台
            #if car.io_data[4]==0 and car.io_data[5]==0:#与台平行
            if car.io_data[6]==1:#前下传感器无障碍/接收不到反光（与台平行）
                #（台在右侧时）
                print("main:台在左侧/右侧,执行右转90度")        
                car.make_a_turn(1,0.55)#右转90度
                #（台在在左侧时）
                if car.io_data[6]==0 and car.io_data[7]==0:#面壁
                    print("main:右转90度后面壁,执行右转180度,前进0.8s")
                    car.make_a_turn(1,1)#右转180度
                    car.move_up()
                    time.sleep(0.8)
                    continue
                print("main:找到台,执行前进0.8s")  
                car.move_up()
                time.sleep(0.8)
                continue

            if car.io_data[6]==0 and car.io_data[7]==0:#前面壁
                if car.io_data[4]==1 and car.io_data[5]==0:#左面壁
                    print("main:前和左面壁,右转90度,前进0.8s,再右转90度,前进0.8s")
                    car.make_a_turn(1,0.55)
                    car.move_up(100,100)
                    time.sleep(0.8)
                    car.make_a_turn(1,0.55)
                    car.move_up(100,100)
                    time.sleep(0.8)
                    continue
                if car.io_data[4]==0 and car.io_data[5]==1:#右面壁
                    print("main:前和右面壁,左转90度,前进0.8s,再左转90度,前进0.8s")
                    car.make_a_turn(-1,0.55)
                    car.move_up(100,100)
                    time.sleep(0.8)
                    car.make_a_turn(-1,0.55)
                    car.move_up(100,100)
                    time.sleep(0.8)
                    continue
                if car.io_data[4]==0 and car.io_data[5]==0:#左右面壁,面对角落
                    print("main:前和左右都面壁,后退0.8s,再右转130度(不确定?),前进0.8s,再右转90度,前进0.8s")
                    car.move_back(0,0)
                    time.sleep(0.8)
                    car.make_a_turn(1,0.7)
                    car.move_up(100,100)
                    time.sleep(0.8)
                    car.make_a_turn(1,0.55)
                    car.move_up(100,100)
                    time.sleep(0.8)
                    continue
                print("main:普通面壁,右转180度,前进0.8s")
                car.make_a_turn(1,1)
                car.move_up(100,100)
                time.sleep(0.8)
                continue
            # if car.io_data[4]==0 and car.io_data[5]==0 and car.io_data[2]==0 and car.io_data[0]==0:
            #     print("sijiao")
            #     car.move_back()
            #     time.sleep(0.3)
            #     car.make_a_turn(1,0.55)
            #     car.move_up(0,0)
            #     time.sleep(1.5)
            #     continue

#             #台下车辆倾覆自救
#             if car.adc_data[3]>2500 and car.flag_going_on_platform==False and car.io_data[1]==0 and car.io_data[3]==0:
#                 print("main:车辆倾覆自救_前部朝上")
#                 car.after_servo_degree(0,car.servo_speed)
#                 time.sleep(0.8)
#                 car.after_servo_degree(90,car.servo_speed)
#                 continue    
#             if car.adc_data[3]>2500 and car.flag_going_on_platform==False and car.io_data[0]==0 and car.io_data[2]==0:
#                 print("main:车辆倾覆自救_尾部朝上")
#                 car.front_servo_degree(0,car.servo_speed)
#                 time.sleep(0.8)
#                 car.front_servo_degree(90,car.servo_speed)
#                 continue
        
        
        if car.is_on_platform==True:#在台上
            car.front_servo_degree(-24,car.servo_speed)#调整前爪至攻击状态
            #判断边缘运动状态变更
            cur = car.edge_control_running
            if pre!=cur:
                print("main:边缘运动状态变更:",cur)
                pre=cur
                if cur==False and car.finding_box_flag==False:#台边缘操作未执行时，且未在找物体时执行。
                    time.sleep(0.3)
                    car.move_up(100,100)
                    print("main:台上前进")
                    #这里不能加continue

    except:
        print("main error")