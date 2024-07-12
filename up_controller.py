'''
=============2024.07.02==============
小车控制类
作者:马思奇
状态:待完成
功能:   1.控制小车移动,爪子摆动
        2.台下寻台,上台
        3.台上边缘检测
        4.获取灰度值,红外光电传感器值
        5.扫码,推箱子
注意：代码为务必看明白再改，每一句话都有用，不要嫌麻烦，看懂了再改，牵一发而动全身
=====================================        
'''

import uptech
import time
import random
import threading
import cv2
import apriltag

class UpController:
    '''
    CDS控制:
    id=[1,2]为BDMC驱动器,位于车底
    id=[3,4,5,6]为CDS系列舵机分别对应四个爪子
    以上id编号可通过舵机调试软件设置;舵机/电机模式可用set_cds_mode()函数设置,也可通过舵机调试软件设置
    对于不同的模式,采取不同的控制策略
    CHASSIS_MODE_SERVO = 1 舵机控制策略
    CHASSIS_MODE_CONTROLLER = 2 电机控制策略
    不过本代码只用到了2策略,1策略没写哈哈哈啊哈,所以这两个没啥用    
    '''

    # chassis_mode 1 for cds5516 系列舵机 ,2 for BDMC系列驱动器及配套电机
    #此处四个爪子采用CDS5516舵机控制，轮子用BDMC驱动器控制（与CDS5516的控制接口通用）
    #这两个参数没有用到哈哈哈哈哈,因为硬件固定了
    CHASSIS_MODE_SERVO = 1          
    CHASSIS_MODE_CONTROLLER = 2     
#初始化
    def __init__(self):
        self.uptech=uptech.UpTech()
        self.uptech.LCD_Open(2)     #打开屏幕，参数2表示屏幕方向
        self.uptech.ADC_IO_Open()   #打开ADC（模拟数字转换器）输入输出
        self.uptech.CDS_Open()      #打开CDS5516舵机和BDMC驱动器

        self.servo_speed=700        #CDS舵机速度,即爪子速度
        self.turn_speed=580         #原地转弯函数make_turn的速度
        self.lspeed=512             #小车左轮速度对应id=1
        self.rspeed=512             #小车右轮速度对应id=2
                
        self.servo_degree_front_left = 500     #前左舵机水平的角度
        self.servo_degree_front_right = 485    #前右舵机水平的角度
        self.servo_degree_after_left = 485     #后左舵机水平的角度
        self.servo_degree_after_right = 500    #后右舵机水平的角度
        
        self.adc_data = []            #存储树莓派ADC口的数据
        self.io_data = []             #存储树莓派IO口数据
        self.is_on_platform=False     #默认不在台上
        self.flag_going_on_platform=None    #True为正在执行上台操作
        self.edge_control_running=None         #边缘时正在后退标志
        self.finding_box_flag=False            #扫码扫到结果标志
        self.tag_id=-1                  #扫码结果



##设置舵机模式(设置CDS是电机还是舵机,对应编号)
    def set_cds_mode(self,ids,mode):
        for id in ids:
            self.uptech.CDS_SetMode(id,mode)

#设置控制模式，1为CDS舵机模式，2为CDS电机模式
    def set_chassis_mode(self,mode):
        '''
        1为CDS舵机模式
        2为CDS电机模式(此处硬件使用BDMC,BDMC控制接口与CDS电机接口通用)
        '''
        self.chassis_mode = mode



#启动灰度，红外检测，台上台下判断线程
    def open_get_ADC(self):    
        ADC_thread = threading.Thread(name = "get_ADC_IO",target=self.get_ADC_IO)
        ADC_thread.setDaemon(True)
        ADC_thread.start()

#启动边缘检测及控制线程
    def open_edge_detect(self):
        edge_thread = threading.Thread(name = "edge_detect_thread",target=self.edge_detect_thread)
        edge_thread.setDaemon(True)
        edge_thread.start()

#启动扫码及推箱子线程
    def open_scan_pull_thread(self):
        scan_thread = threading.Thread(name = "scan_tag_thread",target=self.Scan_Qr_Code_and_Pull_box)
        scan_thread.setDaemon(True)
        scan_thread.start()



#获取灰度传感器，红外传感器的值 
    def get_ADC_IO(self):
        '''
        获取灰度传感器，红外传感器的值
        IO数据存在io_data中(数字信号0/1)
        ADC数据存在adc_data中(模拟信号)
        '''
        pre=False
        while True:
            try:
                #获取ADC数据
                self.adc_data = self.uptech.ADC_Get_All_Channle()#获取ADC数据，共9个，ADC0->左灰度，ADC1->右灰度，ADC8->后红外传感器，列表索引与之对应
                #if __name__=='__main__':
                    #print(self.adc_data)#查看ADC数据
                    #time.sleep(1)

                #位置判断（台上/台下）
                self.is_on_platform = False if self.adc_data[0] < 1400 or self.adc_data[1] < 500 else True#利用ADC获取的灰度值判断是否在平台上
                #print(self.is_on_platform)
                #time.sleep(1)
                cur=self.is_on_platform
                if pre!=cur:
                    print("ADC:小车位置改变至",cur)
                    pre=cur
                    #i=0
                    while self.flag_going_on_platform==True:
                        #time.sleep(0.5)
                        continue#上台过程中不需要检测灰度
                    # if self.edge_running==False and self.flag_going_on_platform==False:
                    #     time.sleep(0.5)
                
                
                #获取IO数据
                io_all_input = self.uptech.ADC_IO_GetAllInputLevel()#获取所有口的IO数据，得到的是一个十进制整数
                io_array = '{:08b}'.format(io_all_input)#将整数处理成二进制0/1(每一位代表一个IO数值)，共8个
                '''
                #用于查看IO原始数值
                #print("type=",type(io_all_input),"data=",io_all_input)#查看io_all_input
                #time.sleep(1)
                '''
                self.io_data = [int(char) for char in io_array[::-1]]
            except:
                print("ADC or IO error")

#边缘检测及边缘控制
    def edge_detect_thread(self):
        #开始边缘检测
        while True:
            try:
                if self.is_on_platform==True:   #仅在台上时执行
                    if self.io_data[0]==1 and self.io_data[2]==1 and self.io_data[6]==0:#前面都悬空
                        print("edge:前面都悬空")
                        self.edge_control_running=True
                        self.move_back()#后退
                        time.sleep(0.9)
                        self.make_a_turn(1,0.7)#介于90-180度之间，不知道多少
                        #self.move_up(100,0)
                        self.edge_control_running=False
                        print("edge:前操作完成")
                        continue
                    
                    if self.io_data[0]==0 and self.io_data[2]==1:#仅左前悬空
                        if self.io_data[3]==1:
                            print("edge:左前左后悬空,右转90度")
                            self.edge_control_running=True
                            self.move_stop()
                            self.make_a_turn(1,0.55)
                            self.edge_control_running=False
                            print("edge:左前左后悬空操作完成")
                            continue
                        print('edge:仅左前悬空')
                        self.edge_control_running=True
                        self.move_stop()
                        self.move_back()#后退
                        time.sleep(0.9)
                        print("edge:后退操作完成")
                        self.move_stop()
                        self.make_a_turn(1,0.55)#速度为580mm/s时，0.55s相当于转90度
                        self.edge_control_running=False
                        print("edge:仅左前操作完成")
                        continue

                    if self.io_data[2]==0 and self.io_data[0]==1:#仅右前悬空
                        if self.io_data[1]==1:
                            print("edge:右前右后悬空,左转90度")
                            self.edge_control_running=True
                            self.move_stop()
                            self.make_a_turn(-1,0.55)
                            self.edge_control_running=False
                            print("edge:右前右后悬空操作完成")
                            continue
                        print("仅右前悬空")
                        self.edge_control_running=True
                        self.move_stop()
                        self.move_back()
                        time.sleep(0.9)
                        self.move_stop()
                        self.make_a_turn(1,0.61)
                        self.edge_control_running=False
                        print("仅右前操作完成")
                        continue
                    
                #这里最好还是写点判断，可以解决后退过多的问题
#                     if self.io_data[4]==0 and self.io_data[5]==1:#仅左后悬空
#                         continue
#                     if self.io_data[5]==0 and self.io_data[4]==1:#仅右后悬空
#                         continue
#                     if self.io_data[4]==1 and self.io_data[5]==1:#后面都悬空
#                         continue
                
                
                '''
            #依据灰度传感器判断边缘
            while True:
                if up.adc_data[0]<=1000 or up.adc_data[1]<=1000:#阈值还得调整
                    print("到边缘啦")
                    self.up.CDS_SetSpeed(1, 0)
                    self.up.CDS_SetSpeed(2, 0)
                    up.move_stop()
                    print("停止")
                    break
                if up.is_on_platform==False:    #掉台以后退出循环
                    break
                '''
            except:
                print("边缘检测出错")
                continue
    #扫码及扫码后控制
    def Scan_Qr_Code_and_Pull_box(self):
        cap = cv2.VideoCapture(0)
        at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))
        #摄像头分辨率640*480
        w = 640;h = 480;weight =320
        cap.set(3,640)
        cap.set(4,480)
        cup_w = int((w - weight) / 2) 
        cup_h = int((h - weight) / 2) + 50

        while True:
            _ , frame = cap.read()
            #print(frame)
            frame = frame[cup_h:cup_h + weight,cup_w:cup_w + weight]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(gray)
            self.tag_id = tags[0].tag_id if len(tags) else -1   #扫码结果
            if self.is_on_platform==True:   #在台上
                #找到箱子/小车
                if self.io_data[6]==0 and self.edge_control_running==False:#前下摄像头被挡（前上不被挡说明箱子被推下？）且未边缘操作时执行
                    #要推的箱子
                    if self.tag_id==2:
                        print("Qr:识别到2号箱子")
                        #这里的循环是防止掉台,将小车运动交给边缘检测接管
                        while self.edge_control_running==False:
                            print("Qr:将加速推箱子")
                            self.move_up(400,400)
                            if self.is_on_platform==False:    #掉台以后退出循环
                                break
                        continue
                    #不推的箱子
                    if self.tag_id==1:
                        print("Qr:识别到不要推的1号箱子,远离")
                        while self.edge_control_running==False:
                            print("Qr:将远离箱子")
                            self.move_back()
                            if self.is_on_platform==False:    #掉台以后退出循环
                                break
                        self.make_a_turn(1,random.uniform(0.55,0.9))
                        continue
                    #遇到车
                    if self.tag_id==-1:
                        if self.io_data[6]==0 or self.io_data[7]==0:
                            print("Qr:识别到车,攻击")
                            while self.edge_control_running==False:
                                print("Qr:将加速推车")
                                self.move_up(300,300)
                                if self.is_on_platform==False:    #掉台以后退出循环
                                    break
                        continue
                


                #找箱子（转弯可能要改成差速？2024.07.04 8：44）
                if self.io_data[5]==0 and self.edge_control_running==False:           #左侧摄像头被挡，有障碍物
                    print("左侧有物体")
                    self.finding_box_flag=True
                    self.make_a_turn(-1,0.4)    #向左转
                    #self.move_up(100,100)
                    self.finding_box_flag=False
                    print("左侧找物体操作完成")
                    continue
                
                if self.io_data[4]==0 and self.edge_control_running==False:           #右侧摄像头被挡，有障碍物
                    print("右侧有物体")
                    self.finding_box_flag=True
                    self.make_a_turn(1,0.4)     #向右转
                    #self.move_up(100,100)
                    self.finding_box_flag=False
                    print("右侧找物体操作完成")
                    continue


            cv2.imshow('video', frame)
            c = cv2.waitKey(1) & 0xff
            if c == 27:#ESC botton
                cap.release()
                break
            #time.sleep(0.5)
        cap.release()
        cv2.destroyAllWindows()




#前舵机角度调整
    '''
    参数1:+90度,表示水平方向向上90度
    参数2:舵机速度
    ===2024.6.24===
    '''
    def front_servo_degree(self,degree,speed):
        self.uptech.CDS_SetAngle(6,int(self.servo_degree_front_right+(10/3)*degree),speed)
        self.uptech.CDS_SetAngle(7,int(self.servo_degree_front_left-(10/3)*degree),speed)

#后舵机角度调整 
    '''
    参数1:+90度,表示水平方向向上90度
    参数2:舵机速度
    ===2024.6.24===
    '''
    def after_servo_degree(self,degree,speed):
        self.uptech.CDS_SetAngle(4,int(self.servo_degree_front_right+(10/3)*degree),speed)
        self.uptech.CDS_SetAngle(5,int(self.servo_degree_front_right-(10/3)*degree),speed)

##前进
    def move_up(self,lspeed_add=0,rspeed_add=0):
        '''
        参数1和2省略时left_speed_add=0,right_speed_add=0按原速度运动
        参数1:lspeed_add:左轮速度增量
        参数2:rspeed_add:右轮速度增量
        '''
        self.uptech.CDS_SetSpeed(1, self.lspeed+lspeed_add)
        self.uptech.CDS_SetSpeed(2, -self.rspeed-rspeed_add)

##后退
    def move_back(self,lspeed_add=0,rspeed_add=0):
        '''
        参数1和2省略时left_speed_add=0,right_speed_add=0按原速度运动
        参数1:lspeed_add:左轮速度增量
        参数2:rspeed_add:右轮速度增量
        '''
        self.uptech.CDS_SetSpeed(1, -self.lspeed-lspeed_add)
        self.uptech.CDS_SetSpeed(2, self.rspeed+rspeed_add)

##停止
    def move_stop(self):
        self.uptech.CDS_SetSpeed(1, 0)
        self.uptech.CDS_SetSpeed(2, 0)
        
#原地转弯函数
    def make_a_turn(self,direction = 1, times = 1.0):
        '''
        参数1:顺时针1,逆时针-1
        参数2:时间
        speed=580代表580mm/s
        speed=580 times=0.55 90 degrees
        speed=580 times=1  180 degrees
        speed=580 times=1.8 360 degrees
        '''
        self.uptech.CDS_SetSpeed(1, self.turn_speed * direction)
        self.uptech.CDS_SetSpeed(2, self.turn_speed * direction)
        time.sleep(1 * times)
        self.uptech.CDS_SetSpeed(1, 0)
        self.uptech.CDS_SetSpeed(2, 0)
#前上台
    def go_up_platform(self):
        '''
        该函数只有上台逻辑,是否符合上台条件还需判断！！！
        '''
        self.move_up()
        time.sleep(1.2)
        self.front_servo_degree(-60,self.servo_speed)
        time.sleep(0.8)
        self.front_servo_degree(0,self.servo_speed)
        time.sleep(0.8)
        self.after_servo_degree(-60,self.servo_speed)
        time.sleep(0.8)
        self.after_servo_degree(0,self.servo_speed)

#后上台
    def go_up_back_platform(self):
        '''
        该函数只有上台逻辑,是否符合上台条件还需判断！！！
        '''
        self.move_back()
        time.sleep(1.2)
        self.after_servo_degree(-60,self.servo_speed)
        time.sleep(0.9)
        self.after_servo_degree(0,self.servo_speed)
        time.sleep(0.9)
        self.front_servo_degree(-60,self.servo_speed)
        time.sleep(0.9)
        self.front_servo_degree(0,self.servo_speed)

##lcd打印
    def lcd_display(self,content):
        self.uptech.LCD_PutString(30, 0, content)
        self.uptech.LCD_Refresh()
        self.uptech.LCD_SetFont(self.uptech.FONT_8X14)


if __name__ == '__main__':
    up_controller = UpController()
    up_controller.set_chassis_mode(2)
    #up_controller.get_ADC_IO()
    
    
    #up_controller.go_up_platform()
    #up_controller.front_servo_stand()
    #up_controller.front_servo_laying()
    #up_controller.after_servo_stand()
    #up_controller.after_servo_laying()
    #up_controller.make_a_turn(1,0.3)
    
    up_controller.move_back()
    time.sleep(0.9)
    up_controller.move_stop()
    #up_controller.open_get_ADC()
    #up_controller.open_edge_detect()
    #up_controller.move_up(200,200)
    while True:
        #print(up_controller.adc_data)
        #time.sleep(0.5)
#         if up_controller.edge_running==True:
#             time.sleep(1)
#             up_controller.move_up(100,100)
        
        pass
    #up_controller.move_up()

    





