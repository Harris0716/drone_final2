#!/usr/bin/env python
# -*- coding: utf-8 -*-

### 此為行為樹behavior tree範例程式 ### 
### 該程式針對先前的pass_example.py 使用行為樹進行修改, ### 
### 該程式針對無人機過框的過程區分為3個條件節點, 6個動作節點 ###
### 定義了節點的階層與順序 ###
### >> 表示順序節點 Sequence node
### | 表示選擇節點 Selector node
### 請搭配先前的 pass_example中的 test_h264_sub.py 以及 simple_tello.py 使用 ###
### behave 請放在同一目錄下 ###
### 如有新增節點, 建議可以在節點加入 print, 觀察其在 terminal 上 的流程 ###
 
import rospy                        # 導入套件: rospy
import simple_tello                 # 導入 simple_tello

from geometry_msgs.msg import Twist # 導入 geometry_msgs 裡的 Twist 訊息格式
from std_msgs.msg import Empty      # 導入 std_msgs 裡的 Empty 訊息格式
from behave import *                # 導入 behave 中的所有class
from time import sleep              # 導入 time 裡的 sleep

#建立 simple_tello.Tello_drone 變數 t1
t1 = simple_tello.Tello_drone()

# class bt_mission
# 建立 基於行為樹之任務模型
class bt_mission:

    # bool 變數 isContinue, 當 True 時 bt 會持續 tick, False 時則會停止tick, 並回到main中往後執行
    isContinue = True
    
    # 定義 影像的中心點座標, 影像大小為 (960, 720)
    center = (480, 200)

    # 初始 dx, dy 
    dx = -1
    dy = -1

    # 初始化 
    def __init__(self):
    
        # behavior tree
        # 修改：在doAddSp後加入doTurnLeft和doLongHover
        self.tree = (
            (self.isNotDataReceived >> self.doHover)
            | (self.isReceivePass >> self.doAddSp >> self.doTurnLeft >> self.doLongHover >> self.doBtStop)
            | (self.doComputeData >> (self.isForward >> self.doForward) | (self.doCorrection)) 
        )    

    @condition
    def isNotDataReceived(self):
        print("condition: isNotDataReceived")
        return t1.state.target_x == -1 and t1.state.target_y == -1

    @action
    def doHover(self):
        print("action: doHover")
        msg = Twist()
        t1.controler.move(msg, 0.5)

    @condition
    def isReceivePass(self):
        print("condition: isReceivePass")
        return t1.state.canPass == 1

    @action
    def doAddSp(self):
        print("action: doAddSp")
        msg = Twist()
        msg.linear.y = 0.4
        t1.controler.move(msg, 3)
      
        msg = Twist()
        msg.linear.y = 0.5
        t1.controler.move(msg, 3)

    # 新增：左轉90度動作節點
    @action
    def doTurnLeft(self):
        print("action: doTurnLeft")
        msg = Twist()
        msg.angular.z = 0.5  # 正值為右轉，負值為左轉
        t1.controler.move(msg, 1.8)  # 持續約1.8秒可轉90度

    # 新增：懸停5秒動作節點
    @action
    def doLongHover(self):
        print("action: doLongHover")
        msg = Twist()
        t1.controler.move(msg, 5.0)

    @action
    def doBtStop(self):
        print("action: doBtStop")
        bt_mission.isContinue = False

    @action
    def doComputeData(self):
        print("action: doComputeData")
        bt_mission.dx = t1.state.target_x - bt_mission.center[0]
        bt_mission.dy = t1.state.target_y - bt_mission.center[1]
        print("doComputeData: ", bt_mission.dx, bt_mission.dy)
     
    @condition
    def isForward(self):
        print("condition: isForward")
        return abs(bt_mission.dx) < 30 and abs(bt_mission.dy) < 30
    
    @action
    def doForward(self):
        print("action: doForward")
        msg = Twist()
        msg.linear.y = 0.2
        t1.controler.move(msg, 0.5)
    
    @action
    def doCorrection(self):
        print("action: doCorrection")
        msg = Twist()
        if bt_mission.dx != 0:
          msg.linear.x = bt_mission.dx / abs(bt_mission.dx) * 0.1
        if bt_mission.dy != 0:
          msg.linear.z = -bt_mission.dy / abs(bt_mission.dy) * 0.2
        t1.controler.move(msg, 0.5)
    
    def run(self):
        while True:
            if bt_mission.isContinue == False:
                break
            bb = self.tree.blackboard(1)
            state = bb.tick()
            print("state = %s\n" % state)
            while state == RUNNING:
                state = bb.tick()
                print("state = %s\n" % state)
            assert state == SUCCESS or state == FAILURE

def main():
    while t1.state.is_flying == False:
      t1.controler.takeoff()
  
    while t1.state.fly_mode != 6:
      print("wait...")
 
    btCm_n = bt_mission()
    sleep(2)
    print("bt start...")
    btCm_n.run()
    
    while t1.state.fly_mode != 6:
      print("wait...") 
    
    while t1.state.is_flying == True:
      t1.controler.land()        

if __name__ == "__main__":
    rospy.init_node('h264_pub', anonymous=True)
    main()