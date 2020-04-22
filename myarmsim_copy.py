#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""

from numpy import asarray, array, dot, arctan, cos, arccos, sin, degrees
from joy.plans import Plan
from joy.decl import *
from p2sim import ArmAnimatorApp
from vis3d import FourViewPlot, xyzCube, iCube, iFace, plotVE
import time

class MyArmSim(ArmAnimatorApp):
    def __init__(self,Tp2ws):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###
      Tws2w = asarray([
           [1,0,0,  0],
           [0,1,0, -5],
           [0,0,1,-12],
           [0,0,0,  1]
      ]) 
      ###
      ### Arm specification
      ###
      armSpec = asarray([
              [0,0.01,1,  5,  0],
              [0,   1,0,  5,  0],
              [0,   1,0,  4,  0],
          ]).T
      ArmAnimatorApp.__init__(self,armSpec,Tws2w,Tp2ws)
      self.hWorld = []

    def show(self,fvp):
      fvp.plot3D([0],[0],[0],'^k',ms=10) # Plot black triangle at origin
      if self.square != []:
        lt = dict(marker='+',color='r')
        for point in self.square:
          fvp.plot3D(point[0],point[1],point[2],**lt)
      return ArmAnimatorApp.show(self,fvp)
    
    def append_point(self, paperCoord):
      ###
      ### add points to square
      ###
      self.square.append(self.paperToWorld(paperCoord))
      return

    def paperToWorld(self, paperCoord):
      hPaper = array(list(paperCoord) + [1]) # mage homogenous coord
      hWorld = dot(self.Tp2w, hPaper)
      # self.hWorld = hWorld

      print("\nhpaper: \n{}, \nTp2w: \n{}, \nhWorld: \n{}".format(hPaper, self.Tp2w, hWorld))
      return hWorld

    def onStart(self):
      ArmAnimatorApp.onStart(self)
      ###
      ### TEAM CODE GOES HERE
      ###
      seg_2 = 5 # segment 2 length 
      seg_3 = 4 # segment 3 length 
      self.square = []
      self.corner_1 = []
      self.corner_2 = []
      self.corner_3 = []
      self.corner_4 = []
      self.corners = []
      self.corners.append(self.corner_1)
      self.corners.append(self.corner_2)
      self.corners.append(self.corner_3)
      self.corners.append(self.corner_4)
      self.movePlan = MovePlan(app, self.arm, seg_2, seg_3, self.square)
      
      
    def onEvent(self,evt):
      ###
      ### TEAM CODE GOES HERE
      ###    Handle events as you see fit, and return after
      # Ignore everything except keydown events
      if evt.type != KEYDOWN:
        return

      if evt.key == K_1:
        print("\nset corner 1")
        # x_pos has not been set
        if not self.corner_1:
          self.corner_1.append(self.arm[0].get_goal())
          self.corner_1.append(self.arm[1].get_goal())
          self.corner_1.append(self.arm[2].get_goal())
          print("\n{},{},{}".format(self.corner_1[0],self.corner_1[1],self.corner_1[2]))
          return
        
        # already set, override
        self.corner_1[0] = self.arm[0].get_goal()
        self.corner_1[1] = self.arm[1].get_goal()
        self.corner_1[2] = self.arm[2].get_goal()
        print("\n{},{},{}".format(self.corner_1[0],self.corner_1[1],self.corner_1[2]))
        return

      if evt.key == K_2:
        print("\nset corner 2")
        # x_pos has not been set
        if not self.corner_2:
          self.corner_2.append(self.arm[0].get_goal())
          self.corner_2.append(self.arm[1].get_goal())
          self.corner_2.append(self.arm[2].get_goal())
          print("\n{},{},{}".format(self.corner_2[0],self.corner_2[1],self.corner_2[2]))
          return
        
        # already set, override
        self.corner_2[0] = self.arm[0].get_goal()
        self.corner_2[1] = self.arm[1].get_goal()
        self.corner_2[2] = self.arm[2].get_goal()
        print("\n{},{},{}".format(self.corner_2[0],self.corner_2[1],self.corner_2[2]))
        return
      
      if evt.key == K_3:
        print("\nset corner 3")
        # x_pos has not been set
        if not self.corner_3:
          self.corner_3.append(self.arm[0].get_goal())
          self.corner_3.append(self.arm[1].get_goal())
          self.corner_3.append(self.arm[2].get_goal())
          print("\n{},{},{}".format(self.corner_3[0],self.corner_3[1],self.corner_3[2]))
          return
        
        # already set, override
        self.corner_3[0] = self.arm[0].get_goal()
        self.corner_3[1] = self.arm[1].get_goal()
        self.corner_3[2] = self.arm[2].get_goal()
        print("\n{},{},{}".format(self.corner_3[0],self.corner_3[1],self.corner_3[2]))
        return
      
      if evt.key == K_4:
        print("\nset corner 4")
        # x_pos has not been set
        if not self.corner_4:
          self.corner_4.append(self.arm[0].get_goal())
          self.corner_4.append(self.arm[1].get_goal())
          self.corner_4.append(self.arm[2].get_goal())
          print("\n{},{},{}".format(self.corner_4[0],self.corner_4[1],self.corner_4[2]))
          return
        
        # already set, override
        self.corner_4[0] = self.arm[0].get_goal()
        self.corner_4[1] = self.arm[1].get_goal()
        self.corner_4[2] = self.arm[2].get_goal()
        print("\n{},{},{}".format(self.corner_4[0],self.corner_4[1],self.corner_4[2]))
        return

      # move to programmed corners in plan 
      if evt.key == K_SPACE:
        # self.movePlan.square = self.square
        self.movePlan.start()
        return

      # append points and begin drawing square
      if evt.key == K_5:
        self.append_point([8,1,0])
        self.append_point([8,5,0])
        self.append_point([8,10,0])
        
        return
      return ArmAnimatorApp.onEvent(self,evt)

class MovePlan(Plan):
  def __init__(self, app, arm, seg_2, seg_3, square):
    Plan.__init__(self, app)
    self.arm = arm
    self.corners = []
    self.seg_2 = seg_2
    self.seg_3 = seg_3
    self.square = square

  def behavior(self):
    
    print("\nDrawing square")
    print(self.square)
    for target in self.square:
      ###
      ### move all motor joints
      ###
      
      angle_1 = 100 * degrees(arctan(target[1]/(target[0])))  # arctan y/x convert to deg
      
      # keep moving until difference reaches zero
      while abs(self.arm[0].get_goal() - angle_1) > 0:
        
        # actual pos within 5 deg of goal
        if abs(angle_1 - self.arm[0].get_goal()) < 500:
          break
        
        self.arm[0].set_pos(self.arm[0].get_goal() + (angle_1 - self.arm[0].get_goal())/3) # increment by 1/10 * abs differnce 
        # print("\ncurr angle {}".format(self.arm[0].get_goal()))
        yield self.forDuration(1)

      print("\nangle 1: {}\n  goal:{}\n  actual:{}".format(angle_1,
                                                          self.arm[0].get_goal(),
                                                          self.arm[0].get_pos()))
      
      ###
      ### use inverse kinematics and cosine rule to find second and third angles
      ###

      # find third motor joint

      angle_3 = 100 * degrees(-arccos((target[0]**2 + target[2]**2 - self.seg_2**2 - self.seg_3**2) / (2 * self.seg_2 * self.seg_3)))

      # find second motor joint 
      angle_2 = 100 * degrees(arctan(target[2]/target[0]) - arctan((self.seg_3 * sin(angle_3) / (self.seg_2 + self.seg_3 * cos(angle_3)))))

      # prevent looping
      #if angle_2 < -18000:
      #  angle_2 = 360000 - abs(angle_2)
      
      #if angle_3 < -18000:
      #  angle_3 = 360000 - abs(angle_3)

      print("ang 2: {}, ang 3: {}".format(angle_2, angle_3))

      # keep moving motor 2 until difference reaches zero
      # use pos for angle 2
      while abs(self.arm[1].get_goal() - angle_2) > 0:

        # actual pos within 5 deg of goal
        if abs(angle_2 - self.arm[1].get_goal()) < 500:
          break

        # increment by 1/5 * abs differnce 
        self.arm[1].set_pos(self.arm[1].get_goal() + (angle_2 - self.arm[1].get_goal()) / 5)

        # print("\ncurr angle {}".format(self.arm[1].get_goal()))
        yield self.forDuration(1) 

      print("\nangle 2: {}\n  goal:{}\n  actual:{}".format(angle_2,
                                                          self.arm[1].get_goal(),
                                                          self.arm[1].get_pos()))

      # keep moving motor 3 until difference reaches zero
      while abs(self.arm[2].get_goal() - angle_3) > 0:

        # actual pos within 3 deg of goal
        if abs(angle_3 - self.arm[2].get_goal()) < 300:
          break
        
        # increment by 1/5 * abs differnce
        self.arm[2].set_pos(self.arm[2].get_goal() + (angle_3 - self.arm[2].get_goal()) / 5)  
        # print("\ncurr angle {}".format(self.arm[2].get_goal()))
        yield self.forDuration(1)

      print("\nangle 3: {}\n  goal:{}\n  actual:{}".format(angle_3,
                                                          self.arm[2].get_goal(),
                                                          self.arm[2].get_pos()))

      yield self.forDuration(1)

    print("\nfinished drawing square")

    
    

if __name__=="__main__":
  # Transform of paper coordinates to workspace
  Tp2ws = asarray([
       [0.7071,0,-0.7071,0],
       [0,     1,      0,0],
       [0.7071,0, 0.7071,0],
       [0,     0,      0,1]
  ])
  
  
  app = MyArmSim(Tp2ws)
  app.run()