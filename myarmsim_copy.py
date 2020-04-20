#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""

from numpy import asarray
from joy.plans import Plan
from joy.decl import *
from p2sim import ArmAnimatorApp
import time

class MyArmSim(ArmAnimatorApp):
    def __init__(self,Tp2ws):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###
      Tws2w = asarray([
           [1,0,0,  0],
           [0,1,0, -5],
           [0,0,1,-10],
           [0,0,0,  1]
      ]) 
      ###
      ### Arm specification
      ###
      armSpec = asarray([
              [0,0.01,1,  5,  0],
              [0,   1,0,  5,  0],
              [0,   1,0,  5,  0],
          ]).T
      ArmAnimatorApp.__init__(self,armSpec,Tws2w,Tp2ws)

    def show(self,fvp):
      fvp.plot3D([0],[0],[0],'^k',ms=10) # Plot black triangle at origin
      return ArmAnimatorApp.show(self,fvp)
    
    def onStart(self):
      ArmAnimatorApp.onStart(self)
      ###
      ### TEAM CODE GOES HERE
      ###
      self.corner_1 = []
      self.corner_2 = []
      self.corner_3 = []
      self.corner_4 = []
      self.corners = []
      self.corners.append(self.corner_1)
      self.corners.append(self.corner_2)
      self.corners.append(self.corner_3)
      self.corners.append(self.corner_4)
      self.movePlan = MovePlan(app, self.arm)
      
      
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
        print("\nmoving to corners")
        self.movePlan.corners = self.corners
        self.movePlan.start()
        
        return

      return ArmAnimatorApp.onEvent(self,evt)

class MovePlan(Plan):
  def __init__(self, app, arm):
    Plan.__init__(self, app)
    self.arm = arm
    self.corners = []

  def behavior(self):
    if self.corners != []:
      # iterate through list of saved corners
      for i in self.corners:
        print("\nmoving to corner {}".format(1 + self.corners.index(i)))
        self.arm[0].set_pos(i[0])
        self.arm[1].set_pos(i[1])
        self.arm[2].set_pos(i[2])
        # duration of each move
        yield self.forDuration(5)
        print("\nfinished moving to corner {}".format(1 + self.corners.index(i)))
      print("\nfinished moving to corners")
    

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