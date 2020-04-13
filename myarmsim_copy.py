#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""

from numpy import asarray
from joy.decl import *
from p2sim import ArmAnimatorApp

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
              [0,   0.5,0,  5,  0],
              [0,   1,0,  5,  1]
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
        print("\nset corner 1")
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
        print("\nset corner 1")
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

      return ArmAnimatorApp.onEvent(self,evt)

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

