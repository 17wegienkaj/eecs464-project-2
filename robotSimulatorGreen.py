# file robotSimulator.py simulates a robot in an arena

from sensorPlanTCP import SensorPlanTCP
from robotSimIX import SimpleRobotSim,RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT, lineSensorResponse, lineDist
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from pylab import randn,dot,mean,exp,newaxis,rand,ones,asarray,find,prctile,linspace,arange,argmax
from joy.plans import AnimatorPlan
import math

current_angle = 0
        
class yMovePlan(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Angle to turn [rad]
    self.ang = 1+0j
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    progress("Pos: %s" % (s.tagPos))
    s.ang = self.ang
    s.yVis = True
    s.xVis = False
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.move(step)
      yield self.forDuration(dt)

class xMovePlan(Plan):
  """
  Plan simulates robot turning over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Angle to turn [rad]
    self.ang = 1j
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10
    # Distance to travel RIGHT/LEFT
    self.dist = 10

  def behavior(self):
    s = self.simIX
    s.ang = self.ang
    s.yVis = False
    s.xVis = True
    # Compute rotation step
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.move(step)
      yield self.forDuration(dt)
      
class Auto(Plan):
    """
    Autonomous mode for simulation of robot
    """
    def __init__(self,app,simIX,yMoveP,xMoveP):
        Plan.__init__(self,app)
        self.simIX = simIX
        self.sensor = self.app.sensor
        # Duration of travel [sec]
        self.dur = 3.0
        # Number of intermediate steps
        self.N = 10
        # Distance to travel
        self.dist = 10
        #Plans
	self.yMove = yMoveP
	self.xMove = xMoveP
	
    def negY(self):
	self.yMove.dist = 50
	yield self.yMove.start()
	yield self.forDuration(2.5)
	yield self.yMove.stop()

    def posY(self):
	self.yMove.dist = -50
	yield self.yMove.start()
	yield self.forDuration(2.5)
	yield self.yMove.stop()

    def posX(self):
	self.xMove.dist = 50
	yield self.xMove.start()
	yield self.forDuration(2.5)
	yield self.xMove.stop()

    def negX(self):
	self.xMove.dist = -50
	yield self.xMove.start()
	yield self.forDuration(2.5)
	yield self.xMove.stop()
	
            
    def behavior(self):
	dictBacktrack = {'negY': self.posY, 'posY': self.negY, 'posX': self.negX, 'negX': self.posX}	
	history = []
	ts,f,b = self.sensor.lastSensor
	key1 = 'headDown' #Default dictionary
        while True:
	    s = self.simIX

	    ts, f, b = self.sensor.lastSensor
	    old_f = f
	    old_b = b
	    
	    ts, w = self.sensor.lastWaypoints
	    old_wx = w[0][0]
	    old_wy = w[0][1]
	    new_wx = w[1][0]
	    new_wy = w[1][1]
	
	    old_w = dot(asarray(s.matTransform),asarray([[old_wx],[old_wy],[1]]))
	    old_wx = old_w[0]
	    old_wy = old_w[1]
	    new_w = dot(asarray(s.matTransform),asarray([[new_wx],[new_wy],[1]]))
	    new_wx = new_w[0]
	    new_wy = new_w[1]	    

	    diffy = abs(new_wy - old_wy)
	    diffx = abs(new_wx - old_wx)

	    if(diffy > diffx and old_wy > new_wy): #new waypoint is in -y dir
		key1 = 'headDown'
            elif(diffy > diffx and old_wy < new_wy): #new waypoint is in +y dir
		key1 = 'headUp'
	    elif(diffy < diffx and old_wx < new_wx): #new waypoint is in +x dir
		if key1 == 'headRight':
                    key1 = 'headRight'
                elif key1 == 'headRightAlt':
                    key1 = 'headRightAlt'
                else:
                    key1 = 'headRight'
	    elif(diffy < diffx and old_wx > new_wx): #new waypoint is in +x dir
		if key1 == 'headLeft':
                    key1 = 'headLeft'
                elif key1 == 'headLeftAlt':
                    key1 = 'headLeftAlt'
                else:
                    key1 = 'headLeft'
	    else: #TODO figure out what to do if new waypoint is in 45 deg from old waypoint
		continue
	     
            if(key1 == 'headDown'):
		    progress('Dict = %s ' % (key1))
		    yield self.negY()
		    history.append('negY')
		    if f > b and ((f < 150) or (b < 150)):
			progress("F = %d > B = %d" % (f,b))
			yield self.posX()
			history.append('posX');
		    elif b > f and ((f < 150) or (b < 150)):
			progress("F = %d < B = %d" % (f,b))
			yield self.negX()
			history.append('negX')
		    elif b < 10 or f < 10:
			progress("BACKTRACK")
			yield dictBacktrack[history.pop()]()
		    else:
			progress("F = %d, B = %d" % (f,b))
			yield self.negY()
			history.append('negY')

     
            elif(key1 == 'headUp'):
		    progress('Dict = %s ' % (key1))
		    yield self.posY()
		    history.append('posY')
		    if f > b and ((f < 150) or (b < 150)):
			progress("F = %d > B = %d" % (f,b))
			yield self.posX()
			history.append('posX');
		    elif b > f and ((f < 150) or (b < 150)):
			progress("F = %d < B = %d" % (f,b))
			yield self.negX()
			history.append('negX')
		    elif b < 10 or f < 10:
			progress("BACKTRACK")
			yield dictBacktrack[history.pop()]()
		    else:
			progress("F = %d, B = %d" % (f,b))
			yield self.posY()
			history.append('posY')

            elif(key1 == 'headRight'):
		    progress('Dict = %s ' % (key1))
		    yield self.posX()
		    history.append('posX')
		    if f > b and ((f < 150) or (b < 150)):
			progress("F = %d > B = %d" % (f,b))
			yield self.negY()
			history.append('negY');
		    elif b > f and ((f < 150) or (b < 150)):
			progress("F = %d < B = %d" % (f,b))
			yield self.posY()
			history.append('posY')
		    elif b < 10 or f < 10:
			progress("BACKTRACK")
			yield dictBacktrack[history.pop()]()
		    else:
			progress("F = %d, B = %d" % (f,b))
			yield self.posX()
			history.append('posX')
		    ts, f, b = self.sensor.lastSensor
		    if (f < old_f) or (b < old_b):
                        key1 = 'headRightAlt'

            elif(key1 == 'headRightAlt'):
		    progress('Dict = %s ' % (key1))
		    yield self.posX()
		    history.append('posX')
		    if f > b and ((f < 150) or (b < 150)):
			progress("F = %d > B = %d" % (f,b))
			yield self.posY()
			history.append('posY');
		    elif b > f and ((f < 150) or (b < 150)):
			progress("F = %d < B = %d" % (f,b))
			yield self.negY()
			history.append('negY')
		    elif b < 10 or f < 10:
			progress("BACKTRACK")
			yield dictBacktrack[history.pop()]()
		    else:
			progress("F = %d, B = %d" % (f,b))
			yield self.posX()
			history.append('posX')
		    ts, f, b = self.sensor.lastSensor
		    if (f < old_f) or (b < old_b):
                        key1 = 'headRight'          

            elif(key1 == 'headLeft'):
		    progress('Dict = %s ' % (key1))
		    yield self.negX()
		    history.append('negX')
		    if f > b and ((f < 150) or (b < 150)):
			progress("F = %d > B = %d" % (f,b))
			yield self.negY()
			history.append('negY');
		    elif b > f and ((f < 150) or (b < 150)):
			progress("F = %d < B = %d" % (f,b))
			yield self.posY()
			history.append('posY')
		    elif b < 10 or f < 10:
			progress("BACKTRACK")
			yield dictBacktrack[history.pop()]()
		    else:
			progress("F = %d, B = %d" % (f,b))
			yield self.negX()
			history.append('negX')
		    ts, f, b = self.sensor.lastSensor
		    if (f < old_f) or (b < old_b):
                        key1 = 'headLeftAlt'

            elif(key1 == 'headLeftAlt'):
		    progress('Dict = %s ' % (key1))
		    yield self.negX()
		    history.append('negX')
		    if f > b and ((f < 150) or (b < 150)):
			progress("F = %d > B = %d" % (f,b))
			yield self.posY()
			history.append('posY');
		    elif b > f and ((f < 150) or (b < 150)):
			progress("F = %d < B = %d" % (f,b))
			yield self.negY()
			history.append('negY')
		    elif b < 10 or f < 10:
			progress("BACKTRACK")
			yield dictBacktrack[history.pop()]()
		    else:
			progress("F = %d, B = %d" % (f,b))
			yield self.negX()
			history.append('negX')
		    ts, f, b = self.sensor.lastSensor
		    if (f < old_f) or (b < old_b):
				key1 = 'headLeft'
	    else:
		    progress('Dict = %s ' % (key1))
		    continue

class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.

     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """
  def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
    """
    Initialize the simulator
    """
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      )
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)
    # ADD pre-startup initialization here, if you need it

  def onStart( self ):
    """
    Sets up the JoyApp and configures the simulation
    """
    ### DO NOT MODIFY ------------------------------------------
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
    self.sensor.start()
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now
    ### MODIFY FROM HERE ------------------------------------------
    self.robSim = SimpleRobotSim(fn=None)
    self.yMove = yMovePlan(self,self.robSim)
    self.xMove = xMovePlan(self,self.robSim)
    self.auto = Auto(self, self. robSim, self.yMove, self.xMove)

  def showSensors( self ):
    """
    Display sensor readings
    """
    # This code should help you understand how you access sensor information
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )

  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    #### DO NOT MODIFY --- it WILL break the simulator
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg.encode("ascii"), self.srvAddr)

  def onEvent( self, evt ):
    #### DO NOT MODIFY --------------------------------------------
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus():
      #self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame():
      self.emitTagMessage()
    #### MODIFY FROM HERE ON ----------------------------------------
    if evt.type == KEYDOWN:
      if evt.key == K_UP and not self.yMove.isRunning():
        self.yMove.dist = 100.0
        self.yMove.start()
        return progress("(say) Move forward")
      elif evt.key == K_DOWN and not self.yMove.isRunning():
        self.yMove.dist = -100.0
        self.yMove.start()
        return progress("(say) Move back")
      if evt.key == K_LEFT and not self.xMove.isRunning():
        self.xMove.dist = 100.0
        self.xMove.start()
        return progress("(say) Turn left")
      if evt.key == K_RIGHT and not self.xMove.isRunning():
        self.xMove.dist = -100.0
        self.xMove.start()
        return progress("(say) Turn right")
      if evt.key == K_a and not self.auto.isRunning():
        self.auto.start()
      if evt.key == K_s:
	self.yMove.stop()
	self.xMove.stop()
	self.auto.stop()
	return progress("(say) Stopped")
      if evt.key != K_s and evt.key != K_a and evt.type == KEYUP:
	self.yMove.stop()
	self.xMove.stop()
	return progress("(say) Stopped")

    ### DO NOT MODIFY -----------------------------------------------
      else:# Use superclass to show any other events
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events

if __name__=="__main__":
  print("""
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer.
  """)
  import sys
  if len(sys.argv)>1:
      app=RobotSimulatorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST, cfg={'windowSize' : [160,120]})
  app.run()
