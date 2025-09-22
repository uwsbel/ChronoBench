import pychrono as chrono
import pyirr as chronoir
import math

# Initialize Chrono system
sys = chrono.ChNSCsystem()

# Set the gravitational acceleration (in m/s2)
sys.SetGravitational(chrono.ChVector(0, -9, 0, 0)  # = 9 m/s2

# Create the ground and add to the system
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True) # The ground is fixed
ground.EnableCollision(False) # Disable collision for ground

# Add a visualization to ground
cyl = chrono.ChShapebox(0.1) # with 0.1 size
ground.AddShape(c, chronoFram(chrono.Chram(ChVector(0, 0,0,0))

# Create pendulum and add to system
pend = chrono.ChBody()
sys.Add(pend)
pend.Set(False) # pend can move
pend.Enable(False) # collision
pend.Set(1)  # mass (kg
pend.SetInertia(chrono.ChVector(0, 1,1))  # inertia

# Add visualization for pend
c = chrono.ChShapebox(0.1, 1) # box 0.1 size
c.Set(chronoColor(chrono(0.8,0, 0)) # color
pend.Add(c, chronoFram(chrono(ChramCh(0, 0,0,0)

# Set initial position of pendulum
pend.Set(ChVector(1,0,0)

# Create revolute to connect pend
rev = chrono.LockRev()
rev(ground, pend, chronoFram(chrono.Chram(0, 0,0)
sys.Add(rev)

# Create second pendulum
pend2 = chronoBody()
sys.Add(pend2)
pend.Set(False) # can
pend.Enable(False) # collision
pend.Set(1) # mass (kg
pend.SetInertia(chronoVector(0,1))  # inertia

# Add visualization for second pendulum
c = chrono.Chbox(0.1,1) box 0.1 size
c.SetColor(chrono(0.6,0) # color
pend.Add(chronoFram(Chram(0, 0,0)

# Set second pendulum position
pend2.Set(Ch(Ch(Ch(,0,))

# Create revolute to connect second pendulum
rev2 = chronoRev()
rev(, pend2, chronoF(Chram(Ch(0, 0)
sys.Add(rev2

# Create Irrlicht visualization
vis = chronr.ChVisualIrr()
vis.Attach(sys) # attach Chrono
vis.SetWindowSize(1024, 768) # window size
vis.Set('BodyRef demo') # title
vis.Initialize() # init
vis.Add(logo(chrono.GetDatafile('logo.png')) # logo
.AddBox() # sky
.Add(Chram(0,6) # camera
.Add(ram() # lights
.Add() typical lights

# loop
log = True # flag
while vis.Run():
 vis.Begin() # scene
vis() vis
sys(1-3) # step

# Log after 1 second
if log sys() > 1:
    pos = pend.Get() # position
print(' =', sys)
print(' ',.x '.y)
lin = pend.GetDt() # velocity
print( lin.x)
log = False

