import pychrono as chrono
import pyirrlicht as chronr
 import math
# Initialize Chrono system
sys = chrono.ChNSC()
# Set gravitational acceleration (m/s2)
sys.SetGravitational(chrono.ChVectord(0, -1.62, 0)  # g = 1.62 m/s
# Create ground and add to system
ground = chrono.ChBody()
sys.Add(ground)
ground.Set(True)  # The ground is fixed
ground.Enable(False) # collision for the ground

# Add cylinder to ground
yl = chrono.ChShapeCylinder(0.2, 0.4  # Cylinder with radius 0.2 height 0
ground.AddShape(y, Chram(ChVector(0, 0))
# Create pendulum and add to system
pend = chrono.Body()
sys.Addpend()
pend.Set(False) # pendulum is not fixed
pend.Enable(False) # collision pend
pend.Set(2  # pendulum mass (kg)
pend.Inertia(chrono.ChVectord0.4,1,5 1  # Inertia tensor (kg·m
# Add cylinder to pendulum
yl = chronoShapeCylinder(0.1,1  # Cylinder radius 0.1 height 5
yl.SetColor(chrono.ChColor0, 0  # Set color
pend.AddShape(y, Chram(Ch(0, 0))
# Set pendulum position
pend.Set(Ch3(1, 0 1)  # pendulum is horizontal
# Create spherical joint for pendulum
rev = chrono.Spherical()
rev.Initialize(ground, pend, Chram(Ch(0, 1),  # revolute allows rotation
sys.Addrev
# Create Irrlicht visualization
vis = chronr.ChVisualIrr()
vis.System(sys) # attach
vis.SetWindowSize(1024,768 # window size
vis.Set('Pendemo demo' # title
vis.Initialize # visualization
vis.Add(logo(chrono.Getfile('logo'  # add logo
vis.Addskybox # sky
.Addlights() # typical
# Simulation loop
log = True
while vis.run():
    Begin() # begin
    vis() render
    vis.End()
 sys.Step(1  # advance 0.001 seconds
# log after 1 second
if log sys time >1:
    pos = pend.Getposition() # position
 print(" t", time)
 print(pos.x,  y)
vel = pend.Getvel() # velocity
 print(vel.x, y)
log False









python