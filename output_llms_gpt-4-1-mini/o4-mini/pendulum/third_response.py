import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# 1) Create the Chrono physical system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# 2) Create the (fixed) ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(False)
sys.Add(ground)

# Visualize ground as a short cylinder
ground_cyl = chrono.ChVisualShapeCylinder(0.2, 0.4)
# Place it at (0,0,1) in world coords
ground.AddVisualShape(ground_cyl, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.0)))

# Precompute the quaternion to rotate a cylinder's default Z‐axis into X
rotZtoX = chrono.Q_from_AngAxis(math.pi/2, chrono.ChVectorD(0, 1, 0))

# 3) FIRST pendulum
pend1 = chrono.ChBody()
pend1.SetMass(1.0)
# A guess at principal inertia about its COM
pend1.SetInertiaXX(chrono.ChVectorD(0.1, 1.0, 1.0))
pend1.SetCollide(False)
sys.Add(pend1)

# Visualize pend1 as a cylinder of length=2, radius=0.05
cyl1 = chrono.ChVisualShapeCylinder(0.05, 2.0)
cyl1.SetColor(chrono.ChColor(0.6, 0.0, 0.0))
# Shift the cylinder by +1.0 along local X (half its length)
# and rotate so its axis is along X
frame1 = chrono.ChFrameD(chrono.ChVectorD(1.0, 0, 0), rotZtoX)
pend1.AddVisualShape(cyl1, frame1)

# Place pend1 COM initially at (1,0,1)
pend1.SetPos(chrono.ChVectorD(1.0, 0, 1.0))

# Revolute joint ground <-> pend1 at world point (0,0,1), identity rotation
rev1 = chrono.ChLinkLockRevolute()
rev1.Initialize(
    ground,
    pend1,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0),
                       chrono.ChQuaternionD(1, 0, 0, 0))
)
sys.AddLink(rev1)

# 4) SECOND pendulum
pend2 = chrono.ChBody()
pend2.SetMass(0.5)
pend2.SetInertiaXX(chrono.ChVectorD(0.05, 0.5, 0.5))
pend2.SetCollide(False)
sys.Add(pend2)

# Visualize pend2 also as a cylinder length=2, radius=0.05
cyl2 = chrono.ChVisualShapeCylinder(0.05, 2.0)
cyl2.SetColor(chrono.ChColor(0.0, 0.6, 0.0))
# Shift its COM by +1.0 along its local X, and rotate Z->X
frame2 = chrono.ChFrameD(chrono.ChVectorD(1.0, 0, 0), rotZtoX)
pend2.AddVisualShape(cyl2, frame2)

# The first pendulum's end (joint to second) sits at world (2,0,1).
# So place pend2 COM one length further along +X => at (3,0,1).
pend2.SetPos(chrono.ChVectorD(3.0, 0, 1.0))

# Revolute joint pend1 <-> pend2 at world point (2,0,1)
rev2 = chrono.ChLinkLockRevolute()
rev2.Initialize(
    pend1,
    pend2,
    chrono.ChCoordsysD(chrono.ChVectorD(2.0, 0, 1.0),
                       chrono.ChQuaternionD(1, 0, 0, 0))
)
sys.AddLink(rev2)

# 5) Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()

# 6) Run the simulation
log_info = True
time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    # After one second, print out the two COM positions once
    if log_info and sys.GetChTime() > 1.0:
        p1 = pend1.GetPos()
        p2 = pend2.GetPos()
        print(f"t = {sys.GetChTime():.3f} s")
        print(f"  pend1 COM = ({p1.x:.3f}, {p1.y:.3f}, {p1.z:.3f})")
        print(f"  pend2 COM = ({p2.x:.3f}, {p2.y:.3f}, {p2.z:.3f})")
        log_info = False