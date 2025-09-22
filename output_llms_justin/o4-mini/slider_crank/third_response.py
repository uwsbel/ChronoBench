import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# (uncomment and set your own path if you need Chrono data)
# chrono.SetChronoDataPath('path/to/data')

# ----------------------------------------------------------------------
# 1) System and basic geometry
# ----------------------------------------------------------------------
sys = chrono.ChSystemNSC()

crank_center = chrono.ChVectorD(-1, 0.5, 0)
crank_rad    = 0.4
crank_thick  = 0.1
rod_length   = 1.5

# Floor
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVectorD(0, -0.5, 0))
mfloor.SetBodyFixed(True)
sys.Add(mfloor)

# Crank (a short cylinder around the Y‐axis, rotated into the X–Y plane)
mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -crank_thick/2))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Rod (a long slender box)
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length/2, 0, 0))
sys.Add(mrod)

# Piston (cylinder that must slide/rotate in the plane)
mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# ----------------------------------------------------------------------
# 2) Joints
# ----------------------------------------------------------------------

# 2a) Motor at the crank‐floor interface (revolute + preset speed)
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(
    mcrank, 
    mfloor,
    chrono.ChCoordsysD(crank_center, chrono.QUNIT)
)
# constant angular speed = pi rad/s
my_motor.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_PI))
sys.Add(my_motor)

# 2b) Crank‐rod: spherical joint instead of revolute
jointA = chrono.ChLinkSpherical()
jointA.Initialize(
    mcrank, 
    mrod,
    chrono.ChCoordsysD(
        crank_center + chrono.ChVectorD(crank_rad, 0, 0),
        chrono.QUNIT
    )
)
sys.Add(jointA)

# 2c) Rod‐piston: spherical joint instead of revolute
jointB = chrono.ChLinkSpherical()
jointB.Initialize(
    mrod,
    mpiston,
    chrono.ChCoordsysD(
        crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0),
        chrono.QUNIT
    )
)
sys.Add(jointB)

# 2d) Piston‐floor: planar joint (allows translation in x,y and rotation about z)
jointC = chrono.ChLinkLockPlanar()
jointC.Initialize(
    mpiston,
    mfloor,
    chrono.ChCoordsysD(
        crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0),
        chrono.QUNIT
    )
)
sys.Add(jointC)

# ----------------------------------------------------------------------
# 3) Visualization setup
# ----------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank–Rod–Piston (with spherical & planar joints)')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1,1,3), chrono.ChVectorD(0,1,0))
vis.AddTypicalLights()

# ----------------------------------------------------------------------
# 4) Data collection arrays
# ----------------------------------------------------------------------
array_time  = []
array_angle = []
array_pos   = []
array_speed = []

# ----------------------------------------------------------------------
# 5) Simulation loop
# ----------------------------------------------------------------------
step_size = 1e-3
while vis.Run():
    t = sys.GetChTime()
    # record data
    array_time.append(t)
    # use GetMotorRot() to get current angle of the speed‐controlled motor
    array_angle.append(my_motor.GetMotorRot())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPos_dt().x)

    # render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # advance
    sys.DoStepDynamics(step_size)

    # stop after 2 seconds
    if t > 2.0:
        vis.GetDevice().closeDevice()

# ----------------------------------------------------------------------
# 6) Plot results: piston position & speed vs. crank angle
# ----------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(array_angle, array_pos)
ax1.set(ylabel='position [m]')
ax1.grid(True)

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='speed [m/s]', xlabel='angle [rad]')
ax2.grid(True)

# nicer π ticks
plt.xticks(
    np.linspace(0, 2*np.pi, 5),
    ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$']
)
plt.show()