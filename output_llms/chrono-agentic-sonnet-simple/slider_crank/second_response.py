import os
import pychrono.core as chrono  # core Chrono library
import pychrono.irrlicht as chronoirr  # Irrlicht visualization

sys = chrono.ChSystemNSC()  # NSC system, no collision needed for pure jointed MBS

# Shared parameters
crank_center = chrono.ChVector3d(-1, 0.5, 0)  # crankshaft center
crank_rad = 0.4   # crank radius [m]
crank_thick = 0.1  # crank thickness [m]
rod_length = 1.5   # connecting rod length [m]

# Floor truss (fixed reference body)
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)  # 3x1x3 m box, density 1000 kg/m^3
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))   # centered at y=-0.5
mfloor.SetFixed(True)
sys.Add(mfloor)

# Crank flywheel (cylinder along Z after rotation)
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # vertical cyl
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  # offset z so disc faces front
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # rotate to lie in XY plane (spin about Z)
sys.Add(mcrank)

# Connecting rod (box)
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # 1.5x0.1x0.1 m
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  # center of rod
sys.Add(mrod)

# Piston (cylinder along X after rotation)
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  # radius 0.2, height 0.3
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  # at rod end
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # rotate to align along X (slide direction)
sys.Add(mpiston)

# Motor: spins crank at π rad/s (a full motor-link, no extra revolute needed)
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))  # motor at crank center
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # π rad/s = 180°/s
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Crank-rod revolute joint (crank pin)
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Rod-piston revolute joint (wrist pin)
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Piston-floor prismatic joint (slider along X-axis)
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor,
                   chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
                                   chrono.Q_ROTATE_Z_TO_X))  # local +Z aligned to world +X
sys.Add(mjointC)

# Irrlicht visualization (Initialize first, then add scene elements)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()  # FIRST — then add scene elements
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))  # eye, target
vis.AddTypicalLights()

# Arrays for storing values to plot (crank angle, piston position, piston speed)
array_time = []
array_angle = []
array_pos = []
array_speed = []

time_step = 1e-3   # physics time step [s]
sim_end = 20.0     # stop simulation after 20 seconds
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # frames per render


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        t = sys.GetChTime()
        angle = my_motor.GetMotorAngle()   # crank rotation angle [rad]
        pos = mpiston.GetPos().x           # piston position along X [m]
        speed = mpiston.GetPosDt().x       # piston speed along X [m/s]
        array_time.append(t)
        array_angle.append(angle)
        array_pos.append(pos)
        array_speed.append(speed)
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
