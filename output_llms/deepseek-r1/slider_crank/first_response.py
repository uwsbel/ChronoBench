import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Create the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  # Gravity

# Create ground body (truss)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.AddBody(ground)

# Create crankshaft (rotating part)
crank = chrono.ChBodyEasyCylinder(0.02, 0.15, 7800)  # Radius, height, density
crank.SetPos(chrono.ChVectorD(0, 0, 0))
crank.SetRot(chrono.Q_from_AngY(chrono.CH_C_PI_2))  # Orient horizontally
system.AddBody(crank)

# Create connecting rod
rod = chrono.ChBodyEasyBox(0.5, 0.015, 0.015, 7800)  # Length, width, height
rod.SetPos(chrono.ChVectorD(0.25, 0, 0))
system.AddBody(rod)

# Create piston (slider)
piston = chrono.ChBodyEasyCylinder(0.04, 0.06, 7800)
piston.SetPos(chrono.ChVectorD(0.7, 0, 0))
piston.SetRot(chrono.Q_from_AngY(chrono.CH_C_PI_2))  # Orient horizontally
system.AddBody(piston)

# Create joints
# 1. Revolute joint between ground and crankshaft (at origin)
revolute_ground_crank = chrono.ChLinkLockRevolute()
revolute_ground_crank.Initialize(ground, crank, 
                                chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(revolute_ground_crank)

# 2. Revolute joint between crankshaft and connecting rod
revolute_crank_rod = chrono.ChLinkLockRevolute()
revolute_crank_rod.Initialize(crank, rod, 
                             chrono.ChCoordsysD(chrono.ChVectorD(0.15, 0, 0)))
system.AddLink(revolute_crank_rod)

# 3. Revolute joint between connecting rod and piston
revolute_rod_piston = chrono.ChLinkLockRevolute()
revolute_rod_piston.Initialize(rod, piston, 
                              chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0)))
system.AddLink(revolute_rod_piston)

# 4. Prismatic joint between ground and piston (constrain to X-axis)
prismatic_ground_piston = chrono.ChLinkLockPrismatic()
prismatic_ground_piston.Initialize(ground, piston, 
                                  chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)),
                                  chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
prismatic_ground_piston.SetMotionAxis(chrono.ChVectorD(1, 0, 0))  # X-axis motion
system.AddLink(prismatic_ground_piston)

# Add motor to crankshaft
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  # 180°/s (π rad/s)
system.AddLink(motor)

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, -1, 0.2), chrono.ChVectorD(0.5, 0, 0))
vis.AddTypicalLights()

# Add visual assets
texture = chrono.ChTexture(chrono.GetChronoDataFile('textures/bluewhite.png'))
ground.AddVisualShape(chrono.ChBoxShape(0.2, 0.2, 0.1)).SetTexture(texture)
crank.GetVisualShape(0).SetTexture(texture)
rod.GetVisualShape(0).SetTexture(texture)
piston.GetVisualShape(0).SetTexture(texture)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)