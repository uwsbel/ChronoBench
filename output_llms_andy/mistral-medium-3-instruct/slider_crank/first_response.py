import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# 1. Initialize PyChrono environment
chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  # Update this path

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Create the floor (truss)
floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
floor.GetVisualModel().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(floor)

# 3. Create the crankshaft
crank = chrono.ChBodyEasyCylinder(0.2, 1, 10, True, True)
crank.SetPos(chrono.ChVectorD(0, 2, 0))
crank.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  # Align along x-axis
system.Add(crank)

# 4. Create the connecting rod
rod_length = 3.0
rod = chrono.ChBodyEasyBox(0.1, 0.1, rod_length, 5, True, True)
rod.SetPos(chrono.ChVectorD(0, 2 + rod_length/2, 0))
system.Add(rod)

# 5. Create the piston
piston = chrono.ChBodyEasyBox(0.5, 0.2, 0.5, 5, True, True)
piston.SetPos(chrono.ChVectorD(0, 2 + rod_length, 0))
system.Add(piston)

# 6. Create joints
# Revolute joint between ground and crank (at origin)
joint_crank_ground = chrono.ChLinkLockRevolute()
joint_crank_ground.Initialize(floor, crank, chrono.ChCoordinatesysD(chrono.ChVectorD(0, 2, 0)))
system.Add(joint_crank_ground)

# Revolute joint between crank and rod
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(crank, rod, chrono.ChCoordinatesysD(chrono.ChVectorD(1, 2, 0)))  # 1m from origin
system.Add(joint_crank_rod)

# Revolute joint between rod and piston
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(rod, piston, chrono.ChCoordinatesysD(chrono.ChVectorD(0, 2 + rod_length, 0)))
system.Add(joint_rod_piston)

# Prismatic joint between piston and ground (constrain to move only vertically)
joint_piston_ground = chrono.ChLinkLockPrismatic()
joint_piston_ground.Initialize(floor, piston, chrono.ChCoordinatesysD(chrono.ChVectorD(0, 2 + rod_length, 0)))
joint_piston_ground.SetLimitSpeed(0, 0)  # Lock rotation
system.Add(joint_piston_ground)

# 7. Add motor to crankshaft
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0, 2, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # 1 rad/s constant speed
system.Add(motor)

# 8. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 2, 0))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 10, chrono.ChColor(1, 1, 1))

# 9. Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by 0.01 seconds
    system.DoStepDynamics(0.01)

    # Print piston position
    print("Piston position:", piston.GetPos().y)