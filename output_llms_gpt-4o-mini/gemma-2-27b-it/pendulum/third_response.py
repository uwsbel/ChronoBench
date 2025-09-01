import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True) 
ground.EnableCollision(False) 

# Add a visualization cylinder to the ground
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4) 
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

# Create the first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False) 
pend_1.EnableCollision(False) 
pend_1.SetMass(1) 
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1)) 

# Add a visualization cylinder to the first pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2) 
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0)) 
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the first pendulum
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Create the second pendulum body and add it to the simulation
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False) 
pend_2.EnableCollision(False) 
pend_2.SetMass(1) 
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1)) 

# Add a visualization cylinder to the second pendulum
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2) 
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6)) 
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the second pendulum
pend_2.SetPos(chrono.ChVector3d(3, 0, 1))

# Create a revolute joint to connect the first pendulum to the ground
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(1, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)

# Create a revolute joint to connect the second pendulum to the first
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(2, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys) 
vis.SetWindowSize(1024, 768) 
vis.SetWindowTitle('Double Pendulum demo') 
vis.Initialize() 
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 
vis.AddSkyBox() 
vis.AddCamera(chrono.ChVector3d(0, 3, 6)) 
vis.AddTypicalLights() 

# Simulation loop
log_info = True 
while vis.Run():
    vis.BeginScene() 
    vis.Render() 
    vis.EndScene()
    sys.DoStepDynamics(1e-3) 

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos() 
        print("t = ", sys.GetChTime())
        print("     Pendulum 1: ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt() 
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)

        pos_2 = pend_2.GetPos() 
        print("     Pendulum 2: ", pos_2.x, "  ", pos_2.y)
        lin_vel_2 = pend_2.GetPosDt() 
        print("     ", lin_vel_2.x, "  ", lin_vel_2.y)

        log_info = False