import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono system
sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath("chrono_data/")

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Create ground body
ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetFixed(True)
ground.EnableCollision(True)

# Add visual shape to ground
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(10, 1, 10))
ground_shape.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
ground.AddVisualShape(ground_shape)
sys.Add(ground)

# Create pendulum body
pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVector3d(0, 1, 0))
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))

# Add visual shape to pendulum
pendulum_shape = chrono.ChVisualShapeSphere(0.1)
pendulum_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.3))
pendulum.AddVisualShape(pendulum_shape)

# Create revolute joint between ground and pendulum
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(math.pi / 2)))

# Add bodies and joint to the system
sys.Add(pendulum)
sys.Add(rev_joint)

# Set up contact reporting for logging
class LogCallback(chrono.ReportContactCallback):
    def OnReportContact(self, pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB):
        print(f"Pendulum position: {pendulum.GetPos()}")
        print(f"Pendulum velocity: {pendulum.GetPos_dt()}")

sys.GetContactContainer().RegisterAddContactCallback(LogCallback())

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(0.01)
    time.sleep(0.01)