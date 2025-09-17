# Import necessary modules
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine

# Create the simulation system and set the collision margin
system = chrono.ChSystemNSC()
system.SetCollisionMargin(0.001)

# Create the Irrlicht visualization system
visualizer = chronoirr.ChVisualSystemIrrlicht(system)
visualizer.SetWindowSize(800, 600)
visualizer.SetWindowTitle("Epicyclic Gears Simulation")
visualizer.SetCameraDistance(2.5)
visualizer.SetCameraAzimuth(90)
visualizer.SetCameraElevation(30)

# Create the fixed truss
truss = chrono.ChBodyEasyBox(system, 1, 0.1, 0.1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
system.Add(truss)

# Create the rotating bar
bar = chrono.ChBodyEasyBox(system, 2, 0.1, 0.1, 1000, True, True)
bar.SetPos(chrono.ChVectorD(1, 0, 0))
bar.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))
system.Add(bar)

# Create the gear motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, system.GetGroundBody())
motor.SetSpeed(chrono.CH_C_PI)
system.Add(motor)

# Create the gears
gear1 = chrono.ChBodyEasyCylinder(system, 0.5, 0.1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(2, 0, 0))
gear1.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(system, 0.5, 0.1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(3, 0, 0))
gear2.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))
system.Add(gear2)

# Add gear-gear contact
contact = chrono.ChContactSurfaceNSC()
contact.SetMaterial(chrono.ChMaterialSurfaceNSC())
contact.SetCompliance(0.01)
contact.SetFriction(0.5)
system.AddContactMaterial(contact)
system.AddContact(gear1, gear2, contact)

# Add gear-bar contact
contact = chrono.ChContactSurfaceNSC()
contact.SetMaterial(chrono.ChMaterialSurfaceNSC())
contact.SetCompliance(0.01)
contact.SetFriction(0.5)
system.AddContactMaterial(contact)
system.AddContact(gear1, bar, contact)
system.AddContact(gear2, bar, contact)

# Set visualization materials and lighting
visualizer.AddSkyBox()
visualizer.AddTypicalLights()
visualizer.AddCameraVerticalAll()
visualizer.SetShowAxes(True)
visualizer.SetShowContactForces(True)
visualizer.SetShowContactPoints(True)

# Run the simulation
visualizer.Run()
system.SetTimestep(0.01)
while visualizer.Run():
    system.DoStepDynamics(0.01)