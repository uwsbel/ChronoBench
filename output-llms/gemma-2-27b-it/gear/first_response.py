import pychrono as chrono
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# 2. Add the required physical systems and objects

# Create the truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(truss)

# Create the rotating bar
bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
bar.SetPos(chrono.ChVectorD(0, 1, 0))
my_system.Add(bar)

# Create the gears
gear1 = chrono.ChBodyEasyCylinder(0.5, 0.1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(1, 1, 0))
my_system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.3, 0.1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(2, 1, 0))
my_system.Add(gear2)

# Create a gear motor to enforce constant rotation speed
motor = chrono.ChLinkMotorRotationSpeed()
motor.SetBody1(bar)
motor.SetBody2(gear1)
motor.SetTargetSpeed(10)
my_system.Add(motor)

# 3. Set necessary default parameters

# Define contact materials
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.2)

# Create contact pairs
truss_bar_contact = chrono.ChContactSurfaceNodeNSC()
truss_bar_contact.SetMaterialSurface(material)
truss_bar_contact.AddBody(truss)
truss_bar_contact.AddBody(bar)
my_system.AddContactSurface(truss_bar_contact)

gear1_bar_contact = chrono.ChContactSurfaceNodeNSC()
gear1_bar_contact.SetMaterialSurface(material)
gear1_bar_contact.AddBody(gear1)
gear1_bar_contact.AddBody(bar)
my_system.AddContactSurface(gear1_bar_contact)

gear1_gear2_contact = chrono.ChContactSurfaceNodeNSC()
gear1_gear2_contact.SetMaterialSurface(material)
gear1_gear2_contact.AddBody(gear1)
gear1_gear2_contact.AddBody(gear2)
my_system.AddContactSurface(gear1_gear2_contact)

# 4. Visualization setup

# Create the Irrlicht visualization system
vis = chronoirr.ChIrrApp(my_system, "Epicyclic Gears", chrono.ChVectorD(0, 0, 10))
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.SetWindowSize(800, 600)
vis.SetStep(0.01)

# Set material properties for visualization
truss.AddAsset(chronoirr.ChColorAsset(1, 0, 0))
bar.AddAsset(chronoirr.ChColorAsset(0, 1, 0))
gear1.AddAsset(chronoirr.ChColorAsset(0, 0, 1))
gear2.AddAsset(chronoirr.ChColorAsset(1, 1, 0))

# Run the simulation
vis.Run()