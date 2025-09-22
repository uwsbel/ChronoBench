import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Create the Chrono simulation system
sys = chrono.ChSystemNSC()

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)

# Create a truss (fixed body)
truss = chrono.ChBody()
sys.Add(truss)
truss.SetFixed(True)
truss.EnableCollision(False)

# Add a visual representation of the truss
truss.AddVisualShape(chrono.ChVisualShapeSphere(0.05))

# Create a rotating bar (body A)
barA = chrono.ChBody()
sys.Add(barA)
barA.SetFixed(False)
barA.SetPos(chrono.ChVector3d(0, 0, 0))
barA.EnableCollision(False)

# Add a visual representation of the rotating bar
barA.AddVisualShape(chrono.ChVisualShapeCylinder(0.05, 0.2))

# Create a gear (body B)
gearB = chrono.ChBody()
sys.Add(gearB)
gearB.SetFixed(False)
gearB.SetPos(chrono.ChVector3d(0.2, 0, 0))
gearB.EnableCollision(False)

# Add a visual representation of the gear
gearB.AddVisualShape(chrono.ChVisualShapeCylinder(0.05, 0.1))

# Create a second gear (body C)
gearC = chrono.ChBody()
sys.Add(gearC)
gearC.SetFixed(False)
gearC.SetPos(chrono.ChVector3d(0.1, 0, 0))
gearC.EnableCollision(False)

# Add a visual representation of the second gear
gearC.AddVisualShape(chrono.ChVisualShapeCylinder(0.05, 0.15))

# Create a revolute joint between the truss and bar A
revA = chrono.ChLinkLockRevolute()
sys.Add(revA)
revA.Initialize(truss, barA, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0)))

# Create a revolute joint between bar A and gear B
revB = chrono.ChLinkLockRevolute()
sys.Add(revB)
revB.Initialize(barA, gearB, chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0), chrono.ChVector3d(0, 1, 0)))

# Create a revolute joint between gear B and gear C
revC = chrono.ChLinkLockRevolute()
sys.Add(revC)
revC.Initialize(gearB, gearC, chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0), chrono.ChVector3d(0, 1, 0)))

# Create a gear constraint between gear B and gear C
gearBC = chrono.ChLinkGear()
sys.Add(gearBC)
gearBC.Initialize(gearB, gearC, chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0), chrono.ChVector3d(0, 1, 0)))
gearBC.SetName('GBC')
gearBC.SetTimestep(1e-3)
gearBC.SetGearRatio(2)

# Create a gear constraint between gear A and gear B
gearAB = chrono.ChLinkGear()
sys.Add(gearAB)
gearAB.Initialize(gearA, gearB, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0)))
gearAB.SetName('GAB')
gearAB.SetTimestep(1e-3)
gearAB.SetGearRatio(1)

# Create a motor to enforce rotation between gear A and gear B
motorA = chrono.ChLinkMotorRotationSpeed()
sys.Add(motorA)
motorA.Initialize(gearA, gearB, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0)))
motorA.SetSpeed(chrono.CH_PI_2)  # Set the rotation speed to π/2 rad/s

# Set the simulation time step
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# Run the simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)  # Advance the simulation by one time step of 1 ms