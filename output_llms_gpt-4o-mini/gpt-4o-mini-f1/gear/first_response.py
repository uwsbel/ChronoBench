import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.particle as particle
import pychrono.gear as gear

# Create the physical system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the truss and add it to the system
truss = chrono.ChBodyEasyBox(10, 10, 10, 1000, True, True)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
truss.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(truss)

# Create the rotating bar and add it to the system
bar = chrono.ChBodyEasyBox(0.5, 10, 0.5, 1000, True, True)
bar.SetPos(chrono.ChVector3d(0, 0, 0))
bar.SetRot(chrono.Q_from_AngZ(chrono.CH_PI_2))
bar.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(bar)

# Create the first gear and add it to the system
gear1 = gear.ChGearBody()
gear1.SetBody(bar)
gear1.SetPos(chrono.ChVector3d(0, 5, 0))
gear1.SetRot(chrono.Q_from_AngZ(chrono.CH_PI_2))
gear1.SetGearRatio(1)
gear1.SetPitchDiameter(2)
gear1.SetToothWidth(1)
gear1.SetToothAngle(chrono.CH_PI / 6)
gear1.SetMass(1)
gear1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/gear.png"))
system.Add(gear1)

# Create the second gear and add it to the system
gear2 = gear.ChGearBody()
gear2.SetPos(chrono.ChVector3d(0, 0, 0))
gear2.SetRot(chrono.Q_from_AngZ(chrono.CH_PI_2))
gear2.SetGearRatio(1)
gear2.SetPitchDiameter(2)
gear2.SetToothWidth(1)
gear2.SetToothAngle(chrono.CH_PI / 6)
gear2.SetMass(1)
gear2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/gear.png"))
system.Add(gear2)

# Create the gear motor and add it to the system
gear_motor = gear.ChGearMotor()
gear_motor.SetBodyA(gear1)
gear_motor.SetBodyB(gear2)
gear_motor.SetSpeed(chrono.CH_PI / 2)
gear_motor.SetMass(1)
gear_motor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/gear.png"))
system.Add(gear_motor)

# Create the Irrlicht visualization system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(chrono.ChVector2i(1280, 720))
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, -15))
vis.AddTypicalLights()

# Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)