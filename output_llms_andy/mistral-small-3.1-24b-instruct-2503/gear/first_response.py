import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as engine

# Initialize the Chrono::Engine system
chrono.SetChronoDataPath('../data/')

# Create a Chrono::Engine system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create the Irrlicht visualization system
visualization = chronoirr.ChIrrApp(system, 'Epicyclic Gears', chronoirr.dimension2du(1280, 720))

# Add a camera to visualize the scene
camera = visualization.getIrrlichtCamera()
camera.setPosition(chrono.ChVectorD(0, -1, 0.5))
camera.setTarget(chrono.ChVectorD(0, 0, 0))

# Create a fixed truss
truss = chrono.ChBodyEasyBox(1, 1, 0.1, 1000, False, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0.05))
truss.SetBodyFixed(True)
system.Add(truss)

# Create a rotating bar
bar = chrono.ChBodyEasyCylinder(0.05, 0.5, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 0.3))
bar.SetRot(chrono.Q_from_AngAxis(chrono.ChMatrix33D(1, 0, 0, 0, 1, 0, 0, 0, 1)))
system.Add(bar)

# Create two gears
gear1 = chrono.ChBodyEasyCylinder(0.1, 0.1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0.2, 0, 0.3))
gear1.SetRot(chrono.Q_from_AngAxis(chrono.ChMatrix33D(1, 0, 0, 0, 1, 0, 0, 0, 1)))
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.1, 0.1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(-0.2, 0, 0.3))
gear2.SetRot(chrono.Q_from_AngAxis(chrono.ChMatrix33D(1, 0, 0, 0, 1, 0, 0, 0, 1)))
system.Add(gear2)

# Create a gear motor to enforce a constant rotation speed
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, gear1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI / 2))  # 90 degrees per second
system.Add(motor)

# Set materials and lighting
truss.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
bar.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/steel.jpg'))
gear1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/steel.jpg'))
gear2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/steel.jpg'))

visualization.AddTypicalSky()
visualization.AddTypicalLights()
visualization.AddLogo(chrono.GetChronoDataFile('logo_pychrono_small.png'), 10, 10)

# Run the simulation
visualization.SetTimestep(0.01)
visualization.SetTryRealTime(True)

while visualization.Run():
    system.DoStepDynamics(0.01)