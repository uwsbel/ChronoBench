import pychrono as chrono
import pychrono.irrlicht as irrlicht

# Parameters
sun_radius = 1.0
planet_radius = 0.5
bar_length = sun_radius + planet_radius  # 1.5 meters

# System setup
system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Create fixed truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetCollide(False)
system.Add(truss)

# Create sun gear (fixed to truss)
sun_gear = chrono.ChBody()
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_gear.SetBodyFixed(True)
sun_gear.SetCollide(False)
sun_shape = chrono.ChCylinderShape()
sun_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.2)
sun_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.2)
sun_shape.GetCylinderGeometry().rad = sun_radius
sun_gear.AddAsset(sun_shape)
sun_gear.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.6, 0.2, 0.2)))
system.Add(sun_gear)

# Create rotating bar
bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetMass(1.0)
bar.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
bar.SetCollide(False)
bar_shape = chrono.ChBoxShape()
bar_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.1, bar_length, 0.1))
bar.AddAsset(bar_shape)
bar.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.2, 0.5, 0.8)))
system.Add(bar)

# Motor to rotate bar at constant speed
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, bar, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  # PI rad/s rotation
system.Add(motor)

# Create planet gear
planet_gear = chrono.ChBody()
planet_gear.SetPos(chrono.ChVectorD(0, bar_length, 0))
planet_gear.SetMass(0.5)
planet_gear.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
planet_gear.SetCollide(False)
planet_shape = chrono.ChCylinderShape()
planet_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.2)
planet_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.2)
planet_shape.GetCylinderGeometry().rad = planet_radius
planet_gear.AddAsset(planet_shape)
planet_gear.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.2, 0.8, 0.2)))
system.Add(planet_gear)

# Revolute joint between bar and planet gear
planet_joint = chrono.ChLinkRevolute()
planet_joint.Initialize(bar, planet_gear, 
                        chrono.ChCoordsysD(chrono.ChVectorD(0, bar_length, 0), 
                                           chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))
system.Add(planet_joint)

# Gear constraint between motor and planet joint
gear = chrono.ChLinkGear()
gear.Initialize(motor, planet_joint)
gear.SetTau(-sun_radius / planet_radius)  # Negative ratio for opposite rotation
system.Add(gear)

# Visualization setup
vis = irrlicht.ChIrrApp(system, "Epicyclic Gears", 800, 600)
vis.AddTypicalLights(irrlicht.vector3df(30, 50, 30), irrlicht.vector3df(30, -50, 30), 200, 200)
vis.AddTypicalCamera(chrono.ChVectorD(3, 3, 3))
vis.AssetBindAll()
vis.AssetUpdateAll()

# Simulation loop
vis.Initialize()
vis.Run()

timestep = 0.005
while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.DoStepDynamics(timestep)
    vis.EndScene()