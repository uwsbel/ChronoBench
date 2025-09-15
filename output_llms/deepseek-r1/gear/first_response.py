import pychrono as chrono
import pychrono.irrlicht as irr

# Create the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable gravity

# 1. Initialize core components
# -----------------------------

# Create a fixed truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
system.Add(truss)

# 2. Create physical systems
# --------------------------

# Parameters
sun_radius = 0.5
planet_radius = 0.3
bar_length = 1.5
gear_height = 0.2

# Create rotating bar (carrier)
bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(bar)

# Connect bar to truss with revolute joint
revolute_bar = chrono.ChLinkRevolute()
revolute_bar.Initialize(truss, bar, 
                       chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                         chrono.Q_from_AngY(chrono.CH_C_PI_2)))
system.Add(revolute_bar)

# Create sun gear (fixed to truss)
sun_gear = chrono.ChBody()
sun_gear.SetBodyFixed(True)
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_shape = chrono.ChCylinderShape(sun_radius, gear_height)
sun_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.0))
sun_gear.AddAsset(sun_shape)
system.Add(sun_gear)

# Create planet gear
planet_gear = chrono.ChBody()
planet_gear.SetPos(chrono.ChVectorD(bar_length, 0, 0))
planet_shape = chrono.ChCylinderShape(planet_radius, gear_height)
planet_shape.SetColor(chrono.ChColor(0.2, 0.8, 0.0))
planet_gear.AddAsset(planet_shape)
system.Add(planet_gear)

# Connect planet gear to bar with revolute joint
revolute_planet = chrono.ChLinkRevolute()
revolute_planet.Initialize(bar, planet_gear,
                          chrono.ChCoordsysD(chrono.ChVectorD(bar_length, 0, 0),
                                           chrono.Q_from_AngY(chrono.CH_C_PI_2)))
system.Add(revolute_planet)

# 3. Set up gear constraints and motor
# ------------------------------------

# Create gear constraint between sun and planet
gear_ratio = -(sun_radius/planet_radius)  # External gear ratio
gear_constraint = chrono.ChLinkLockGear()
gear_constraint.Initialize(sun_gear, planet_gear,
                          chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)),
                          chrono.ChCoordsysD(chrono.ChVectorD(bar_length, 0, 0)))
gear_constraint.SetRatio(gear_ratio)
system.Add(gear_constraint)

# Create rotation motor to drive the carrier
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, bar, 
                chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI / 3))  # 60 deg/s
system.Add(motor)

# 4. Visualization setup
# ----------------------
vis = irr.ChIrrApp(system, "Epicyclic Gear System", irr.dimension2du(1024, 768))
vis.AddTypicalLights(irr.vector3df(30, 100, 30), irr.vector3df(-30, -100, -30), 500, 500)
vis.AddCamera(chrono.ChVectorD(3, 1.5, 3), chrono.ChVectorD(0, 0, 0))
vis.AssetBindAll()
vis.AssetUpdateAll()

# Simulation loop
vis.Run()

while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.DoStepDynamics(1e-3)
    vis.EndScene()