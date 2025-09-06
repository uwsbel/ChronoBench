import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math

# --- Simulation Setup ---
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chrono.SetChronoDataPath("../chrono/data/")  # Adjust path if needed

# --- Visualization Setup ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 2, -5))
vis.AddTypicalLights()

# --- Physical Constants ---
gear_radius = 0.2
gear_width = 0.1
motor_speed = 1  # rad/s
fixed_gear_radius = 0.4

# --- Materials ---
material_steel = chrono.ChContactMaterialNSC()
material_steel.SetFriction(0.3)
material_steel.SetDampingF(0.1)

# --- Fixed Truss ---
truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetMass(10)
truss.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
truss_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.1, 0.1, 0.5))
truss_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
truss.AddVisualShape(truss_shape)
sys.Add(truss)

# --- Rotating Bar ---
bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
bar.SetPos(chrono.ChVector3d(0, 0, 0))
bar_shape = chrono.ChVisualShapeCylinder(0.05, 0.5)
bar_shape.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
bar.AddVisualShape(bar_shape)
sys.Add(bar)

# --- Gear 1 (Motor-Driven) ---
gear1 = chrono.ChBody()
gear1.SetMass(1)
gear1.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
gear1.SetPos(chrono.ChVector3d(0.5, 0, 0))
gear1_shape = chrono.ChVisualShapeCylinder(gear_radius, gear_width)
gear1_shape.SetColor(chrono.ChColor(0, 0, 1))
gear1.AddVisualShape(gear1_shape)
sys.Add(gear1)

# --- Gear 2 (Driven Gear) ---
gear2 = chrono.ChBody()
gear2.SetMass(1)
gear2.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
gear2.SetPos(chrono.ChVector3d(1, 0, 0))
gear2_shape = chrono.ChVisualShapeCylinder(gear_radius, gear_width)
gear2_shape.SetColor(chrono.ChColor(1, 0, 0))
gear2.AddVisualShape(gear2_shape)
sys.Add(gear2)

# --- Motor ---
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, truss, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(motor_speed))
motor.SetTorqueFunction(chrono.ChFunction_Const(10))
sys.Add(motor)

# --- Gear Connection (Revolute Joint) ---
joint1 = chrono.ChLinkLockRevolute()
joint1.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVector3d(gear_radius, 0, 0), chrono.QUNIT))
sys.Add(joint1)

# --- Fixed Gear Connection ---
fixed_gear = chrono.ChBody()
fixed_gear.SetBodyFixed(True)
fixed_gear.SetPos(chrono.ChVector3d(0,0,0))
fixed_gear.SetMass(1)
fixed_gear.SetInertiaXX(chrono.ChVector3d(0.1,0.1,0.1))
fixed_gear_shape = chrono.ChVisualShapeCylinder(fixed_gear_radius, gear_width)
fixed_gear_shape.SetColor(chrono.ChColor(0,1,0))
fixed_gear.AddVisualShape(fixed_gear_shape)
sys.Add(fixed_gear)

joint2 = chrono.ChLinkLockRevolute()
joint2.Initialize(bar, fixed_gear, chrono.ChCoordsysD(chrono.ChVector3d(0,0,0), chrono.QUNIT))
sys.Add(joint2)

# --- Simulation Loop ---
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)