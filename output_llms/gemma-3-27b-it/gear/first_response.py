import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess

# ---
#  Create the simulation system and add items
# ---

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# ---
# Create the fixed truss
# ---
body_truss = chrono.ChBodyEasy()
body_truss.SetBodyFixed(True)
body_truss.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(body_truss)

# Create a box shape for the truss
truss_shape = chrono.ChBoxShape()
truss_shape.GetBoxGeometry().SetMinMax(chrono.ChVectorD(-0.5, -0.5, -0.5), chrono.ChVectorD(0.5, 0.5, 0.5))
body_truss.AddAsset(truss_shape)
body_truss.SetCollide(True)
body_truss.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())

# ---
# Create the rotating bar
# ---
body_bar = chrono.ChBodyEasy()
body_bar.SetPos(chrono.ChVectorD(1, 0, 0))
body_bar.SetBodyFixed(False)
system.Add(body_bar)

# Create a cylinder shape for the bar
bar_shape = chrono.ChCylinderShape()
bar_shape.GetCylinderGeometry().SetRadius(0.1)
bar_shape.GetCylinderGeometry().SetHeight(1)
body_bar.AddAsset(bar_shape)
body_bar.SetCollide(True)
body_bar.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())

# ---
# Create the gears
# ---

# Gear 1 (driven by motor)
body_gear1 = chrono.ChBodyEasy()
body_gear1.SetPos(chrono.ChVectorD(2, 0, 0))
system.Add(body_gear1)

gear1_shape = chrono.ChGearShape()
gear1_shape.GetGearGeometry().SetRadius(0.3)
gear1_shape.GetGearGeometry().SetHeight(0.2)
gear1_shape.GetGearGeometry().SetNumTeeth(20)
body_gear1.AddAsset(gear1_shape)
body_gear1.SetCollide(True)
body_gear1.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())

# Gear 2 (driven gear)
body_gear2 = chrono.ChBodyEasy()
body_gear2.SetPos(chrono.ChVectorD(3, 0, 0))
system.Add(body_gear2)

gear2_shape = chrono.ChGearShape()
gear2_shape.GetGearGeometry().SetRadius(0.3)
gear2_shape.GetGearGeometry().SetHeight(0.2)
gear2_shape.GetGearGeometry().SetNumTeeth(40)
body_gear2.AddAsset(gear2_shape)
body_gear2.SetCollide(True)
body_gear2.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())

# ---
# Create a gear pair constraint
# ---
gear_pair = chrono.ChLinkGear()
gear_pair.Initialize(body_gear1, body_gear2, chrono.ChCoordsys(chrono.ChVectorD(0,0,0)))
system.Add(gear_pair)

# ---
# Create a motor to drive gear 1
# ---
motor = chrono.ChMotorLinearSpring()
motor.Set_Rest_Spring_K(1000)
motor.Set_Rest_Spring_R(10)
motor.Set_Motor_Speed(1) #rad/s
motor.Set_Motor_Torque(1)
motor.Initialize(body_gear1, body_truss, chrono.ChCoordsys(chrono.ChVectorD(0,0,0)))
system.Add(motor)

# ---
# Visualization
# ---

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddTypicalCamera(chrono.ChVectorD(2, 1.5, -2))
vis.AddTypicalLights()

# ---
# Simulation loop
# ---
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)