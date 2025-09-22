import pychrono.core as chrono
import pychrono.vehicle as veh  # if needed for advanced visualization or controls

# 1. Initialize the PyChrono system and visualization
system = chrono.ChSystemNSC()

# Set default collision envelope and update sizes
system.SetCollisionSystemType(chrono.ChSystem::CollisionSystemType::CHRONO_COLLISION_SYSTEM_TYPE_HYDRO)
system.GetCollisionSystem().SetDefaultSuggestedEnvelope(0.01)
system.GetCollisionSystem().SetDefaultSuggestedMargin(0.005)

# 2. Create the Irrlicht visualization application
application = chrono.ChIrrApp(system, "Epicyclic Gears Simulation", chrono.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.vector3d(0, 1.5, -3))
application.AssetBindAll()
application.AssetUpdateAll()

# 3. Define materials for visualization
material_gear = chrono.ChMaterialSurfaceSMC()
material_gear.SetFriction(0.4)
material_truss = chrono.ChMaterialSurfaceSMC()
material_truss.SetFriction(0.9)

# 4. Create a fixed truss (base)
truss_body = chrono.ChBodyEasyBox(0.2, 0.05, 0.2, 1000, True, True)
truss_body.SetPos(chrono.ChVectorD(0, 0, 0))
truss_body.SetBodyFixed(True)
truss_body.GetMaterialSurface().SetFriction(0.9)
system.Add(truss_body)

# 5. Create the rotating bar (arm)
arm_length = 0.4
arm_thickness = 0.04
arm = chrono.ChBodyEasyBox(arm_length, arm_thickness, 0.02, 1000, True, True)
arm.SetPos(chrono.ChVectorD(0, 0.1, 0))
system.Add(arm)

# Create a motor to rotate the arm at constant speed
motor_speed = chrono.VECT_X  # Define axis of rotation
rotation_speed_rad_per_sec = 2 * 3.14159 / 5  # 1 full rotation in 5 seconds

# 6. Create gears
# Gear 1 - connected to the arm via a revolute joint
radius_gear1 = 0.05
gear1 = chrono.ChBodyEasyCylinder(radius_gear1, 0.02, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0, 0.1, 0))
gear1.SetMaterialSurface(material_gear)
system.Add(gear1)

# Gear 2 - fixed on the truss
radius_gear2 = 0.05
gear2 = chrono.ChBodyEasyCylinder(radius_gear2, 0.02, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(0.2, 0.1, 0))
gear2.SetMaterialSurface(material_gear)
system.Add(gear2)

# 7. Connect the arm to Gear 1 with a revolute joint and motor to enforce rotation
revolute_arm_gear1 = chrono.ChLinkRevolute()
revolute_arm_gear1.SetBodyA(arm)
revolute_arm_gear1.SetBodyB(gear1)
revolute_arm_gear1.SetConstrained(True)
system.Add(revolute_arm_gear1)

# Motor to rotate the arm at constant speed
motor = chrono.ChLinkMotorRotationSpeed()
motor.SetSpindleArm(revolute_arm_gear1.GetRevolute())
motor.SetSpeed(rotation_speed_rad_per_sec)
system.Add(motor)

# 8. Connect Gear 1 and Gear 2 to simulate gear engagement
# Use a gear constraint if gears are meshed; in Chrono, approximate with a gear constraint
gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.1, 0), chrono.QUNIT))
system.Add(gear_constraint)

# Alternatively, if gear constraint is not preferred, use a gear ratio via a motor or a custom constraint
# For simplicity, we'll proceed with gear constraint

# 9. Finalize the setup
application.AssetBindAll()
application.AssetUpdateAll()

# 10. Simulation loop
while application.GetDeviceStream():
    application.BeginScene()
    application.DrawAll()
    application.DoStepHousing(0.01)
    application.EndScene()

# 11. End of simulation
application.GetDevice().closeDevice()