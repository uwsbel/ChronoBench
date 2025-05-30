import pychrono.core as chrono
import pychrono.vehicle as veh  


system = chrono.ChSystemNSC()


system.SetCollisionSystemType(chrono.ChSystem::CollisionSystemType::CHRONO_COLLISION_SYSTEM_TYPE_HYDRO)
system.GetCollisionSystem().SetDefaultSuggestedEnvelope(0.01)
system.GetCollisionSystem().SetDefaultSuggestedMargin(0.005)


application = chrono.ChIrrApp(system, "Epicyclic Gears Simulation", chrono.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.vector3d(0, 1.5, -3))
application.AssetBindAll()
application.AssetUpdateAll()


material_gear = chrono.ChMaterialSurfaceSMC()
material_gear.SetFriction(0.4)
material_truss = chrono.ChMaterialSurfaceSMC()
material_truss.SetFriction(0.9)


truss_body = chrono.ChBodyEasyBox(0.2, 0.05, 0.2, 1000, True, True)
truss_body.SetPos(chrono.ChVectorD(0, 0, 0))
truss_body.SetBodyFixed(True)
truss_body.GetMaterialSurface().SetFriction(0.9)
system.Add(truss_body)


arm_length = 0.4
arm_thickness = 0.04
arm = chrono.ChBodyEasyBox(arm_length, arm_thickness, 0.02, 1000, True, True)
arm.SetPos(chrono.ChVectorD(0, 0.1, 0))
system.Add(arm)


motor_speed = chrono.VECT_X  
rotation_speed_rad_per_sec = 2 * 3.14159 / 5  



radius_gear1 = 0.05
gear1 = chrono.ChBodyEasyCylinder(radius_gear1, 0.02, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0, 0.1, 0))
gear1.SetMaterialSurface(material_gear)
system.Add(gear1)


radius_gear2 = 0.05
gear2 = chrono.ChBodyEasyCylinder(radius_gear2, 0.02, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(0.2, 0.1, 0))
gear2.SetMaterialSurface(material_gear)
system.Add(gear2)


revolute_arm_gear1 = chrono.ChLinkRevolute()
revolute_arm_gear1.SetBodyA(arm)
revolute_arm_gear1.SetBodyB(gear1)
revolute_arm_gear1.SetConstrained(True)
system.Add(revolute_arm_gear1)


motor = chrono.ChLinkMotorRotationSpeed()
motor.SetSpindleArm(revolute_arm_gear1.GetRevolute())
motor.SetSpeed(rotation_speed_rad_per_sec)
system.Add(motor)



gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.1, 0), chrono.QUNIT))
system.Add(gear_constraint)





application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDeviceStream():
    application.BeginScene()
    application.DrawAll()
    application.DoStepHousing(0.01)
    application.EndScene()


application.GetDevice().closeDevice()