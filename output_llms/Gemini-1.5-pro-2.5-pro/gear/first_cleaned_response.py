import pychrono as chrono
import pychrono.irrlicht as irr
import math







system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0)) 


solver = chrono.ChSolverSOR()
system.SetSolver(solver)






sun_radius = 0.5         
planet_radius = 0.25     
carrier_length = sun_radius + planet_radius 
gear_thickness = 0.1     


carrier_angular_speed = math.pi / 2.0  








truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetName("TRUSS")
system.Add(truss)



carrier_arm = chrono.ChBody()
carrier_arm.SetName("CARRIER_ARM")
system.Add(carrier_arm)

carrier_arm.SetPos(chrono.ChVector3d(carrier_length / 2.0, 0, 0))
carrier_arm.SetMass(1.0)
carrier_arm.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))



sun_gear = chrono.ChBody()
sun_gear.SetName("SUN_GEAR")
system.Add(sun_gear)
sun_gear.SetPos(chrono.ChVector3d(0, 0, 0))
sun_gear.SetMass(2.0)
sun_gear.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))



planet_gear = chrono.ChBody()
planet_gear.SetName("PLANET_GEAR")
system.Add(planet_gear)

planet_gear.SetPos(chrono.ChVector3d(carrier_length, 0, 0))
planet_gear.SetMass(0.5)
planet_gear.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))









revolute_carrier_truss = chrono.ChLinkLockRevolute()
revolute_carrier_truss.Initialize(
    truss,
    carrier_arm,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(math.pi/2)) 
)
system.Add(revolute_carrier_truss)



lock_sun_truss = chrono.ChLinkLockLock()
lock_sun_truss.Initialize(truss, sun_gear, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
system.Add(lock_sun_truss)



revolute_planet_carrier = chrono.ChLinkLockRevolute()
revolute_planet_carrier.Initialize(
    carrier_arm,
    planet_gear,
    chrono.ChFramed(chrono.ChVector3d(carrier_length, 0, 0), chrono.Q_from_AngX(math.pi/2))
)
system.Add(revolute_planet_carrier)




gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(sun_gear, planet_gear, chrono.ChFramed(chrono.ChVector3d(0,0,0)))

gear_constraint.SetRatio(sun_radius / planet_radius)

gear_constraint.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.Q_from_AngX(math.pi/2)))
gear_constraint.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.Q_from_AngX(math.pi/2)))
system.Add(gear_constraint)



motor_function = chrono.ChFunctionConst(carrier_angular_speed)
carrier_motor = chrono.ChLinkMotorRotationSpeed()
carrier_motor.Initialize(
    truss,                  
    carrier_arm,            
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(math.pi/2)) 
)
carrier_motor.SetSpeedFunction(motor_function)
system.Add(carrier_motor)







mat_blue = chrono.ChVisualMaterial()
mat_blue.SetDiffuseColor(chrono.ChColor(0.1, 0.1, 0.8))

mat_green = chrono.ChVisualMaterial()
mat_green.SetDiffuseColor(chrono.ChColor(0.1, 0.8, 0.1))

mat_red = chrono.ChVisualMaterial()
mat_red.SetDiffuseColor(chrono.ChColor(0.8, 0.1, 0.1))

mat_gray = chrono.ChVisualMaterial()
mat_gray.SetDiffuseColor(chrono.ChColor(0.4, 0.4, 0.4))


vis_truss = chrono.ChVisualShapeCylinder(0.1, 0.5)
truss.AddVisualShape(vis_truss, chrono.ChFramed(chrono.ChVector3d(0,-0.25,0)))
truss.GetVisualShape(0).SetMaterial(0, mat_gray)


vis_carrier = chrono.ChVisualShapeBox(carrier_length, 0.05, 0.05)
carrier_arm.AddVisualShape(vis_carrier)
carrier_arm.GetVisualShape(0).SetMaterial(0, mat_blue)


vis_sun_gear = chrono.ChVisualShapeCylinder(sun_radius, gear_thickness)
sun_gear.AddVisualShape(vis_sun_gear, chrono.ChFramed(Vect=chrono.ChVector3d(0,0,0), Rot=chrono.Q_from_AngX(math.pi/2)))
sun_gear.GetVisualShape(0).SetMaterial(0, mat_red)


vis_planet_gear = chrono.ChVisualShapeCylinder(planet_radius, gear_thickness)
planet_gear.AddVisualShape(vis_planet_gear, chrono.ChFramed(Vect=chrono.ChVector3d(0,0,0), Rot=chrono.Q_from_AngX(math.pi/2)))
planet_gear.GetVisualShape(0).SetMaterial(0, mat_green)







application = irr.ChIrrApp(
    system,
    "Epicyclic Gear System Simulation",
    irr.dimension2du(1280, 720)
)


application.AddCamera(pos=chrono.ChVector3d(0, 1.5, -2.5), lookat=chrono.ChVector3d(0, 0, 0))


application.AddLight(pos=chrono.ChVector3d(3, 3, -3), radius=5)


application.AssetBindAll()
application.AssetUpdateAll()





application.SetTimestep(0.005)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    irr.draw_chrono_plugin(system, "link_frames_csys")
    
    application.DoStep()
    application.EndScene()

print("Simulation finished.")