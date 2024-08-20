import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import math







system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)



chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)



ground_material = chrono.ChMaterialSurfaceNSC()
ground_material.SetFriction(0.3)
ground_material.SetRestitution(0.1)

ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, ground_material)
ground.SetBodyFixed(True)
system.Add(ground)


rover = chrono.ChBodyAuxRef()
rover.SetBodyFixed(False)
rover.SetMass(100)
rover.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
system.Add(rover)


rover_vis = chrono.ChObjShapeFile(chrono.GetChronoDataFile("vehicle/viper/obj/viper.obj"))
rover_vis.SetScale(chrono.ChVectorD(0.0254, 0.0254, 0.0254))
rover.AddAsset(rover_vis)


wheel_radius = 0.2
wheel_width = 0.1
wheel_mass = 5

wheel_material = chrono.ChMaterialSurfaceNSC()
wheel_material.SetFriction(1.0)
wheel_material.SetRestitution(0.1)

wheels = []
for i in range(4):
    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, wheel_radius, wheel_width, wheel_mass, True, True, wheel_material)
    wheel.SetBodyFixed(False)
    system.Add(wheel)
    wheels.append(wheel)


wheel_vis = chrono.ChObjShapeFile(chrono.GetChronoDataFile("vehicle/viper/obj/viper_wheel.obj"))
wheel_vis.SetScale(chrono.ChVectorD(0.0254, 0.0254, 0.0254))

for wheel in wheels:
    wheel.AddAsset(wheel_vis)


susp_spring_k = 20000
susp_damping = 500

suspensions = []
for i in range(4):
    suspension = chrono.ChLinkTSDA()
    suspensions.append(suspension)

suspensions[0].Initialize(wheels[0], rover, False, chrono.ChVectorD(-0.6, -0.3, 0.5), chrono.ChVectorD(0, 0, 0))
suspensions[1].Initialize(wheels[1], rover, False, chrono.ChVectorD(0.6, -0.3, 0.5), chrono.ChVectorD(0, 0, 0))
suspensions[2].Initialize(wheels[2], rover, False, chrono.ChVectorD(-0.6, -0.3, -0.5), chrono.ChVectorD(0, 0, 0))
suspensions[3].Initialize(wheels[3], rover, False, chrono.ChVectorD(0.6, -0.3, -0.5), chrono.ChVectorD(0, 0, 0))

for suspension in suspensions:
    suspension.SetSpringCoefficient(susp_spring_k)
    suspension.SetDampingCoefficient(susp_damping)
    system.Add(suspension)


driver = chrono.ChDriver(system)
driver.Initialize(rover)


rover.SetPos(chrono.ChVectorD(0, 1, 0))


vis = chronoirr.ChIrrApp(system, "Viper Rover Simulation", chronoirr.dimension2du(1024, 768))
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.SetCameraPosition(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))


time_step = 1e-3
time_end = 10

vis.AssetBindAll()
vis.AssetUpdateAll()

vis.Initialize()
vis.SetTimestep(time_step)
vis.SetTryRealtime(True)

while vis.Run():
    time = system.GetChTime()

    
    steering_angle = math.sin(time * 2 * math.pi / time_end) * math.pi / 6
    driver.SetSteering(steering_angle)

    
    system.DoStepDynamics(time_step)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()