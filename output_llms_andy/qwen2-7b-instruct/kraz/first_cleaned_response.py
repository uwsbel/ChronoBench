from pychrono.core import ChSystem, ChVector
from pychrono.vehicle import ChKraz, ChDriverSystem
from pychrono.irrlicht import ChIrrApp, irr
from pychrono.geometry import ChTerrain, ChTerrainMaterial


chrono = ChSystem()
chrono.Set_G_acc(ChVector<>(0, -9.81, 0))


vehicle_mass = 1000.0
terrain_height = 100.0
terrain_width = 100.0
terrain_length = 100.0
terrain_friction = 0.8
terrain_restitution = 0.1


kraz = ChKraz()
kraz.SetMass(vehicle_mass)
kraz.Initialize(chrono, ChVector<>(0, 0, 0), ChVector<>(1, 1, 1))


kraz.SetSteeringAngle(0)
kraz.SetThrottle(0)
kraz.SetBrake(0)


terrain_material = ChTerrainMaterial()
terrain_material.SetFriction(terrain_friction)
terrain_material.SetRestitution(terrain_restitution)
terrain = ChTerrain()
terrain.Initialize(chrono, ChVector<>(-terrain_width / 2, -terrain_height / 2, 0),
                   ChVector<>(terrain_length, terrain_height, terrain_width),
                   terrain_material)


driver = ChDriverSystem()
driver.Initialize(chrono, kraz)


app = ChIrrApp(chrono, "PyChrono Kraz Simulation", irr.svideo_device.create_device(irr.svideo_device.DEVICE_TYPE_3D))
app.AddTypicalSky()
app.AddTypicalLogo("logo.pcx")
app.AddTypicalCamera(ChVector<>(100, 100, 50))
app.AddTypicalLights()


app.SetCameraPosition(ChVector<>(10, 10, 10))
app.SetCameraTarget(ChVector<>(0, 0, 0))
app.SetCameraUpVector(ChVector<>(0, 1, 0))


app.SetTimestep(chrono.GetTimestep())


while app.GetDevice().run():
    
    chrono.DoStepDynamics(1)

    
    steering_angle = driver.ControlSteering(chrono.GetChTime())
    throttle = driver.ControlThrottle(chrono.GetChTime())
    brake = driver.ControlBrake(chrono.GetChTime())
    kraz.SetSteeringAngle(steering_angle)
    kraz.SetThrottle(throttle)
    kraz.SetBrake(brake)

    
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()