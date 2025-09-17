import chrono
from chrono import ChSystemNSC, ChVisualization, ChMaterialSurfaceNSC, ChContactMethod, ChLoadContainer
from chrono.vehicle import ChVehicleModelRoad, ChTMeasyTire, ChSteering, ChWheel, ChDriver, ChAssimpImporter
from chrono.irrlicht import ChIrrApp
from pychrono.irrlicht import ChIrrTools


my_system = ChSystemNSC()



ground = my_system.AddBody(ChBody.BOX(100, 1, 100, 1000, 1000, 1000))
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


tire_mat = ChMaterialSurfaceNSC(1e6, 
                                1.0, 
                                0.1, 
                                0.1) 


road = ChBody.Box(20, 0.2, 50, 1000, 1000, 1000)
road.SetPos(chrono.ChVectorD(0, -4.9, 0))
road.SetBodyFixed(True)
road.SetMaterialSurface(tire_mat)
my_system.AddBody(road)


assimp_imp = ChAssimpImporter()
vehicle_model = assimp_imp.ImportFile('path/to/MAN_10t_truck.dae')
vehicle_model.SetName('vehicle')
vehicle_model.SetPos(chrono.ChVectorD(0, 0, 0))
vehicle_model.SetBodyFixed(False)
my_system.Add(vehicle_model)


for part in vehicle_model.GetSubSystems():
    if isinstance(part, ChVisual):
        part.SetCollide(False)
    elif isinstance(part, ChCollisionModel):
        part.SetCollide(True)


tire_radius = 1.0
tire_width = 0.4
tire_positions = [(1.8, -1.2), (-1.8, -1.2), (1.8, 1.2), (-1.8, 1.2)]
tire_axles = [0, 0, 1, 1]
tire_chassis_connections = [(0, 0), (1, 1), (2, 0), (3, 1)]
tire_names = ['FL', 'FR', 'RL', 'RR']
for i in range(4):
    tire_body = my_system.AddBody(ChBody.BOX(tire_width, tire_radius, tire_width, 1000, 1000, 1000))
    tire_body.SetPos(chrono.ChVectorD(*tire_positions[i]))
    tire_body.SetBodyFixed(False)
    tire_body.SetCollide(True)
    tire_body.SetMaterialSurface(tire_mat)

    tire_model = ChTMeasyTire()
    tire_model.SetTireWidth(tire_width)
    tire_model.SetTireRadius(tire_radius)
    tire_model.SetChassisConnectionPoint(chrono.ChVectorD(*tire_chassis_connections[i]))
    tire_model.Initialize(my_system, tire_body, vehicle_model, tire_axles[i])
    my_system.Add(tire_model)



steering = ChSteering()
steering.SetSteerAngle(0)
steering.SetSteerRate(1)
throttle = 0
brake = 0
driver = ChDriver(steering, throttle, brake)
vehicle_model.SetDriver(driver)


my_vis = ChIrrApp(my_system, 'MAN 10t Truck Simulation', 800, 600, False)


cam_pos = chrono.ChVectorD(0, 10, -15)
cam_target = chrono.ChVectorD(0, 0, 0)
my_vis.AddCamera(cam_pos, cam_target)


dir_light = chrono.ChVector(1, -1, -1)
dir_light.Normalize()
my_vis.AddLight(dir_light, 0.8, 0.8, 0.8)


my_vis.AddSkyBox(chrono.ChColor(0.8, 0.9, 1.0))





my_vis.Run()