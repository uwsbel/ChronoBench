import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print('Copyright (c) 2023')


veh.SetChronoDataPath(chrono.GetChronoDataPath())
my_vehicle = veh.MAN_10t(veh.SteeringTypeWHEEL, veh.RigidTerrainType_BOX)


terrain = veh.RigidTerrain(my_vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(-0.2)))
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetNormalMap(veh.GetDataFile("terrain/textures/grass_normal.png"), 200, 200)
terrain.Initialize()


my_vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
my_vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
my_vehicle.SetChassisFixed(False)
my_vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-5, 0, 1.0), chrono.Q_from_AngZ(0)))
my_vehicle.SetTireType(veh.TireType_TMEASY)
my_vehicle.SetTireStepSize(1e-3)
my_vehicle.Initialize()


driver = veh.ChDriver(my_vehicle.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddLightDirectional(1, 1, 1, 0, -1, -1)
vis.AttachVehicle(my_vehicle.GetVehicle())


while vis.Run():
    time = my_vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    driver_inputs.m_steering = 0.0
    driver_inputs.m_throttle = 0.3
    driver_inputs.m_braking = 0.0
    
    
    driver.Synchronize(time)
    
    my_vehicle.Synchronize(time, driver_inputs, terrain.GetPlot())
    
    vis.Synchronize('', driver_inputs)
    
    my_vehicle.Advance(0.01)
    vis.Advance(0.01)