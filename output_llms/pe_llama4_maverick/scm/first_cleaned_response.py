import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math


out_dir = "HMMWV_SCM_OUTPUT"


terrain_length = 100.0  
terrain_width = 100.0   
soil_params = {
    'cohesion': 10e3,  
    'friction_angle': math.radians(30),  
    'youngs_modulus': 2e7,  
    'poisson_ratio': 0.3,  
    'shear_modulus': 1e7  
}


init_loc = chrono.ChVector3d(0, 0.5, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  


fps = 50  
time_step = 1e-3  
end_time = 1000  


my_system = chrono.ChSystemSMC()


vehicle = veh.HMMWV_Full(my_system)
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.SCMDeformableTerrain(my_system)
terrain.SetSoilParameters(
    soil_params['cohesion'],
    soil_params['friction_angle'],
    soil_params['youngs_modulus'],
    soil_params['poisson_ratio'],
    soil_params['shear_modulus']
)
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                          terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("soil_textures/grass.jpg"), 200, 200)
patch.SetPlotType(veh.SCMPlotType_SINKAGE, 0, 0.1)
terrain.Initialize()


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), 10, 10)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 10, 10))


driver = veh.ChIrrGuiDriver(vis, vehicle.GetVehicle())


time = 0
while vis.Run() and time < end_time:
    time = my_system.GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.SetDriverInputs(driver_inputs)
    vehicle.Update(time_step)
    terrain.Update(time_step)
    my_system.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    time += time_step


del vis
del vehicle
del terrain
del my_system