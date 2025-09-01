import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.terrain as terrain
import numpy as np


system = chrono.ChSystemNSC()


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
hmmwv.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)


terrain_length = 100.0  
terrain_width = 100.0   

soil_params = terrain.SCMDeformableSoilParameters()
soil_params.Bekker_Kphi = 0.2e6
soil_params.Bekker_Kc = 0
soil_params.Bekker_n = 1.1
soil_params.Mohr_cohesion = 0
soil_params.Mohr_friction = 30
soil_params.Janosi_shear = 0.01
soil_params.elastic_K = 4e7
soil_params.damping_R = 3e4


scm_terrain = terrain.SCMDeformableTerrain(system)
scm_terrain.SetSoilParameters(soil_params)


scm_terrain.Initialize(terrain_length, terrain_width, 0.1)


def terrain_height(x, y):
    return 0.0

scm_terrain.SetPlane(terrain_height)


driver = veh.ChIrrGuiDriver(hmmwv.GetVehicle())
driver.Initialize()


vis = veh.ChWheeledVehicleIrrApp(hmmwv.GetVehicle(), 'HMMWV Demo')

vis.SetHUDLocation(500, 20)
vis.SetSkyBox()
vis.AddTypicalLogo()
vis.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
vis.AddTypicalCamera(irr.vector3df(6, 3, -6), irr.vector3df(0, 0, 0))
vis.SetTimestep(hmmwv.GetSystem().GetStep())


step_size = 1e-3
end_time = 20


time = 0
while (vis.GetDevice().run() and time < end_time):
    
    driver_inputs = driver.GetInputs()
    
    
    time = hmmwv.GetSystem().GetChTime()
    driver.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, scm_terrain)
    scm_terrain.Synchronize(time)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    
    driver.Advance(step_size)
    hmmwv.Advance(step_size)
    scm_terrain.Advance(step_size)
    vis.Advance(step_size)

    
    time += step_size

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()


vis.GetDevice().drop()