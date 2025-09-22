import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.suspension as veh_susp
import pychrono.vehicle.wheeled_vehicle as veh_whl
import pychrono.vehicle.driver as veh_drv
import pychrono.vehicle.terrain as veh_terrain
import pychrono.irrlicht as chronoirr
import os





chrono.SetChronoDataPath(chrono.GetChronoDataPath())  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))






initLoc = chrono.ChVectorD(0, 0, 1.0)  
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(veh.VehicleTireType.RIGID)  
vehicle.SetTireCollision(False)  
vehicle.SetInitWheelRotations(True)  
vehicle.SetUseMeshVisualization(True)  

vehicle.Initialize(system)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)





terrain = veh_terrain.SCMTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))


soil_params = {
    "Bekker_Kphi": 150000,     
    "Bekker_Kc": 20000,        
    "Bekker_n": 1.1,           
    "Mohr_cohesion": 0,        
    "Mohr_friction": 30,       
    "Janosi_shear": 56000,     
    "Elastic_K": 20000000,     
    "Damping_R": 1500,         
}

terrain.SetSoilParameters(
    soil_params["Bekker_Kphi"],
    soil_params["Bekker_Kc"],
    soil_params["Bekker_n"],
    soil_params["Mohr_cohesion"],
    soil_params["Mohr_friction"],
    soil_params["Janosi_shear"],
    soil_params["Elastic_K"],
    soil_params["Damping_R"],
)


terrain.SetPlotType(veh_terrain.SCMTerrain.PlotType_SINKAGE)


terrain.EnableMovingPatch(True)
terrain.SetMovingPatchVehicle(vehicle.GetChassis().GetBody())
terrain.SetPatchDimensions(6, 6)  

terrain.Initialize()





vis = chronoirr.ChIrrApp(vehicle.GetVehicle(), "HMMWV Full Vehicle on SCM Terrain", chronoirr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddTypicalCamera(chronoirr.vector3df(0, -10, 2), chronoirr.vector3df(0, 0, 1))
vis.AssetBindAll()
vis.AssetUpdateAll()
vis.SetTimestep(0.02)  





driver = veh_drv.ChInteractiveDriverIrrlicht(vis)
driver.SetInputDelay(0.0)  
driver.SetThrottleDelta(0.05)  
driver.SetSteeringDelta(0.05)  
driver.SetBrakingDelta(0.1)  





step_size = 0.02  
render_step_size = 1.0 / 50  


time = 0
render_frame = 0





vis.SetEasyCamera(True)

while vis.Run():
    
    driver.Synchronize(time)
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.BeginScene()
    vis.DrawAll()

    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    driver.Advance(step_size)
    system.DoStepDynamics(step_size)

    vis.EndScene()

    time += step_size