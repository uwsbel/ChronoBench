import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.scm as veh_scm
import pychrono.vehicle.utils as veh_utils
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  


step_size = 1e-3  


sim_time = 20.0  





hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))  
hmmwv.SetTimestep(step_size)
hmmwv.Initialize()





terrain = veh_scm.SCMDeformableTerrain(hmmwv.GetSystem())


soil_params = veh_scm.SCMDeformableTerrain.SoilParameters()
soil_params.Kphi = 1.5e6      
soil_params.n = 0.6           
soil_params.Kc = 1.2e5        
soil_params.phi = 30.0        
soil_params.cohesion = 2000  
soil_params.E_elastic = 4e7   
soil_params.nu = 0.3          
soil_params.constant_penetration_depth = 0.02  

terrain.SetSoilParameters(soil_params)


terrain.SetPlotType(veh_scm.SCMDeformableTerrain.PLOT_PRESSURE)
terrain.Initialize(10.0, 10.0, 0.05)  



height_map_data = veh_utils.LoadHeightMap(terrain, terrain.GetLengthX(), terrain.GetLengthY(), 0.0)








target_speed = 10  
driver = veh.ChDriver(hmmwv)  

driver.SetInputDelay(0.01)
driver.Initialize()




application = chronoirr.ChIrrApp(hmmwv.GetVehicle(), "HMMWV SCM Terrain Demo", chronoirr.dimension2du(1280, 720))
application.AddTypicalLights()
application.AddTypicalSky()
application.AddTypicalLogo()
application.SetChaseCamera(chrono.ChVectorD(0.0, -6.0, 1.5), chrono.ChVectorD(0, 0, 0.8))
application.AssetBindAll()
application.AssetUpdateAll()





time = 0
application.SetTimestep(step_size)

while application.GetDevice().run() and (time < sim_time):
    application.BeginScene()
    application.DrawAll()

    
    driver_inputs = driver.GetInputs()

    
    time = hmmwv.GetSystem().GetChTime()

    
    driver.Synchronize(time)

    
    hmmwv.Synchronize(time, driver_inputs, terrain)

    
    terrain.Synchronize(time)

    
    driver.Advance(step_size)
    hmmwv.Advance(step_size)
    terrain.Advance(step_size)

    application.DoStep()

    application.EndScene()


print(f"Simulation complete at time {time:6.3f} seconds")
print(f"Final vehicle pos : {hmmwv.GetVehicle().GetPos()}")