import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.postprocess as postprocess


chrono.SetChronoDataPath("/path/to/chrono/data/")


init_loc = chrono.ChVectorD(0, 0, 1.0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle = veh.HMMWV_Full()
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e7,  
                           0,      
                           1.1,    
                           0,      
                           30,     
                           0.01,   
                           2e8,    
                           3e4     
                          )
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time - vehicle.GetSystem().GetChTime())
    terrain.Advance(time - vehicle.GetSystem().GetChTime())
    vis.Advance(0.02)
    vis.Render()