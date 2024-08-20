import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh






time_step = 1e-3
render_frame_rate = 50


vehicle_file = "hmmwv/hmmwv.json"
tire_model = "RIGID"  
tire_vis = "MESH"


terrain_type = "SCM"
terrain_length = 30.0  
terrain_width = 10.0
terrain_height = 1.0


Bekker_Kphi = 0.2e6  
Bekker_Kc = -20000    
Bekker_n = 1.1        
 Mohr_cohesion = 20000   
 Mohr_friction = 0.8      
 Janosi_shear = 1.4e4  
soil_density = 1500     


use_moving_patch = True


mesh_visualization = True  
sinkage_visualization = True  





system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetTimestepperType(chrono.ChTimestepper.Type.HHT)
system.SetStep(time_step)






vehicle = veh.WheeledVehicle(system, vehicle_file, tire_model, tire_vis)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.6, 0), chrono.ChQuaternionD(1, 0, 0, 0)))  





if terrain_type == "SCM":
    terrain = veh.SCMDeformableTerrain(system)
    terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, -terrain_height, 0), chrono.QUNIT))
    terrain.Initialize(terrain_length, terrain_width, 0.04, terrain_height)

    my_params = terrain.GetSCMParameters()
    my_params.m_Bekker_Kphi = Bekker_Kphi
    my_params.m_Bekker_Kc = Bekker_Kc
    my_params.m_Bekker_n = Bekker_n
    my_params.m_Mohr_cohesion = Mohr_cohesion
    my_params.m_Mohr_friction = Mohr_friction
    my_params.m_Janosi_shear = Janosi_shear
    my_params.m_rho = soil_density
    terrain.SetSCMParameters(my_params)

    if use_moving_patch:
        terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0.5, 2), chrono.ChVectorD(5, 3, 3))

    if sinkage_visualization:
        terrain.EnablePlotHeight(True)
        terrain.SetPlotType(veh.SCMDeformableTerrain.PlotType_DISPLACEMENT)
else:
    raise NotImplementedError("Only SCM terrain is currently supported.")





app = chronoirr.ChIrrApp(system, "HMMWV Simulation", chronoirr.dimension2du(1024, 768))
app.SetTimestep(time_step)
app.SetTryRealtime(True)
app.SetBackground(chrono.ChColor(0.5, 0.5, 0.5))
app.AddTypicalSky()
app.AddTypicalCamera(chronoirr.vector3df(0.0, 2.0, -6.0), chronoirr.vector3df(0.0, 0.0, 0.0))
app.AddLightWithShadow(chronoirr.vector3df(10.0, 20.0, 10.0), chronoirr.vector3df(0, 0, 0), 30, 8, 16, 40, 512, chronoirr.SColorf(1, 1, 1))

app.AssetBindAll()
app.AssetUpdateAll()











while app.GetDevice().run():
    time = system.GetChTime()

    
    vehicle.Update(time)
    if terrain_type == "SCM":
        terrain.Update(time)

    
    system.DoStepDynamics(time_step)

    
    app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()

    
    app.SetFramerate(render_frame_rate)