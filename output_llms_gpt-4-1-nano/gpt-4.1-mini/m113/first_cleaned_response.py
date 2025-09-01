import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr





chrono.SetChronoDataPath(chrono.GetChronoDataPath())  

time_step = 1.0 / 1000  


system = chrono.ChSystemSMC()






terrain = veh.RigidTerrain(system)


patch = terrain.AddPatch(
    chrono.ChVectorD(0, 0, 0),                 
    chrono.ChVectorD(0, 0, 1),                 
    100, 100                                   
)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetMaterialSurface(
    chrono.ChMaterialSurfaceSMC()
)
patch.GetMaterialSurface().SetFriction(0.9)
patch.GetMaterialSurface().SetRestitution(0.01)

terrain.Initialize()


m113 = veh.M113Vehicle(system)
m113.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
m113.SetChassisVisualizationType(veh.VisualizationType_MESH)
m113.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)
m113.Initialize()







driver = veh.ChIrrGuiDriver(m113.GetVehicle())


driver.SetInputTimeResponse(1.0, 1.0, 0.5)


driver.Initialize()





vis = veh.ChWheeledVehicleIrrApp(m113.GetVehicle(), "M113 Vehicle Simulation")


vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddLogo()
vis.AddHelp()


vis.SetChaseCamera(trackPoint=chrono.ChVectorD(0, 0, 1.75),
                   chaseDist=7.0,
                   chaseHeight=1.75,
                   chasePitch=0.3)

vis.GetDevice().GetVideoDriver().getMaterialRenderer(chrono.material.E_MATERIAL_FLAG::EMF_WIREFRAME)


vis.AssetBindAll()
vis.AssetUpdateAll()
vis.SetTimestep(time_step)
vis.Initialize()





while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()

    
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking = driver.GetBraking()

    
    time = system.GetChTime()
    driver.Synchronize(time)
    m113.Synchronize(time, steering, throttle, braking, terrain)
    terrain.Synchronize(time)
    vis.Synchronize(time, steering, throttle, braking)

    
    driver.Advance(time_step)
    m113.Advance(time_step)
    terrain.Advance(time_step)
    vis.Advance(time_step)

    
    system.DoStepDynamics(time_step)

    vis.EndScene()