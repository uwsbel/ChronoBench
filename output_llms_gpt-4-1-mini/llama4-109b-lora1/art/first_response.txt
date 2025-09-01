import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.vehicle.utils as veh_utils

# Initialize PyChrono environment and core components
chrono.SetChairsDriver(True)
chrono.SetChronoDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initialize Irrlicht visualization
vis_size = 600
vis_title = 'ARTcar Simulation'
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(veh.GetSystem())
vis.SetWindowSize(vis_size, vis_size)
vis.SetWindowTitle(vis_title)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, -6.0, 1.5))
vis.AddLightDirectional()

# Initialize vehicle
artcar = veh.ARTcar(chrono.ChContactMethod_SMC)
veh.SetArtcar(artcar)
veh.SetSystem(artcar.GetSystem())

# Set vehicle parameters
veh.SetLocation(chrono.ChVector3d(0, 0, 1.75))
veh.SetOrientation(chrono.QuatFromAngleY(chrono.C_PI / 2))

# Initialize terrain
terrain = veh.RigidTerrain(veh.GetSystem())
terrain.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
terrain.SetInitHeight(chrono.ChVector3d(0, 0, 0.75))
terrain.SetTexture(chrono.GetChronoDataPath() + 'vehicle/artcar/texture/artcar_tire.jpg', 200, 200)
terrain.Initialize()

# Initialize driver system
driver = veh.ChInteractiveDriverIRR(vis)
veh.SetDriver(driver)
driver.Initialize()

# Initialize output
out_dir = veh.GetChronoDataPath() + 'vehicle/artcar/'
veh.SetOutputDataPath(out_dir)
veh.SetOutput(veh.GetSystem(), 0.1)

# Initialize visualization type
vis_type = veh.ChVehicleVisualSystem.Type_Irrlicht
veh.SetVisualization(vis_type)

# Initialize contact method
contact_method = chrono.ChContactMethod_SMC
veh.SetContactMethod(contact_method)

# Initialize simulation loop
while vis.Run():
    time_step = 1 / 50.0
    veh.GetSystem().DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    driver.Synchronize(time_step)
    driver.Update()
    veh_utils.UpdateInfo(veh, time_step)
    veh_utils.PrintStats(veh, time_step)
```