import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


chrono.SetChronoDataPath("../../../data/")
veh.SetDataPath("../../../data/vehicle/")


system = chrono.ChSystemNSC()


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ARTcar Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AttachSystem(system)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()


vehicle = veh.ARTCar(system,
                     veh.GetDataFile("ARTcar/ARTcar.json"),
                     chrono.ChContactMethod_NSC)


vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngZ(0)))


powertrain = veh.ARTPowertrain(veh.GetDataFile("ARTcar/ARTpowertrain.json"))
vehicle.InitializePowertrain(powertrain)


driver = veh.ChInteractiveDriverIRR(vis)


driver.SetUserInputFile(veh.GetDataFile("ARTcar/driver/ARTcar_inputs.json"))


driver.Initialize()


step_size = 1e-3
render_step_size = 1 / 50  


time = 0
while vis.Run():
    time += step_size

    
    if vis.ShouldRender():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)

    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)

    
    vis.UpdateVisualModel()


vis.Shutdown()