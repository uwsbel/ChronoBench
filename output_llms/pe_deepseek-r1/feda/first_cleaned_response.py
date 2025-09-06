import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.SetChronoDataPath(chrono.GetChronoDataPath())


vehicle = veh.FEDA()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0, 0.5),  
    chrono.Q_from_AngZ(0)           
))
vehicle.SetTireType(veh.TireModelType_TMEASY)  
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.8)  
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        100, 50)  
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 50)  
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEDA Vehicle Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddTypicalLights()
vis.AddSkyBox()


vis.AddCamera(chrono.ChVector3d(-5, 0, 2), chrono.ChVector3d(0, 0, 0.5))


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


step_size = 0.002
render_step = 1.0 / 50  
frame_number = 0


while vis.Run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    
    
    if frame_number % int(render_step / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(vehicle.GetChassisBody().GetFrame_REF_to_abs())
        vis.EndScene()
        
        
        pos = vehicle.GetPos()
        speed = vehicle.GetSpeed()
        print(f"Time: {time:.2f}s | Position: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) | Speed: {speed * 3.6:.1f} km/h")
    
    frame_number += 1