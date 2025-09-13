import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 200, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_ROTATE_Y_TO_Z))
bus.SetTireType(veh.TireModelType_RIGID)
bus.SetTireStepSize(0.001)
bus.Initialize()


bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 3, -10), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, -5), chrono.ChVectorD(0, 0, 0), 50, 3, 10, 40, 512)


driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0  
driver.SetSteeringDelta(0.02 / steering_time)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


step_size = 0.002
render_step = 1.0 / 50  
time = 0
frame_number = 0

while vis.Run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    
    
    bus.Synchronize(time, driver.GetInputs(), terrain)
    
    
    driver.Advance(step_size)
    bus.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.RenderFrame(chrono.ChFrameD(), 2)  
    vis.EndScene()
    
    
    if time >= frame_number * render_step:
        frame_number += 1

print("Simulation completed")