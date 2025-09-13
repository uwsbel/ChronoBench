import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# 1. Initialize environment and core components
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Create systems
system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)

# 2. Create rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 200, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# 3. Create CityBus vehicle
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_ROTATE_Y_TO_Z))
bus.SetTireType(veh.TireModelType_RIGID)
bus.SetTireStepSize(0.001)
bus.Initialize()

# Visualization settings
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)

# 4. Create Irrlicht visualization
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

# 5. Create interactive driver
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0  # seconds to full steer
driver.SetSteeringDelta(0.02 / steering_time)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# 6. Simulation loop setup
step_size = 0.002
render_step = 1.0 / 50  # 50 FPS
time = 0
frame_number = 0

while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver.Synchronize(time)
    
    # Update vehicle
    bus.Synchronize(time, driver.GetInputs(), terrain)
    
    # Advance simulation
    driver.Advance(step_size)
    bus.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.RenderFrame(chrono.ChFrameD(), 2)  # Reference frame
    vis.EndScene()
    
    # Maintain 50 FPS rendering
    if time >= frame_number * render_step:
        frame_number += 1

print("Simulation completed")