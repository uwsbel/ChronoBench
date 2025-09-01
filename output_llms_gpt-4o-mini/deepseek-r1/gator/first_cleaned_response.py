import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os


chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
contact_method = chrono.ChContactMethod_SMC
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(20, 20, 0.1), 
                        "textures/terrain.png")
patch.SetTexture(20, 20)
terrain.Initialize()


gator = veh.Gator(system)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), 
                                        chrono.QUNIT))
gator.SetTireVisualizationType(veh.VisualizationType_MESH)
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.Initialize()


tire_params = veh.TMeasyParams()
tire_params.mass = 15.0
tire_params.Inertia = chrono.ChVectorD(0.6, 1.0, 0.6)
tire_params.disc_locs = [-0.1, 0.1]

for axle in [veh.LEFT, veh.RIGHT]:
    gator.SetTireType(tire_params, veh.FRONT, axle)
    gator.SetTireType(tire_params, veh.REAR, axle)


driver = veh.InteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


step_size = 0.005
render_step = 1.0 / 50  
frame_number = 0
time = 0

while vis.Run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    
    system.DoStepDynamics(step_size)
    
    
    if time >= frame_number * render_step:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame_number += 1