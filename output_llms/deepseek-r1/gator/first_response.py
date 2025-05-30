import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# Initialize PyChrono environment
chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
contact_method = chrono.ChContactMethod_SMC
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Create rigid terrain
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(20, 20, 0.1), 
                        "textures/terrain.png")
patch.SetTexture(20, 20)
terrain.Initialize()

# Initialize Gator vehicle
gator = veh.Gator(system)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), 
                                        chrono.QUNIT))
gator.SetTireVisualizationType(veh.VisualizationType_MESH)
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.Initialize()

# Create TMeasy tires
tire_params = veh.TMeasyParams()
tire_params.mass = 15.0
tire_params.Inertia = chrono.ChVectorD(0.6, 1.0, 0.6)
tire_params.disc_locs = [-0.1, 0.1]

for axle in [veh.LEFT, veh.RIGHT]:
    gator.SetTireType(tire_params, veh.FRONT, axle)
    gator.SetTireType(tire_params, veh.REAR, axle)

# Create interactive driver
driver = veh.InteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# Simulation parameters
step_size = 0.005
render_step = 1.0 / 50  # 50 FPS
frame_number = 0
time = 0

while vis.Run():
    time = system.GetChTime()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    
    # Update visualization
    if time >= frame_number * render_step:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame_number += 1