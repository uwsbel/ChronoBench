import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
from pychrono.vehicle import UAZBUS, ChIrrGuiDriver

# 1) Set up Chrono
chrono.SetChronoDataPath("")   # adjust if needed, or set CHRONO_DATA_PATH env var
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2) Create the UAZBUS vehicle
vehicle = UAZBUS(system)
# Initialize at the origin, pointing along X, with a small Z offset
init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))
vehicle.Initialize(chrono.ChCoordsysD(init_pos, init_rot))

# Choose how the chassis and wheels are visualized
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

# 3) Create a rigid terrain
terrain = veh.RigidTerrain(system)
# Define a default NSC (non‐smooth) material for the patch
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.1)
terrain.SetContactMaterial(patch_mat)
# Create a 200×200 patch, lying in the X–Y plane
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                       chrono.Q_from_AngX(chrono.CH_C_PI / 2)),
    200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(chrono.GetChronoDataPath() + "texture/grass.jpg", 50, 50)
terrain.Initialize()

# 4) Set up the Irrlicht visual system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("PyChrono UAZBUS Simulation")
vis.SetWindowSize(1280, 720)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
vis.AddSkyBox()
# Place the camera behind and above the vehicle, looking at its chassis
vis.AddCamera(chrono.ChVectorD(-10, 5, 1),   # position
              chrono.ChVectorD(0, 0, 0))     # target
# Add a single directional light
vis.AddLightDirectional(chrono.ChVectorD(-1, -1, -1), 0.8)

# 5) Create a GUI driver to steer the vehicle
driver = ChIrrGuiDriver(vis.GetDevice(), vehicle)
driver.SetSteeringDelta(0.04)   # per time step
driver.SetThrottleDelta(0.1)
driver.SetBrakeDelta(0.1)
driver.Initialize()

# 6) Final system settings
step_size = 1e-2
system.SetTimestepperType(chrono.ChTimestepper.Type.EULER_IMPLICIT_LINEARIZED)

# 7) Main simulation loop
while vis.Run():
    t = system.GetChTime()

    # Get current driver inputs
    inputs = driver.GetInputs()

    # Synchronize each module at time t
    driver.Synchronize(t)                              # reading GUI
    vehicle.Synchronize(t, inputs, terrain)            # vehicle to terrain & driver
    terrain.Synchronize(t)                             # (nothing internal to sync here)
    vis.BeginScene()                                   # start rendering
    vis.Render()
    vis.EndScene()

    # Advance each module by one time step
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)