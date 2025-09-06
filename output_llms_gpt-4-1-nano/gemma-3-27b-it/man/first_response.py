import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np

# ---------------------------------------------------------------------
#
#  Demo code about using the MAN 10t truck model
#  in a simple environment.
#
# ---------------------------------------------------------------------

# --- Set the path to the Chrono data directory ---
chrono.SetChronoDataPath("./chrono_data/")  # Replace with your actual data path

# --- Create a physical system ---
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --- Set gravity ---
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

# --- Create the terrain ---
terrain = veh.RigidTerrain(sys)
terrain.SetMeshFile("terrain/terrain.obj")  # Use a terrain mesh file
terrain.SetTextureFile("terrain/grass.jpg")  # Use a texture for the terrain
terrain.SetContactOffset(0.01)
terrain.Initialize()

# --- Create the vehicle ---
vehicle = veh.MAN_10t(sys)

# --- Set vehicle initial position ---
initial_position = chrono.ChVector3d(0, 1, 0)
initial_rotation = chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))  # No initial rotation
vehicle.SetInitPosition(chrono.ChCoordsysD(initial_position, initial_rotation))
vehicle.Initialize()

# --- Set tire model ---
vehicle.SetTireModel(veh.TMEASY)

# --- Configure visualization settings ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, -5))  # Chase camera position
vis.AddTypicalLights()

# --- Set chase camera ---
chase_cam = vis.GetChaseCamera()
chase_cam.SetVerticalAngle(0.3)
chase_cam.SetDistance(5)

# --- Driver controls ---
driver = veh.SimpleDriver(vehicle)

# --- Simulation loop ---
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # --- Update driver controls ---
    driver.Update(time_step)

    # --- Step the simulation ---
    sys.DoStepDynamics(time_step)