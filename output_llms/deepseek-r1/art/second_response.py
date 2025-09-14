```python
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (MODIFIED)
initLoc = chrono.ChVector3d(1, 0, 0.5)  # Changed x-coordinate from 0 to 1
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (MODIFIED)
vis_type = veh.VisualizationType_PRIMITIVES  # Changed from MESH to PRIMITIVES

# Collision type for chassis (MODIFIED)
chassis_collision_type = veh.CollisionType_MESH  # Changed from NONE to MESH

# Type of tire model (MODIFIED)
tire_model = veh.TireModelType_FIALA  # Changed from TMEASY to FIALA

# Rigid terrain
terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.2)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create systems

# Create the ARTcar vehicle, set parameters, and initialize
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetMaxMotorVoltageRatio(0.16)
vehicle.SetStallTorque(0.3)
vehicle.SetTireRollingRes