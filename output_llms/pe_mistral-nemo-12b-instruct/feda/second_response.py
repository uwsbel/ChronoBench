import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Updated initial vehicle location and orientation
initLoc = chrono.ChVector3d(-50, 0, 0.5)
initRot = chrono.Quaterniond(1, 0, 0, 0)

# ... (other parameters remain unchanged)

# Create the FEDA vehicle, set parameters, and initialize
vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# ... (vehicle visualization and system settings remain unchanged)

# Create the terrain with updated length
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)  # Increased terrain length

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the path-follower, cruise-control driver system
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle())
driver.SetTargetSpeed(10.0)  # Set target speed

# Create the path for the double lane change maneuver
path = chrono.ChFunction_Composite()
path.Add(chrono.ChFunction_Const(0.0), 0.0, 0.0)
path.Add(chrono.ChFunction_SegY(chrono.ChVector3d(-50, 5, 0), chrono.ChVector3d(-25, 5, 0), chrono.ChVector3d(-25, -5, 0)), 0.0, 5.0)
path.Add(chrono.ChFunction_SegY(chrono.ChVector3d(-25, -5, 0), chrono.ChVector3d(0, -5, 0), chrono.ChVector3d(0, 5, 0)), 5.0, 10.0)
path.Add(chrono.ChFunction_SegY(chrono.ChVector3d(0, 5, 0), chrono.ChVector3d(25, 5, 0), chrono.ChVector3d(25, -5, 0)), 10.0, 15.0)
path.Add(chrono.ChFunction_SegY(chrono.ChVector3d(25, -5, 0), chrono.ChVector3d(50, -5, 0), chrono.ChVector3d(50, 5, 0)), 15.0, 20.0)
driver.SetPath(path)

# Configure the steering controller
driver.SetSteeringLookAheadDistance(5.0)
driver.SetSteeringKp(1.0)
driver.SetSteeringKd(0.1)

# Configure the speed controller
driver.SetSpeedKp(0.5)
driver.SetSpeedKi(0.1)
driver.SetSpeedKd(0.01)

# ... (other driver system settings remain unchanged)

# Initialize driver system
driver.Initialize()

# ... (simulation loop remains unchanged)