import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# 1. Initialize the PyChrono environment and core components.

chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # Ensure the data path is set

# Simulation parameters
step_size = 1.0 / 50  # 50 FPS

# 2. Add the required physical systems and objects as specified.

# Create the vehicle system using the FEDA vehicle from Chrono::Vehicle
contact_method = chrono.ChContactMethod_NSC  # Could also be SMC, using NSC here

# Initial location and orientation of the vehicle
init_loc = chrono.ChVectorD(0, 0, 0.5)  # Start half meter above terrain
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # No rotation initially

vehicle = veh.FEDA(contact_method)
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_MESH)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

# Set mesh visualization for all vehicle parts
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# 3. Set necessary default parameters such as positions, forces, and interactions.

# Create the rigid terrain with custom texture
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
                        100, 100)  # 100x100 m patch
patch.SetContactFriction(0.9f)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 50, 50)
terrain.Initialize()

# Interactive driver system
driver = veh.ChInteractiveDriver(vehicle.GetVehicle())
driver.SetStepSize(step_size)
driver.Initialize()

# Initialize the Irrlicht visualization
application = veh.ChIrrApp(vehicle.GetVehicle(), "FEDA Vehicle on Rigid Terrain", irr.dimension2du(1280, 720))
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(-8, -8, 5), chrono.ChVectorD(0, 0, 0.5))

# Adjust camera to follow the vehicle from slightly behind and above
camera_target = vehicle.GetChassisBody().GetPos()
camera_pos = camera_target + chrono.ChVectorD(-8, -8, 5)
application.SetCamera(irr.vector3df(camera_pos.x, camera_pos.y, camera_pos.z),
                      irr.vector3df(camera_target.x, camera_target.y, camera_target.z))

application.SetTimestep(step_size)
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
while application.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    application.Synchronize("FEDA Vehicle Simulation", driver)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    application.Advance(step_size)

    application.BeginScene()
    application.DrawAll()
    application.EndScene()