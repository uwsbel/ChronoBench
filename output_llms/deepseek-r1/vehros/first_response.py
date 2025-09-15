import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
import numpy as np

# =============================================================================
# ROS Node Initialization
# =============================================================================
ros_manager = chros.ChROSPyManager()
ros_manager.RegisterHandler(chros.ROSTimestampHandler())
ros_manager.RegisterHandler(chros.ROSParamsHandler())
ros_manager.Initialize()

# =============================================================================
# Simulation Parameters
# =============================================================================
contact_method = chrono.ChContactMethod_NSC
step_size = 0.002
tire_model = veh.TireModelType_TMEASY
engine_model = veh.EngineModelType_SIMPLE_MAP
drive_type = veh.DrivelineTypeWV_RWD
init_pos = chrono.ChVectorD(0, 0, 0.5)

# =============================================================================
# Create Chrono System
# =============================================================================
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)

# =============================================================================
# Initialize Vehicle System
# =============================================================================
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos))
hmmwv.SetEngineType(engine_model)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
hmmwv.SetDriveType(drive_type)
hmmwv.SetTireType(tire_model)
hmmwv.Initialize()

# Configure visualization
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

vehicle = hmmwv.GetVehicle()

# =============================================================================
# Initialize Terrain
# =============================================================================
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         200, 400)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# =============================================================================
# Driver System and ROS Integration
# =============================================================================
driver = veh.ChInteractiveDriverIRR(vehicle)  # ROS inputs will override this
ros_manager.RegisterHandler(
    chros.VehicleInputsHandler(0.5, vehicle, driver, "~/controls")
)

# Vehicle state publisher
state_handler = chros.VehicleStateHandler(vehicle, "~/vehicle_state")
ros_manager.RegisterHandler(state_handler)

# =============================================================================
# Simulation Loop
# =============================================================================
time = 0
while ros_manager.ProcessEvents():
    time = system.GetChTime()
    
    # Synchronize and process ROS messages
    ros_manager.Synchronize(time)
    
    # Update driver inputs
    driver.Synchronize(time)
    
    # Advance simulation
    vehicle.Synchronize(time)
    terrain.Synchronize(time)
    system.DoStepDynamics(step_size)
    
    # Update ROS data
    ros_manager.Update(time, step_size)

print("Simulation completed.")