import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.gator as gator

# =============================================================================
# Set up the simulation system
# =============================================================================

# Create the simulation system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Set solver parameters
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetMaxItersSolverSpeed(40)
sys.SetMaxItersSolverStab(40)
sys.SetTolForce(1e-5)

# =============================================================================
# Create the terrain
# =============================================================================

# Create a flat rigid terrain
terrain_length = 100.0
terrain_width = 100.0
terrain = veh.RigidTerrain(sys)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)

patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0),
                         chrono.ChVectorD(terrain_length, terrain_width, 0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
terrain.Initialize()

# =============================================================================
# Create the Gator vehicle
# =============================================================================

# Set initial vehicle location and orientation
vehicle_pos = chrono.ChVectorD(0, 0, 0.5)
vehicle_rot = chrono.ChQuaternionD(1, 0, 0, 0)

# Create the Gator vehicle
gator = gator.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.CoordSysD(vehicle_pos, vehicle_rot))
gator.SetTireType(veh.TMeasyTire)
gator.SetTireStepSize(0.01)
gator.Initialize()

# Add mesh visualization for all components
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)

gator.GetChassisBody().AddVisualModel(gator.GetChassisMesh())
for wheel in gator.GetWheels():
    wheel.AddVisualModel(wheel.GetMesh())

# =============================================================================
# Create the driver system
# =============================================================================

driver = veh.ChInteractiveDriverIRR(sys, gator)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.05)
driver.Initialize()

# =============================================================================
# Create the visualization system
# =============================================================================

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gator Vehicle Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(5, 5, 3), chrono.ChVectorD(0, 0, 0.5))
vis.SetCameraMovementType(chronoirr.CameraMovementType_FREE)

# =============================================================================
# Simulation loop
# =============================================================================

# Simulation parameters
step_size = 0.02  # 50 FPS
sim_time = 0
max_sim_time = 100

while vis.Run() and sim_time < max_sim_time:
    # Get current time
    time = sys.GetChTime()

    # Update driver inputs
    driver.Synchronize(time)
    driver.SetSteering(gator.GetSteeringAngle())
    driver.SetThrottle(gator.GetMotorFunctionType() == veh.ChFunction_Motor::MOT_FORWARD)
    driver.SetBraking(gator.GetMotorFunctionType() == veh.ChFunction_Motor::MOT_BACKWARD)

    # Update vehicle
    gator.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(step_size)
    sim_time += step_size

# Clean up
vis.RemoveAllVisualModels()