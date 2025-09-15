import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# New class to manage SCM terrain parameters
class SCMTerrainParameters:
    def __init__(self, configuration="mid"):
        # Define different configurations
        self.configurations = {
            "soft": {
                "Kphi": 1e6,     # Bekker Kphi
                "Kc": 0,        # Bekker Kc
                "n": 1.0,       # Bekker n exponent
                "cohesive": 0,  # Mohr cohesive limit (Pa)
                "friction": 30, # Mohr friction limit (degrees)
                "shear": 0.01,  # Janosi shear coefficient (m)
                "stiffness": 1e8, # Elastic stiffness (Pa/m)
                "damping": 1e4   # Damping (Pa s/m)
            },
            "mid": {
                "Kphi": 2e6,
                "Kc": 0,
                "n": 1.1,
                "cohesive": 0,
                "friction": 30,
                "shear": 0.01,
                "stiffness": 2e8,
                "damping": 3e4
            },
            "hard": {
                "Kphi": 5e6,
                "Kc": 0,
                "n": 1.2,
                "cohesive": 0,
                "friction": 30,
                "shear": 0.01,
                "stiffness": 5e8,
                "damping": 5e4
            }
        }
        
        # Set parameters based on selected configuration
        if configuration in self.configurations:
            params = self.configurations[configuration]
            self.Kphi = params["Kphi"]
            self.Kc = params["Kc"]
            self.n = params["n"]
            self.cohesive = params["cohesive"]
            self.friction = params["friction"]
            self.shear = params["shear"]
            self.stiffness = params["stiffness"]
            self.damping = params["damping"]
        else:
            raise ValueError("Invalid terrain configuration. Choose 'soft', 'mid', or 'hard'.")

def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

    # Initial vehicle location and orientation
    initLoc = chrono.ChVector3d(-8, 0, 0.6)
    initRot = chrono.ChQuaterniond(1, 0, 0, 0)

    # Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
    vis_type = veh.VisualizationType_MESH

    # Collision type for chassis (PRIMITIVES, MESH, or NONE)
    chassis_collision_type = veh.CollisionType_NONE

    # Type of tire model (RIGID, TMEASY)
    tire_model = veh.TireModelType_RIGID

    # Rigid terrain
    terrainHeight = 0      # terrain height
    terrainLength = 100.0  # size in X direction
    terrainWidth = 100.0   # size in Y direction

    # Poon chassis tracked by the camera
    trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

    # Contact method
    contact_method = chrono.ChContactMethod_SMC
    contact_vis = False

    # Simulation step sizes
    step_size = 1e-3
    tire_step_size = step_size

    # Time interval between two render frames
    render_step_size = 1.0 / 50  # FPS = 50

    # Create the HMMWV vehicle, set parameters, and initialize
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(contact_method)
    vehicle.SetChassisCollisionType(chassis_collision_type)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    vehicle.SetTireType(tire_model)
    vehicle.SetTireStepSize(tire_step_size)

    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(vis_type)
    vehicle.SetSuspensionVisualizationType(vis_type)
    vehicle.SetSteeringVisualizationType(vis_type)
    vehicle.SetWheelVisualizationType(vis_type)
    vehicle.SetTireVisualizationType(vis_type)

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create and initialize terrain with new parameter class
    terrain_params = SCMTerrainParameters("mid")  # Choose configuration: "soft", "mid", or "hard"
    terrain = veh.SCMTerrain(vehicle.GetSystem())
    terrain.SetSoilParameters(
        terrain_params.Kphi,
        terrain_params.Kc,
        terrain_params.n,
        terrain_params.cohesive,
        terrain_params.friction,
        terrain_params.shear,
        terrain_params.stiffness,
        terrain_params.damping
    )

    # Optionally, enable moving patch feature (single patch around vehicle chassis)
    terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

    # Set plot type for SCM (false color plotting)
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

    # Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
    terrain.Initialize(20, 20, 0.02)

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())

    # Create the driver system
    driver = veh.ChInteractiveDriverIRR(vis)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # ---------------
    # Simulation loop
    # ---------------

    # output vehicle mass
    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

    # Number of simulation steps between miscellaneous events
    render_steps = math.ceil(render_step_size / step_size)

    # Initialize simulation frame counter s
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    render_frame = 0

    while vis.Run():
        time = vehicle.GetSystem().GetChTime()

        # Render scene and output POV-Ray data
        if (step_number % render_steps == 0):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        # Increment frame number
        step_number += 1

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()

terrain_params = SCMTerrainParameters("soft")  # For soft terrain
# or
terrain_params = SCMTerrainParameters("hard")  # For hard terrain