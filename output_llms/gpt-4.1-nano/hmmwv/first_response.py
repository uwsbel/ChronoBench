import pychrono.core as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import time

def main():
    # ----------------------------------------------
    # Initialize the Chrono system
    # ----------------------------------------------
    chrono.ChSystemNS.equalsleep = True  # Enable sleeping for better performance
    
    # ----------------------------------------------
    # Create the physical system
    # ----------------------------------------------
    sys = chrono.ChSystemNS()
    
    # ----------------------------------------------
    # Create a flat rigid terrain
    # ----------------------------------------------
    terrain = vehicle.RigidTerrain(sys)
    # Set terrain dimensions: width, length, height
    terrain_dim_x = 50  # meters
    terrain_dim_y = 50  # meters
    terrain_dim_z = 2   # height of the terrain mesh
    
    # Add a flat terrain with a texture
    terrain.Initialize(terrain_dim_x, terrain_dim_y, 0.3)
    # Apply texture
    texture = chrono.ChTexture()
    texture.SetTextureFilename('textures/concrete.jpg')  # Ensure this texture exists or replace
    terrain.GetGroundMaterial().AddTexture(texture)
    
    # ----------------------------------------------
    # Create the vehicle
    # ----------------------------------------------
    # Load the HMMWV prototype
    # Using the Chrono provided vehicle model: vehicle.HMMWV_Vehicle
    vehicle_left_location = chrono.ChVectorD(0, 0, 1.2)  # Starting position
    vehicle_heading = chrono.Q_from_Euler123(0, 0, 0)     # No initial rotation
    
    # Create the vehicle with specified parameters
    my_vehicle = vehicle.HMMWV_Vehicle(
        sys,
        True,  # init_vehicle, to create default vehicle
        vehicle_left_location,
        vehicle_heading,
        vehicle.CylinderTubeType.Primitive,  # Primitive visualization
        vehicle.TireModel.TMEASY,
        False,
        0.0
    )
    my_vehicle.SetContactMethod(chrono.ChContactMethod.SAT)  # Contact method
    my_vehicle.GetChassis().SetName("HMMWV_Chassis")
    my_vehicle.GetTireModelType()  # TMEASY tires are default
    
    # Initialize the vehicle
    my_vehicle.Initialize()
    # Optionally, set tire pressure or other parameters here
    
    # ----------------------------------------------
    # Create the Irrlicht visualization
    # ----------------------------------------------
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowTitle('PyChrono HMMWV on Flat Terrain')
    vis.SetWindowSize(1280, 720)
    vis.SetSymbolscale(1.0)
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    
    # ----------------------------------------------
    # Create an interactive driver (for keyboard control)
    # ----------------------------------------------
    driver = vehicle.ChIrrGuiDriver(vis)
    driver.Initialize()
    driver.SetInputMode(vehicle.ChIrrGuiDriver.KEYBOARD)
    driver.SetChaseCamera(track_point=my_vehicle.GetChassis())
    
    # ----------------------------------------------
    # Simulation loop parameters
    # ----------------------------------------------
    fps = 50  # frames per second
    wait_time = 1.0 / fps
    
    # Run the simulation
    while vis.Run():
        start_time = time.time()
        
        # Update driver inputs
        driver.Interact()
        # Apply driver controls to the vehicle
        my_vehicle.GetDriver().SubmitDrivingInput(
            driver.GetThrottle(),
            driver.GetSteering(),
            driver.GetBraking()
        )
        
        # Step the system
        sys.DoStepDynamics(wait_time)
        
        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Maintain real-time
        elapsed = time.time() - start_time
        sleep_time = wait_time - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == "__main__":
    main()