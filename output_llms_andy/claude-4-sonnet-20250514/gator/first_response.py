import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os

def main():
    # Set the Chrono data path (adjust this to your PyChrono installation)
    chrono.SetChronoDataPath("C:/Program Files/Chrono/data/")
    
    # Initialize the PyChrono environment
    print("Copyright (c) 2017 projectchrono.org")
    print("Chrono version: ", chrono.CHRONO_VERSION)

    # Create the Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # Set contact method
    contact_method = chrono.ChContactMethod_NSC
    contact_vis = False

    # Create the terrain
    terrain = veh.RigidTerrain(system)
    
    # Create a patch for the terrain
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    
    # Define terrain dimensions
    terrain_length = 200.0
    terrain_width = 200.0
    terrain_height = 0.1
    
    patch = terrain.AddPatch(patch_mat, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -terrain_height/2), 
                                            chrono.QUNIT),
                           terrain_length, 
                           terrain_width)
    
    # Set terrain texture (optional)
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    
    terrain.Initialize()

    # Vehicle initial location and orientation
    initLoc = chrono.ChVectorD(0, 0, 0.5)
    initRot = chrono.ChQuaternionD(1, 0, 0, 0)

    # Create the Gator vehicle
    gator = veh.Gator()
    gator.SetContactMethod(contact_method)
    gator.SetChassisCollisionType(veh.CollisionType_NONE)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.SetTireStepSize(1e-3)
    gator.Initialize()

    # Enable mesh visualization for all vehicle parts
    gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create the driver system
    driver = veh.ChInteractiveDriverIRR()

    # Set the time response for steering and throttle inputs
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    
    driver.SetSteeringDelta(1.0 / steering_time)
    driver.SetThrottleDelta(1.0 / throttle_time)
    driver.SetBrakingDelta(1.0 / braking_time)

    # Initialize the driver
    driver.Initialize()

    # Create the Irrlicht visualization system
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Gator Vehicle Simulation")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(gator.GetVehicle())

    # Simulation parameters
    step_size = 1e-3
    tire_step_size = step_size
    render_step_size = 1.0 / 50  # 50 FPS

    # Initialize simulation frame counters
    render_steps = int(math.ceil(render_step_size / step_size))

    # Initialize time
    time = 0
    render_frame = 0

    print("Vehicle mass: ", gator.GetVehicle().GetMass())

    # Simulation loop
    while vis.Run():
        time = gator.GetSystem().GetChTime()

        # Render scene and output POV-Ray data
        if render_frame % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        vis.Advance(step_size)

        # Increment frame number
        render_frame += 1

if __name__ == "__main__":
    main()