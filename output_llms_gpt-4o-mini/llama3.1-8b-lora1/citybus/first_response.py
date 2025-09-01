import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # --------------
    # Create the various modules
    # --------------

    # Create the vehicle, set parameters, and initialize
    vehicle = veh.CityBus()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False) 
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.75), chrono.QUNIT))
    vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH);
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES);
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES);
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH);

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    if (veh.HasOFFAPI()):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch = terrain.AddPatch(patch_mat, 
                                 chrono.ChCoordsysd(chrono.VNULL, chrono.QUNIT), 
                                 200., 100.)
        patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('CityBus')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1 for opposite side)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(10 * steering_time)
    driver.SetThrottleDelta(10 * throttle_time)
    driver.SetBrakingDelta(10 * braking_time)
    driver.Initialize()

    # --------------
    # Simulation loop
    # --------------

    # Number of simulation steps between miscellaneous events
    step_number = 0

    while vis.Run() :
        # At each step, render the scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Take data from vehicle parts (e.g. chassis)
        # (this is where you would normally access vehicle data)

        # Perform step on the rest of the vehicle system (process input from driver, update modules, simulate)
        driver_inputs = driver.GetInputs()
        vehicle.Update(driver_inputs, step_number)
        terrain.Update()

        # Advance simulation for one timestep for all modules
        vehicle.GetSystem().DoStepDynamics(0.01)

        # Increment the step number
        step_number += 1

# This test driver is for CityBus vehicle
main()