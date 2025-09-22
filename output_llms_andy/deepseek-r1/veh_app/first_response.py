#!/usr/bin/env python
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

def main():
    # ==========================================================================
    # Initialize PyChrono environment
    # ==========================================================================
    chrono.SetChronoDataPath("C:/path/to/chrono/data/")  # Set your actual data path
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # ==========================================================================
    # Create terrain
    # ==========================================================================
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # ==========================================================================
    # Create and initialize Gator vehicle
    # ==========================================================================
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.SetTireStepSize(0.001)
    gator.Initialize()

    # Set visualization modes for vehicle parts
    gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # ==========================================================================
    # Create interactive driver
    # ==========================================================================
    driver = veh.InteractiveDriver(gator.GetVehicle())
    driver.Initialize()

    # ==========================================================================
    # Create sensor manager and attach sensors
    # ==========================================================================
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChColorF(1.0, 1.0, 1.0), 5000)

    # Camera sensor attached to chassis
    camera = sens.ChCameraSensor(
        gator.GetChassisBody(),  # parent body
        10,                      # update rate (Hz)
        chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0.5), chrono.Q_from_AngZ(0)),  # offset pose
        1280,                    # image width
        720,                     # image height
        1.0                      # fov (radians)
    )
    camera.SetName("Vehicle Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())  # For accessing images
    manager.AddSensor(camera)

    # Point light attached to vehicle
    point_light = sens.ChPointLightSensor(
        gator.GetChassisBody(),  # parent body
        10,                      # update rate (Hz)
        chrono.ChFrameD(chrono.ChVectorD(-0.5, 0, 0.5), chrono.QUNIT),  # offset pose
        50.0                     # maximum range
    )
    manager.AddSensor(point_light)

    # ==========================================================================
    # Simulation loop
    # ==========================================================================
    step_size = 0.001
    realtime_step = True
    time = 0

    while time < 100:  # Simulate for 100 seconds
        time = system.GetChTime()

        # Update interactive driver
        driver.Synchronize(time)

        # Update terrain
        terrain.Synchronize(time)

        # Update vehicle systems
        gator.Synchronize(time, driver.GetInputs(), terrain)

        # Update sensor manager
        manager.Update()

        # Advance simulation
        gator.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)

        # Optional: Render camera images
        if camera.GetMostRecentRGBA8Buffer().HasData():
            img = camera.GetMostRecentRGBA8Buffer().GetRGBA8Data()
            # Process image data here if needed

if __name__ == "__main__":
    main()