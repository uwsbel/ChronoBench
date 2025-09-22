#!/usr/bin/env python3
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # 1. Initialize PyChrono environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))

    # 2. Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -0.5), chrono.QUNIT), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
    terrain.Initialize()

    # 3. Create and initialize UAZ vehicle
    vehicle = veh.UAZBUS()
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize()

    # Set visualization mode and chassis color
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # 4. Create driver system
    driver = veh.InteractiveDriver(vehicle.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    # 5. Set up Irrlicht visualization
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('UAZBUS Simulation')
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddTypicalLights()
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AttachVehicle(vehicle.GetVehicle())

    # 6. Simulation loop
    step_size = 2e-3
    realtime_step = True

    while vis.Run():
        time = system.GetChTime()

        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)

        # Update vehicle systems
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        # Advance system state
        system.DoStepDynamics(step_size)
        
        # Maintain real-time synchronization
        if realtime_step:
            vis.SetRealtimeStep(step_size)

    return 0

if __name__ == "__main__":
    main()