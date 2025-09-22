import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

"""
**** Set this path before running the demo!
Chrono data directory (containing vehicle meshes, etc.)
"""
# \Ch_Opt is relative to the directory containing this file
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    

def main():
    # Create the Kraz vehicle chassis (sedan model)
    vehicle = veh.Kraz()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.10), chrono.ChQuaterniond(1, 0, 0, 0)))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeTSAR_4WD) # Kraz 4WD 
    vehicle.SetSteeringType(veh.SteeringTypeTSAR_PEDALS) # Kraz 2WS (only front wheels)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()

    # Set visualization types for vehicle parts
    vehicle.SetChassisVisibility(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisibility(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisibility(veh.VisualizationType_MESH)
    vehicle.SetWheelVisibility(veh.VisualizationType_MESH)
    vehicle.SetTireVisibility(veh.VisualizationType_MESH)

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch = terrain.AddPatch(patch_mat, 
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        40.0, 40.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tarmac.jpg"), 40, 40)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Kraz Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())


    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # output vehicle mass
    print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

    # Solver and integrator settings
    s = vehicle.GetSystem().GetSettings()
    s.solver.max_iteration_bilateral = 100

    # Set the time step for the simulation
    time_step = 1e-3
    time = 0

    # Initialize the simulation clock
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()
        # get driver inputs
        driver_inputs = driver.GetInputs()

        # update vehicle system
        vehicle.Synchronize(time, driver_inputs, terrain)
        vehicle.Advance(time_step)

        # update the visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Drive the simulation for the specified timestep
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        if not vis.Run():
            break
        vehicle.GetSystem().DoStepDynamics(time_step)
        # Increment the simulation time counter
        time += time_step

        # Ensure the simulation runs in real-time
        realtime_timer.Spin(time_step)
if __name__ == "__main__":
    main()