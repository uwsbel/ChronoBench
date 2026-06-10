import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

# Simulation step sizes
step_size = 2e-3                                                      # main sim time step
tire_step_size = 1e-3                                                 # tire sub-step size
sim_end = 30.0                                                        # simulation end time

def main():
    # Create HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                # NSC for rigid terrain
    hmmwv.SetChassisFixed(False)                                      # MANDATORY - fixed chassis won't move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire model
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # report total vehicle mass

    # Create the terrain with multiple patches
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    # Patch 1: flat tile terrain, position changed from (-16, 0, 0) to (-20, 5, 0)
    patch1_mat = chrono.ChContactMaterialNSC()
    patch1_mat.SetFriction(0.9)
    patch1_mat.SetRestitution(0.01)
    patch1 = terrain.AddPatch(
        patch1_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(-20, 5, 0), chrono.QUNIT),  # updated position
        32,
        20
    )
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

    # Patch 2: flat concrete terrain, position changed from (16, 0, 0.15) to (20, -5, 0.2)
    patch2_mat = chrono.ChContactMaterialNSC()
    patch2_mat.SetFriction(0.9)
    patch2_mat.SetRestitution(0.01)
    patch2 = terrain.AddPatch(
        patch2_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(20, -5, 0.2), chrono.QUNIT),  # updated position
        32,
        30
    )
    patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
    patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

    # Patch 3: mesh bump terrain, position changed from (0, -42, 0) to (5, -45, 0)
    patch3_mat = chrono.ChContactMaterialNSC()
    patch3_mat.SetFriction(0.9)
    patch3_mat.SetRestitution(0.01)
    patch3 = terrain.AddPatch(
        patch3_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(5, -45, 0), chrono.QUNIT),  # updated position
        veh.GetDataFile("terrain/meshes/bump.obj")
    )
    patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
    patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

    # Patch 4: heightmap grass terrain, position changed from (0, 42, 0) to (10, 40, 0)
    patch4_mat = chrono.ChContactMaterialNSC()
    patch4_mat.SetFriction(0.9)
    patch4_mat.SetRestitution(0.01)
    patch4 = terrain.AddPatch(
        patch4_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(10, 40, 0), chrono.QUNIT),  # updated position
        veh.GetDataFile("terrain/height_maps/bump64.bmp"),
        64.0,
        64.0,
        0.0,
        3.0
    )
    patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)

    terrain.Initialize()

    # Create the vehicle Irrlicht visual system
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()                                          # vehicle demos use directional light
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    hmmwv.GetVehicle().EnableRealtime(True)

    render_fps = 50.0                                                  # target render rate
    render_every = max(1, round(1.0 / (render_fps * step_size)))      # render cadence (untagged)

    step_number = 0
    while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
        time = hmmwv.GetSystem().GetChTime()

        if step_number % render_every == 0:                           # throttled render
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1


    return 0


main()
