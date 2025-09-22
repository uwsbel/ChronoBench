#!/usr/bin/env python3

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.rigidterrain as rigidterrain
import pychrono.irrlicht as irr
from pychrono_vehicle.driver import ChIrrGuiDriver

def main():
    # ------------------------------------------------------------------------
    # 1) INITIALIZE CHRONO ENGINE AND SET DATA PATHS
    # ------------------------------------------------------------------------
    # adjust this path to where your Chrono data (models, textures) live
    chrono.SetChronoDataPath("/usr/local/share/chrono/data/")  
    # vehicle.GetDataFile() will look underneath ChronoDataPath/vehicle
    # so e.g. your ART vehicle JSON should be in 
    # /usr/local/share/chrono/data/vehicle/ART/vehicle.json

    # create the Chrono system with non‐smooth contact
    system = chrono.ChSystemNSC()

    # ------------------------------------------------------------------------
    # 2) CREATE AND INITIALIZE THE ARTcar VEHICLE
    # ------------------------------------------------------------------------
    contact_method = chrono.ContactMethod_NSC

    # initial location & orientation of the chassis
    init_loc = chrono.ChVectorD(0, 0.04, 0)       # x, y, z
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)   # identity rotation

    # instantiate the ARTcar from a JSON specification
    vehicle_json = veh.GetDataFile("ART/vehicle.json")
    vehicle = veh.ArticulatedTrackedVehicle(vehicle_json, contact_method)

    # now initialize: attach to the Chrono system, give initial chassis pose
    vehicle.Initialize(system, chrono.ChCoordsysD(init_loc, init_rot))

    # set up visualization for chassis and wheels/tracks
    vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
    vehicle.SetTrackAssemblyVisualizationType(veh.VisualizationType.MESH)

    # ------------------------------------------------------------------------
    # 3) CREATE A RIGID TERRAIN
    # ------------------------------------------------------------------------
    terrain = rigidterrain.RigidTerrain(system)

    # create one flat patch 200 m × 100 m, zero thickness
    patch = terrain.AddPatch(
        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
        200,          # half‐length in X
        100,          # half‐length in Y
        0             # half‐thickness in Z
    )

    # apply a grass texture tiled over the patch
    texture_file = veh.GetDataFile("terrain/grass.jpg")
    patch.SetTexture(texture_file, 16, 16)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetContactFrictionCoefficient(0.9)

    terrain.Initialize()

    # ------------------------------------------------------------------------
    # 4) SET UP IRRLICHT VISUALIZATION
    # ------------------------------------------------------------------------
    app = irr.ChIrrApp(
        system,
        "ARTcar on Rigid Terrain",
        irr.dimension2du(1280, 720),
        irr.VerticalDir_Z
    )

    app.AddTypicalLogo()
    app.AddTypicalLights()
    app.AddTypicalCamera(
        irr.vector3df(-20, 10, 5),  # camera position
        irr.vector3df(0, 0, 0)      # look‐at point
    )

    # bind and update all assets (textures, meshes, etc)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # ------------------------------------------------------------------------
    # 5) CREATE AN INTERACTIVE DRIVER
    # ------------------------------------------------------------------------
    driver = ChIrrGuiDriver(app)
    # you can tune steering, throttle and braking response speeds if desired
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.05)
    driver.SetBrakingDelta(0.05)
    driver.Initialize()

    # ------------------------------------------------------------------------
    # 6) RUN THE SIMULATION LOOP AT ~50 FPS
    # ------------------------------------------------------------------------
    step_size     = 1e-3         # simulation step size
    render_fps    = 50.0
    render_step   = 1.0 / render_fps
    time          = 0.0
    next_render   = 0.0

    while app.GetDevice().run():
        time = system.GetChTime()

        # render scene only at the desired frame rate
        if time >= next_render:
            app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
            app.DrawAll()      # draw all Irrlicht scene nodes (vehicle, terrain, etc)
            app.EndScene()
            next_render += render_step

        # collect driver inputs
        driver.Synchronize(time)

        # synchronize all modules to the current time
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        app.Synchronize("", driver.GetInputs())

        # advance all modules by one time step
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        app.Advance(step_size)

        # note: ChVehicle::Advance() internally calls system.DoStepDynamics(step_size)

    # eof simulation loop

if __name__ == "__main__":
    main()