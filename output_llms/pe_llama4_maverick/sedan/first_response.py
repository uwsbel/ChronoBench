import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# ==============================================================================

def main():
    # Create the vehicle system
    vehicle = veh.WheeledVehicle(chrono.GetChronoDataFile('vehicle/BMW_E90/BMW_E90.json'), chrono.ChContactMethod_NSC)
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QuatFromAngleAxis(chrono.CH_C_PI / 2, chrono.ChVector3d(0, 1, 0))))
    vehicle.Initialize()

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
    patch.SetTexture(chrono.GetChronoDataFile('textures/dirt.jpg'), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('BMW E90 Sedan Simulation')
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    # Create the interactive driver system
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # Simulation loop
    while vis.Run():
        time = vehicle.GetSystem().GetChTime()

        # Driver inputs
        driver_inputs = driver.GetInputs()
        vehicle.Advance(time)
        terrain.Advance(time)
        vis.Advance(time)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    return 0

if __name__ == "__main__":
    main()