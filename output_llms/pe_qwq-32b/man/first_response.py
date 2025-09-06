import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    # Initialize simulation system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("MAN 10t Truck Simulation")
    vis.SetSymbolscale(0.01)
    vis.SetAntialiasing(True)
    vis.AttachSystem(system)
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.Initialize()
    vis.AddCamera(chronoirr.ChVectorD(0, 5, 30))
    vis.GetCamera().LookAt(chronoirr.ChVectorD(0, 0, 0))

    # Terrain creation
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.AddPatch(patch_mat,
                    chrono.ChCoordsysD(chrono.ChVectorD(0, -1, 0)),
                    100.0, 100.0)  # Size X and Z
    terrain.Initialize()
    patch = terrain.GetGroundPatch()
    patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)

    # Vehicle creation
    vehicle = veh.MAN13()  # Assuming MAN13 is the MAN 10t model
    vehicle.SetChassisCollisionType(veh.ChassisCollisionType.BOX)
    vehicle.SetTireType(veh.TireModelType.TMEASY)  # TMEASY tire model
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0.5),
                                              chrono.Q_from_Euler123(chrono.ChVectorD(0, 0, 0))))
    vehicle.Initialize()

    # Driver setup
    driver = veh.ChDriver()
    vehicle.SetDriver(driver)

    # Simulation loop
    time_step = 0.005
    while vis.Run():
        # Update driver inputs from keyboard
        key = vis.GetKeyboard()
        driver.SetThrottle(key.KeyIsPressed(chronoirr.KEY_KEY_UP))
        driver.SetBrake(key.KeyIsPressed(chronoirr.KEY_KEY_DOWN))
        steering = 0.0
        if key.KeyIsPressed(chronoirr.KEY_KEY_LEFT):
            steering -= 1.0
        if key.KeyIsPressed(chronoirr.KEY_KEY_RIGHT):
            steering += 1.0
        driver.SetSteering(steering)

        # Update camera to chase the vehicle
        chassis = vehicle.GetChassis()
        cam = vis.GetCamera()
        cam.SetPos(chassis.GetPos() + chrono.ChVectorD(0, 5, 10))
        cam.LookAt(chassis.GetPos())

        # Advance simulation step
        system.DoStepDynamics(time_step)

        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == '__main__':
    main()