import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
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

    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.AddPatch(patch_mat,
                    chrono.ChCoordsysD(chrono.ChVectorD(0, -1, 0)),
                    100.0, 100.0)  
    terrain.Initialize()
    patch = terrain.GetGroundPatch()
    patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)

    
    vehicle = veh.MAN13()  
    vehicle.SetChassisCollisionType(veh.ChassisCollisionType.BOX)
    vehicle.SetTireType(veh.TireModelType.TMEASY)  
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0.5),
                                              chrono.Q_from_Euler123(chrono.ChVectorD(0, 0, 0))))
    vehicle.Initialize()

    
    driver = veh.ChDriver()
    vehicle.SetDriver(driver)

    
    time_step = 0.005
    while vis.Run():
        
        key = vis.GetKeyboard()
        driver.SetThrottle(key.KeyIsPressed(chronoirr.KEY_KEY_UP))
        driver.SetBrake(key.KeyIsPressed(chronoirr.KEY_KEY_DOWN))
        steering = 0.0
        if key.KeyIsPressed(chronoirr.KEY_KEY_LEFT):
            steering -= 1.0
        if key.KeyIsPressed(chronoirr.KEY_KEY_RIGHT):
            steering += 1.0
        driver.SetSteering(steering)

        
        chassis = vehicle.GetChassis()
        cam = vis.GetCamera()
        cam.SetPos(chassis.GetPos() + chrono.ChVectorD(0, 5, 10))
        cam.LookAt(chassis.GetPos())

        
        system.DoStepDynamics(time_step)

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == '__main__':
    main()