import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystem()


ua_zbus = veh.UAZBUS()
ua_zbus.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
ua_zbus.AddDrivers()
ua_zbus.AddTires()
ua_zbus.AddRigidBodies()
ua_zbus.AddMarkers()
sys.Add(ua_zbus)


terrain = chrono.ChBodyEasyBox(sys, 10, 10, 0.1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterial().SetFriction(0.9)
terrain.GetMaterial().SetRestitution(0.1)
sys.Add(terrain)


driver = veh.ChIrrlichtDriver()
driver.Initialize(ua_zbus)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()
vis.SetWindowSize(1024, 768)
vis.SetCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))
vis.SetLogo("")


time = 0
dt = 0.01
while time < 10:
    
    ua_zbus.Synchronize(dt)
    terrain.Synchronize(dt)
    driver.Synchronize(dt)
    vis.Synchronize(dt)
    
    
    sys.DoStepDynamics(dt)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    time += dt


sys.Delete()
vis.Delete()