import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = irr.ChIrrApp(sys, 'ARTcar on Rigid Terrain', irr.dimension2du(1024, 768))
vis.SetCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


terrain = veh.RigidTerrain(sys)
material = chrono.ChMaterialSurfaceNSC()
patch = terrain.AddPatch(material, chrono.ChVectorD(0, -1, 0), chrono.ChCoordsysD(chrono.ChVectorD(0, -1, 0)))
patch.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 20, 20)  
terrain.Initialize()
sys.Add(terrain)


vehicle = veh.ARTcar(sys)
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1))  
vehicle.SetInitFwdVector(chrono.ChVectorD(1, 0, 0))  
vehicle.SetContactMethod(chrono.vehicle.ChContactMethod.NSC)  
vehicle.SetVisualizationType(veh.ChVisualizationType.MESH)
vehicle.Initialize()


vis.Add(vehicle.GetVehicle())


driver = veh.InteractiveDriver()
vehicle.SetDriver(driver)


vis.SetTimestep(0.02)  
vis.SetRealTimeFactor(1)  


while vis.Run():
    driver.Update()  
    sys.DoStepDynamics(vis.GetTimestep())  
    vis.BeginScene()
    vis.Render()
    vis.EndScene()