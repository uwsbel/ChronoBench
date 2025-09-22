import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import time


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())


system = chrono.ChSystemNSC()


initLoc = chrono.ChVectorD(0, 0, 1.0)  
initRot = chrono.QUNIT  
kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisFixed(False)
kraz.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
kraz.SetInitFwdVel(0.0)
kraz.Initialize()


terrain = veh.RigidTerrain(system)
terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetFriction(0.9)      
terrain_mat.SetRestitution(0.01)  

patch = terrain.AddPatch(
    terrain_mat,
    chrono.ChVectorD(0, 0, 0),  
    chrono.ChVectorD(0, 0, 1),  
    200, 200                    
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


driver = veh.ChDriverInputs()
driver_system = veh.ChIrrGuiDriver(kraz.GetVehicle())
driver_system.SetSteeringDelta(0.04)
driver_system.SetThrottleDelta(0.02)
driver_system.SetBrakingDelta(0.1)
driver_system.Initialize()


vis = veh.ChWheeledVehicleIrrApp(
    kraz.GetVehicle(),
    "Kraz Vehicle Demo",
    chronoirr.dimension2du(1024, 768)
)
vis.SetSkyBox()
vis.AddTypicalLights(chrono.ChVectorD(30, 30, 100), chrono.ChVectorD(-30, -30, 100), 250, 130)
vis.AddTypicalLogo()
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.SetTimestep(0.01)
vis.AssetBindAll()
vis.AssetUpdateAll()


step_size = 0.01
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    vis.BeginScene()
    vis.DrawAll()

    
    driver_inputs = driver_system.GetInputs()

    
    time = kraz.GetVehicle().GetChTime()
    driver_system.Synchronize(time)
    kraz.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize("Kraz Vehicle Demo", driver_inputs)

    
    driver_system.Advance(step_size)
    kraz.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    vis.EndScene()
    realtime_timer.Spin(step_size)

    
    
    


vis.GetDevice().closeDevice()