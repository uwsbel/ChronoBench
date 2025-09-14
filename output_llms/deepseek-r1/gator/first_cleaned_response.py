import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], ''))






gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.Initialize()


gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(100, 100, 0.2), 
                        chrono.ChVectorD(0, 0, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()






app = irr.ChIrrApp(system, "Gator Simulation", irr.dimension2du(1280, 720))
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 6, 1.5))
app.SetTimestep(0.001)


driver = veh.ChInteractiveDriverIRR(app.GetDevice())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()





app.AssetBindAll()
app.AssetUpdateAll()

realtime_timer = chrono.ChRealtimeStepTimer()
time_step = 0.001
frame_time = 1/50.0  
elapsed_time = 0.0

while app.GetDevice().run():
    time = system.GetChTime()
    
    
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.GetDevice().getGUIEnvironment().drawAll()
    
    
    driver_inputs = driver.GetInputs()
    gator.DriverInputs(driver_inputs)
    
    
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("", driver_inputs)
    
    
    app.DoStepDynamics(time_step)
    
    
    elapsed_time += time_step
    realtime_timer.Spin(time_step)
    
    
    if elapsed_time >= frame_time:
        app.EndScene()
        elapsed_time = 0.0


app.GetDevice().closeDevice()