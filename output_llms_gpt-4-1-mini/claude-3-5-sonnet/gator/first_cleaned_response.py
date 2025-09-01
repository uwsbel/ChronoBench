import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-math.pi/2)),
                         200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()


vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)
driver.Initialize()


app = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), 'Gator Demo')

app.SetSkyBox()
app.AddTypicalLights()
app.AddCamera(chrono.ChVectorD(6, 3, -6), chrono.ChVectorD(0, 0, 0))
app.SetTimestep(1e-3)


realtime_timer = chrono.ChRealtimeStepTimer()
while app.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("", driver_inputs)

    
    driver.Advance(1e-3)
    terrain.Advance(1e-3)
    vehicle.Advance(1e-3)
    app.Advance(1e-3)

    
    realtime_timer.Spin(1e-3)

    
    if vehicle.GetVehicle().GetChassisBody().GetPos().y() < -0.5:
        break


app.GetDevice().drop()