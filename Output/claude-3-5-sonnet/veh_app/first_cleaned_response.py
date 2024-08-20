import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import math


system = chrono.ChSystemNSC()


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(0, 0, 0), 
                         chrono.ChVectorD(0, 0, 1),
                         200, 200)
terrain.Initialize()


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(1e-3)
gator.Initialize()

gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_NONE)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChIrrGuiDriver(gator.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


app = veh.ChWheeledVehicleIrrApp(gator.GetVehicle(), 'Gator Demo')
app.SetSkyBox()
app.AddTypicalLights(irr.vector3df(+130, +130, 130), irr.vector3df(-130, +130, 130), 120, 120, irr.SColorf(0.7, 0.7, 0.7, 1.0), irr.SColorf(0.3, 0.3, 0.3, 1.0))
app.AddTypicalLogo()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
app.SetTimestep(1e-3)
app.AssetBindAll()
app.AssetUpdateAll()


manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChVectorF(intensity, intensity, intensity), 500.0)


offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),  
    30.0,  
    offset_pose,  
    1280,  
    720,  
    3.14159 / 3  
)
cam.SetName("Camera Sensor")
cam.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(cam)


while app.GetDevice().run():
    
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(system.GetChTime())
    terrain.Synchronize(system.GetChTime())
    gator.Synchronize(system.GetChTime(), driver_inputs, terrain)
    app.Synchronize("", driver_inputs)

    
    driver.Advance(1e-3)
    terrain.Advance(1e-3)
    gator.Advance(1e-3)
    app.Advance(1e-3)
    manager.Update()

    
    system.DoStepDynamics(1e-3)

    
    camera_data = cam.GetMostRecentRGBA8Buffer()
    if camera_data.HasData():
        img = camera_data.GetRGBA8Data()
        

    
    if not app.GetDevice().run():
        break


app.GetDevice().drop()