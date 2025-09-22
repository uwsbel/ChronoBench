import pychrono as chrono
import pychrono.postprocess as postprocess


chrono.SetChronoDataPath("./")  
app = chrono.ChApp()


terrain = chrono.ChBodyEasyBox(10, 10, 1, 1000)
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetBodyFixed(True)
app.Add(terrain)


gator = chrono.ChBodyEasyCylinder(0.5, 1, 1000)
gator.SetPos(chrono.ChVectorD(0, 0, 1))
gator.SetBodyFixed(False)
app.Add(gator)




wheel = chrono.ChBodyEasySphere(0.2, 100)
wheel.SetPos(chrono.ChVectorD(0.3, 0, 0))
wheel.SetBodyFixed(False)
app.Add(wheel)


chassis = chrono.ChBodyEasyBox(1, 1, 0.5, 1000)
chassis.SetPos(chrono.ChVectorD(0, 0, 0.5))
chassis.SetBodyFixed(False)
app.Add(chassis)





sensor_manager = chrono.ChSensorManager()
point_light = chrono.ChPointLight()
point_light.SetPos(chrono.ChVectorD(1, 2, 3))
sensor_manager.Add(point_light)

camera = chrono.ChCamera()
camera.SetPos(chrono.ChVectorD(0, 0, 2))
camera.SetTarget(chrono.ChVectorD(0, 0, 0))
sensor_manager.Add(camera)


while app.GetContinue():
    app.GetSystem().DoStepDynamics(0.01)  
    app.Render()  
    app.Sync()  


postprocess.SaveState(app.GetSystem(), "gator_simulation.xml")

app.Close()