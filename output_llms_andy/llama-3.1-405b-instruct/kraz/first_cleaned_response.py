import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('/path/to/chrono/data/')


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys,  
                              100, 100, 2,  
                              1000,  
                              True,  
                              True,  
                              chrono.ChMaterialSurfaceNSC()  
                              )
ground.SetBodyFixed(True)
sys.Add(ground)


vehicle = veh.ChPart_Kraz(sys)
vehicle.SetChassisPosition(chrono.ChVectorD(0, 0, 1.5))
vehicle.SetChassisRotation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.Initialize()


driver = veh.ChIrrNodeDriver(vehicle)
driver.Initialize()


app = irr.ChIrrApp(sys, 'PyChrono Simulation', irr.dimension2du(800, 600))
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalCamera(chrono.ChVectorD(0, 0, 1.5),  
                     chrono.ChVectorD(0, 0, 0)  
                     )
app.SetSymbolscale(0.02)
app.SetShowInfos(True)


while app.GetDevice().run():
    
    sys.DoStepDynamics(0.01)
    vehicle.Synchronize(0.01)
    driver.Advance(0.01)
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()