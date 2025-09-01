import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data/')  


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys,  
                              10, 10, 1,  
                              1000,  
                              True,  
                              True,  
                              chrono.ChVectorD(0, 0, -1))  
ground.SetBodyFixed(True)  


mass = chrono.ChBodyEasySphere(sys,  
                               0.5,  
                               1000,  
                               True,  
                               True,  
                               chrono.ChVectorD(0, 0, 2))  


link = chrono.ChLinkTSDA()
link.Initialize(mass, ground, True, chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 0, -1))
sys.Add(link)


link.SetSpringCoefficient(100.0)  
link.SetDampingCoefficient(10.0)  


app = chronoirr.ChIrrApp(sys, 'Mass-Spring-Damper System', chronoirr.dimension2du(800, 600))


app.AddTypicalSky()
app.AddTypicalLights()
app.AddCamera(chrono.ChVectorD(0, 5, -10))
app.AssetBindAll()
app.AssetUpdateAll()


app.SetTimestep(0.01)
app.SetTryRealtime(True)
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()