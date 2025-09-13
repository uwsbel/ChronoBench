import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")


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


rover = chrono.ChBodyEasyBox(sys,  
                             2, 1, 1,  
                             1000,  
                             True,  
                             True,  
                             chrono.ChMaterialSurfaceNSC()  
                             )
rover.SetPos(chrono.ChVectorD(0, 0, 1.5))  
sys.Add(rover)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rover,  
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),  
                                 chrono.QUNIT)  
                 )
sys.Add(motor)


app = chronoirr.ChIrrApp(sys,  
                         "Curiosity Rover Simulation",  
                         chronoirr.dimension2du(800, 600),  
                         chronoirr.VerticalFlip,  
                         chronoirr.SkyBox,  
                         chronoirr.SkyLight  
                         )


app.AddCamera(chronoirr.vector3df(0, 0, 5),  
              chronoirr.vector3df(0, 0, 0))  
app.AddTypicalLights()  
app.AddLightWithShadow(chronoirr.vector3df(2, 4, 2),  
                       chronoirr.vector3df(0, 0, 0),  
                       3,  
                       2,  
                       10,  
                       512,  
                       1000)  


app.AssetBindAll()
app.AssetUpdateAll()


sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  
sys.SetStep(0.01)  
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)  


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()