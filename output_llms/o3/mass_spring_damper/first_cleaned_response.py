import pychrono as chrono
import pychrono.irrlicht as irr




sys = chrono.ChSystemSMC()          
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


ground_visual = chrono.ChBodyEasyBox(3.0, 0.1, 3.0,          
                                     1000,                   
                                     True, True)             
ground_visual.SetBodyFixed(True)
ground_visual.SetPos(chrono.ChVectorD(0, -0.05, 0))
sys.Add(ground_visual)




mass          = 1.0                          
size          = 0.2                          
initial_pos   = chrono.ChVectorD(0, 0.5, 0)  

cube = chrono.ChBodyEasyBox(size, size, size,
                            mass,
                            True,  True)      
cube.SetPos(initial_pos)



Ixx = Iyy = Izz = (1/6) * mass * size**2
cube.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz))

sys.Add(cube)




k   = 100.0   
c   =   5.0   
rest_length = 0.40  


pos_ground = chrono.ChVectorD(0, 0.00, 0)                 
pos_cube   = chrono.ChVectorD(0, +size/2, 0)              

tsda = chrono.ChLinkTSDA()
tsda.Initialize(ground, cube, False, pos_ground, pos_cube)
tsda.SetSpringCoefficient(k)
tsda.SetDampingCoefficient(c)
tsda.SetRestLength(rest_length)



tsda.AddVisualShapeSpring(0.02, 20, chrono.ChColor(0.4, 0.4, 0.9))

sys.Add(tsda)




app = irr.ChIrrApp(sys,
                   "Single mass–spring–damper (ChLinkTSDA) demo",
                   irr.dimension2du(1024, 768))

app.AddTypicalLogo()
app.AddTypicalSky()


app.AddTypicalLights(chrono.ChVectorD(+2, 4, -2),
                     chrono.ChVectorD(-2, 4, +2), 9, 1)


app.AddCamera(chrono.ChVectorD(1.5, 1.0, 1.5),
              chrono.ChVectorD(0, 0.25, 0))


app.AssetBindAll()
app.AssetUpdateAll()




time_step = 1e-3
app.SetTimestep(time_step)

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()          
    app.EndScene()