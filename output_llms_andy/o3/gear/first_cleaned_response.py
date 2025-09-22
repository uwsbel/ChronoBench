import pychrono as chrono
import pychrono.irrlicht as irr
import math




chrono.SetChronoDataPath( chrono.GetChronoDataPath() )   

sys = chrono.ChSystemNSC()
sys.Set_G_acc( chrono.ChVectorD(0, 0, -9.81) )


mat = chrono.ChMaterialSurfaceNSC()
mat.SetFriction(0.3)






truss = chrono.ChBodyEasyBox(0.20, 0.20, 0.20,      
                             1000,                  
                             True, True, mat)       
truss.SetBodyFixed(True)
truss.SetPos( chrono.ChVectorD(0,0,0) )
sys.Add(truss)


bar_len   = 0.60
bar_width = 0.05
bar = chrono.ChBodyEasyBox(bar_len, bar_width, bar_width,
                           800, True, True, mat)

bar.SetPos( chrono.ChVectorD(bar_len*0.5, 0, 0) )
sys.Add(bar)


rev_ground_bar = chrono.ChLinkLockRevolute()
rev_ground_bar.Initialize(truss, bar,
        chrono.ChCoordsysD( chrono.ChVectorD(0,0,0),
                            chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,1,0)) ))
sys.Add(rev_ground_bar)


r_sun   = 0.10
thick   = 0.05
sun = chrono.ChBodyEasyCylinder(r_sun, thick,
                                7800, True, True, mat)
sun.SetPos( chrono.ChVectorD(0,0,0) )
sys.Add(sun)


motor_sun = chrono.ChLinkMotorRotationSpeed()
motor_sun.Initialize(sun, truss,
        chrono.ChFrameD( chrono.ChVectorD(0,0,0), chrono.QUNIT ) )

const_speed = chrono.ChFunction_Const( 2*math.pi )   
motor_sun.SetSpeedFunction(const_speed)
sys.Add(motor_sun)


r_planet = 0.20
planet   = chrono.ChBodyEasyCylinder(r_planet, thick,
                                     7800, True, True, mat)

planet_center = chrono.ChVectorD( r_sun + r_planet, 0, 0 )   
planet.SetPos( planet_center )
sys.Add(planet)



rev_bar_planet = chrono.ChLinkLockRevolute()
rev_bar_planet.Initialize(bar, planet,
        chrono.ChCoordsysD( planet_center, chrono.QUNIT ))
sys.Add(rev_bar_planet)



ratio = - r_sun / r_planet
gear_link = chrono.ChLinkGear()

gear_frame = chrono.ChFrameD( chrono.ChVectorD(0,0,0), chrono.QUNIT )
gear_link.Initialize(sun, planet,
                     gear_frame,                   
                     chrono.ChVectorD(0,0,1),      
                     chrono.ChVectorD(1,0,0),      
                     ratio,                        
                     False)                        
sys.Add(gear_link)




app = irr.ChIrrApp(sys, "Epicyclic Gear Set", irr.dimension2du(1024,768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera( irr.vector3df(0.6,0.6,0.6), irr.vector3df(0,0,0) )
app.AddTypicalLights( irr.vector3df( 3, 3, 3 ),
                      irr.vector3df(-3,-3, 5 ), 120.0, 1.0 )


app.AssetBindAll()
app.AssetUpdateAll()




app.SetTimestep(1e-3)

while(app.GetDevice().run()):
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()