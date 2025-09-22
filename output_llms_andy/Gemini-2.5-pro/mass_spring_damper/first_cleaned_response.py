import pychrono as chrono
import pychrono.irrlicht as chronoirr
import os




chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'chrono_data', ''))








system = chrono.ChSystemNSC()  


system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))




ground = chrono.ChBody()
ground.SetBodyFixed(True)  
ground.SetPos(chrono.ChVectorD(0, 0, 0)) 
system.Add(ground)


ground_marker = chrono.ChSphereShape(0.05)
ground_marker.SetColor(chrono.ChColor(0.6, 0.6, 0.6)) 
ground.AddVisualShape(ground_marker, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0))) 




mass_val = 1.0  
mass_radius = 0.2 
initial_mass_pos = chrono.ChVectorD(0, 1.5, 0) 

mass = chrono.ChBody()
mass.SetMass(mass_val)
mass.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01)) 
mass.SetPos(initial_mass_pos)
system.Add(mass)


mass_shape = chrono.ChSphereShape(mass_radius)
mass_shape.SetColor(chrono.ChColor(0.3, 0.4, 0.8)) 
mass.AddVisualShape(mass_shape)






pt1_mass_local = chrono.ChVectorD(0, 0, 0)



pt2_ground_local = chrono.ChVectorD(0, 0.5, 0)


spring_stiffness = 100.0  
damping_coefficient = 5.0  


rest_length = 0.7 


tsda = chrono.ChLinkTSDA()
tsda.Initialize(mass,              
                ground,            
                False,             
                pt1_mass_local,    
                pt2_ground_local)  

tsda.SetSpringCoefficient(spring_stiffness)
tsda.SetDampingCoefficient(damping_coefficient)
tsda.SetRestLength(rest_length)
system.Add(tsda)


spring_shape = chrono.ChSpringShape(0.03, 80, 15) 
spring_shape.SetColor(chrono.ChColor(0.8, 0.8, 0.2)) 
tsda.AddVisualShape(spring_shape)





myapplication = chronoirr.ChIrrApp(system, "Mass-Spring-Damper Demo", chronoirr.dimension2du(1024, 768))
myapplication.AddTypicalLights()
myapplication.AddTypicalSky()
myapplication.AddTypicalCamera(chronoirr.vector3df(1.5, 1.5, -2.5), 
                               chronoirr.vector3df(0, 0.8, 0))    


myapplication.AssetBindAll()
myapplication.AssetUpdateAll()




myapplication.SetTimestep(0.005) 
myapplication.SetTryRealtime(True)

print("Starting simulation... Press Esc to exit.")

while myapplication.GetDevice().run():
    myapplication.BeginScene(True, True, chronoirr.SColor(255, 140, 160, 190)) 
    myapplication.DrawAll()
    
    
    if system.GetChTime() % 0.1 < myapplication.GetTimestep(): 
        print(f"Time: {system.GetChTime():.2f} s, Mass Y: {mass.GetPos().y:.3f} m, Spring Length: {tsda.GetLength():.3f} m")

    myapplication.DoStep()
    myapplication.EndScene()

print("Simulation finished.")

import pychrono.tools.paths as p
    chrono.SetChronoDataPath(os.path.join(p.get_chrono_data_path(), ''))