import pychrono as chrono
import pychrono.irrlicht as irr
import math
import os




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', 'chrono_data/'))








print("Copyright (c) 2017 projectchrono.org")


system = chrono.ChSystemNSC() 


system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
system.SetSolverMaxIterations(150)
system.SetTimestep(0.005) 





crank_radius = 0.1  
conrod_length = 0.25 
piston_mass = 0.5    
crank_mass = 0.2     
conrod_mass = 0.3    
element_thickness = 0.02 


motor_speed_rpm = 60.0 
motor_speed_rad_s = motor_speed_rpm * (2 * math.pi) / 60.0


initial_crank_angle = 0.0 







floor = chrono.ChBodyEasyBox(1.0, 0.1, 0.5, 1000, True, True, None) 
floor.SetPos(chrono.ChVector3d(0, -0.05, 0)) 
floor.SetBodyFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
system.Add(floor)


crank_pos_x = 0
crank_pos_y = 0 
crank_pos_z = 0.0 

crankshaft = chrono.ChBodyEasyBox(crank_radius * 2, element_thickness, element_thickness, 1000, True, True, None)
crankshaft.SetMass(crank_mass)




crank_rotation_point = chrono.ChVector3d(0, 0, crank_pos_z)
crankshaft.SetPos(crank_rotation_point + chrono.ChVector3d(crank_radius * math.cos(initial_crank_angle),
                                                          crank_radius * math.sin(initial_crank_angle),
                                                          0))
crankshaft.SetRot(chrono.Q_from_AngZ(initial_crank_angle)) 
crankshaft.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
crankshaft.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.3, 0.8)) 
system.Add(crankshaft)


conrod = chrono.ChBodyEasyBox(conrod_length, element_thickness * 0.8, element_thickness * 0.8, 1000, True, True, None)
conrod.SetMass(conrod_mass)


crank_pin_x = crank_rotation_point.x + crank_radius * math.cos(initial_crank_angle)
crank_pin_y = crank_rotation_point.y + crank_radius * math.sin(initial_crank_angle)
crank_pin_z = crank_pos_z






conrod_angle = math.asin(-crank_radius * math.sin(initial_crank_angle) / conrod_length)
conrod_cog_x = crank_pin_x + (conrod_length / 2) * math.cos(conrod_angle)
conrod_cog_y = crank_pin_y + (conrod_length / 2) * math.sin(conrod_angle)

conrod.SetPos(chrono.ChVector3d(conrod_cog_x, conrod_cog_y, crank_pos_z))
conrod.SetRot(chrono.Q_from_AngZ(conrod_angle))
conrod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/ลายไม้.jpg')) 
conrod.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.4, 0.2)) 
system.Add(conrod)


piston = chrono.ChBodyEasyBox(element_thickness * 2.5, element_thickness * 2.5, element_thickness * 2.5, 1000, True, True, None)
piston.SetMass(piston_mass)


piston_x = crank_pin_x + conrod_length * math.cos(conrod_angle)
piston_y = crank_rotation_point.y 
piston_z = crank_pos_z

piston.SetPos(chrono.ChVector3d(piston_x, piston_y, piston_z))
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/rock.jpg'))
piston.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.6)) 
system.Add(piston)








joint_crank_floor_pos = crank_rotation_point

joint_axis = chrono.ChVector3d(0, 0, 1)

rev_crank_floor = chrono.ChLinkRevolute()
rev_crank_floor.Initialize(crankshaft,         
                           floor,              
                           True,               
                           chrono.ChFrameD(joint_crank_floor_pos, chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X))) 
system.AddLink(rev_crank_floor)



joint_crank_conrod_pos = chrono.ChVector3d(crank_pin_x, crank_pin_y, crank_pin_z)

rev_crank_conrod = chrono.ChLinkRevolute()
rev_crank_conrod.Initialize(crankshaft,
                            conrod,
                            True,
                            chrono.ChFrameD(joint_crank_conrod_pos, chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X)))
system.AddLink(rev_crank_conrod)





joint_conrod_piston_pos = chrono.ChVector3d(piston_x, piston_y, piston_z) 

rev_conrod_piston = chrono.ChLinkRevolute()
rev_conrod_piston.Initialize(conrod,
                             piston,
                             True,
                             chrono.ChFrameD(joint_conrod_piston_pos, chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X)))
system.AddLink(rev_conrod_piston)




joint_piston_floor_pos = chrono.ChVector3d(piston_x, piston_y, piston_z)




prismatic_piston_floor = chrono.ChLinkPrismatic()
prismatic_piston_floor.Initialize(piston,
                                  floor,
                                  True, 
                                  chrono.ChFrameD(joint_piston_floor_pos, chrono.Q_from_AngY(math.pi/2)), 
                                  chrono.ChFrameD(joint_piston_floor_pos, chrono.Q_from_AngY(math.pi/2))) 
system.AddLink(prismatic_piston_floor)






motor_function = chrono.ChFunction_Const(motor_speed_rad_s) 




motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft,            
                 floor,                 
                 chrono.ChFrameD(joint_crank_floor_pos, chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X))) 
motor.SetSpeedFunction(motor_function)
system.AddLink(motor)





myapplication = irr.ChIrrApp(system, "Crank-Slider Mechanism", irr.dimension2du(1024, 768))
myapplication.SetTimestep(system.GetTimestep()) 
myapplication.SetTryRealtime(True)


myapplication.AddTypicalSky(chrono.GetChronoDataFile('skybox/'))


myapplication.GetSceneManager().addLight(
    None, 
    chrono.ChVector3d(30, 100, 30), 
    chrono.ChColor(0.8, 0.8, 0.8),  
    300.0 
)
myapplication.GetSceneManager().addLight(
    None, 
    chrono.ChVector3d(-30, 100, -30), 
    chrono.ChColor(0.6, 0.7, 0.8),  
    300.0 
)




camera = myapplication.GetSceneManager().getActiveCamera()
camera.setPosition(irr.vector3df(0.3, 0.4, 0.8)) 
camera.setTarget(irr.vector3df(0.15, 0, 0))     


myapplication.GetGUIEnvironment().addImage(
    myapplication.GetVideoDriver().getTexture(chrono.GetChronoDataFile('logo_pychrono_alpha.png')),
    irr.position2di(10, 10) 
)


myapplication.AssetBindAll()
myapplication.AssetUpdateAll()





simulation_time = 10.0 
time = 0

while myapplication.GetDevice().run():
    myapplication.BeginScene(True, True, irr.SColor(255, 140, 160, 190)) 
    myapplication.DrawAll()

    
    

    
    myapplication.DoStep() 

    myapplication.EndScene()

    time = system.GetChTime()
    if time > simulation_time:
        myapplication.GetDevice().closeDevice()

del myapplication 
print("Simulation finished.")