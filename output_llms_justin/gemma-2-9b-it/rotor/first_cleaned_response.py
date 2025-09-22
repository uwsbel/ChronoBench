import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("..")  


world = chrono.World()


beam_params = chrono.ChBodyEasyBox(0.1, 0.05, 0.01, 1000)  
beam_params.SetPos(chrono.ChVectorD(0, 0, 0))  


beam = chrono.ChBeamIga(beam_params, 
                       chrono.ChVectorD(0, 0, 0),  
                       chrono.ChVectorD(1, 0, 0),  
                       10,  
                       10,  
                       chrono.ChVectorD(0, 0, 0),  
                       chrono.ChVectorD(0, 0, 0))  

world.Add(beam)


flywheel_params = chrono.ChBodyEasySphere(0.05, 1000)  
flywheel_params.SetPos(chrono.ChVectorD(0.5, 0, 0))  

flywheel = chrono.ChBodyEasy(flywheel_params)
beam.AddChild(flywheel)


motor_params = chrono.ChBodyEasyCylinder(0.02, 0.1, 1000)  
motor_params.SetPos(chrono.ChVectorD(0, 0, 0))  

motor = chrono.ChBodyEasy(motor_params)
motor.SetPos(chrono.ChVectorD(0, 0, 0))  
beam.AddChild(motor)


motor_joint = chrono.ChLinkLockPrismatic(motor, beam,
                                        chrono.ChVectorD(0, 0, 0),
                                        chrono.ChVectorD(1, 0, 0))
motor_joint.SetPos(chrono.ChVectorD(0, 0, 0))
world.Add(motor_joint)


world.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  
world.Set_Speed(1000)  


vis = chronoirr.ChIrrApp(world, 'Jeffcott Rotor Simulation', chronoirr.vector3df(800, 600, 0))


beam.AddVisualization(vis.GetSceneNode())


vis.SetFullscreen(True)
vis.AssetBindAll()
vis.Start()
chrono.ChSystem.Run(10)