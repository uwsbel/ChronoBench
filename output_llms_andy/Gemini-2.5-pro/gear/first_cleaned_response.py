import pychrono as chrono
import pychrono.irrlicht as irr
import math


chrono.SetChronoDataPath('./chrono_data/') 


system = chrono.ChSystemNSC() 


system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81)) 


sun_radius = 0.5  
planet_radius = 0.25 
gear_thickness = 0.1 
carrier_arm_length = sun_radius + planet_radius 
carrier_arm_visual_radius = 0.05 
motor_speed_rad_s = math.pi / 2  


mat_grey = chrono.ChVisualMaterial()
mat_grey.SetDiffuseColor(chrono.ChColor(0.3, 0.3, 0.3))
mat_grey.SetSpecularColor(chrono.ChColor(0.5,0.5,0.5))

mat_blue = chrono.ChVisualMaterial()
mat_blue.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.8))
mat_blue.SetSpecularColor(chrono.ChColor(0.5,0.5,0.8))

mat_green = chrono.ChVisualMaterial()
mat_green.SetDiffuseColor(chrono.ChColor(0.2, 0.8, 0.2))
mat_green.SetSpecularColor(chrono.ChColor(0.5,0.8,0.5))

mat_red = chrono.ChVisualMaterial()
mat_red.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))
mat_red.SetSpecularColor(chrono.ChColor(0.8,0.5,0.5))



def create_gear_body(name, radius, thickness, position, material, is_fixed=False):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 
                                     radius, thickness, 1000, 
                                     True, True) 
    body.SetName(name)
    body.SetPos(position)
    
    body.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
    if is_fixed:
        body.SetFixed(True)
    body.GetVisualShape(0).SetMaterial(0, material)
    system.Add(body)
    return body






truss = chrono.ChBodyEasyBox(0.1, 0.1, 0.2, 1000, True, False) 
truss.SetName("Truss (Ground)")
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
truss.GetVisualShape(0).SetMaterial(0, mat_grey)
system.Add(truss)



sun_gear_pos = chrono.ChVector3d(0, 0, 0)
sun_gear = create_gear_body("SunGear", sun_radius, gear_thickness, sun_gear_pos, mat_blue)




carrier_arm_body = chrono.ChBodyEasyBox(carrier_arm_length * 1.5, 
                                     carrier_arm_visual_radius * 2, 
                                     carrier_arm_visual_radius * 2, 
                                     1000, True, True)
carrier_arm_body.SetName("CarrierArm")
carrier_arm_body.SetPos(chrono.ChVector3d(carrier_arm_length * 0.75 / 2, 0, 0)) 
carrier_arm_body.GetVisualShape(0).SetMaterial(0, mat_green)
system.Add(carrier_arm_body)





planet_gear_pos_initial = chrono.ChVector3d(carrier_arm_length, 0, 0)
planet_gear = create_gear_body("PlanetGear", planet_radius, gear_thickness, planet_gear_pos_initial, mat_red)







z_rot_frame = chrono.QuatFromAngleY(chrono.CH_PI_2) 

rev_sun_truss = chrono.ChLinkRevolute()
rev_sun_truss.Initialize(sun_gear,              
                         truss,                 
                         True,                  
                         chrono.ChFrameD(sun_gear_pos, z_rot_frame)) 
system.Add(rev_sun_truss)


rev_carrier_truss = chrono.ChLinkRevolute()
rev_carrier_truss.Initialize(carrier_arm_body,
                             truss,
                             True,
                             chrono.ChFrameD(chrono.ChVector3d(0,0,0), z_rot_frame))
system.Add(rev_carrier_truss)













rev_planet_carrier = chrono.ChLinkRevolute()
rev_planet_carrier.Initialize(planet_gear,
                              carrier_arm_body,
                              True, 
                              chrono.ChFrameD(planet_gear_pos_initial, z_rot_frame))
system.Add(rev_planet_carrier)





link_gear_sun_planet = chrono.ChLinkGear()


link_gear_sun_planet.Initialize(rev_sun_truss, rev_planet_carrier, chrono.ChFrameD(chrono.ChVector3d(0,0,0), z_rot_frame))




transmission_ratio = -sun_radius / planet_radius
link_gear_sun_planet.SetTransmissionRatio(transmission_ratio)
link_gear_sun_planet.SetEnforcePhase(True) 
link_gear_sun_planet.SetPhase(0)
system.Add(link_gear_sun_planet)



motor_function = chrono.ChFunctionConst(motor_speed_rad_s) 



sun_motor = chrono.ChLinkMotorRotationSpeed()
sun_motor.Initialize(sun_gear,      
                     truss,         
                     chrono.ChFrameD(sun_gear_pos, z_rot_frame)) 
sun_motor.SetSpeedFunction(motor_function)
system.Add(sun_motor)



myapplication = irr.ChVisualSystemIrrlicht()
myapplication.AttachSystem(system)
myapplication.SetWindowSize(1024, 768)
myapplication.SetWindowTitle('PyChrono Epicyclic Gear Demo')
myapplication.Initialize()
myapplication.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddSkyBox()
myapplication.AddTypicalLights()

myapplication.AddCamera(chrono.ChVector3d(0, -2.5 * carrier_arm_length, 0.5 * carrier_arm_length), 
                        chrono.ChVector3d(0, 0, 0))                   






timestep = 0.005
system.SetTimestep(timestep)





while myapplication.GetDevice().run():
    myapplication.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.15)) 
    myapplication.Render()
    myapplication.DrawAll() 

    
    

    system.DoStepDynamics(timestep)
    myapplication.EndScene()


del myapplication
del system