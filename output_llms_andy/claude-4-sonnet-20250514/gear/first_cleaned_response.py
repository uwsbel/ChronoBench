import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mat_steel = chrono.ChMaterialSurfaceNSC()
mat_steel.SetFriction(0.3)
mat_steel.SetRestitution(0.1)


truss = chrono.ChBodyEasyBox(2.0, 0.2, 2.0, 1000, True, True, mat_steel)
truss.SetPos(chrono.ChVectorD(0, -0.5, 0))
truss.SetBodyFixed(True)
truss.SetName("Truss")
system.Add(truss)


bar_length = 1.5
bar_width = 0.1
bar_height = 0.1
rotating_bar = chrono.ChBodyEasyBox(bar_length, bar_width, bar_height, 500, True, True, mat_steel)
rotating_bar.SetPos(chrono.ChVectorD(0, 0.2, 0))
rotating_bar.SetName("RotatingBar")
system.Add(rotating_bar)


joint_bar_truss = chrono.ChLinkRevolute()
joint_bar_truss.Initialize(rotating_bar, truss, 
                          chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                           chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.VECT_X)))
system.Add(joint_bar_truss)


sun_gear_radius = 0.15
planet_gear_radius = 0.1
ring_gear_radius = sun_gear_radius + 2 * planet_gear_radius


sun_gear = chrono.ChBodyEasyCylinder(sun_gear_radius, 0.05, 1000, True, True, mat_steel)
sun_gear.SetPos(chrono.ChVectorD(0, 0.3, 0))
sun_gear.SetName("SunGear")
system.Add(sun_gear)


joint_sun = chrono.ChLinkRevolute()
joint_sun.Initialize(sun_gear, truss,
                    chrono.ChCoordsysD(chrono.ChVectorD(0, 0.3, 0),
                                     chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.VECT_X)))
system.Add(joint_sun)


ring_gear_outer = chrono.ChBodyEasyCylinder(ring_gear_radius + 0.02, 0.05, 1000, True, True, mat_steel)
ring_gear_inner = chrono.ChBodyEasyCylinder(ring_gear_radius - 0.02, 0.06, 1000, True, True, mat_steel)
ring_gear = chrono.ChBodyEasyCylinder(ring_gear_radius, 0.04, 1000, True, True, mat_steel)
ring_gear.SetPos(chrono.ChVectorD(0, 0.3, 0))
ring_gear.SetBodyFixed(True)  
ring_gear.SetName("RingGear")
system.Add(ring_gear)


planet_positions = []
planet_gears = []
planet_joints = []


num_planets = 3
for i in range(num_planets):
    angle = 2 * chrono.CH_C_PI * i / num_planets
    planet_distance = sun_gear_radius + planet_gear_radius
    x_pos = planet_distance * math.cos(angle)
    z_pos = planet_distance * math.sin(angle)
    
    
    planet_gear = chrono.ChBodyEasyCylinder(planet_gear_radius, 0.05, 800, True, True, mat_steel)
    planet_gear.SetPos(chrono.ChVectorD(x_pos, 0.3, z_pos))
    planet_gear.SetName(f"PlanetGear_{i}")
    system.Add(planet_gear)
    planet_gears.append(planet_gear)
    
    
    planet_joint = chrono.ChLinkRevolute()
    planet_joint.Initialize(planet_gear, rotating_bar,
                           chrono.ChCoordsysD(chrono.ChVectorD(x_pos, 0.3, z_pos),
                                            chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.VECT_X)))
    system.Add(planet_joint)
    planet_joints.append(planet_joint)


for i, planet_gear in enumerate(planet_gears):
    gear_constraint = chrono.ChLinkGear()
    gear_constraint.Initialize(sun_gear, planet_gear, truss,
                              chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    gear_constraint.Set_tau(sun_gear_radius / planet_gear_radius)
    system.Add(gear_constraint)


for i, planet_gear in enumerate(planet_gears):
    gear_constraint_ring = chrono.ChLinkGear()
    gear_constraint_ring.Initialize(planet_gear, ring_gear, truss,
                                   chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    gear_constraint_ring.Set_tau(-planet_gear_radius / ring_gear_radius)
    system.Add(gear_constraint_ring)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sun_gear, truss,
                chrono.ChFrameD(chrono.ChVectorD(0, 0.3, 0),
                               chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.VECT_X)))
motor_function = chrono.ChFunction_Const(chrono.CH_C_PI)  
motor.SetSpeedFunction(motor_function)
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1, 2))
vis.AddTypicalLights()



truss_asset = truss.GetAssets()[0]
if isinstance(truss_asset, chrono.ChVisualizationShapeBox):
    truss_asset.SetColor(chrono.ChColor(0.5, 0.5, 0.5))


bar_color = chrono.ChColorAsset()
bar_color.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
rotating_bar.AddAsset(bar_color)


sun_color = chrono.ChColorAsset()
sun_color.SetColor(chrono.ChColor(0.9, 0.9, 0.2))
sun_gear.AddAsset(sun_color)


ring_color = chrono.ChColorAsset()
ring_color.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ring_gear.AddAsset(ring_color)


for planet in planet_gears:
    planet_color = chrono.ChColorAsset()
    planet_color.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
    planet.AddAsset(planet_color)


vis.EnableContactDrawing(chrono.ContactsDrawMode_CONTACT_DISTANCES)
vis.EnableBodyFrameDrawing(True)


time_step = 0.01
simulation_time = 0

print("Starting epicyclic gear simulation...")
print("Sun gear (yellow) rotates at constant speed")
print("Planet gears (green) orbit around the sun gear")
print("Ring gear (red) is fixed")
print("Carrier bar (blue) rotates with the planet gears")

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)
    simulation_time += time_step
    
    
    if int(simulation_time * 50) % 100 == 0:
        sun_rot = sun_gear.GetRot().Q_to_Euler123().y
        bar_rot = rotating_bar.GetRot().Q_to_Euler123().y
        print(f"Time: {simulation_time:.2f}s, Sun rotation: {sun_rot:.2f} rad, Carrier rotation: {bar_rot:.2f} rad")

print("Simulation completed!")