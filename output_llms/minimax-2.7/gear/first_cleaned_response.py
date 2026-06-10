import math
import os






import pychrono as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath("../../../data/")


def create_epicyclic_gear_system():
    
    
    
    
    
    
    
    my_system = chrono.ChSystemNSC()
    my_system.SetCollisionSystemType(chrono.ChCollisionSystem_Type_BULLET)
    
    
    my_system.Set_Gravity(chrono.ChVector3d(0, 0, -9.81))
    
    
    time_step = 0.001
    output_fps = 50
    
    
    
    
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, 0))
    ground.SetVisualizationObject(
        chrono.ChVisualizationShapeFile(chrono.GetChronoDataFile("sensor/vlp16/velodyne_mount.obj"))
    )
    
    
    ground_mat = chrono.ChVisualizationShapeMaterial()
    ground_mat.SetDiffuseColor(chrono.ChColor(0.3, 0.3, 0.3))
    ground_mat.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))
    
    ground_shape = chrono.ChVisualizationShapeBox(20, 20, 0.1)
    ground_shape.SetMaterial(ground_mat)
    ground.AddVisualizationShape(ground_shape)
    
    
    ground_coll = chrono.ChCollisionShapeBox(10, 10, 0.05)
    ground.AddCollisionShape(ground_coll, chrono.ChFrameD(chrono.ChVector3d(0, 0, -0.05)))
    
    my_system.AddBody(ground)
    
    
    
    
    
    
    truss = chrono.ChBody()
    truss.SetBodyFixed(True)
    truss.SetPos(chrono.ChVector3d(0, 0, 0.5))
    
    truss_mat = chrono.ChVisualizationShapeMaterial()
    truss_mat.SetDiffuseColor(chrono.ChColor(0.4, 0.4, 0.5))
    truss_mat.SetSpecularColor(chrono.ChColor(0.2, 0.2, 0.2))
    
    
    truss_shape = chrono.ChVisualizationShapeCylinder(2.0, 0.1)
    truss_shape.SetMaterial(truss_mat)
    truss.AddVisualizationShape(truss_shape)
    
    my_system.AddBody(truss)
    
    
    
    
    
    
    sun_radius = 0.3           
    planet_radius = 0.2        
    ring_inner_radius = 0.75   
    gear_thickness = 0.08      
    
    
    sun_teeth = 30
    planet_teeth = 20
    ring_teeth = 70
    
    
    sun_position = chrono.ChVector3d(0, 0, 1.0)
    planet_offset = sun_radius + planet_radius  
    
    
    
    
    
    sun_gear = chrono.ChBody()
    sun_gear.SetPos(sun_position)
    sun_gear.SetMass(5.0)
    sun_gear.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
    
    
    sun_mat = chrono.ChVisualizationShapeMaterial()
    sun_mat.SetDiffuseColor(chrono.ChColor(0.9, 0.3, 0.1))  
    sun_mat.SetSpecularColor(chrono.ChColor(0.5, 0.2, 0.1))
    sun_mat.SetShininess(60)
    
    
    sun_shape = chrono.ChVisualizationShapeGear(
        sun_radius,           
        sun_teeth,            
        gear_thickness,       
        0.02,                 
        0.0,                  
        True                  
    )
    sun_shape.SetMaterial(sun_mat)
    sun_gear.AddVisualizationShape(sun_shape)
    
    
    shaft_shape = chrono.ChVisualizationShapeCylinder(gear_thickness/2, 0.15)
    shaft_shape.SetMaterial(sun_mat)
    sun_gear.AddVisualizationShape(shaft_shape)
    
    
    sun_coll = chrono.ChCollisionShapeCylinder(chrono.ChAxis_Y, sun_radius, gear_thickness)
    sun_gear.AddCollisionShape(sun_coll, chrono.ChFrameD())
    
    my_system.AddBody(sun_gear)
    
    
    
    
    
    num_planets = 3
    planet_gears = []
    
    for i in range(num_planets):
        angle = 2 * math.pi * i / num_planets
        
        planet = chrono.ChBody()
        planet.SetPos(chrono.ChVector3d(
            sun_position.x + planet_offset * math.cos(angle),
            sun_position.y + planet_offset * math.sin(angle),
            sun_position.z
        ))
        planet.SetMass(2.0)
        planet.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
        
        
        planet_mat = chrono.ChVisualizationShapeMaterial()
        planet_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.4, 0.8))
        planet_mat.SetSpecularColor(chrono.ChColor(0.3, 0.5, 0.8))
        planet_mat.SetShininess(80)
        
        
        planet_shape = chrono.ChVisualizationShapeGear(
            planet_radius,
            planet_teeth,
            gear_thickness,
            0.015,
            0.0,
            True
        )
        planet_shape.SetMaterial(planet_mat)
        planet.AddVisualizationShape(planet_shape)
        
        
        planet_coll = chrono.ChCollisionShapeCylinder(chrono.ChAxis_Y, planet_radius, gear_thickness)
        planet.AddCollisionShape(planet_coll, chrono.ChFrameD())
        
        my_system.AddBody(planet)
        planet_gears.append(planet)
    
    
    
    
    
    ring_gear = chrono.ChBody()
    ring_gear.SetBodyFixed(True)  
    ring_gear.SetPos(sun_position)
    ring_gear.SetMass(100.0)  
    
    
    ring_mat = chrono.ChVisualizationShapeMaterial()
    ring_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.2))
    ring_mat.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
    ring_mat.SetShininess(40)
    
    
    ring_shape = chrono.ChVisualizationShapeRing(
        ring_inner_radius - 0.1,  
        ring_inner_radius,         
        gear_thickness,            
        ring_teeth,                
        0.02                       
    )
    ring_shape.SetMaterial(ring_mat)
    ring_gear.AddVisualizationShape(ring_shape)
    
    
    ring_coll = chrono.ChCollisionShapeCylinderAnnulus(
        ring_inner_radius - 0.15,
        ring_inner_radius + 0.05,
        gear_thickness
    )
    ring_gear.AddCollisionShape(ring_coll, chrono.ChFrameD())
    
    my_system.AddBody(ring_gear)
    
    
    
    
    
    planet_carrier = chrono.ChBody()
    planet_carrier.SetPos(sun_position)
    planet_carrier.SetMass(3.0)
    planet_carrier.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
    
    
    carrier_mat = chrono.ChVisualizationShapeMaterial()
    carrier_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.7, 0.3))
    carrier_mat.SetSpecularColor(chrono.ChColor(0.2, 0.5, 0.3))
    
    
    for i in range(num_planets):
        angle = 2 * math.pi * i / num_planets
        
        arm_shape = chrono.ChVisualizationShapeBox(
            planet_offset * 0.8,
            0.03,
            0.03
        )
        arm_shape.SetMaterial(carrier_mat)
        planet_carrier.AddVisualizationShape(arm_shape)
    
    
    hub_shape = chrono.ChVisualizationShapeCylinder(gear_thickness/2, 0.1)
    hub_shape.SetMaterial(carrier_mat)
    planet_carrier.AddVisualizationShape(hub_shape)
    
    my_system.AddBody(planet_carrier)
    
    
    
    
    
    
    
    
    for i, planet in enumerate(planet_gears):
        angle = 2 * math.pi * i / num_planets
        
        
        sun_planet_joint = chrono.ChLinkLockRevolute()
        sun_planet_joint.Initialize(
            sun_gear,
            planet,
            chrono.ChFrameD(sun_position + chrono.ChVector3d(
                planet_offset * math.cos(angle),
                planet_offset * math.sin(angle),
                0
            ))
        )
        my_system.AddLink(sun_planet_joint)
    
    
    for i, planet in enumerate(planet_gears):
        angle = 2 * math.pi * i / num_planets
        
        
        planet_carrier_joint = chrono.ChLinkLockRevolute()
        planet_carrier_joint.Initialize(
            planet,
            planet_carrier,
            planet.GetFrame()
        )
        my_system.AddLink(planet_carrier_joint)
    
    
    
    
    
    
    
    
    
    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(
        sun_gear,                    
        ground,                      
        chrono.ChFrameD(sun_position, chrono.ChQuaterniond(1, 0, 0, 0))  
    )
    
    
    motor.SetMotorSpeed(chrono.ChFunction_Const(10.0))  
    my_system.AddLink(motor)
    
    
    
    
    
    
    out_dir = "epicyclic_gear_output"
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    
    return my_system, {
        'system': my_system,
        'sun_gear': sun_gear,
        'planet_gears': planet_gears,
        'ring_gear': ring_gear,
        'planet_carrier': planet_carrier,
        'motor': motor,
        'ground': ground,
        'time_step': time_step,
        'output_fps': output_fps
    }


def run_visualization(system_data):
    
    
    my_system = system_data['system']
    
    
    
    
    
    
    my_application = irr.ChIrrApp(
        my_system,
        "Epicyclic Gear System Simulation",
        irr.dimension2du(1280, 720)
    )
    
    
    my_application.AddTypicalSky()
    my_application.AddTypicalCamera(
        irr.vector3df(3, 3, 3),      
        irr.vector3df(0, 0, 1.5)      
    )
    
    
    
    
    
    
    my_application.AddLight(
        irr.vector3df(5, 5, 10),
        1.0,
        irr.SColorf(0.3, 0.3, 0.3)
    )
    
    
    my_application.AddTypicalLight(
        irr.ChIrrApp.LIGHT_DIRECTIONAL,
        irr.vector3df(1, 1, 1),
        1.0,
        irr.SColorf(0.8, 0.8, 0.8)
    )
    
    
    my_application.AddLight(
        irr.vector3df(-2, -2, 4),
        0.5,
        irr.SColorf(0.5, 0.5, 0.6)
    )
    
    
    
    
    
    
    ground_grid = irr.ChIrrAssetConverter.AddGrid(
        my_system.Get_bodylist()[0],
        2.0,              
        2.0,              
        20,               
        20,               
        irr.SColor(100, 100, 100, 100),
        True              
    )
    
    
    
    
    
    
    my_application.AddHUDText(
        "Epicyclic Gear System",
        irr.vector2di(10, 10),
        irr.SColor(255, 255, 255, 255),
        True
    )
    
    my_application.AddHUDText(
        "Motor: Sun gear rotating at 10 rad/s (~95 RPM)",
        irr.vector2di(10, 30),
        irr.SColor(255, 200, 200, 200),
        True
    )
    
    
    
    
    
    
    my_application.SetTimestep(system_data['time_step'])
    my_application.SetVideoframeRate(60)
    
    
    print("Starting visualization...")
    print("Controls:")
    print("  - Left mouse button: Select object")
    print("  - Right mouse button: Rotate view")
    print("  - Mouse wheel: Zoom")
    print("  - Press 'q' or close window to quit")
    
    while my_application.Run():
        
        my_application.BeginScene()
        my_application.DrawAll()
        
        
        sun_speed = system_data['sun_gear'].GetAngVelLocal().z
        carrier_speed = system_data['planet_carrier'].GetAngVelLocal().z
        
        time_text = f"Time: {my_system.GetChTime():.2f} s"
        my_application.AddHUDText(
            time_text,
            irr.vector2di(10, 50),
            irr.SColor(255, 255, 255, 255),
            False
        )
        
        rpm_text = f"Sun gear RPM: {abs(sun_speed) * 60 / (2 * math.pi):.1f}"
        my_application.AddHUDText(
            rpm_text,
            irr.vector2di(10, 70),
            irr.SColor(255, 255, 255, 255),
            False
        )
        
        carrier_text = f"Carrier RPM: {abs(carrier_speed) * 60 / (2 * math.pi):.1f}"
        my_application.AddHUDText(
            carrier_text,
            irr.vector2di(10, 90),
            irr.SColor(255, 255, 255, 255),
            False
        )
        
        
        my_application.DoStep()
        
        
        my_application.EndScene()
    
    print("Visualization closed.")


def run_batch_simulation(system_data, duration=10.0):
    
    
    my_system = system_data['system']
    
    print(f"Running batch simulation for {duration} seconds...")
    print(f"Time step: {system_data['time_step']} s")
    
    
    sim_time = 0.0
    step = 0
    
    while sim_time < duration:
        my_system.DoStepDynamics(system_data['time_step'])
        sim_time = my_system.GetChTime()
        
        step += 1
        
        if step % 100 == 0:
            
            sun_angle = system_data['sun_gear'].GetRot().GetAngle()
            carrier_angle = system_data['planet_carrier'].GetRot().GetAngle()
            sun_angvel = system_data['sun_gear'].GetAngVelLocal().z
            carrier_angvel = system_data['planet_carrier'].GetAngVelLocal().z
            
            print(f"t={sim_time:.2f}s | "
                  f"Sun: angle={sun_angle:.2f} rad, ω={sun_angvel:.2f} rad/s | "
                  f"Carrier: angle={carrier_angle:.2f} rad, ω={carrier_angvel:.2f} rad/s")
    
    
    print("\n=== Final State ===")
    print(f"Sun gear final angle: {system_data['sun_gear'].GetRot().GetAngle():.2f} rad")
    print(f"Sun gear final angular velocity: {system_data['sun_gear'].GetAngVelLocal().z:.2f} rad/s")
    print(f"Carrier final angle: {system_data['planet_carrier'].GetRot().GetAngle():.2f} rad")
    print(f"Carrier final angular velocity: {system_data['planet_carrier'].GetAngVelLocal().z:.2f} rad/s")


def main():
    
    
    print("=" * 60)
    print("PyChrono Epicyclic Gear System Simulation")
    print("=" * 60)
    
    
    system_data = create_epicyclic_gear_system()
    
    
    mode = 'visual'
    
    if mode == 'visual':
        print("\nStarting interactive visualization mode...")
        run_visualization(system_data)
    else:
        print("\nStarting batch simulation mode...")
        run_batch_simulation(system_data, duration=5.0)
    
    print("\nSimulation complete.")


if __name__ == "__main__":
    main()