import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


def main():
    
    
    
    print("Initializing PyChrono environment...")
    
    
    chrono.SetChronoDataPath("../../../data/")
    
    
    my_system = chrono.ChSystemNSC()
    my_system.SetSolverType(chrono.ChSolver.SPSOR)
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
    my_system.SetMaxiter(100)
    my_system.SetIterStoppingCriterion(1e-6)
    
    
    timestep = 0.001
    my_system.SetTimestepperType(chrono.ChTimestepper.HHT)
    
    print("  - System created with gravity: (0, -9.81, 0)")
    print("  - Solver: SPSOR")
    print("  - Timestep: {}".format(timestep))
    
    
    
    
    
    crank_length = 0.5          
    crank_radius = 0.05         
    crank_density = 7800        
    
    
    rod_length = 1.5            
    rod_width = 0.04            
    rod_density = 2700          
    
    
    piston_radius = 0.15        
    piston_height = 0.2         
    piston_density =2700        
    
    
    cylinder_length = 2.0       
    
    
    motor_speed = 10.0          
    
    print("\nMechanism Parameters:")
    print("  - Crank length: {} m".format(crank_length))
    print("  - Connecting rod length: {} m".format(rod_length))
    print("  - Piston radius: {} m".format(piston_radius))
    print("  - Motor speed: {} rad/s (~{} RPM)".format(motor_speed, motor_speed * 60 / (2 * math.pi)))
    
    
    
    
    print("\nCreating floor (truss)...")
    
    floor = chrono.ChBody()
    floor.SetBodyFixed(True)  
    floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
    floor.SetCollide(True)
    floor.SetName("Floor")
    floor.SetVisualizationType(chrono.VisualizationType.VIZ_MESH)
    
    
    floor_mat = chrono.ChMaterialSurfaceNSC()
    floor_mat.SetFriction(0.4)
    floor_mat.SetRestitution(0.1)
    floor_mat.SetCompliance(0.0)
    
    
    floor_shape = chrono.ChBoxShape(8, 0.1, 4)
    floor_shape.SetMaterialSurface(floor_mat)
    floor.AddShape(floor_shape)
    
    
    floor_color = chrono.ChColorAsset()
    floor_color.SetColor(chrono.ChColor(0.4, 0.4, 0.4))  
    floor.AddAsset(floor_color)
    
    my_system.AddBody(floor)
    print("  - Floor created and fixed")
    
    
    
    
    print("\nCreating crankshaft...")
    
    crankshaft = chrono.ChBody()
    crankshaft.SetName("Crankshaft")
    crankshaft.SetPos(chrono.ChVectorD(0, crank_length, 0))
    crankshaft.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    crankshaft.SetBodyFixed(False)
    crankshaft.SetCollide(False)
    crankshaft.SetMass(crank_density * math.pi * crank_radius * crank_radius * crank_length)
    
    
    shaft_shape = chrono.ChCylinderShape(crank_radius, crank_length)
    shaft_shape.SetRotation(chrono.ChRotationD(chrono.VECT_Y, 90))  
    shaft_color = chrono.ChColorAsset()
    shaft_color.SetColor(chrono.ChColor(0.7, 0.2, 0.2))  
    crankshaft.AddAsset(shaft_color)
    crankshaft.AddAsset(shaft_shape)
    
    
    crank_web_shape = chrono.ChBoxShape(crank_radius * 3, crank_radius * 2, crank_radius * 2)
    crank_web_shape.SetColor(chrono.ChColor(0.6, 0.15, 0.15))
    crankshaft.AddAsset(crank_web_shape)
    
    
    crank_pin_shape = chrono.ChCylinderShape(crank_radius * 0.8, crank_radius * 1.5)
    crank_pin_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.3))
    
    crank_pin_asset = chrono.ChVisualShape()
    crankshaft.AddAsset(crank_pin_shape)
    
    my_system.AddBody(crankshaft)
    print("  - Crankshaft created (mass: {:.3f} kg)".format(crankshaft.GetMass()))
    
    
    
    
    print("\nCreating connecting rod...")
    
    connecting_rod = chrono.ChBody()
    connecting_rod.SetName("Connecting Rod")
    
    connecting_rod.SetPos(chrono.ChVectorD(rod_length / 2, crank_length, 0))
    connecting_rod.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    connecting_rod.SetBodyFixed(False)
    connecting_rod.SetCollide(False)
    
    
    rod_volume = rod_length * rod_width * rod_width
    connecting_rod.SetMass(rod_density * rod_volume)
    
    
    rod_shape = chrono.ChBoxShape(rod_length, rod_width, rod_width)
    rod_color = chrono.ChColorAsset()
    rod_color.SetColor(chrono.ChColor(0.3, 0.5, 0.7))  
    connecting_rod.AddAsset(rod_color)
    connecting_rod.AddAsset(rod_shape)
    
    
    big_end_shape = chrono.ChCylinderShape(crank_radius * 1.2, rod_width * 1.5)
    big_end_shape.SetColor(chrono.ChColor(0.25, 0.4, 0.6))
    connecting_rod.AddAsset(big_end_shape)
    
    
    small_end_shape = chrono.ChCylinderShape(crank_radius, rod_width * 1.5)
    small_end_shape.SetColor(chrono.ChColor(0.35, 0.55, 0.75))
    connecting_rod.AddAsset(small_end_shape)
    
    my_system.AddBody(connecting_rod)
    print("  - Connecting rod created (mass: {:.3f} kg)".format(connecting_rod.GetMass()))
    
    
    
    
    print("\nCreating piston...")
    
    piston = chrono.ChBody()
    piston.SetName("Piston")
    
    piston_x = crank_length + rod_length
    piston.SetPos(chrono.ChVectorD(piston_x, crank_length, 0))
    piston.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    piston.SetBodyFixed(False)
    piston.SetCollide(True)
    
    
    piston_volume = math.pi * piston_radius * piston_radius * piston_height
    piston.SetMass(piston_density * piston_volume)
    
    
    piston_mat = chrono.ChMaterialSurfaceNSC()
    piston_mat.SetFriction(0.3)
    piston_mat.SetRestitution(0.0)
    piston_mat.SetCompliance(0.0)
    
    
    piston_shape = chrono.ChCylinderShape(piston_radius, piston_height)
    piston_shape.SetMaterialSurface(piston_mat)
    piston_shape.SetRotation(chrono.ChRotationD(chrono.VECT_Z, 90))  
    piston.AddShape(piston_shape)
    
    
    piston_color = chrono.ChColorAsset()
    piston_color.SetColor(chrono.ChColor(0.2, 0.6, 0.3))  
    piston.AddAsset(piston_color)
    
    
    ring_shape = chrono.Ch torus or ChBoxShape approximating rings
    for i in range(3):
        ring_visual = chrono.ChBoxShape(piston_radius * 2.02, 0.01, piston_radius * 2.02)
        ring_color = chrono.ChColorAsset()
        ring_color.SetColor(chrono.ChColor(0.1, 0.1, 0.1))  
        piston.AddAsset(ring_visual)
    
    my_system.AddBody(piston)
    print("  - Piston created (mass: {:.3f} kg)".format(piston.GetMass()))
    
    
    
    
    print("\nCreating piston rod...")
    
    piston_rod = chrono.ChBody()
    piston_rod.SetName("Piston Rod")
    piston_rod.SetPos(chrono.ChVectorD(piston_x + piston_height / 2 + cylinder_length / 2, crank_length, 0))
    piston_rod.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    piston_rod.SetBodyFixed(False)
    piston_rod.SetCollide(False)
    
    
    rod_volume = cylinder_length * 0.02 * 0.02
    piston_rod.SetMass(piston_density * rod_volume)
    
    
    rod_visual_shape = chrono.ChCylinderShape(0.02, cylinder_length)
    rod_visual_shape.SetRotation(chrono.ChRotationD(chrono.VECT_Z, 90))  
    rod_color = chrono.ChColorAsset()
    rod_color.SetColor(chrono.ChColor(0.5, 0.5, 0.5))  
    piston_rod.AddAsset(rod_color)
    piston_rod.AddAsset(rod_visual_shape)
    
    my_system.AddBody(piston_rod)
    print("  - Piston rod created")
    
    
    
    
    print("\nCreating slider block...")
    
    slider = chrono.ChBody()
    slider.SetName("Slider")
    slider.SetPos(chrono.ChVectorD(piston_x + piston_height / 2 + cylinder_length, crank_length, 0))
    slider.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    slider.SetBodyFixed(False)
    slider.SetCollide(True)
    slider.SetMass(5.0)
    
    
    slider_mat = chrono.ChMaterialSurfaceNSC()
    slider_mat.SetFriction(0.3)
    
    
    slider_shape = chrono.ChBoxShape(0.15, 0.1, 0.15)
    slider_shape.SetMaterialSurface(slider_mat)
    slider.AddShape(slider_shape)
    
    
    slider_color = chrono.ChColorAsset()
    slider_color.SetColor(chrono.ChColor(0.6, 0.6, 0.2))  
    slider.AddAsset(slider_color)
    
    my_system.AddBody(slider)
    print("  - Slider created")
    
    
    
    
    print("\nCreating joints...")
    
    
    crank_joint = chrono.ChLinkLockRevolute()
    crank_joint.Initialize(
        crankshaft,           
        floor,                
        chrono.ChCoordsysD(
            chrono.ChVectorD(0, crank_length, 0),  
            chrono.ChQuaternionD(1, 0, 0, 0)       
        )
    )
    my_system.AddLink(crank_joint)
    print("  - Crankshaft-Ground revolute joint created")
    
    
    crank_pin_pos = chrono.ChVectorD(crank_length, crank_length, 0)
    rod_crank_joint = chrono.ChLinkLockRevolute()
    rod_crank_joint.Initialize(
        connecting_rod,       
        crankshaft,           
        chrono.ChCoordsysD(crank_pin_pos, chrono.ChQuaternionD(1, 0, 0, 0))
    )
    my_system.AddLink(rod_crank_joint)
    print("  - Crankshaft-Connecting rod revolute joint created")
    
    
    
    piston_pin_pos = chrono.ChVectorD(crank_length + rod_length, crank_length, 0)
    rod_piston_joint = chrono.ChLinkLockSpherical()
    rod_piston_joint.Initialize(
        piston,               
        connecting_rod,       
        chrono.ChCoordsysD(piston_pin_pos, chrono.ChQuaternionD(1, 0, 0, 0))
    )
    my_system.AddLink(rod_piston_joint)
    print("  - Connecting rod-Piston spherical joint created")
    
    
    piston_rod_joint = chrono.ChLinkLockPrismatic()
    piston_rod_joint.Initialize(
        piston_rod,           
        piston,               
        chrono.ChCoordsysD(
            piston.GetPos() + chrono.ChVectorD(piston_height/2, 0, 0),
            chrono.ChQuaternionD(chrono.VECT_Y, 90)  
        )
    )
    my_system.AddLink(piston_rod_joint)
    print("  - Piston-Piston rod prismatic joint created")
    
    
    slider_joint = chrono.ChLinkLockRevolute()
    slider_joint.Initialize(
        slider,               
        piston_rod,           
        chrono.ChCoordsysD(
            slider.GetPos(),
            chrono.ChQuaternionD(chrono.VECT_Z, 90)  
        )
    )
    my_system.AddLink(slider_joint)
    print("  - Piston rod-Slider revolute joint created")
    
    
    slider_ground_joint = chrono.ChLinkLockPrismatic()
    slider_ground_joint.Initialize(
        slider,               
        floor,                
        chrono.ChCoordsysD(
            slider.GetPos(),
            chrono.ChQuaternionD(chrono.VECT_Y, 90)  
        )
    )
    my_system.AddLink(slider_ground_joint)
    print("  - Slider-Ground prismatic joint created")
    
    
    
    
    print("\nAdding motor to crankshaft...")
    
    
    motor_link = chrono.ChLinkMotorRotation()
    
    
    motor_link.Initialize(
        crankshaft,           
        floor,                
        chrono.ChCoordsysD(
            chrono.ChVectorD(0, crank_length, 0),  
            chrono.ChQuaternionD(1, 0, 0, 0)
        )
    )
    
    
    motor_function = chrono.ChFunction_Const(motor_speed)
    motor_link.SetMotorFunction(motor_function)
    
    
    
    
    my_system.AddLink(motor_link)
    print("  - Motor created with angular velocity: {} rad/s".format(motor_speed))
    
    
    
    
    print("\nSetting up Irrlicht visualization...")
    
    
    myapplication = chronoirr.ChIrrApp(
        my_system,                                    
        "Crank-Slider Mechanism Simulation",         
        chronoirr.dimension2du(1280, 720),            
        chronoirr.bool_(True)                         
    )
    
    
    myapplication.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    myapplication.AddTypicalSky()
    myapplication.AddTypicalCamera(
        chronoirr.vector3df(2.5, 1.5, -3),           
        chronoirr.vector3df(1.5, 0.5, 0)             
    )
    myapplication.AddTypicalLight(
        chronoirr.vector3df(5, 8, -5),                
        chronoirr.colorf(1.0, 1.0, 1.0, 1.0),         
        40.0                                          
    )
    myapplication.AddTypicalLight(
        chronoirr.vector3df(-3, 5, 3),               
        chronoirr.colorf(0.5, 0.5, 0.5, 1.0),
        20.0
    )
    
    
    myapplication.AddShadowCube()
    
    
    myapplication.SetUserCameraLookAt(chronoirr.vector3df(1.5, 0.5, 0))
    myapplication.SetUserCameraPosition(chronoirr.vector3df(2.5, 1.5, -3))
    
    
    grid = chrono.ChVisualShapeGrid()
    grid.SetPos(chrono.ChVectorD(0, 0, 0))
    grid.SetXSpacing(0.2)
    grid.SetZSpacing(0.2)
    grid.SetXLines(True)
    grid.SetZLines(True)
    grid.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
    floor.AddVisualShape(grid)
    
    print("  - Irrlicht application configured")
    print("  - Camera, lighting, and grid added")
    print("  - Press 'c' to change camera view")
    print("  - Use mouse to orbit camera")
    
    
    
    
    print("\n" + "="*60)
    print("Starting simulation...")
    print("="*60)
    
    
    simulation_time = 0.0
    end_time = 10.0  
    
    
    myapplication.BeginScene()
    myapplication.DrawAll()
    
    while myapplication.GetDevice().run():
        myapplication.BeginScene()
        myapplication.DrawAll()
        
        
        simulation_time = my_system.GetChTime()
        
        
        crank_angle = crankshaft.GetRot().GetCardanoAngles().y  
        
        
        piston_pos = piston.GetPos()
        
        
        myapplication.GetDevice().GetGUIEnvironment().addStaticText(
            "Time: {:.3f} s | Crank Angle: {:.1f} deg | Piston X: {:.3f} m".format(
                simulation_time,
                chrono.CH_C_DEG_TO_RAD * crank_angle if hasattr(chrono, 'CH_C_DEG_TO_RAD') else crank_angle * 180 / 3.14159,
                piston_pos.x
            ),
            chronoirr.rect<s32>(10, 10, 400, 30),
            True
        )
        
        
        myapplication.DoStep()
        myapplication.EndScene()
        
        
        if simulation_time > end_time:
            print("\nSimulation complete!")
            print("Final crank angle: {:.2f} degrees".format(crank_angle * 180 / 3.14159))
            print("Piston final position: ({:.3f}, {:.3f}, {:.3f})".format(
                piston_pos.x, piston_pos.y, piston_pos.z))
            break
    
    print("\n" + "="*60)
    print("Simulation finished successfully!")
    print("="*60)
    
    return 0


if __name__ == "__main__":
    main()