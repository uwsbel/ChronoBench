import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import pychrono.vehicle as veh
import math as m




def main():
    

    
    rotor = cascade.JeffcottRotorIGA()
    rotor.SetContactMethod(contact_method)
    rotor.SetCollisionType(coll_type)
    rotor.SetChassisFixed(False) 
    rotor.SetChassisLocation(chrono.ChVector3d(0, 0, 0))
    rotor.SetDriveType(cascade.JeffcottRotor.DrivelineType_SIMPLE)
    rotor.SetShaft1Fixed(False)
    rotor.SetShaft1Location(chrono.ChVector3d(0, 0, 0))
    rotor.SetRotorPrecomputedMesh(mesh_file)
    rotor.Initialize()

    
    motor = cascade.SimpleMotorConstantAngularVelocity()
    motor.SetAngularVelocity(100)
    rotor.SetMotor(motor)

    
    flywheel = cascade.FlywheelSimpleDisc()
    flywheel.SetMass(0)
    flywheel.SetInertia(0.015)
    flywheel.SetColor(chrono.ChColor(1, 1, 1))
    flywheel.Initialize()
    rotor.SetFlywheel(flywheel)

    
    beam = cascade.BeamsFEM()
    beam.SetYoungModulus(2e11)
    beam.SetCrossSectionFile(mesh_file)
    beam.Initialize()
    rotor.SetBeam(beam)

    
    rotor_system = cascade.CascadeSystem()
    rotor_system.Initialize()

    
    vehicle = cascade.Cascadevehicle()
    vehicle.Initialize()

    
    ground = chrono.ChBodyEasyBox(1000, 1000, 1, 1000, True, True)
    ground.SetPos(chrono.ChVector3d(0, 0, 0))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
    vehicle.GetSystem().Add(ground)

    
    
    

    vis = cascade.ChFEMVisualSystemIrrlicht()
    vis.SetWindowTitle('Jeffcott rotor')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75 * 0.8), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachSystem(vehicle.GetSystem())

    
    
    

    
    
    
    
    

    
    
    
    
    
    

    
    output = chrono.ChOutputVTK(output_dir)
    output.SetPrecision(6)
    output.SetStepSize(step_size)
    output.SetFilename("jeffcott")
    output.AttachSystem(vehicle.GetSystem())
    output.Initialize()

    
    
    

    
    steps_between_events = int(step_size / delta_t)

    
    frames = 0

    
    time = 0

    
    step_number = 0

    
    while vis.Run() :
        time = rotor.GetSystem().GetChTime()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        rotor.Synchronize(time, driver_inputs, beam, flywheel)
        beam.Synchronize(time, rotor)
        flywheel.Synchronize(time, rotor)
        vehicle.Synchronize(time, driver_inputs)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(delta_t)
        rotor.Advance(delta_t)
        beam.Advance(delta_t)
        flywheel.Advance(delta_t)
        vehicle.Advance(delta_t)
        vis.Advance(delta_t)

        
        step_number += 1
        if (step_number % steps_between_events == 0) :
            frames += 1

    return 0









mesh_dir = chrono.GetChronoDataPath() + 'cascade/'
contact_method = chrono.ChContactMethod_NSC
coll_type = chrono.ChCollisionType_BULLET
step_size = 1e-3
delta_t = 1.0 / 50  
output_dir =  './IGA_Jeffcott_Rotor'
output_file = output_dir + '/output.json'
mesh_file = mesh_dir + 'iga/rotor/rotor_iga_16mm.msh'




driver = veh.ChDriver()




print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

main()