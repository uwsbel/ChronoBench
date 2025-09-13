import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess


chrono.SetChronoDataPath("../chrono/data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



beam_length = 1.0
beam_radius = 0.02
num_segments = 20
beam_young = 2.0e11  
beam_density = 7850  


beam = chrono.ChBodyEasy()
beam.SetBodyFixed(False)
beam.SetCollide(True)
beam.SetMaterialDensity(beam_density)
beam.SetMass(1.0) 
beam.SetPos(chrono.ChVectorD(0, 0, 0))


iga_beam = chrono.ChIGABeamSection()
iga_beam.SetDimensions(beam_length, beam_radius)
iga_beam.SetYoungModulus(beam_young)
iga_beam.Generate(num_segments)

beam.AddAsset(iga_beam)

system.Add(beam)


flywheel_radius = 0.05
flywheel_mass = 0.5
flywheel = chrono.ChBodyEasy()
flywheel.SetBodyFixed(False)
flywheel.SetCollide(True)
flywheel.SetMaterialDensity(beam_density)
flywheel.SetMass(flywheel_mass)
flywheel.SetInertia(chrono.ChVectorD(1,1,1) * flywheel_mass * flywheel_radius * flywheel_radius)
flywheel.SetPos(chrono.ChVectorD(beam_length / 2, 0, 0))
flywheel.SetShape(chrono.ChSphereShape())
flywheel.SetShapeRadius(flywheel_radius)

system.Add(flywheel)



joint = chrono.ChLinkRevolute()
joint.Initialize(beam, flywheel, chrono.ChCoordsys(chrono.ChVectorD(beam_length / 2, 0, 0)))
system.Add(joint)



motor_torque = 1.0
motor = chrono.ChMotorRotationSpeed()
motor.Set_Torque(motor_torque)
motor.Set_Speed(10.0)  
motor.Attach(beam)
system.Add(motor)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor with IGA Beam')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 2, -2))
vis.AddTypicalLights()



time_step = 0.001
time_end = 10.0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

    if system.GetChTime() > time_end:
        break

print("Simulation finished.")