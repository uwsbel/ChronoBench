import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


simulation_time = 10.0  
time_step = 0.001      
beam_length = 1.0      
beam_radius = 0.01     
flywheel_mass = 2.0    
flywheel_radius = 0.1  
motor_speed = 30       


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)


beam_section = fea.ChBeamSectionCosserat(
    beam_area=np.pi * beam_radius**2,
    beam_Iyy=(np.pi/4) * beam_radius**4,
    beam_Izz=(np.pi/4) * beam_radius**4,
    beam_Jxx=(np.pi/2) * beam_radius**4,
    beam_E=210e9,  
    beam_G=80e9,   
)


builder = fea.ChBuilderBeamIGA()


start_point = chrono.ChVectorD(0, 0, 0)
end_point = chrono.ChVectorD(beam_length, 0, 0)


n_control_points = 10
beam_nodes = builder.BuildBeam(
    mesh=mesh,
    section=beam_section,
    n_control_points=n_control_points,
    start_point=start_point,
    end_point=end_point,
    fixed_A=True,   
    fixed_B=False   
)


system.Add(mesh)


for beam_element in builder.GetLastBeamElements():
    beam_element.AddVisualShape(chrono.ChColorAsset(0.0, 0.6, 0.2))


middle_node_idx = n_control_points // 2
middle_node = beam_nodes[middle_node_idx]


flywheel = chrono.ChBodyEasyCylinder(
    flywheel_radius,  
    0.05,             
    7800,             
    True,             
    True              
)
flywheel.SetPos(chrono.ChVectorD(beam_length/2, 0, 0))
flywheel.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1)))
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVectorD(
    (1/4)*flywheel_mass*flywheel_radius**2 + (1/12)*flywheel_mass*0.05**2,
    (1/4)*flywheel_mass*flywheel_radius**2 + (1/12)*flywheel_mass*0.05**2,
    (1/2)*flywheel_mass*flywheel_radius**2
))
flywheel.AddAsset(chrono.ChColorAsset(0.6, 0.2, 0.2))
system.Add(flywheel)


constraint = chrono.ChLinkMateFix()
constraint.Initialize(middle_node, flywheel)
system.Add(constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    beam_nodes[0],                    
    chrono.ChFrameD(start_point),     
    chrono.ChAxisD(chrono.ChVectorD(1, 0, 0))  
)
motor_fun = chrono.ChFunction_Ramp(0, motor_speed)  
motor.SetSpeedFunction(motor_fun)
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Jeffcott Rotor IGA Beam Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.0), chrono.ChVectorD(0.5, 0, 0))
vis.AddTypicalLights()


system.SetSolverType(chrono.ChSolver.Type_MINRES)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-5)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
integrator = system.GetTimestepper()
integrator.SetAlpha(-0.2)
integrator.SetMaxiters(5)
integrator.SetAbsTolerances(1e-5)
integrator.SetMode(chrono.ChTimestepperHHT.POSITION)
integrator.SetScaling(True)


time = 0
while time < simulation_time and vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    info = "Time: {:.2f} s".format(time)
    vis.DrawAll()
    chronoirr.drawAllContactPoints(vis, 4, chrono.ChColor(0, 1, 0))
    chronoirr.drawAllLinks(vis, 0.2)
    chronoirr.drawAllCOGs(vis, 0.1)
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)
    time += time_step

print("Simulation completed!")