import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

    
    beam_material = fea.ChBeamMaterialEuler()
    beam_material.SetDensity(7800)       
    beam_material.SetYoungModulus(210e9) 
    beam_material.SetBeamRaleyghDamping(0.01)  

    
    length = 2.0       
    num_elements = 16  
    num_knots = num_elements + 1
    order = 3          

    
    beam = fea.ChBeamIGA()
    beam.SetOrder(order)
    beam.SetMaterial(beam_material)

    
    for i in range(num_knots):
        y = i * length / (num_knots - 1)
        point = chrono.ChVectorD(0, y, 0)
        beam.AddPoint(point)
    
    
    beam.SetupInitial()

    
    system.Add(beam)

    
    flywheel = chrono.ChBodyEasyCylinder(0.15, 0.05, 1000)  
    flywheel.SetPos(chrono.ChVectorD(0, length/2, 0))
    flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.2))  
    system.Add(flywheel)

    
    node_mid = beam.GetNode(int(num_knots/2))
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(node_mid, flywheel)
    system.Add(constraint)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(beam.GetNode(0), chrono.ChFrameD())
    motor_speed = chrono.ChFunction_Ramp(0, 5.0)  
    motor.SetSpeedFunction(motor_speed)
    system.Add(motor)

    
    node_end = beam.GetNode(num_knots - 1)
    constraint_fix = fea.ChLinkPointFrame()
    constraint_fix.Initialize(node_end, system.GetGroundBody())
    system.Add(constraint_fix)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Jeffcott Rotor IGA Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1.5, length/2, 1.5), chrono.ChVectorD(0, length/2, 0))
    vis.AddTypicalLights()

    
    beam_vis = fea.ChVisualizationFEAmesh(beam)
    beam_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    beam_vis.SetColorscaleMinMax(-100, 100)
    beam_vis.SetSmoothFaces(True)
    beam_vis.SetWireframe(False)
    beam.AddAsset(beam_vis)

    
    time_step = 0.005
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    system.SetSolverType(chrono.ChSolver.Type_MINRES)
    system.GetSolver().AsIterative().SetMaxIterations(100)
    system.GetSolver().AsIterative().SetTolerance(1e-10)

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()