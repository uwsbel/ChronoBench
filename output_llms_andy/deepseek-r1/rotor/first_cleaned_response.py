import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht
import pychrono.vehicle as vehicle

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

    
    length = 2.0  
    num_control_points = 5  
    order = 2  

    
    beam_section = fea.ChBeamSectionEuler()
    beam_diameter = 0.05  
    beam_area = 3.1416 * (beam_diameter ** 2) / 4.0
    beam_material = chrono.ChMaterialSurfaceNSC()
    beam_section.SetDensity(7800)  
    beam_section.SetYoungModulus(210e9)  
    beam_section.SetGshearModulus(80e9)  
    beam_section.SetBeamRaleyghDamping(0.01)  
    beam_section.SetAsCircularSection(beam_diameter)

    
    builder = fea.ChBuilderBeamIGA()
    builder.BuildBeam(system,                     
                      beam_section,               
                      num_control_points,         
                      chrono.ChVectorD(0, 0, 0),  
                      chrono.ChVectorD(0, 0, length),  
                      order)                      
    beam = builder.GetLastBeam()

    
    flywheel_radius = 0.3
    flywheel_thickness = 0.1
    flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, flywheel_thickness,  
                                         7800,  
                                         True,  
                                         True)  
    flywheel.SetPos(chrono.ChVectorD(0, 0, length / 2))  
    system.Add(flywheel)

    
    center_node_idx = num_control_points // 2
    center_node = beam.GetNode(center_node_idx)
    constraint = chrono.ChLinkPointFrame()
    constraint.Initialize(center_node, flywheel)
    system.Add(constraint)

    
    motor_function = chrono.ChFunction_Ramp(0, 10.0)  
    motor = chrono.ChLinkMotorRotationAngle()
    motor.Initialize(beam.GetNode(0),  
                     flywheel,         
                     chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))  
    )
    motor.SetAngleFunction(motor_function)
    system.Add(motor)

    
    vis = irrlicht.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Jeffcott Rotor Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1.5, 0.5, 1.0), chrono.ChVectorD(0, 0, length / 2))
    vis.AddTypicalLights()

    
    vis_fem = fea.ChVisualizationFEAmesh(beam)
    vis_fem.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    vis_fem.SetColorscaleMinMax(-500, 500)
    vis_fem.SetSmoothFaces(True)
    vis_fem.SetWireframe(False)
    beam.AddAsset(vis_fem)

    
    time_step = 0.001
    time_end = 10.0
    time = 0

    
    while vis.Run() and time < time_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
        time += time_step

if __name__ == "__main__":
    main()