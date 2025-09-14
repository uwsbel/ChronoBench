import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht
import pychrono.mkl as mkl

def main():
    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    mesh = fea.ChMesh()
    material = fea.ChMaterialBeamEuler(chrono.ChMaterialSurface.SMC)
    beam_section = fea.ChBeamSectionEulerAdvanced()
    
    
    density = 7850
    E = 2e11
    nu = 0.3
    ks = 0.5
    area = 0.05*0.05
    Iyy = (0.05**4)/12
    Izz = Iyy

    material.SetYoungModulus(E)
    beam_section.SetDensity(density)
    beam_section.SetAsRectangularSection(0.05, 0.05)
    
    
    builder = fea.ChBuilderBeamEuler()
    node_start = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    mesh.AddNode(node_start)
    
    builder.BuildBeam(mesh,                      
                      material,                  
                      beam_section,              
                      5.0,                       
                      0.1,                       
                      chrono.ChVectorD(1, 0, 0), 
                      fea.ANCF_BEZIER_STAIRS,    
                      node_start)                

    
    fixed = chrono.ChLinkPointFrame()
    fixed.Initialize(node_start, system.Get_bodyframe())
    system.Add(fixed)

    
    class MotorFunction(chrono.ChFunction):
        def __init__(self):
            super().__init__()
            self.time = 0
        def Get_y(self, x):
            return -0.001 * x  

    motor_func = MotorFunction()
    
    
    node_end = builder.GetLastBeamNodes()[1]
    motor = fea.ChLoadMotorLinearPosition()
    motor.SetMotionLaw(motor_func)
    motor.SetMotorNode(node_end)
    motor.SetDirection(chrono.ChVectorD(1, 0, 0))  
    motor.SetMotorSystem(system)
    system.Add(motor)

    
    system.Add(mesh)

    
    solver = mkl.ChSolverMKL()
    system.SetSolver(solver)
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    system.SetTimestepperTolerance(1e-6)

    
    vis = irrlicht.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Beam Buckling FEA')
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(5, 3, 0.5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    
    
    fea_vis = fea.ChVisualizationFEAmesh(mesh)
    fea_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    mesh.AddAsset(fea_vis)

    
    timestep = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(timestep)

if __name__ == "__main__":
    main()