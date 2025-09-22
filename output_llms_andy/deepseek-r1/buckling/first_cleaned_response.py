import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    length = 2.0       
    width = 0.05       
    height = 0.05      
    density = 7800     
    E = 2.0e11         
    nu = 0.3           
    num_elements = 10  

    
    section = fea.ChBeamSectionEuler()
    section.SetAsRectangularSection(width, height)
    section.SetDensity(density)
    section.SetYoungModulus(E)
    section.SetShearModulus(E/(2*(1+nu)))
    material = fea.ChBeamMaterialEuler(section)

    
    builder = fea.ChBuilderBeamEuler()
    builder.BuildBeam(
        mesh,           
        material,       
        num_elements,   
        chrono.ChVectorD(0, 0, 0),        
        chrono.ChVectorD(length, 0, 0),   
        chrono.ChVectorD(0, 0, 1)         
    )
    beam_nodes = builder.GetLastBeamNodes()

    
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(beam_nodes[0], beam_nodes[0])
    mesh.Add(constraint)

    
    perturb_force = fea.ChLoadPointForce(beam_nodes[int(num_elements/2)])
    perturb_force.SetForce(chrono.ChVectorD(0, 1, 0))  
    mesh.Add(perturb_force)

    
    class CompressiveForce(chrono.ChLoad):
        def __init__(self, node):
            chrono.ChLoad.__init__(self, node)
            self.time = 0
            self.max_force = -70000  
            self.ramp_time = 2.0     
            
        def ComputeForce(self, node, force, torque):
            if self.time < self.ramp_time:
                factor = self.time / self.ramp_time
            else:
                factor = 1.0
            force.Set(factor * self.max_force, 0, 0)
            torque.SetNull()
            
        def Update(self, time):
            self.time = time

    compressive_load = CompressiveForce(beam_nodes[-1])
    mesh.Add(compressive_load)

    
    solver = chrono.ChSolverMINRES()
    system.SetSolver(solver)
    solver.SetMaxIterations(100)
    solver.SetTolerance(1e-12)
    system.SetSolverForceTolerance(1e-10)
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    hht_stepper = system.GetTimestepper().AsTimestepperHHT()
    hht_stepper.SetAlpha(-0.2)  

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Beam Buckling Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1.5, 0.5, 1.0), chrono.ChVectorD(0.5, 0, 0))
    vis.AddTypicalLights()

    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()