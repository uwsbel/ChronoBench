import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr



class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.mesh = mesh
        self.system = system

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        
        builder = fea.ChBuilderCableANCF()
        
        for i in range(self.n_chains):
            offset = i * 0.2  
            builder.BuildBeam(
                self.mesh,  
                msection_cable2,  
                15,  
                chrono.ChVector3d(-0.1 + offset, 0.5, -0.1),  
                chrono.ChVector3d(-0.1 + offset, 0.5, 0.5)  
            )

            
            
            end_nodes = builder.GetLastBeamNodes()
            end_node = end_nodes.back()
            end_node.SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(end_node, mtruss)
            self.system.Add(constraint_hinge)  

            
            box = chrono.ChBody()
            box.SetMass(0)
            box.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
            box.SetPos(end_node.GetPos())
            box.SetFixed(False)
            box.SetName("box")
            self.system.Add(box)

            
            rev = fea.ChLinkLockRevolute()
            rev.Initialize(box, end_node)
            self.system.Add(rev)

        
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)  

        
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(builder.GetFirstBeamNode(), mtruss)
        self.system.Add(constraint_hinge)  

    def PrintBodyPositions(self):
        end_bodies = []
        for i in range(self.n_chains):
            offset = i * 0.2
            end_body = chrono.CastContactBodyToChBody(self.mesh.GetNthBody(2 + i * 15 + 1))
            end_bodies.append(end_body)
        print("Positions of end bodies:")
        for body in end_bodies:
            print(body.GetPos())



def main():
    
    sys = chrono.ChSystemSMC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    
    mesh = fea.ChMesh()

    
    model = Model1(sys, mesh)
    sys.GetSolver().AsIterative().SetMaxIterations(100)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('FEA cables')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVector3d(0,0.6,-1))
    vis.AddTypicalOrbitCamera(trackPoint=chrono.ChVector3d(0,0.5,0.5))

    
    camTrackRotateTime = 2.0  
    camTrackZoomTime = 1.0   
    vis.SetCameraTrackSpeeds(camTrackRotateTime, camTrackZoomTime)

    
    time = 0
    time_step = 1e-3
    time_end = 10

    
    frame_rate = 0

    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())

    
    render_frames = 0

    while vis.Run() :
        time = sys.GetChTime()

        
        if (render_frames % 10 == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        render_frames += 1

        
        frame_number = int(time / time_step)

        
        if (frame_number <= 0) :
            break

        
        if (frame_number % 20 == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        sys.DoStepDynamics(time_step)

        
        vis.Advance(time_step)

        
        frame_number += 1

    return 0













main()