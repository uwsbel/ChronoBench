import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np






sys = chrono.ChSystemSMC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)








beam_wy = 0.012      
beam_wz = 0.012      


minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)              
minertia.SetArea(beam_wy * beam_wz)    
minertia.SetIyy((1.0 / 12.0) * beam_wz * beam_wy**3)
minertia.SetIzz((1.0 / 12.0) * beam_wy * beam_wz**3)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)     
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetArea(beam_wy * beam_wz)
melasticity.SetIyy((1.0 / 12.0) * beam_wz * beam_wy**3)
melasticity.SetIzz((1.0 / 12.0) * beam_wy * beam_wz**3)
melasticity.SetJ((1.0 / 12.0) * beam_wy * beam_wz * (beam_wy**2 + beam_wz**2))


msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetDrawThickness(beam_wy, beam_wz)





beam_length = 0.6     

builder = fea.ChBuilderBeamIGA()


builder.BuildBeam(
    mesh,                                       
    msection,                                   
    20,                                         
    chrono.ChVector3d(0, 0, 0),                 
    chrono.ChVector3d(beam_length, 0, 0),       
    chrono.ChVector3d(0, 1, 0),                 
    3                                           
)


beam_nodes = builder.GetLastBeamNodes()


node_start = beam_nodes[0]
node_end = beam_nodes[-1]
node_center = beam_nodes[len(beam_nodes) // 2]





flywheel = chrono.ChBody()
flywheel.SetMass(2.0)                            

flywheel.SetInertiaXX(chrono.ChVector3d(0.02, 0.01, 0.02))
flywheel.SetPos(node_center.GetPos())
sys.Add(flywheel)


flywheel_shape = chrono.ChVisualShapeCylinder(0.08, 0.02)
flywheel.AddVisualShape(
    flywheel_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2))
)


constraint_flywheel = fea.ChLinkNodeFrame()
constraint_flywheel.Initialize(node_center, flywheel)
sys.Add(constraint_flywheel)


constraint_flywheel_rot = fea.ChLinkNodeSlopeFrame()
constraint_flywheel_rot.Initialize(node_center, flywheel)
constraint_flywheel_rot.SetDirectionInBodyCoords(chrono.ChVector3d(1, 0, 0))
sys.Add(constraint_flywheel_rot)






truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)


bearing_end = chrono.ChLinkMateGeneric()
bearing_end.Initialize(
    chrono.CastToChBodyFrame(chrono.CastToChBody(node_end)) if False else None,
    None, chrono.ChFramed()
) if False else None  


constraint_end = fea.ChLinkNodeFrame()
constraint_end.Initialize(node_end, truss)
sys.Add(constraint_end)






stub_body = chrono.ChBody()
stub_body.SetMass(0.1)
stub_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
stub_body.SetPos(node_start.GetPos())
sys.Add(stub_body)


constraint_start = fea.ChLinkNodeFrame()
constraint_start.Initialize(node_start, stub_body)
sys.Add(constraint_start)

constraint_start_rot = fea.ChLinkNodeSlopeFrame()
constraint_start_rot.Initialize(node_start, stub_body)
constraint_start_rot.SetDirectionInBodyCoords(chrono.ChVector3d(1, 0, 0))
sys.Add(constraint_start_rot)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    stub_body,
    truss,
    chrono.ChFramed(node_start.GetPos(), chrono.QuatFromAngleY(chrono.CH_PI_2))
)


motor_speed = chrono.ChFunctionConst(20.0)   
motor.SetSpeedFunction(motor_speed)
sys.Add(motor)





sys.Add(mesh)






visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)


visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor with IGA Beam')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.3, 0.3, -0.6),
              chrono.ChVector3d(0.3, 0.0, 0.0))
vis.AddTypicalLights()






solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)


stepper = chrono.ChTimestepperHHT(sys)
stepper.SetStepControl(False)
sys.SetTimestepper(stepper)





time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)
```



This script models a **Jeffcott rotor** with the following elements:

1. **IGA Beam**: Built with `ChBuilderBeamIGA` using a cubic B-spline beam discretized into 20 elements with a Cosserat cross-section (steel material properties).

2. **Flywheel**: A rigid cylindrical body (`ChBody`) attached at the **center** of the beam via `ChLinkNodeFrame` and `ChLinkNodeSlopeFrame` constraints, representing the rotating disk mass.

3. **Rotational Motor**: A `ChLinkMotorRotationSpeed` drives one end of the beam at a constant angular velocity (20 rad/s), simulating the spinning shaft.

4. **Bearings**: The two beam ends are connected to a fixed `truss` (the bearing supports).

5. **Visualization**: FEM visualization shows bending moments (`DataType_ELEM_BEAM_MZ`) as a color map, plus node coordinate systems, rendered in an Irrlicht window.

> **Note**: If `ChSolverPardisoMKL` is unavailable in your build, you can substitute `chrono.ChSolverSparseQR()` or the default solver. Tune the `motor_speed` to pass through the critical (resonant) speed to observe the characteristic Jeffcott whirling behavior.