import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math




print("=" * 60)
print("  Jeffcott Rotor – IGA Beam Demo")
print("=" * 60)

sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
hht = sys.GetTimestepper()
hht.SetAlpha(-0.2)
hht.SetMaxiters(10)
hht.SetAbsTolerances(1e-10, 1e-10)


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(500)
solver.SetTolerance(1e-12)
sys.SetSolver(solver)




mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
sys.Add(mesh)




L_shaft    = 1.0       
r_shaft    = 0.01      
rho_shaft  = 7800.0    
E_shaft    = 210e9     
nu_shaft   = 0.3
G_shaft    = E_shaft / (2.0 * (1.0 + nu_shaft))

A_s  = math.pi * r_shaft**2
Iy_s = math.pi * r_shaft**4 / 4.0
Iz_s = math.pi * r_shaft**4 / 4.0
J_s  = math.pi * r_shaft**4 / 2.0   


minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(rho_shaft)
minertia.SetArea(A_s)
minertia.SetIyy(Iy_s)
minertia.SetIzz(Iz_s)


melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(E_shaft)
melasticity.SetGshearModulus(G_shaft)
melasticity.SetArea(A_s)
melasticity.SetIyy(Iy_s)
melasticity.SetIzz(Iz_s)
melasticity.SetJ(J_s)


msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(r_shaft * 4)   




N_ELEM   = 20    
ORDER    = 3     

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    N_ELEM,
    chrono.ChVectorD(0.0,      0.0, 0.0),   
    chrono.ChVectorD(L_shaft,  0.0, 0.0),   
    chrono.VECT_Y,                           
    ORDER
)

beam_nodes   = builder.GetLastBeamNodes()
n_nodes      = len(beam_nodes)
node_A       = beam_nodes[0]              
node_B       = beam_nodes[n_nodes - 1]   
node_center  = beam_nodes[n_nodes // 2]  

print(f"  IGA shaft: {N_ELEM} elements | {n_nodes} nodes | order {ORDER}")
print(f"  Left end :  {node_A.GetPos()}")
print(f"  Center   :  {node_center.GetPos()}")
print(f"  Right end:  {node_B.GetPos()}")




ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetName("ground")
sys.Add(ground)




rot_speed_hz = 20.0                          
omega_motor  = 2.0 * math.pi * rot_speed_hz  

motor_frame = chrono.ChFrameD(
    node_A.GetPos(),
    chrono.ChQuaternionD(1, 0, 0, 0)         
)
motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("shaft_motor")
motor.Initialize(node_A, ground, motor_frame)
motor.SetSpeedFunction(chrono.ChFunction_Const(omega_motor))
sys.Add(motor)

print(f"  Motor: {rot_speed_hz} Hz  ({rot_speed_hz * 60:.0f} RPM)")





bearing_B = chrono.ChLinkMateGeneric()
bearing_B.SetName("bearing_right")
bearing_B.Initialize(
    node_B, ground, False,
    chrono.ChFrameD(node_B.GetPos()),
    chrono.ChFrameD(node_B.GetPos())
)

bearing_B.SetConstrainedCoords(True, True, True, False, True, True)
sys.Add(bearing_B)




m_disk   = 0.5    
r_disk   = 0.08   
h_disk   = 0.015  
e_disk   = 0.001  


Ix_disk = 0.5  * m_disk * r_disk**2
Iy_disk = 0.25 * m_disk * r_disk**2 + m_disk * h_disk**2 / 12.0
Iz_disk = Iy_disk

disk = chrono.ChBody()
disk.SetName("flywheel")
disk.SetMass(m_disk)
disk.SetInertiaXX(chrono.ChVectorD(Ix_disk, Iy_disk, Iz_disk))
disk.SetPos(node_center.GetPos())
sys.Add(disk)


cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().rad = r_disk
cyl.GetCylinderGeometry().p1  = chrono.ChVectorD(+h_disk / 2, 0, 0)
cyl.GetCylinderGeometry().p2  = chrono.ChVectorD(-h_disk / 2, 0, 0)
disk.AddAsset(cyl)

disk_col = chrono.ChColorAsset()
disk_col.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
disk.AddAsset(disk_col)


eccentricity_sphere = chrono.ChSphereShape()
eccentricity_sphere.GetSphereGeometry().rad    = 0.005
eccentricity_sphere.GetSphereGeometry().center = chrono.ChVectorD(0, e_disk, 0)
disk.AddAsset(eccentricity_sphere)

print(f"  Flywheel: m={m_disk} kg, R={r_disk*1000:.0f} mm, e={e_disk*1000:.1f} mm")




attach = chrono.ChLinkMateGeneric()
attach.SetName("disk_attach")
attach.Initialize(
    disk, node_center, False,
    chrono.ChFrameD(disk.GetPos()),
    chrono.ChFrameD(node_center.GetPos())
)
attach.SetConstrainedCoords(True, True, True, True, True, True)   
sys.Add(attach)





vis_surf = fea.ChVisualizationFEAmesh(mesh)
vis_surf.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
vis_surf.SetSmoothFaces(True)
mesh.AddAsset(vis_surf)


vis_dots = fea.ChVisualizationFEAmesh(mesh)
vis_dots.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
vis_dots.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
vis_dots.SetSymbolsThickness(0.006)
mesh.AddAsset(vis_dots)


vis_csys = fea.ChVisualizationFEAmesh(mesh)
vis_csys.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_CSYS)
vis_csys.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
vis_csys.SetSymbolsThickness(0.003)
vis_csys.SetSymbolsScale(0.025)
mesh.AddAsset(vis_csys)




application = chronoirr.ChIrrApp(
    sys,
    "Jeffcott Rotor – IGA Beam Simulation",
    chronoirr.dimension2du(1280, 720)
)
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(
    chronoirr.vector3df(0.5,  0.5, 1.6),   
    chronoirr.vector3df(0.5,  0.0, 0.0)    
)

application.AssetBindAll()
application.AssetUpdateAll()

dt = 5e-5         
application.SetTimestep(dt)
application.SetTryRealtime(False)




print("\n  Starting simulation loop … (close window to stop)\n")

t_max      = 5.0    
last_print = -0.1   

while application.GetDevice().run():
    t = sys.GetChTime()

    
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()

    
    if t - last_print >= 0.1:
        last_print = t
        p  = node_center.GetPos()
        r_mm = math.sqrt(p.y**2 + p.z**2) * 1e3   
        ang  = motor.GetMotorRot()                   
        rpm  = omega_motor / (2.0 * math.pi) * 60.0
        print(f"  t={t:6.3f} s  |  centre radial disp = {r_mm:.4f} mm  "
              f"|  motor angle = {ang:8.2f} rad  |  {rpm:.0f} RPM")

    
    application.DoStep()
    application.EndScene()

    if t >= t_max:
        print(f"\n  Reached t_max = {t_max} s – stopping.")
        break

print("\n  Simulation complete.")