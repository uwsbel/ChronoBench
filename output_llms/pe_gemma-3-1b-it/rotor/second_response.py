import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True,
                         2)  # for max precision in gravity of FE, suitable for large numbers of contact points with complex behaviors
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0));

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.1456

# Create a material for non-smooth contact interactions, commonly used for rigid body simulations.
# This is a crucial step for ensuring accurate and stable simulations.
material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)

material_smc = chrono.ChContactMaterialSMC()
material_smc.SetFriction(0.5)
material_smc.SetDampingF(0.1)
material_smc.SetCompliance(0.01)

material.SetColor(chrono.ChColor(1, 0, 0))  # Red color
material.SetFriction(0.5)
material.SetDampingF(0.1)
material.SetCompliance(0.01)

material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)

material_smc = chrono.ChContactMaterialSMC()
material_smc.SetFriction(0.5)
material_smc.SetDampingF(0.1)
material_smc.SetCompliance(0.01)

material.SetFriction(0.5)
material.SetDampingF(0.1)
material.SetCompliance(0.01)

material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)

material_smc = chrono.ChContactMaterialSMC()
material_smc.SetFriction(0.5)
material_smc.SetDampingF(0.1)
material_smc.SetCompliance(0.01)

material.SetFriction(0.5)
material.SetDampingF(0.1)
material.SetCompliance(0.01)

material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)

material_smc = chrono.ChContactMaterialSMC()
material_smc.SetFriction(0.5)
material_smc.SetDampingF(0.1)
material_smc.SetCompliance(0.01)

material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)

material_smc = chrono.ChContactMaterialSMC()
material_smc.SetFriction(0.5)
material_smc.SetDampingF(0.1)
material_smc.SetCompliance(0.01)

material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)

material_smc = chrono.ChContactMaterialSMC()
material_smc.SetFriction(0.5)
material_smc.SetDampingF(0.1)
material_smc.SetCompliance(0.01)

material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)

material_smc = chrono.ChContactMaterialSMC()
material_smc.SetFriction(0.5)
material_smc.SetDampingF(0.1)
material_smc.SetCompliance(0.01)

material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)

material_smc = chrono.ChContactMaterialSMC()
material_smc.SetFriction(0.5)
material_smc.SetDampingF(0.1)
material_smc.SetCompliance(0.01)

# Create a section, i.e. thickness and material properties
# for beams. This will be shared among some beams.

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800);
minertia.SetArea(chrono.ChVector3d(0, 0, 0))
minertia.SetSection(beam_L)
minertia.SetYoungModulus(0.01e9)
minertia.SetShearModulusFromPoisson(0.3)
minertia.SetIyy((chrono.ChFramed(chrono.ChVector3d(0, 0, 0))) * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIzz((chrono.ChFramed(chrono.ChVector3d(0, 0, 0))) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetJ((chrono.ChFramed(chrono.ChVector3d(0, 0, 0))) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

# Create the flywheel and attach it to the center of the beam
body_flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
body_flywheel.SetCoordsys(
    chrono.ChCoordsysd(minertia.GetPos() + chrono.ChVector3d(0, 0.05, 0),  # flywheel initial center (plus Y offset)
                       chrono.QuatFromAngleAxis(chrono.ChPI / 2.0, chrono.VECT_Z))
)
sys.Add(body_flywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(minertia, body_floor, chrono.ChFramed())
sys.Add(myjoint)

# Create the end bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(minertia, body_floor, chrono.ChFramed(minertia.GetPos(), chrono.QUNIT))
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(minertia, body_floor, chrono.ChFramed())
rotmotor1.SetMotorFunction(f_ramp)

# Attach a visualization of the FEM mesh.
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)