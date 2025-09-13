import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data/')


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys, 1, 1, 1, 1000, True, True)
sys.Add(ground)
ground.SetBodyFixed(True)


beam_L = 2.0  
beam_rho = 1000  
beam_E = 2.1e7  
beam_nu = 0.3  
beamthickness = 0.1  

beam_FEA = fea.ChElementBeamIGA()
beam_FEA.SetDimensions(beam_L, beamthickness)
beam_FEA.SetDensity(beam_rho)
beam_FEA.SetYoungModulus(beam_E)
beam_FEA.SetPoissonRatio(beam_nu)
beam_FEA.SetNdivisions(10)

sys.Add(beam_FEA)


flywheel = chrono.ChBodyEasyCylinder(sys, 0.5, 0.1, 1000, True, True)
sys.Add(flywheel)
flywheel.SetCoord(chrono.ChVectorD(beam_L/2, 0, 0))
flywheel.SetBodyFixed(False)


constr = chrono.ChLinkMateGeneric()
constr.Initialize(beam_FEA, flywheel, False, chrono.ChVectorD(beam_L/2, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(constr)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam_FEA, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.Add(motor)
motor.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_C_ANG_SPEED, 1.0))


application = chronoirr.ChIrrApp(sys, 'PyChrono Simulation', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(2, 2, 2), chronoirr.vector3df(0, 0, 0))


mvisualizebeam = fea.ChVisualizationFEAmesh(beam_FEA)
mvisualizebeam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED)
mvisualizebeam.SetColorscaleMinMax(-0.5, 0.5)
mvisualizebeam.SetShrinkElements(True, 0.85)
mvisualizebeam.SetSmoothFaces(True)

application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.001)
application.SetTryRealtime(True)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()