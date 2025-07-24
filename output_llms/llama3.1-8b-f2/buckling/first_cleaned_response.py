import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import math as m







use_callbacks = False



print ("Example: Create a beam with multiple finite elements, and use it to build a buckling demo.")


beam = cascade.Beams(chrono.ChVector3d(0,0,0), chrono.ChVector3d(0,1.2,0), 
                     cascade.GetSCSBox(0.02,0.02,0.3), 
                     cascade.GetMaterialElastic(2e7,0.2), 
                     10)


beam.SetVisualType(cascade.Beams.VisualType_PRIMITIVES)


sys = chrono.ChSystemNSC()


fixedPoint = chrono.ChBody()
fixedPoint.SetFixed(True)
sys.Add(fixedPoint)




beam_body = chrono.ChBody()
sys.Add(beam_body)
beam.Initialize(fixedPoint, chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(1,0,0)), chrono.ChVector3d(0,0.2,0))
beam.SetColor(chrono.ChColor(0.8,0.8,0.5))
beam.SetName("my beam")


driver = cascade.ChCascadeDemoDriver(beam)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0,1.5,0), chrono.ChVector3d(0,0,0))
vis.AddTypicalLights()









def motor_callback(t):
    v = 0 if abs(beam.GetAngle() - 0.2) < 0.01 else -0.02
    beam_body.SetAngVelParent(chrono.ChVector3d(0,0,0))
    return v


driver_motor = cascade.ChCascadeDriverMotor(beam_body, driver, motor_callback)


vis.Run()