import pychrono as chrono
import pychrono Irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


mass = chrono.ChBody()
mass.SetMass(1.0)
mass.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
mass.SetPos(chrono.ChVectorD(0, -1, 0))
mass.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
mass.SetBodyFixed(False)
system.Add(mass)



spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(
    ground,                
    mass,                  
    False,                 
    chrono.ChVectorD(0, 0, 0),    
    chrono.ChVectorD(0, 0, 0)     
)


spring_const = 50    
damping_const = 2    
rest_length = 1.0    

spring_damper.Set_springRestLength(rest_length)
spring_damper.Set_springK(spring_const)
spring_damper.Set_springR(damping_const)

system.AddLink(spring_damper)



application = chronoirr.ChIrrApp(system, "Mass-Spring-Damper Example", chronoirr.dimension2du(800, 600))

application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1.5, 1.0, 4), chronoirr.vector3df(0, -1, 0))  


mvisual_ground = chrono.ChVisualShapeBox()
mvisual_ground.SetBoxSize(chrono.ChVectorD(2, 0.05, 2))
mvisual_ground.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(mvisual_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))


box_shape = chrono.ChBoxShape()
box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)
box_shape.SetColor(chrono.ChColor(0.2, 0.7, 0.2))
mass.AddVisualShape(box_shape)


class SpringVisualizer(chrono.ChVisualShape):
    def __init__(self, link_tsda):
        super().__init__()
        self.link = link_tsda

    def Update(self):
        
        pass



line_shape = chrono.ChLineShape()
line_shape.SetColor(chrono.ChColor(0.9, 0.1, 0.1))
application.GetSceneManager().getRootSceneNode().addChild(line_shape)

def update_spring_line():
    try:
        
        pA = spring_damper.GetMarker1().GetAbsCoord().pos
        pB = spring_damper.GetMarker2().GetAbsCoord().pos

        
        vA = chronoirr.vector3df(float(pA.x), float(pA.y), float(pA.z))
        vB = chronoirr.vector3df(float(pB.x), float(pB.y), float(pB.z))

        
        
        
        

    except:
        pass




cyl = chrono.ChBody()
cyl.SetBodyFixed(True)  
system.Add(cyl)

cyl_shape = chrono.ChCylinderShape()
cyl_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
cyl_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 1)  
cyl_shape.GetCylinderGeometry().rad = 0.03
cyl_shape.SetColor(chrono.ChColor(0.7, 0.2, 0.2))
cyl.AddVisualShape(cyl_shape)

def update_cylinder_visual():
    pA = spring_damper.GetMarker1().GetAbsCoord().pos
    pB = spring_damper.GetMarker2().GetAbsCoord().pos

    diff = pB - pA
    length = diff.Length()

    if length < 1e-6:
        length = 1e-6
        diff = chrono.ChVectorD(0, 1, 0)

    
    z_axis = chrono.ChVectorD(0, 0, 1)
    axis = chrono.Vcross(z_axis, diff)
    angle = chrono.Vdot(z_axis, diff)

    
    midpoint = (pA + pB) * 0.5

    
    rot = chrono.Q_from_AngAxis(diff.GetNormalized(), 0)  
    
    import math
    from math import acos, degrees
    dot = z_axis.Dot(diff.GetNormalized())
    if abs(dot) < 1.0:
        rot_axis = z_axis.Cross(diff.GetNormalized()).GetNormalized()
        rot_angle = math.acos(dot)
        rot = chrono.Q_from_AngAxis(rot_angle, rot_axis)
    else:
        
        if dot > 0:
            rot = chrono.QIDENTITY
        else:
            rot = chrono.Q_from_AngAxis(math.pi, chrono.ChVectorD(1, 0, 0))  

    
    cyl.SetPos(midpoint)
    cyl.SetRot(rot)

    
    cyl_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -length / 2)
    cyl_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, length / 2)


def custom_step():
    update_cylinder_visual()



application.SetTimestep(0.01)
application.SetTryRealtime(True)

application.AssetBindAll()
application.AssetUpdateAll()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(application.GetTimestep())
    custom_step()
    application.EndScene()