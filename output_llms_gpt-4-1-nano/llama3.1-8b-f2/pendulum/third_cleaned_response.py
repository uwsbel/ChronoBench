import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math








change_mass = False


L = 1.0


trackPoint = chrono.ChVector3d(0.0, 0.0, 0.75)


step_size = 1e-3
tire_step_size = step_size


tend = 15


render_step_size = 1.0 / 50  


out_dir = "./PENULUM"




chrono.SetChronoDataPath('.')


print( "Copyright (c) 2017 projectchrono.org\n")






system = chrono.ChSystemNSC()


system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)


pendulum1 = chrono.ChBody()
pendulum1.SetMass(20)
pendulum1.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
system.Add(pendulum1)


pendulum2 = chrono.ChBody()
pendulum2.SetMass(15)
pendulum2.SetInertiaXX(chrono.ChVector3d(5, 5, 5))
system.Add(pendulum2)


pendulum3 = chrono.ChBody()
pendulum3.SetMass(10)
pendulum3.SetInertiaXX(chrono.ChVector3d(2, 2, 2))
system.Add(pendulum3)


pendulum4 = chrono.ChBody()
pendulum4.SetMass(5)
pendulum4.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
system.Add(pendulum4)


pendulum5 = chrono.ChBody()
pendulum5.SetMass(1)
pendulum5.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
system.Add(pendulum5)


pendulum6 = chrono.ChBody()
pendulum6.SetMass(0.5)
pendulum6.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
system.Add(pendulum6)


pendulum7 = chrono.ChBody()
pendulum7.SetMass(0.2)
pendulum7.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
system.Add(pendulum7)


pendulum8 = chrono.ChBody()
pendulum8.SetMass(0.1)
pendulum8.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
system.Add(pendulum8)


pendulum9 = chrono.ChBody()
pendulum9.SetMass(0.05)
pendulum9.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
system.Add(pendulum9)


pendulum10 = chrono.ChBody()
pendulum10.SetMass(0.02)
pendulum10.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
system.Add(pendulum10)


pendulum11 = chrono.ChBody()
pendulum11.SetMass(0.01)
pendulum11.SetInertiaXX(chrono.ChVector3d(0.005, 0.005, 0.005))
system.Add(pendulum11)


pendulum12 = chrono.ChBody()
pendulum12.SetMass(0.005)
pendulum12.SetInertiaXX(chrono.ChVector3d(0.002, 0.002, 0.002))
system.Add(pendulum12)


pendulum13 = chrono.ChBody()
pendulum13.SetMass(0.002)
pendulum13.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
system.Add(pendulum13)


pendulum14 = chrono.ChBody()
pendulum14.SetMass(0.001)
pendulum14.SetInertiaXX(chrono.ChVector3d(0.0005, 0.0005, 0.0005))
system.Add(pendulum14)


pendulum15 = chrono.ChBody()
pendulum15.SetMass(0.0005)
pendulum15.SetInertiaXX(chrono.ChVector3d(0.0002, 0.0002, 0.0002))
system.Add(pendulum15)


pendulum16 = chrono.ChBody()
pendulum16.SetMass(0.0002)
pendulum16.SetInertiaXX(chrono.ChVector3d(0.0001, 0.0001, 0.0001))
system.Add(pendulum16)


pendulum17 = chrono.ChBody()
pendulum17.SetMass(0.0001)
pendulum17.SetInertiaXX(chrono.ChVector3d(0.00005, 0.00005, 0.00005))
system.Add(pendulum17)


pendulum18 = chrono.ChBody()
pendulum18.SetMass(0.00005)
pendulum18.SetInertiaXX(chrono.ChVector3d(0.00002, 0.00002, 0.00002))
system.Add(pendulum18)


pendulum19 = chrono.ChBody()
pendulum19.SetMass(0.00002)
pendulum19.SetInertiaXX(chrono.ChVector3d(0.00001, 0.00001, 0.00001))
system.Add(pendulum19)


pendulum20 = chrono.ChBody()
pendulum20.SetMass(0.00001)
pendulum20.SetInertiaXX(chrono.ChVector3d(0.000005, 0.000005, 0.000005))
system.Add(pendulum20)


pendulum21 = chrono.ChBody()
pendulum21.SetMass(0.000005)
pendulum21.SetInertiaXX(chrono.ChVector3d(0.000002, 0.000002, 0.000002))
system.Add(pendulum21)


pendulum22 = chrono.ChBody()
pendulum22.SetMass(0.000002)
pendulum22.SetInertiaXX(chrono.ChVector3d(0.000001, 0.000001, 0.000001))
system.Add(pendulum22)


pendulum23 = chrono.ChBody()
pendulum23.SetMass(0.000001)
pendulum23.SetInertiaXX(chrono.ChVector3d(0.0000005, 0.0000005, 0.0000005))
system.Add(pendulum23)


pendulum24 = chrono.ChBody()
pendulum24.SetMass(0.0000005)
pendulum24.SetInertiaXX(chrono.ChVector3d(0.0000002, 0.0000002, 0.0000002))
system.Add(pendulum24)


pendulum25 = chrono.ChBody()
pendulum25.SetMass(0.0000002)
pendulum25.SetInertiaXX(chrono.ChVector3d(0.0000001, 0.0000001, 0.0000001))
system.Add(pendulum25)


pendulum26 = chrono.ChBody()
pendulum26.SetMass(0.0000001)
pendulum26.SetInertiaXX(chrono.ChVector3d(0.00000005, 0.00000005, 0.00000005))
system.Add(pendulum26)


pendulum27 = chrono.ChBody()
pendulum27.SetMass(0.00000005)
pendulum27.SetInertiaXX(chrono.ChVector3d(0.00000002, 0.00000002, 0.00000002))
system.Add(pendulum27)


pendulum28 = chrono.ChBody()
pendulum28.SetMass(0.00000002)
pendulum28.SetInertiaXX(chrono.ChVector3d(0.00000001, 0.00000001, 0.00000001))
system.Add(pendulum28)


pendulum29 = chrono.ChBody()
pendulum29.SetMass(0.00000001)
pendulum29.SetInertiaXX(chrono.ChVector3d(0.000000005, 0.000000005, 0.000000005))
system.Add(pendulum29)


pendulum30 = chrono.ChBody()
pendulum30.SetMass(0.000000005)
pendulum30.SetInertiaXX(chrono.ChVector3d(0.000000002, 0.000000002, 0.000000002))
system.Add(pendulum30)


pendulum31 = chrono.ChBody()
pendulum31.SetMass(0.000000002)
pendulum31.SetInertiaXX(chrono.ChVector3d(0.000000001, 0.000000001, 0.000000001))
system.Add(pendulum31)


pendulum32 = chrono.ChBody()
pendulum32.SetMass(0.000000001)
pendulum32.SetInertiaXX(chrono.ChVector3d(0.0000000005, 0.0000000005, 0.0000000005))
system.Add(pendulum32)


pendulum33 = chrono.ChBody()
pendulum33.SetMass(0.0000000005)
pendulum33.SetInertiaXX(chrono.ChVector3d(0.0000000002, 0.0000000002, 0.0000000002))
system.Add(pendulum33)


pendulum34 = chrono.ChBody()
pendulum34.SetMass(0.0000000002)
pendulum34.SetInertiaXX(chrono.ChVector3d(0.0000000001, 0.0000000001, 0.0000000001))
system.Add(pendulum34)


pendulum35 = chrono.ChBody()
pendulum35.SetMass(0.0000000001)
pendulum35.SetInertiaXX(chrono.ChVector3d(0.00000000005, 0.00000000005, 0.00000000005))
system.Add(pendulum35)


pendulum36 = chrono.ChBody()
pendulum36.SetMass(0.00000000005)
pendulum36.SetInertiaXX(chrono.ChVector3d(0.00000000002, 0.00000000002, 0.00000000002))
system.Add(pendulum36)


pendulum37 = chrono.ChBody()
pendulum37.SetMass(0.00000000002)
pendulum37.SetInertiaXX(chrono.ChVector3d(0.00000000001, 0.00000000001, 0.00000000001))
system.Add(pendulum37)


pendulum38 = chrono.ChBody()
pendulum38.SetMass(0.00000000001)
pendulum38.SetInertiaXX(chrono.ChVector3d(0.000000000005, 0.000000000005, 0.000000000005))
system.Add(pendulum38)


pendulum39 = chrono.ChBody()
pendulum39.SetMass(0.000000000005)
pendulum39.SetInertiaXX(chrono.ChVector3d(0.000000000002, 0.000000000002, 0.000000000002))
system.Add(pendulum39)


pendulum40 = chrono.ChBody()
pendulum40.SetMass(0.000000000002)
pendulum40.SetInertiaXX(chrono.ChVector3d(0.000000000001, 0.000000000001, 0.000000000001))
system.Add(pendulum40)


pendulum41 = chrono.ChBody()
pendulum41.SetMass(0.000000000001)
pendulum41.SetInertiaXX(chrono.ChVector3d(0.0000000000005, 0.0000000000005, 0.0000000000005))
system.Add(pendulum41)


pendulum42 = chrono.ChBody()
pendulum42.SetMass(0.0000000000005)
pendulum42.SetInertiaXX(chrono.ChVector3d(0.0000000000002, 0.0000000000002, 0.0000000000002))
system.Add(pendulum42)


pendulum43 = chrono.ChBody()
pendulum43.SetMass(0.0000000000002)
pendulum43.SetInertiaXX(chrono.ChVector3d(0.0000000000001, 0.0000000000001, 0.0000000000001))
system.Add(pendulum43)


pendulum44 = chrono.ChBody()
pendulum44.SetMass(0.0000000000001)
pendulum44.SetInertiaXX(chrono.ChVector3d(0.00000000000005, 0.00000000000005, 0.00000000000005))
system.Add(pendulum44)


pendulum45 = chrono.ChBody()
pendulum45.SetMass(0.00000000000005)
pendulum45.SetInertiaXX(chrono.ChVector3d(0.00000000000002, 0.00000000000002, 0.00000000000002))
system.Add(pendulum45)


pendulum46 = chrono.ChBody()
pendulum46.SetMass(0.00000000000002)
pendulum46.SetInertiaXX(chrono.ChVector3d(0.00000000000001, 0.00000000000001, 0.00000000000001))
system.Add(pendulum46)


pendulum47 = chrono.ChBody()
pendulum47.SetMass(0.00000000000001)
pendulum47.SetInertiaXX(chrono.ChVector3d(0.000000000000005, 0.000000000000005, 0.000000000000005))
system.Add(pendulum47)


pendulum48 = chrono.ChBody()
pendulum48.SetMass(0.000000000000005)
pendulum48.SetInertiaXX(chrono.ChVector3d(0.000000000000002, 0.000000000000002, 0.000000000000002))
system.Add(pendulum48)


pendulum49 = chrono.ChBody()
pendulum49.SetMass(0.000000000000002)
pendulum49.SetInertiaXX(chrono.ChVector3d(0.000000000000001, 0.000000000000001, 0.000000000000001))
system.Add(pendulum49)


pendulum50 = chrono.ChBody()
pendulum50.SetMass(0.000000000000001)
pendulum50.SetInertiaXX(chrono.ChVector3d(0.0000000000000005, 0.0000000000000005, 0.0000000000000005))
system.Add(pendulum50)


pendulum51 = chrono.ChBody()
pendulum51.SetMass(0.0000000000000005)
pendulum51.SetInertiaXX(chrono.ChVector3d(0.0000000000000002, 0.0000000000000002, 0.0000000000000002))
system.Add(pendulum51)


pendulum52 = chrono.ChBody()
pendulum52.SetMass(0.0000000000000002)
pendulum52.SetInertiaXX(chrono.ChVector3d(0.0000000000000001, 0.0000000000000001, 0.0000000000000001))
system.Add(pendulum52)


pendulum53 = chrono.ChBody()
pendulum53.SetMass(0.0000000000000001)
pendulum53.SetInertiaXX(chrono.ChVector3d(0.00000000000000005, 0.00000000000000005, 0.00000000000000005))
system.Add(pendulum53)


pendulum54 = chrono.ChBody()
pendulum54.SetMass(0.00000000000000005)
pendulum54