# Copyright (c) 2010 Jean-Paul Calderone
# See LICENSE for details.

import sys

import bullet

def memused():
    return [
        int(line.split(None)[1])
        for line in file('/proc/self/status')
        if line.startswith('VmSize')][0]



def measure(n, params):
    type, args = params[0], params[1:]
    type(*args)
    before = memused()
    print type.__name__, before,
    sys.stdout.flush()
    for i in n:
        type(*args)
    after = memused()
    print after, after - before



def main():
    n = xrange(2 * 10 ** 3)
    for type in [
        (bullet.Vector3, 1, 2, 3),
        (bullet.Box2dShape, bullet.Vector3(1, 2, 3)),
        (bullet.BoxShape, bullet.Vector3(1, 2, 3)),
        (bullet.SphereShape, 3.0,),
        (bullet.CylinderShape, bullet.Vector3(1, 2, 3)),
        (bullet.CylinderShapeX, bullet.Vector3(1, 2, 3)),
        (bullet.CylinderShapeZ, bullet.Vector3(1, 2, 3)),
        (bullet.StaticPlaneShape, bullet.Vector3(1, 2, 3), 1),
        (bullet.CapsuleShape, 3.0, 5.0),
        (bullet.IndexedMesh,),
        (bullet.TriangleIndexVertexArray,),
        (bullet.BvhTriangleMeshShape, bullet.TriangleIndexVertexArray()),
        (bullet.CollisionObject,),
        (bullet.Transform,),
        (bullet.DefaultMotionState,),
        (bullet.RigidBody,),
        (bullet.KinematicCharacterController,
         bullet.BoxShape(bullet.Vector3(1, 2, 3)), 5, 1),
        (bullet.CollisionDispatcher,),
        (bullet.OverlappingPairCache,),
        (bullet.AxisSweep3, bullet.Vector3(0, 0, 0), bullet.Vector3(2, 2, 2)),
        (bullet.SequentialImpulseConstraintSolver,),
        (bullet.CollisionWorld,),
        (bullet.DynamicsWorld,),
        (bullet.DiscreteDynamicsWorld,),
                 ]:
        measure(n, type)


if __name__ == '__main__':
    main()
