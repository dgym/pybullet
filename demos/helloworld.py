# Copyright (c) 2010 Jean-Paul Calderone
# See LICENSE for details.

from bullet.bullet import (
    Vector3, Transform,
    BoxShape,
    DefaultMotionState,
    RigidBody,
    DiscreteDynamicsWorld)

def main():
    dynamicsWorld = DiscreteDynamicsWorld()

    groundShape = BoxShape(Vector3(50, 50, 50))
    groundTransform = Transform()
    groundTransform.setIdentity()
    groundTransform.setOrigin(Vector3(0, -56, 0))
    groundMotion = DefaultMotionState()
    groundMotion.setWorldTransform(groundTransform)
    ground = RigidBody(groundMotion, groundShape)
    dynamicsWorld.addRigidBody(ground)

    ballShape = BoxShape(Vector3(1, 1, 1))
    ballTransform = Transform()
    ballTransform.setIdentity()
    ballTransform.setOrigin(Vector3(2, 10, 0))
    ballMotion = DefaultMotionState()
    ballMotion.setWorldTransform(ballTransform)
    ball = RigidBody(ballMotion, ballShape, 1.0)
    dynamicsWorld.addRigidBody(ball)

    for i in range(100):
        dynamicsWorld.stepSimulation(1.0 / 60.0, 10)

        for obj in ballMotion, groundMotion:
            o = obj.getWorldTransform().getOrigin()
            print 'world pos = %0.6f,%0.6f,%0.6f' % (o.x, o.y, o.z)


main()
