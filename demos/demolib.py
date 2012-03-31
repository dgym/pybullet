
import time

import pygame.display

from OpenGL.GL import (
    GL_TRIANGLE_STRIP, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
    glPushMatrix, glPopMatrix, glColor, glClear,
    glBegin, glEnd, glTranslate, glVertex)
from OpenGL.GLU import gluNewQuadric, gluSphere

from bullet.bullet import (
    Vector3, Transform, BoxShape, SphereShape, DefaultMotionState, RigidBody)


class Ground:
    def __init__(self):
        self.boxHalfExtents = Vector3(20, 2, 20)
        groundShape = BoxShape(self.boxHalfExtents)
        groundTransform = Transform()
        groundTransform.setIdentity()
        groundTransform.setOrigin(Vector3(0, -4, 0))
        groundMotion = DefaultMotionState()
        groundMotion.setWorldTransform(groundTransform)
        self.body = RigidBody(groundMotion, groundShape)
        self.body.setRestitution(0.5)
        self.motion = groundMotion


    def render(self):
        x, y, z = (
            self.boxHalfExtents.x, self.boxHalfExtents.y, self.boxHalfExtents.z)
        o = self.motion.getWorldTransform().getOrigin()
        glColor(0, 0, 255)
        glTranslate(o.x, o.y, o.z)
        glBegin(GL_TRIANGLE_STRIP)
        glVertex(-x, y, -z)
        glVertex(x, y, -z)
        glVertex(-x, y, z)
        glVertex(x, y, z)
        glEnd()



class Ball:
    def __init__(self, position, color, radius=2):
        self.radius = radius
        ballShape = SphereShape(self.radius)
        ballTransform = Transform()
        ballTransform.setIdentity()
        ballTransform.setOrigin(position)
        ballMotion = DefaultMotionState()
        ballMotion.setWorldTransform(ballTransform)
        self.body = RigidBody(ballMotion, ballShape, 2.0)
        self.body.setRestitution(0.9)
        self.motion = ballMotion
        self.quad = gluNewQuadric()
        self.color = color


    def render(self):
        o = self.motion.getWorldTransform().getOrigin()
        glColor(*self.color)
        glTranslate(o.x, o.y, o.z)
        gluSphere(self.quad, self.radius, 25, 25)


def step(world):
    timeStep = fixedTimeStep = 1.0 / 60.0
    world.stepSimulation(timeStep, 1, fixedTimeStep)
    now = time.time()
    delay = now % timeStep
    time.sleep(delay)


def render(objects):
    glPushMatrix()
    for o in objects:
        glPushMatrix()
        o.render()
        glPopMatrix()
    glPopMatrix()

    pygame.display.flip()
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


def simulate(world, objects):
    while True:
        step(world)
        render(objects)

