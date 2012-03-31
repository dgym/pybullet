
import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_DEPTH_TEST, GL_LINES,
    glEnable, glColor,
    glBegin, glEnd, glTranslate, glVertex)
from OpenGL.GLU import gluPerspective

from bullet.bullet import (
    DRAW_WIREFRAME, DRAW_AABB, DRAW_CONTACT_POINTS,
    Vector3, DiscreteDynamicsWorld)

from demolib import Ball, step, render


class DebugDraw:
    mode = DRAW_WIREFRAME | DRAW_AABB | DRAW_CONTACT_POINTS

    def reset(self):
        self.lines = []
        self.contacts = []


    def drawLine(self, *args):
        self.lines.append(args)


    def drawContactPoint(self, *args):
        self.contacts.append(args)


    def setDebugMode(self, mode):
        self.mode = mode


    def getDebugMode(self):
        return self.mode



def main():
    pygame.init()
    pygame.display.set_mode(
        (1024, 768), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
    glTranslate(0, -15, -60)

    objects = []
    dynamicsWorld = DiscreteDynamicsWorld()
    debug = DebugDraw()
    dynamicsWorld.setDebugDrawer(debug)

    b1 = Ball(Vector3(-30, 0, 0), (255, 0, 0))
    b1.body.applyCentralImpulse(Vector3(30, 40, 0))
    objects.append(b1)

    b2 = Ball(Vector3(+30, 0, 0), (0, 255, 0))
    b2.body.applyCentralImpulse(Vector3(-30, 40, 0))
    objects.append(b2)

    for o in objects:
        dynamicsWorld.addRigidBody(o.body)

    while True:
        step(dynamicsWorld)

        debug.reset()
        dynamicsWorld.debugDrawWorld()
        glBegin(GL_LINES)
        for line in debug.lines:
            glColor(*line[6:])
            glVertex(*line[:3])
            glVertex(*line[3:6])
        if debug.contacts:
            print 'Contact!', debug.contacts
        glEnd()

        render(objects)


if __name__ == '__main__':
    main()
