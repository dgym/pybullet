# Copyright (c) PyBullet Team
# See LICENSE for details.

import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_DEPTH_TEST, glEnable, glTranslate)
from OpenGL.GLU import gluPerspective

from bullet.bullet import Vector3, DiscreteDynamicsWorld

from demolib import Ground, Ball, simulate

def main():
    pygame.init()
    pygame.display.set_mode(
        (1024, 768), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
    glTranslate(0, -15, -60)

    objects = []
    dynamicsWorld = DiscreteDynamicsWorld()

    objects.append(Ground())
    objects.append(Ball(Vector3(1, 10, 0), (255, 0, 0)))
    objects.append(Ball(Vector3(0, 20, 1), (0, 255, 0)))
    objects.append(Ball(Vector3(0, 30, 1), (255, 255, 0)))
    objects.append(Ball(Vector3(0, 40, 1), (0, 255, 255, 0)))

    for o in objects:
        dynamicsWorld.addRigidBody(o.body)

    simulate(dynamicsWorld, objects)


if __name__ == '__main__':
    main()
