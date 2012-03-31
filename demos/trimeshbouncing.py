# Copyright (c) PyBullet Team
# See LICENSE for details.

"""
This module demonstrates rigid bodies with a collision shape defined by a
triangle mesh.  The collision shape BvhTriangleMeshShape is defined in terms of
a TriangleIndexVertexArray which is defined in terms of one or more
IndexedMeshes.
"""


from numpy import array

import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_DEPTH_TEST, GL_TRIANGLES,
    glEnable, glColor,
    glBegin, glEnd, glTranslate, glVertex)
from OpenGL.GLU import gluPerspective

from bullet.bullet import (
    Vector3, Transform,
    IndexedMesh, TriangleIndexVertexArray, BvhTriangleMeshShape,
    DefaultMotionState,
    RigidBody,
    DiscreteDynamicsWorld)

from demolib import Ball, simulate

class Ground:
    def __init__(self):
        # Vertices are coordinates in three-space.  These four will define two
        # triangles, with two vertices shared between them.
        self.groundVertices = array([
                0,  0, 0,
                10, 0, 0,
                10, 0, 10,
                0,  0, 10], 'f')

        # These indices are used to find vertices in the groundVertices array.
        # They index a vertex, not an individual float.  So a value of 1 refers
        # to the 2nd triple in groundVertices (ie 10, 0, 0).  Three indices
        # define one triangle, so this array defines two triangles.
        self.groundIndices = array(
            [0, 1, 2,
             2, 3, 0],
            'i')

        # Create a single triangle mesh to use as part of the array of triangle
        # meshes that will define the shape of the ground object.
        groundMesh = IndexedMesh()

        # Set the indices of this mesh.  Specify the number of triangles it
        # will define (2), the stride in bytes between the beginning of index
        # triples, and the array actually containing the data.  Since this
        # index data is tightly packed, the stride is just the size of a single
        # index times the number of indices per triangle (3).
        groundMesh.setIndices(
            2, 3 * self.groundIndices.itemsize, self.groundIndices)

        # Set the vertex data of this mesh.  Specify the number of vertices it
        # will define (4), the stride in bytes between the beginning of vertex
        # triples, and the array actually containing the data.  Since the
        # vertex data is tightly packed, the stride is just the size of a
        # single vertex component (ie x, y, or z value) times the number of
        # components per vertex (3).
        groundMesh.setVertices(
            4, 3 * self.groundVertices.itemsize, self.groundVertices)

        # Create a container for the single IndexedMesh, but we could add more
        # to this if we had any.
        groundStuff = TriangleIndexVertexArray()
        groundStuff.addIndexedMesh(groundMesh)

        # Create a shape based on that array of IndexedMesh instances.
        groundShape = BvhTriangleMeshShape(groundStuff)

        # Since the data is completely specified at this point, tell the shape
        # to build its optimized internal data structure.  XXX This is somewhat
        # bad, and forgetting to do it will cause problems like segfaults.  It
        # would be nice if it were not required.
        groundShape.buildOptimizedBvh()

        # Position the ground at an offset so that it's actually centered at
        # the origin (we could also have used -5s and 5s in the vertex data,
        # instead of 0s and 10s).
        groundTransform = Transform()
        groundTransform.setIdentity()
        groundTransform.setOrigin(Vector3(-5, -5, -5))

        # Install a motion state object which we can inspect later to find out
        # if the ground has moved (it won't, though).
        groundMotion = DefaultMotionState()
        groundMotion.setWorldTransform(groundTransform)

        # Create the actual collision object using the motion state and the
        # shape.
        self.body = RigidBody(groundMotion, groundShape)
        self.motion = groundMotion


    def render(self):
        o = self.motion.getWorldTransform().getOrigin()
        glColor(0, 0, 255)
        glTranslate(o.x, o.y, o.z)
        glBegin(GL_TRIANGLES)
        for i in range(len(self.groundIndices)):
            base = self.groundIndices[i] * 3
            x = self.groundVertices[base]
            y = self.groundVertices[base + 1]
            z = self.groundVertices[base + 2]
            glVertex(x, y, z)
        glEnd()



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
    objects.append(Ball(Vector3(-5.1, 10, -5.1), (255, 0, 0)))
    objects.append(Ball(Vector3(-5.1, 20, 5.1), (0, 255, 0)))
    objects.append(Ball(Vector3(5.1, 30, -5.1), (255, 255, 0)))
    objects.append(Ball(Vector3(5.1, 40, 5.1), (0, 255, 255, 0)))
    objects.append(Ball(Vector3(0, 50, 0), (255, 0, 255)))

    for o in objects:
        dynamicsWorld.addRigidBody(o.body)

    simulate(dynamicsWorld, objects)


if __name__ == '__main__':
    main()

