#include <list>

struct PyBulletCollision
{
    PyBulletCollision()
    {
    }

    PyBulletCollision(float distance, const btVector3 &position, const btVector3 &collision_normal, int triangle_index) :
        m_distance(distance),
        m_position(position),
        m_collision_normal(collision_normal),
        m_triangle_index(triangle_index)
    {
    }

    float m_distance;
    btVector3 m_position;
    btVector3 m_collision_normal;
    int m_triangle_index;
};

struct PyBulletCollisionResults : public btCollisionWorld::ConvexResultCallback, public btCollisionWorld::ContactResultCallback
{
    std::list<PyBulletCollision> m_collisions;

    virtual btScalar addSingleResult(btManifoldPoint &cp, const btCollisionObject *colObj0, int partId0, int index0, const btCollisionObject *colObj1, int partId1, int index1)
    {
        const btVector3 &pos = cp.getPositionWorldOnA();
        m_collisions.push_back(PyBulletCollision(0.0,
                                                 btVector3(pos.getX(),
                                                           pos.getY(),
                                                           pos.getZ()),
                                                 btVector3(cp.m_normalWorldOnB.getX(),
                                                           cp.m_normalWorldOnB.getY(),
                                                           cp.m_normalWorldOnB.getZ()),
                                                 index1));

        return 1.f;
    }

    virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult &convexResult, bool normalInWorldSpace)
    {
        std::list<PyBulletCollision>::iterator iter;
        for (iter = m_collisions.begin(); iter != m_collisions.end(); ++iter)
        {
            if (convexResult.m_hitFraction < iter->m_distance)
                break;
        }

        m_collisions.insert(iter,
                            PyBulletCollision(convexResult.m_hitFraction,
                                              btVector3(convexResult.m_hitPointLocal.getX(),
                                                        convexResult.m_hitPointLocal.getY(),
                                                        convexResult.m_hitPointLocal.getZ()),
                                              btVector3(convexResult.m_hitNormalLocal.getX(),
                                                        convexResult.m_hitNormalLocal.getY(),
                                                        convexResult.m_hitNormalLocal.getZ()),
                                              convexResult.m_localShapeInfo->m_triangleIndex));

        return 1.f;
    }

    bool more()
    {
        return m_collisions.size() > 0;
    }

    PyBulletCollision pop()
    {
        PyBulletCollision col = m_collisions.front();
        m_collisions.pop_front();
        return col;
    }
};

void pybullet_contact(btCollisionWorld *world, btConvexShape *shape, btTransform trans, PyBulletCollisionResults *results)
{
    // Clear previous results
    results->m_collisions.clear();

    btCollisionObject object;
    object.setCollisionShape(shape);
    object.setWorldTransform(trans);

    world->contactTest(&object, *results);
}

void pybullet_sweep(btCollisionWorld *world, btConvexShape *shape, btTransform trans_from, btTransform trans_to, PyBulletCollisionResults *results)
{
    // Clear previous results
    results->m_collisions.clear();

    // The current version of bullet does not report existing contacts
    // during a sweep (only new ones), so we start with a contactTest
    btCollisionObject object;
    object.setCollisionShape(shape);
    object.setWorldTransform(trans_from);

    world->contactTest(&object, *results);

    // Then perform the sweep
    world->convexSweepTest(shape, trans_from, trans_to, *results, 0.f);
}
