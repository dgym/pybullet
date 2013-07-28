#include <list>

struct PyBulletCollision
{
    PyBulletCollision()
    {
    }

    PyBulletCollision(float distance, const btVector3 &position, const btVector3 &collision_normal, int triangle_index, const btCollisionObject *object) :
        m_distance(distance),
        m_position(position),
        m_collision_normal(collision_normal),
        m_triangle_index(triangle_index),
        m_object(object)
    {
    }

    float m_distance;
    btVector3 m_position;
    btVector3 m_collision_normal;
    int m_triangle_index;
    const btCollisionObject *m_object;
};

inline static int triangle_index(const btCollisionWorld::LocalShapeInfo *info)
{
    if (info)
        return info->m_triangleIndex;
    return 0;
}

struct PyBulletCollisionResults : public btCollisionWorld::ConvexResultCallback, public btCollisionWorld::ContactResultCallback, public btCollisionWorld::RayResultCallback
{
    std::list<PyBulletCollision> m_collisions;
    btVector3 m_ray_from;
    btVector3 m_ray_len;

    virtual btScalar addSingleResult(btManifoldPoint &cp, const btCollisionObject *colObj0, int partId0, int index0, const btCollisionObject *colObj1, int partId1, int index1)
    {
        const btVector3 &pos = cp.getPositionWorldOnA();
        int index;
        const btCollisionObject *colObj;

        if (colObj0->getUserPointer())
        {
            index = index0;
            colObj = colObj0;
        }
        else
        {
            index = index1;
            colObj = colObj1;
        }

        m_collisions.push_back(PyBulletCollision(0.0,
                                                 pos,
                                                 cp.m_normalWorldOnB,
                                                 index,
                                                 colObj));

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
                                              convexResult.m_hitPointLocal,
                                              convexResult.m_hitNormalLocal,
                                              triangle_index(convexResult.m_localShapeInfo),
                                              convexResult.m_hitCollisionObject));

        return 1.f;
    }

    virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
    {
        std::list<PyBulletCollision>::iterator iter;
        for (iter = m_collisions.begin(); iter != m_collisions.end(); ++iter)
        {
            if (rayResult.m_hitFraction < iter->m_distance)
                break;
        }

        m_collisions.insert(iter,
                            PyBulletCollision(rayResult.m_hitFraction,
                                              m_ray_from + m_ray_len * rayResult.m_hitFraction,
                                              rayResult.m_hitNormalLocal,
                                              triangle_index(rayResult.m_localShapeInfo),
                                              rayResult.m_collisionObject));

        return 1.f;
    }

    virtual bool needsCollision(btBroadphaseProxy* proxy0) const
    {
        return true;
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

bool pybullet_contact_pair(btCollisionWorld *world, btCollisionObject *a, btCollisionObject *b)
{
    PyBulletCollisionResults results;
    world->contactPairTest(a, b, results);
    return results.m_collisions.size();
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

void pybullet_ray(btCollisionWorld *world, btVector3 from, btVector3 to, PyBulletCollisionResults *results)
{
    // Clear previous results
    results->m_collisions.clear();
    results->m_ray_from = from;
    results->m_ray_len = to - from;

    world->rayTest(from, to, *results);
}
