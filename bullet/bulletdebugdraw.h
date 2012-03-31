
#include "LinearMath/btIDebugDraw.h"

#include "Python.h"

class PythonDebugDraw : public btIDebugDraw {
public:
    PyObject *debugDraw;

    PythonDebugDraw(PyObject *debugDraw) {
        Py_INCREF(debugDraw);
        this->debugDraw = debugDraw;
    }

    virtual ~PythonDebugDraw() {
        Py_DECREF(this->debugDraw);
        this->debugDraw = NULL;
    }

    virtual void drawLine(const btVector3& from,
                          const btVector3& to,
                          const btVector3& color) {
        char method[] = "drawLine";
        char format[] = "(ddddddddd)";
        PyObject_CallMethod(
            this->debugDraw, &method[0], &format[0],
            from.getX(), from.getY(), from.getZ(),
            to.getX(), to.getY(), to.getZ(),
            color.getX(), color.getY(), color.getZ());
    }

    virtual void drawContactPoint(const btVector3& positionOnB,
                                  const btVector3& normalOnB,
                                  btScalar distance, int lifetime,
                                  const btVector3& color) {
        char method[] = "drawContactPoint";
        char format[] = "(ddd ddd di ddd)";
        PyObject_CallMethod(
            this->debugDraw, &method[0], &format[0],
            positionOnB.getX(), positionOnB.getY(), positionOnB.getZ(),
            normalOnB.getX(), normalOnB.getY(), normalOnB.getZ(),
            distance, lifetime,
            color.getX(), color.getY(), color.getZ());
    }

    virtual void reportErrorWarning(const char*) {
    }

    virtual void draw3dText(const btVector3&, const char*) {
    }

    virtual void setDebugMode(int mode) {
        char method[] = "setDebugMode";
        char format[] = "(i)";
        PyObject_CallMethod(this->debugDraw, &method[0], &format[0], mode);
    }

    virtual int getDebugMode() const {
        char method[] = "getDebugMode";
        char format[] = "()";
        PyObject* mode = NULL;
        mode = PyObject_CallMethod(this->debugDraw, &method[0], &format[0]);
        /* TODO Add proper unit tests for this error handling.
         */
        if (mode) {
            return PyLong_AsLong(mode);
        } else {
            return 0;
        }
    }
};
