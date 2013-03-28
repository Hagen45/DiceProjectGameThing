#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif
#include <cyclone/cyclone.h>

#include <iostream>
#include <list>
#include <stdio.h>
#include <cassert>

static int s_Dices = 0;

class Dice : public cyclone::CollisionBox
{
protected:
    GLint m_Name;

public:
    cyclone::CollisionSphere *RoundingSphere;

    Dice( void )
    {
        this->body = new cyclone::RigidBody;
        this->m_Name = s_Dices++;

        this->RoundingSphere = new cyclone::CollisionSphere();
        this->RoundingSphere->body = new cyclone::RigidBody();
    }

    virtual ~Dice( void )
    {
        delete this->body;

        delete this->RoundingSphere;
    }

    virtual void RenderShadow( void )
    {
        GLfloat mat[16];
        body->getGLTransform( mat );

        glPushMatrix();
            glScalef( 1.0, 0, 1.0 );
            glMultMatrixf( mat );
            glScalef( halfSize.x * 2, halfSize.y * 2, halfSize.z * 2 );
            glutSolidCube( 1.0f );
        glPopMatrix();
    }

    virtual void render( void ) = 0;

    virtual void Update( cyclone::real duration ) = 0;
    virtual void DoPlaneCollisionTest( cyclone::CollisionPlane plane, cyclone::CollisionData *collisionData ) = 0;
    virtual void DoDiceCollisionTest( Dice *d, cyclone::CollisionData *collisionData ) = 0;
    virtual void SetState( cyclone::real x, cyclone::real y, cyclone::real z ) = 0;
};