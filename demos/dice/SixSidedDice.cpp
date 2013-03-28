#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif
#include <cyclone/cyclone.h>
#include "Dice.cpp"

#include "app.h"
#include "squadric.h"

#include <iostream>
#include <list>
#include <stdio.h>
#include <cassert>

#define PICK_NAME_DICE_OFFSET 10
#define DICE_ROUNDING_FACTOR 0.75


static bool s_DebugDraw = false;
static bool s_Wireframe = true;

typedef struct
{
    cyclone::Vector3 o, d;
} Ray;

class SixSidedDice : public Dice
{
public:
    cyclone::CollisionSphere *RoundingSphere;

    SixSidedDice( void )
    {
        this->body = new cyclone::RigidBody;

        this->RoundingSphere = new cyclone::CollisionSphere();
        this->RoundingSphere->body = new cyclone::RigidBody();
    }

    ~SixSidedDice( void )
    {
        delete this->body;

        delete this->RoundingSphere;
    }

    void render( void )
    {
        GLfloat mat[16];
        this->body->getGLTransform( mat );

        glPushMatrix();
            glMultMatrixf( mat );
            glPushMatrix();
                if( s_DebugDraw )
                {
                    glScalef( halfSize.x * 2, halfSize.y * 2, halfSize.z * 2 );
                    glutWireCube( 1.0 );
                    glutWireSphere( this->RoundingSphere->radius, 30, 30 );
                }
            glPopMatrix();

            glPushMatrix();
                glScalef( halfSize.x, halfSize.y, halfSize.z );
                glLoadName( this->m_Name + PICK_NAME_DICE_OFFSET );
                sqSolidRoundCube( this->RoundingSphere->radius, 30, 30 );
            glPopMatrix();
        glPopMatrix();
    }

    void Update( cyclone::real duration )
    {
		// Move the dice
        this->body->integrate( duration );

        this->calculateInternals();

        // Update the rounding sphere position
		this->RoundingSphere->body->setPosition( this->body->getPosition() );
    }

    void DoPlaneCollisionTest( cyclone::CollisionPlane plane, cyclone::CollisionData *collisionData )
    {
        if( cyclone::IntersectionTests::boxAndHalfSpace( *this, plane ) && cyclone::IntersectionTests::sphereAndHalfSpace( *this->RoundingSphere, plane ) )
        {
            cyclone::CollisionDetector::boxAndHalfSpace( *this, plane, collisionData );
        }
    }

    void DoDiceCollisionTest( Dice *d, cyclone::CollisionData *collisionData )
    {
        if( cyclone::IntersectionTests::boxAndBox( *this, *d ) && cyclone::IntersectionTests::sphereAndSphere( *this->RoundingSphere, *d->RoundingSphere ) )
        {
            cyclone::CollisionDetector::boxAndBox( *this, *d, collisionData );
        }
    }

    void SetState( cyclone::real x, cyclone::real y, cyclone::real z )
    {
        // Dice body
        {
            this->body->setPosition( x, y, z );
            this->body->setOrientation( 1, 0, 0, 0 );
            this->body->setVelocity( 0, 0, 0 );
            this->body->setRotation( cyclone::Vector3( 0, 0, 0 ) );
            this->halfSize = cyclone::Vector3( 1, 1, 1 );

            assert( this->halfSize.x == this->halfSize.y && this->halfSize.y == this->halfSize.z );

            cyclone::real mass = this->halfSize.x * this->halfSize.y * this->halfSize.z * 8.0f;
            this->body->setMass( mass );

            cyclone::Matrix3 tensor;
            tensor.setBlockInertiaTensor( this->halfSize, mass );
            this->body->setInertiaTensor( tensor );

            this->body->setDamping( 1.0, 1.0 );
            this->body->clearAccumulators();
            this->body->setAcceleration( 0, -10.0, 0 );

            this->body->setCanSleep( false );
            this->body->setAwake();

            this->body->calculateDerivedData();
        }

        // Rounding sphere body
        {
            this->RoundingSphere->radius = this->halfSize.x * DICE_ROUNDING_FACTOR;
        }
    }
};