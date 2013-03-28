#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif
#include <cyclone/cyclone.h>
#include "Dice.cpp"

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

class EightSidedDice : public Dice
{
public:
	cyclone::CollisionSphere *RoundingSphere;

    EightSidedDice( void )
    {
        this->body = new cyclone::RigidBody;

        this->RoundingSphere = new cyclone::CollisionSphere;
        this->RoundingSphere->body = new cyclone::RigidBody();
    }

    ~EightSidedDice( void )
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
                glScalef( halfSize.x * 2, halfSize.y * 2, halfSize.z * 2 );
                glLoadName( this->m_Name + PICK_NAME_DICE_OFFSET );
				sqSolidDoublePyramid( this->RoundingSphere->radius, 30, 20 );
            glPopMatrix();
        glPopMatrix();
    }

    void Update( cyclone::real duration )
    {
        this->body->integrate( duration );
        this->calculateInternals();

        // Update the rounding sphere position
        this->RoundingSphere->body->setPosition( cyclone::Vector3(0,0,0));
    }

    void DoPlaneCollisionTest( cyclone::CollisionPlane plane, cyclone::CollisionData *collisionData )
    {
        if( cyclone::IntersectionTests::sphereAndHalfSpace( *this->RoundingSphere, plane ) )
        {
			PyramidCollision( *this, plane, collisionData );
            //cyclone::CollisionDetector::boxAndHalfSpace( *this, plane, collisionData );
        }
    }

    void DoDiceCollisionTest( Dice *d, cyclone::CollisionData *collisionData )
    {

    }

    void SetState( cyclone::real x, cyclone::real y, cyclone::real z )
    {
        // Dice body
        {
            this->body->setPosition( x, y, z );
            this->body->setOrientation( 1, 0, 0, 0 );
            this->body->setVelocity( 0, 0, 0 );
            this->body->setRotation( cyclone::Vector3( 0.3f, 0.3f, 0.3f ) );
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

	unsigned PyramidCollision( const cyclone::CollisionPrimitive &d, const cyclone::CollisionPlane &plane, cyclone::CollisionData *data )
	{
		if( data->contactsLeft <= 0 ) 
        {
            return 0;
        }

		float vData[12][3] = {
			{0.000001, 0.000000, -1.000000},
			{-0.999999, 0.000001, 0.000000},
			{0.000001, -0.000000, 1.000000},
			{1.000001, -0.000001, -0.000000},
			{-0.000001, -1.000000, 0.000000},
			{0.000001, 0.000000, 0.000000},

			{0.000000, 0.000000, -1.000000},
			{1.000000, 0.000000, 0.000000},
			{-0.000000, 0.000000, 1.000000},
			{-1.000000, 0.000000, -0.000000},
			{0.000000, 1.000000, 0.000000},
			{0.000000, 0.000000, 0.000000},
		};

		cyclone::Contact* contact = data->contacts;
		unsigned contactsUsed = 0;
		for( unsigned i = 0 ; i < 12 ; ++i ) 
        {
			cyclone::Vector3 v( vData[i][0] * halfSize.x*2, vData[i][1] * halfSize.y*2, vData[i][2] * halfSize.z*2 );
			v = d.getTransform().transform( v );

			float vDist = v * plane.direction;

			if(vDist <= plane.offset)
			{
				contact->contactPoint = plane.direction;
				contact->contactPoint *= (vDist - plane.offset);
				contact->contactPoint = v;
				contact->contactNormal = plane.direction;
				contact->penetration = plane.offset - vDist;

				contact->setBodyData( d.body, NULL, data->friction, data->restitution );

				++contact;
				++contactsUsed;
				if( contactsUsed == data->contactsLeft ) 
                {
                    return contactsUsed;
                }
			}
		}

		data->addContacts( contactsUsed );
		return contactsUsed;
	}
};