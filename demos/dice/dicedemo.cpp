#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif
#include <cyclone/cyclone.h>

#include "app.h"
#include "squadric.h"

#include <list>
#include <stdio.h>
#include <cassert>
#include <iostream>

#define DICE_ROUNDING_FACTOR 0.75

#define PICK_NAME_DICE_OFFSET 10
#define PICK_BUFFER_SIZE      256
#define PICK_TOLERANCE        10

static int s_Dices = 0;

Application* getApplication( void );

typedef struct
{
    cyclone::Vector3 o, d;
} Ray;

bool RayBoxIntersection( Ray ray, cyclone::CollisionBox box, cyclone::real &t )
{
    cyclone::Vector3 o = box.body->getPointInLocalSpace( ray.o );
    cyclone::Vector3 d = box.body->getDirectionInLocalSpace( ray.d );

    cyclone::Vector3 tmin = (cyclone::Vector3::Zero - box.halfSize - o) / d;
    cyclone::Vector3 tmax = (box.halfSize - o) / d;
    cyclone::Vector3 temp = tmin;

    tmin = cyclone::Vector3::Min( temp, tmax );
    tmax = cyclone::Vector3::Max( temp, tmax );

    cyclone::real time = tmin.x;
    time = time > tmin.y ? time : tmin.y;
    time = time > tmin.z ? time : tmin.z;

    cyclone::real last = tmax.x;
    last = last < tmax.y ? last : tmax.y;
    last = last < tmax.z ? last : tmax.z;

    t = time;

    return time <= last;
}

class Dice : public cyclone::CollisionBox
{
protected:
    GLint m_Name;

private: 
	cyclone::Vector3 lastPosition; 
	

public:
    cyclone::CollisionSphere *RoundingSphere;
	bool getsDragged; 
	cyclone::Vector3 oldPosition;
	cyclone::Vector3 newPosition;



    Dice( void )
    {
        this->body = new cyclone::RigidBody;
        this->m_Name = s_Dices++;

        this->RoundingSphere = new cyclone::CollisionSphere();
        this->RoundingSphere->body = new cyclone::RigidBody();
		this->getsDragged = false; 
    }

    virtual ~Dice( void )
    {
        delete this->body;

        delete this->RoundingSphere;
    }

	virtual void isGettingDragged(bool state) { 
		this->getsDragged = state; 
	}

	virtual cyclone::Vector3 getOldPosition(){
		return oldPosition;
	}

	virtual cyclone::Vector3 getNewPosition(){
		return newPosition;
	}
	
	virtual void setOldPosition(cyclone::Vector3 pos){
		oldPosition=pos;
	}

	virtual void setNewPosition(cyclone::Vector3 pos){
		setOldPosition(getNewPosition());
		newPosition = pos;
	}

	virtual cyclone::Vector3 getFlingData(){
		return getNewPosition()-getOldPosition();
	}

	virtual void fling(){
		this->body->setVelocity((getNewPosition()*10)-(getOldPosition())*10);
		resetPosition();
		
	}

	virtual void resetPosition(){
	cyclone::Vector3 oldPosition;
	cyclone::Vector3 newPosition;
	}

    virtual void render( void ) = 0;

    virtual void Update( cyclone::real duration ) = 0;
    virtual void DoPlaneCollisionTest( cyclone::CollisionPlane plane, cyclone::CollisionData *collisionData ) = 0;
    virtual void DoDiceCollisionTest( Dice *d, cyclone::CollisionData *collisionData ) = 0;
    virtual void SetState( cyclone::real x, cyclone::real y, cyclone::real z ) = 0;
};

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
        this->RoundingSphere->body->setPosition( this->body->getPosition() );
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
                glScalef( halfSize.x, halfSize.y, halfSize.z );
                glLoadName( this->m_Name + PICK_NAME_DICE_OFFSET );
                sqSolidRoundCube( this->RoundingSphere->radius, 30, 30 );
            glPopMatrix();
        glPopMatrix();
    }

    void Update( cyclone::real duration )
    {
        this->body->integrate( duration );
        this->calculateInternals();
		//std::cout << "dicePos: " << this->body->getPosition().x << ", " << this->body->getPosition().y << ", " << this->body->getPosition().z << ", " << std::endl; 
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

class DiceDemo : public RigidBodyApplication
{
private:
    std::list<Dice*> m_Dices;

    bool m_IsDragging;
    cyclone::PointJoint *m_DragJoint;
    cyclone::Vector3 m_DragPoint;
    Dice *m_DragDice;
	cyclone::real m_DragTime;

    unsigned int m_PickBuffer[PICK_BUFFER_SIZE];
public:
    DiceDemo( void );
    virtual ~DiceDemo( void );
    
    // Returns the window title
    virtual const char* GetTitle( void );

    // Initializes graphics
    virtual void InitGraphics( void );

    // Build the contacts for the current situation
    virtual void GenerateContacts( void );
    // Processes the objects in the simulation forward in time
    virtual void UpdateObjects( cyclone::real duration );

    // Resets the world
    virtual void Reset( void );

    // Do GL picking
    virtual void Select( int x, int y );
    
    // Display the world
    virtual void Display( void );
    // Handles mouse actions
    virtual void Mouse( int button, int state, int x, int y );
    // Handles mouse drag
    virtual void MouseDrag( int x, int y );
};

DiceDemo::DiceDemo( void )
{
    Dice *d;

    this->m_IsDragging = false;
	this->m_DragJoint = NULL;
	this->m_DragDice = NULL;

	for( int i = 0; i < 1; ++i )
	{
		this->m_Dices.push_back( d = new SixSidedDice() );
		d->SetState( i, i*2, i );
	}
	for( int j = 0; j < 0; ++j )
	{
		this->m_Dices.push_back( d = new EightSidedDice() );
		d->SetState( j, j*2, j );
	}
}

static bool deleteElm( Dice *d )
{
    delete d; return true;
}

DiceDemo::~DiceDemo()
{
    this->m_Dices.remove_if( deleteElm );
}

void DiceDemo::Display( void )
{
    const static GLfloat lightPosition[] = {-1, 1, 0, 0};
    
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    gluLookAt( -25.0, 8.0, 5.0, 0.0, 5.0, 0.0, 0.0, 1.0, 0.0 );

    // Draw some scale circles
    glColor3f( 0.75, 0.75, 0.75 );
    for( unsigned i = 1 ; i < 20 ; ++i )
    {
        glBegin( GL_LINE_LOOP );
        for( unsigned j = 0 ; j < 32 ; ++j )
        {
            float theta = 3.1415926 * j / 16.0;
            glVertex3f( i * cosf( theta ), 0.0, i * sinf( theta ) );
        }
        glEnd();
    }
    glBegin( GL_LINES);
        glVertex3f( -20, 0 ,0 );
        glVertex3f( 20, 0, 0 );
        glVertex3f( 0, 0, -20 );
        glVertex3f( 0, 0, 20 );
    glEnd();

	std::list<Dice*>::const_iterator it;

    // Draw the dice
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_LIGHTING );
    glLightfv( GL_LIGHT0, GL_POSITION, lightPosition );
    glColorMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE );
    glEnable( GL_COLOR_MATERIAL );
    glColor3f( 0.70, 0.75, 0.70 );
   

    for( it = this->m_Dices.begin() ; it != this->m_Dices.end() ; ++it )
    {
        (*it)->render();
    }

    glDisable( GL_COLOR_MATERIAL );
    glDisable( GL_LIGHTING );
    glDisable( GL_DEPTH_TEST );

    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    // Render some text
    glColor3f( 0.0, 0.0, 0.0 );
    this->RenderText( 10.0, 10.0, "Mouse click to pick up the dice" );
}

const char *DiceDemo::GetTitle()
{
    return "Playing with Dices";
}

void DiceDemo::InitGraphics( void )
{
    GLfloat lightAmbient[] = {0.8f, 0.8f, 0.8f, 1.0f};
    GLfloat lightDiffuse[] = {0.9f, 0.95f, 1.0f, 1.0f};

    glLightfv( GL_LIGHT0, GL_AMBIENT, lightAmbient );
    glLightfv( GL_LIGHT0, GL_DIFFUSE, lightDiffuse );

    glEnable( GL_LIGHT0 );

    Application::InitGraphics();
}

// Check if we are selecting a dice
void DiceDemo::Select( int x, int y )
{  
	if(m_DragJoint==NULL){
		GLdouble model[16], proj[16];
		GLint view[4];

		GLdouble oX, oY, oZ, eX, eY, eZ;
		oX = oY = oZ = eX = eY = eZ = 0;

		glGetDoublev( GL_MODELVIEW_MATRIX, model );
		glGetDoublev( GL_PROJECTION_MATRIX, proj );
		glGetIntegerv( GL_VIEWPORT, view );

		assert( gluUnProject( x, view[3] - y, 0.0, model, proj, view, &oX, &oY, &oZ ) != GLU_FALSE );
		assert( gluUnProject( x, view[3] - y, 1.0, model, proj, view, &eX, &eY, &eZ ) != GLU_FALSE );

		Ray r;
		//r.o seems to be the original spot that's been clicked on.
		r.o = cyclone::Vector3( oX, oY, oZ );
		//r.d appears to keep track of the point selected after it's been dragged.eX - oX
		r.d = cyclone::Vector3( eX - oX, eY - oY, eZ - oZ );
		r.d.normalise();

		std::list<Dice*>::const_iterator it;

		//Loops through all dices and checks which dice you have clicked.
		for( it = this->m_Dices.begin() ; it != this->m_Dices.end() ; ++it )
		{
			if( RayBoxIntersection( r, *(*it), this->m_DragTime ) )
			{
				cyclone::Vector3 pos = r.o + r.d * this->m_DragTime;
				cyclone::Vector3 bpos = (*it)->body->getPosition();
				(*it)->isGettingDragged(true);

				this->m_IsDragging = true;

				this->m_DragDice = *it;
				cyclone::PointJoint *p = new cyclone::PointJoint( (*it)->body, bpos - pos );
				this->m_DragJoint = p;

				break;
			}
		}
	}
}

// Check if there is a usable mouse action 
void DiceDemo::Mouse( int button, int state, int x, int y )
{
    if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN) )
    {	
        this->Select( x, y );
		this->MouseDrag(x,y);
	}
    else if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_UP) )
    {
        this->m_IsDragging = false;
        this->m_DragDice = 0;

		if( this->m_DragJoint != NULL )
		{
			    std::list<Dice*>::const_iterator it;

				for( it = this->m_Dices.begin() ; it != this->m_Dices.end() ; ++it )
				{
					if(((*it)->getsDragged)==true){
						(*it)->isGettingDragged(false);
						(*it)->fling();
						std::cout << "Fling speed: " <<"("<<(*it)->getFlingData().x <<","<<(*it)->getFlingData().y << 
							","<<(*it)->getFlingData().z<< ")"<< std::endl; 
					}
				}

			delete this->m_DragJoint;
			this->m_DragJoint = NULL;
		}
    }
}

void DiceDemo::MouseDrag( int x, int y )
{
	if( m_DragJoint != NULL )
	{
		GLdouble model[16], proj[16];
		GLint view[4];

		GLdouble oX, oY, oZ, eX, eY, eZ;
		oX = oY = oZ = eX = eY = eZ = 0;

		glGetDoublev( GL_MODELVIEW_MATRIX, model );
		glGetDoublev( GL_PROJECTION_MATRIX, proj );
		glGetIntegerv( GL_VIEWPORT, view );

		assert( gluUnProject( x, view[3] - y, 0.0, model, proj, view, &oX, &oY, &oZ ) != GLU_FALSE );
		assert( gluUnProject( x, view[3] - y, 1.0, model, proj, view, &eX, &eY, &eZ ) != GLU_FALSE );

		Ray r;
		r.o = cyclone::Vector3( oX, oY, oZ );
		r.d = cyclone::Vector3( eX - oX, eY - oY, eZ - oZ );
		r.d.normalise();

		cyclone::Vector3 pos = r.o + r.d * this->m_DragTime;
		this->m_DragJoint->SetWorldPosition( pos );
		//std::cout << "mousepos: " << pos.x << ", " << pos.y << ", " << pos.z << ", " << std::endl; 
	}
}

void DiceDemo::GenerateContacts( void )
{
    // Create a ground plane
    cyclone::CollisionPlane plane;
    plane.direction = cyclone::Vector3( 0, 1, 0 );
    plane.offset = 0;
	
    this->m_CollisionData.reset( RigidBodyApplication::s_MaxContacts );
    this->m_CollisionData.friction = (cyclone::real) 0.9;
    this->m_CollisionData.restitution = (cyclone::real) 0.1;
    this->m_CollisionData.tolerance = (cyclone::real) 0.1;

    if( m_DragDice != NULL )
    {
        this->m_CollisionData.addContacts( this->m_DragJoint->addContact( this->m_CollisionData.contacts, this->m_CollisionData.contactsLeft ) );
    }

    std::list<Dice*>::const_iterator it, ti;
    for( it = this->m_Dices.begin() ; it != this->m_Dices.end() ; ++it )
    {
		(*it)->DoPlaneCollisionTest( plane, &this->m_CollisionData );



        for( ti = this->m_Dices.begin() ; ti != this->m_Dices.end() ; ++ti )
        {
            if( (*ti) != (*it) )
            {
                (*it)->DoDiceCollisionTest( (*ti), &this->m_CollisionData );
            }
        }
    }
}

void DiceDemo::UpdateObjects( cyclone::real duration )
{
    std::list<Dice*>::const_iterator it;
    for( it = this->m_Dices.begin() ; it != this->m_Dices.end() ; ++it )
    {	
		if((*it)->getsDragged==true){
	//		std::cout << "Fling speed: " <<"("<<(*it)->getFlingData().x <<","<<(*it)->getFlingData().y << 
	//						","<<(*it)->getFlingData().z<< ")"<< std::endl; 
	
			std::cout << "mousepos: " << (*it)->getNewPosition().x << ", " << (*it)->getNewPosition().y << ", " << (*it)->getNewPosition().z << ", " << std::endl; 
			(*it)->setNewPosition((*it)->body->getPosition());
		}
        (*it)->Update( duration );
    }
}

void DiceDemo::Reset( void )
{

}

Application* getApplication( void )
{
    return new DiceDemo();
}