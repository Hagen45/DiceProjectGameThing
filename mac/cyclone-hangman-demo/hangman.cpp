#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif
#include <cyclone/cyclone.h>
#include "app.h"

#include <stdio.h>
#include <cassert>

const unsigned int PARTICLE_ARRAY_LENGTH = 13;
const unsigned int PARTICLE_ARRAY_STRIDE = 2;

const unsigned int CABLE_INDEX_ARRAY_LENGTH = 16;
const unsigned int CABLE_INDEX_ARRAY_STRIDE = 2;

const unsigned int SUPPORT_ARRAY_LENGTH = 3;
const unsigned int SUPPORT_ARRAY_STRIDE = 2;
const unsigned int SUPPORT_INDEX_ARRAY_STRIDE = 2;

const cyclone::real DEFAULT_PARTICLE_DAMPING = 0.9;

const cyclone::real DEFAULT_CABLE_LENGTH = 1.0;
const cyclone::real DEFAULT_CABLE_RESTITUTION = 0.1;

const cyclone::real DEFAULT_SUPPORT_LENGTH = 1.0;
const cyclone::real DEFAULT_SUPPORT_RESTITUTION = 0.1;

const unsigned int DEFAULT_PARTICLE_MOVE_INDEX_A = 11;
const unsigned int DEFAULT_PARTICLE_MOVE_INDEX_B = 12;

const cyclone::real DEFAULT_INTERACTION_FORCE = 1.;

const cyclone::real DEFAULT_SCALE = 0.2;
const int DEFAULT_DISPLACEMENT = 8;

/* 5 4 3 2 101 2 3 4 5
 *
 *    0A0  1A1  2A2     0
 *     |    |    |      1
 *     |    |    |      2
 *     |    0    |      3
 *     |    |    |      4
 *     |    1    |      5
 *     |    |    |      6
 *     2    3    4      7
 *    / \  / \  / \     8
 *   /  /5-----6\  \    9
 *  /  / |     | \  \   10
 * 7--/  |     |  \--8  11
 *       |     |        12
 *       9-----0        13
 *      /       \       14
 *     /         \      15
 *    1           2     16
 */
cyclone::real PARTICLE_ARRAY[] = {
    0.0, 6.0, 0.0, 10.0, -6.0, 14.0, 0.0, 14.0, 6.0, 14.0, -4.0, 18.0, 4.0, 
    18.0, -10.0, 22.0, 10.0, 22.0, -4.0, 26.0, 4.0, 26.0, -6.0, 32.0, 6.0, 32.0,
};

unsigned int CABLE_INDEX_ARRAY[] = {
    0,  1, 
    1,  3, 
    2,  7, 
    2,  5, 
    5,  7, 
    3,  5, 
    3,  6, 
    5,  6, 
    4,  6, 
    4,  8, 
    6,  8, 
    5,  9, 
    6,  10, 
    9,  10, 
    9,  11, 
    10, 12
};

cyclone::real SUPPORT_ARRAY[] = {
    -6.0, 0.0, 0.0, 0.0, 6.0, 0.0
};

unsigned int SUPPORT_INDEX_ARRAY[] = {
    0, 2, 1, 0, 2, 4
};

Application* getApplication( void );

inline cyclone::real VectorLength( cyclone::real x0, cyclone::real y0, cyclone::real x1, cyclone::real y1 )
{
    cyclone::real x = x0 - x1;
    cyclone::real y = y0 - y1;
    
    return sqrt( (x * x) + (y * y) );
}

class HangmanDemo : public MassAggregateApplication
{
    cyclone::ParticleCableConstraint *m_Supports;
    cyclone::ParticleRod *m_Cables;
public:
    HangmanDemo( void );
    virtual ~HangmanDemo( void );
    
    virtual const char* GetTitle( void );
    
    virtual void Display( void );
    virtual void Update( void );
    virtual void Key( unsigned char key );
};

HangmanDemo::HangmanDemo( void ) : MassAggregateApplication( PARTICLE_ARRAY_LENGTH ), m_Cables( 0 ), m_Supports( 0 )
{
    for( unsigned i = 0 ; i < PARTICLE_ARRAY_LENGTH ; ++i )
    {
        const cyclone::real x = PARTICLE_ARRAY[i * PARTICLE_ARRAY_STRIDE] * DEFAULT_SCALE;
        const cyclone::real y = PARTICLE_ARRAY[i * PARTICLE_ARRAY_STRIDE + 1] * DEFAULT_SCALE;
        
        this->m_ParticleArray[i].setPosition( x, DEFAULT_DISPLACEMENT - y, -1 );
        this->m_ParticleArray[i].setVelocity( 0, 0, 0 );
        this->m_ParticleArray[i].setDamping( DEFAULT_PARTICLE_DAMPING );
        this->m_ParticleArray[i].setMass( 0.05 );
        this->m_ParticleArray[i].setAcceleration( cyclone::Vector3::GRAVITY );
        this->m_ParticleArray[i].clearAccumulator();
    }
    
    this->m_Cables = new cyclone::ParticleRod[CABLE_INDEX_ARRAY_LENGTH];
    for( unsigned i = 0 ; i < CABLE_INDEX_ARRAY_LENGTH ; ++i )
    {
        unsigned int a = CABLE_INDEX_ARRAY[i * CABLE_INDEX_ARRAY_STRIDE];
        unsigned int b = CABLE_INDEX_ARRAY[i * CABLE_INDEX_ARRAY_STRIDE + 1];
        
        cyclone::Vector3 ap, bp;
        ap = this->m_ParticleArray[a].getPosition();
        bp = this->m_ParticleArray[b].getPosition();
        
        this->m_Cables[i].particle[0] = &this->m_ParticleArray[a];
        this->m_Cables[i].particle[1] = &this->m_ParticleArray[b];
        this->m_Cables[i].length = VectorLength( ap.x, ap.y, bp.x, bp.y );
        this->m_World.getContactGenerators().push_back( &this->m_Cables[i] );
        
        this->m_Cables[i].particle[0]->clearAccumulator();
        this->m_Cables[i].particle[1]->clearAccumulator();
    }
    
    this->m_Supports = new cyclone::ParticleCableConstraint[SUPPORT_ARRAY_LENGTH];
    for( unsigned i = 0 ; i < SUPPORT_ARRAY_LENGTH ; ++i )
    {
        int supportIndex = SUPPORT_INDEX_ARRAY[i * SUPPORT_ARRAY_STRIDE];
        int particleIndex = SUPPORT_INDEX_ARRAY[i * SUPPORT_ARRAY_STRIDE + 1];
     
        this->m_Supports[i].particle = &this->m_ParticleArray[particleIndex];
        this->m_Supports[i].anchor = cyclone::Vector3( 
            SUPPORT_ARRAY[supportIndex * SUPPORT_INDEX_ARRAY_STRIDE] * DEFAULT_SCALE, 
            DEFAULT_DISPLACEMENT - SUPPORT_ARRAY[supportIndex * SUPPORT_INDEX_ARRAY_STRIDE + 1], 
            -1
        );
        
        cyclone::Vector3 ap, bp;
        ap = this->m_Supports[i].anchor;
        bp = this->m_Supports[i].particle->getPosition();
        
        this->m_Supports[i].maxLength = VectorLength( ap.x, ap.y, bp.x, bp.y );
        
        this->m_Supports[i].restitution = DEFAULT_SUPPORT_RESTITUTION;
        this->m_World.getContactGenerators().push_back( &this->m_Supports[i] );
    }
}

HangmanDemo::~HangmanDemo()
{
    if( this->m_Cables ) 
    {
        delete[] this->m_Cables;
    }
    if( this->m_Supports ) 
    {
        delete[] this->m_Supports;
    }
}

void HangmanDemo::Display( void )
{
    MassAggregateApplication::Display();
    
    glBegin( GL_LINES );
        for( unsigned i = 0 ; i < CABLE_INDEX_ARRAY_LENGTH ; ++i )
        {
            cyclone::Particle **particles = this->m_Cables[i].particle;
            const cyclone::Vector3 &p0 = particles[0]->getPosition();
            const cyclone::Vector3 &p1 = particles[1]->getPosition();
            
            glColor3d( 0, 1.0 / CABLE_INDEX_ARRAY_LENGTH * i, 0 );
            glVertex3d( p0.x, p0.y, p0.z );
            glVertex3d( p1.x, p1.y, p1.z );
        }
        
        for( unsigned i = 0 ; i < SUPPORT_ARRAY_LENGTH ; ++i )
        {
            const cyclone::Vector3 &p0 = this->m_Supports[i].particle->getPosition();
            const cyclone::Vector3 &p1 = this->m_Supports[i].anchor;
            
            glColor3d( 1.0 / SUPPORT_ARRAY_LENGTH * i, 0, 1 );
            glVertex3d( p0.x, p0.y, p0.z );
            glVertex3d( p1.x, p1.y, p1.z );
        }
    glEnd();
}

void HangmanDemo::Update()
{
    MassAggregateApplication::Update();
}

const char *HangmanDemo::GetTitle()
{
    return "Hangman";
}

void HangmanDemo::Key( unsigned char key )
{
    cyclone::Particle *a, *b;
    a = &this->m_ParticleArray[DEFAULT_PARTICLE_MOVE_INDEX_A];
    b = &this->m_ParticleArray[DEFAULT_PARTICLE_MOVE_INDEX_B];
    
    cyclone::Vector3 ap, bp;
    ap = a->getVelocity();
    bp = b->getVelocity();
    
    switch( key )
    {
        case 's': case 'S':
            ap.y -= DEFAULT_INTERACTION_FORCE; bp.y -= DEFAULT_INTERACTION_FORCE;
            break;
        case 'w': case 'W':
            ap.y += DEFAULT_INTERACTION_FORCE; bp.y += DEFAULT_INTERACTION_FORCE;
            break;
        case 'a': case 'A':
            ap.x -= DEFAULT_INTERACTION_FORCE; bp.x -= DEFAULT_INTERACTION_FORCE;
            break;
        case 'd': case 'D':
            ap.x += DEFAULT_INTERACTION_FORCE; bp.x += DEFAULT_INTERACTION_FORCE;
            break;
        default:
            MassAggregateApplication::Key( key );
    }
    
    a->setVelocity( ap );
    b->setVelocity( bp );
}

Application* getApplication( void )
{
    return new HangmanDemo();
}