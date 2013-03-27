#include <cstdlib>

#include <cyclone/cyclone.h>

class Application
{
protected:
    int m_Height;
    int m_Width;
public:
    virtual const char* GetTitle( void );
    
    virtual void InitGraphics( void );
    
    virtual void SetView( void );
    
    virtual void Deinit( void );
    
    virtual void Display( void );
    virtual void Update( void );
    virtual void Key( unsigned char key );
    virtual void Resize( int width, int height );
    virtual void Mouse( int button, int state, int x, int y );
    virtual void MouseDrag( int x, int y );
    
    void RenderText( float x, float y, const char *text, void* font = 0 );
};

class MassAggregateApplication : public Application
{
protected:
    cyclone::ParticleWorld m_World;
    cyclone::Particle *m_ParticleArray;
    cyclone::GroundContacts m_GroundContactGenerator;
public:
    MassAggregateApplication( unsigned int particleCount );
    virtual ~MassAggregateApplication( void );
    
    virtual void Update( void );
    virtual void InitGraphics( void );
    virtual void Display( void );
};

class RigidBodyApplication : public Application
{
protected:
    const static unsigned s_MaxContacts = 256;
    
    cyclone::Contact m_Contacts[s_MaxContacts];
    cyclone::CollisionData m_CollisionData;
    cyclone::ContactResolver m_Resolver;
    
    float m_Theta;
    float m_Phi;
    
    int m_LastX, m_LastY;
    
    bool m_RenderDebugInfo;
    bool m_PauseSimulation;
    bool m_AutoPauseSimulation; 
    
    virtual void GenerateContacts( void ) = 0;
    virtual void UpdateObjects( cyclone::real duration ) = 0;
    
    void DrawDebug( void );
    
    virtual void Reset( void ) = 0;
public:
    RigidBodyApplication();
    
    virtual void Display( void );
    virtual void Update( void );
    virtual void Mouse( int button, int state, int x, int y );
    virtual void MouseDrag( int x, int y );
    virtual void Key( unsigned char key );
};