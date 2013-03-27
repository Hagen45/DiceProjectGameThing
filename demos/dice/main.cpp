#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif

#include "app.h"
#include "timing.h"

extern Application* getApplication();

Application* app;

void CreateWindow( const char* title );
void Update( void );
void Display( void );
void Mouse( int button, int state, int x, int y );
void Reshape( int width, int height );
void Keyboard( unsigned char key, int x, int y );
void Motion( int x, int y );

void CreateWindow( const char* title )
{
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE );
    glutInitWindowSize( 1024, 768 );
    glutInitWindowPosition( 0, 0 );
    glutCreateWindow( title );
}

void Update( void )
{
	TimingData::get().Update();
    app->Update();
}

void Display( void )
{
    app->Display();
    
    glFlush();
    glutSwapBuffers();
}

void Mouse( int button, int state, int x, int y )
{
    app->Mouse( button, state, x, y );
}

void Reshape( int width, int height )
{
    app->Resize( width, height );
}

void Keyboard( unsigned char key, int x, int y )
{ 
    app->Key( key );
}

void Motion( int x, int y )
{
    app->MouseDrag( x, y );
}

int main( int argc, char** argv )
{
    glutInit( &argc, argv );
    TimingData::Init();
    
    app = getApplication();
    CreateWindow( app->GetTitle() );
    
    glutReshapeFunc( Reshape );
    glutKeyboardFunc( Keyboard );
    glutDisplayFunc( Display );
    glutIdleFunc( Update );
    glutMouseFunc( Mouse );
    glutMotionFunc( Motion );
    
    app->InitGraphics();
    glutMainLoop();
    
    app->Deinit();
    
    delete app;
    TimingData::Deinit();
}