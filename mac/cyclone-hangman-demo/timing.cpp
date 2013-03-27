#include "timing.h"
#ifndef _WIN32
#include <sys/time.h>
#include <sys/types.h>
#include <sys/sysctl.h>

unsigned int SystemTime( void );
unsigned long SystemClock( void );

unsigned int SystemTime( void )
{
    uint32_t lo, hi;
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    return ((uint64_t)hi << 32 | lo) / 1000000;
}

unsigned long SystemClock( void )
{
    unsigned long x;
    __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
    return x;
}

unsigned TimingData::GetTime( void )
{
    return SystemTime();
}

unsigned long TimingData::GetClock( void )
{
    return SystemClock();
}

static TimingData *s_TimingData = NULL;

TimingData& TimingData::get( void )
{
    return (TimingData&) *s_TimingData;
}

void TimingData::Update( void )
{
    if( !s_TimingData ) 
    {
        return;
    }
        
    if( !s_TimingData->IsPaused )
    {
        ++s_TimingData->FrameNumber;
    }
    
    unsigned thisTime = SystemTime();
    s_TimingData->LastFrameDuration = thisTime - s_TimingData->LastFrameTimestamp;
    s_TimingData->LastFrameTimestamp = thisTime;
    
    unsigned long thisClock = SystemClock();
    s_TimingData->LastFrameClockTicks = thisClock - s_TimingData->LastFrameClockstamp;
    s_TimingData->LastFrameClockstamp = thisClock;
    
    if( s_TimingData->FrameNumber > 1 ) 
    {
        if( s_TimingData->AverageFrameDuration <= 0 )
        {
            s_TimingData->AverageFrameDuration = (double) s_TimingData->LastFrameDuration;
        }
        else
        {
            s_TimingData->AverageFrameDuration *= 0.99;
            s_TimingData->AverageFrameDuration += 0.01 * (double) s_TimingData->LastFrameDuration;
            
            s_TimingData->Fps = (float) (1000.0 / s_TimingData->AverageFrameDuration);
        }
    }
}

void TimingData::Init( void )
{
    if( !s_TimingData ) 
    {
        s_TimingData = new TimingData();
    }
        
    s_TimingData->FrameNumber = 0;
    
    s_TimingData->LastFrameTimestamp = SystemTime();
    s_TimingData->LastFrameDuration = 0;
    
    s_TimingData->LastFrameClockstamp = SystemClock();
    s_TimingData->LastFrameClockTicks = 0;
    
    s_TimingData->IsPaused = false;
    
    s_TimingData->AverageFrameDuration = 0;
    s_TimingData->Fps = 0;
}

void TimingData::Deinit( void )
{
    delete s_TimingData;
    s_TimingData = 0;
}
#else

/*
 * Timing functions, frame management and profiling.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Ian Millington 2003-2006. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include "timing.h"

// Import the high performance timer (c. 4ms).
#include <windows.h>
#include <mmsystem.h>

// Hold internal timing data for the performance counter.
static bool qpcFlag;
static double qpcFrequency;

// Internal time and clock access functions
unsigned systemTime()
{
    if(qpcFlag)
    {
        static LONGLONG qpcMillisPerTick;
        QueryPerformanceCounter((LARGE_INTEGER*)&qpcMillisPerTick);
        return (unsigned)(qpcMillisPerTick * qpcFrequency);
    }
    else
    {
        return unsigned(timeGetTime());
    }
}

unsigned TimingData::getTime()
{
    return systemTime();
}

unsigned long systemClock()
{
    __asm {
        rdtsc;
    }
}

unsigned long TimingData::getClock()
{
    return systemClock();
}

// Sets up the timing system and registers the performance timer.
void initTime()
{
    LONGLONG time;

    qpcFlag = (QueryPerformanceFrequency((LARGE_INTEGER*)&time) > 0);

    // Check if we have access to the performance counter at this
    // resolution.
    if (qpcFlag) qpcFrequency = 1000.0 / time;
}


// Holds the global frame time that is passed around
static TimingData *timingData = NULL;

// Retrieves the global frame info instance
TimingData& TimingData::get()
{
    return (TimingData&)*timingData;
}

// Updates the global frame information. Should be called once per frame.
void TimingData::Update()
{
    if (!timingData) return;

    // Advance the frame number.
    if (!timingData->isPaused)
    {
        timingData->frameNumber++;
    }

    // Update the timing information.
    unsigned thisTime = systemTime();
    timingData->lastFrameDuration = thisTime -
        timingData->lastFrameTimestamp;
    timingData->lastFrameTimestamp = thisTime;

    // Update the tick information.
    unsigned long thisClock = systemClock();
    timingData->lastFrameClockTicks =
    thisClock - timingData->lastFrameClockstamp;
    timingData->lastFrameClockstamp = thisClock;

    // Update the RWA frame rate if we are able to.
    if (timingData->frameNumber > 1) {
        if (timingData->averageFrameDuration <= 0)
        {
            timingData->averageFrameDuration =
                (double)timingData->lastFrameDuration;
        }
        else
        {
            // RWA over 100 frames.
            timingData->averageFrameDuration *= 0.99;
            timingData->averageFrameDuration +=
                0.01 * (double)timingData->lastFrameDuration;

            // Invert to get FPS
            timingData->fps =
                (float)(1000.0/timingData->averageFrameDuration);
        }
    }
}

void TimingData::Init()
{
    // Set up the timing system.
    initTime();

    // Create the frame info object
    if (!timingData) timingData = new TimingData();

    // Set up the frame info structure.
    timingData->frameNumber = 0;

    timingData->lastFrameTimestamp = systemTime();
    timingData->lastFrameDuration = 0;

    timingData->lastFrameClockstamp = systemClock();
    timingData->lastFrameClockTicks = 0;

    timingData->isPaused = false;

    timingData->averageFrameDuration = 0;
    timingData->fps = 0;
}

void TimingData::Deinit()
{
        delete timingData;
        timingData = NULL;
}
#endif