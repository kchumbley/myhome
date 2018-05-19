/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the 
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/

#ifndef _BEBOP_SAMPLE_IHM_H_
#define _BEBOP_SAMPLE_IHM_H_

#include <curses.h>
#include <libARSAL/ARSAL.h>

typedef enum
{
    IHM_INPUT_EVENT_NONE,
    IHM_INPUT_EVENT_EXIT,
    IHM_INPUT_EVENT_EMERGENCY,
    IHM_INPUT_EVENT_TAKEOFF,
    IHM_INPUT_EVENT_LAND,
    IHM_INPUT_EVENT_UP,
    IHM_INPUT_EVENT_DOWN,
    IHM_INPUT_EVENT_RIGHT,
    IHM_INPUT_EVENT_LEFT,
    IHM_INPUT_EVENT_FORWARD,
    IHM_INPUT_EVENT_BACK,
    IHM_INPUT_EVENT_ROLL_LEFT,
    IHM_INPUT_EVENT_ROLL_RIGHT,
    IHM_INPUT_EVENT_HOME,
    IHM_INPUT_EVENT_FLAT_TRIM,
    IHM_INPUT_EVENT_TOGGLE_STICKS,
    IHM_INPUT_EVENT_NUM,
    IHM_INPUT_EVENT_GOTO_CUR_WP,
    IHM_INPUT_EVENT_TOGGLE_VERBOSE_MODE,
    IHM_INPUT_EVENT_TOGGLE_BOUNDING_MODE,
}eIHM_INPUT_EVENT;

typedef void (*IHM_onInputEvent_t) (eIHM_INPUT_EVENT event, void *customData);

typedef struct
{
    WINDOW *mainWindow;
    ARSAL_Thread_t inputThread;
    int run;
    IHM_onInputEvent_t onInputEventCallback;
    void *customData;
}IHM_t;

IHM_t *IHM_New (IHM_onInputEvent_t onInputEventCallback);
void IHM_Delete (IHM_t **ihm);

void IHM_setCustomData(IHM_t *ihm, void *customData);

void IHM_PrintHeader(IHM_t *ihm, char *headerStr);
void IHM_PrintInfo(IHM_t *ihm, char *infoStr);
void IHM_PrintBattery(IHM_t *ihm, uint8_t percent);
void IHM_PrintSticks(IHM_t *ihm, char *sticksStr);
void IHM_PrintGPS(IHM_t *ihm, double latitude, double longitude, double altitude);
void IHM_PrintAttitude(IHM_t *ihm, float roll, float pitch, float yaw);
void IHM_PrintSpeed(IHM_t *ihm, float speedX, float speedY, float speedZ);
int getNum(void);


#endif /* _BEBOP_SAMPLE_IHM_H_ */
