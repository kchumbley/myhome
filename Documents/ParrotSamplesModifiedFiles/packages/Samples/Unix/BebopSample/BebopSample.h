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

#ifndef _BEBOP_SAMPLE_H_
#define _BEBOP_SAMPLE_H_

#include <ihm.h>

// called when the state of the device controller has changed
void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *customData);

// called when a command has been received from the drone
void commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, void *customData);

// IHM updates from commands
void batteryStateChanged (uint8_t percent);
void gpsStateChanged (double latitude, double longitude, double altitude);
void attitudeStateChanged (float roll, float pitch, float yaw);
void speedStateChanged (float speedX, float speedY, float speedZ);

// IHM updates from commands
void onGPSCommandReceived (ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
void onAttitudeCommandReceived (ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
void onSpeedCommandReceived (ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
void onMoveEndCommandReceived (ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
void onSetMaxPitchRollCommmandReceived(ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
void onSetMaxVerticalSpeedCommmandReceived(ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
void onSetMaxPitchRollRotationSpeedCommmandReceived(ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
void onSetMaxRotationSpeedCommmandReceived(ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);


// called when a streaming frame has been received
eARCONTROLLER_ERROR didReceiveFrameCallback (ARCONTROLLER_Frame_t *frame, void *customData);

eARCONTROLLER_ERROR decoderConfigCallback (ARCONTROLLER_Stream_Codec_t codec, void *customData);

/* IHM callbacks: */
void performRelativeMove(ARCONTROLLER_Device_t *deviceController);
void startFlightplan(ARCONTROLLER_Device_t *deviceController);
void performTimedFlightTest(ARCONTROLLER_Device_t *deviceController);
void performBackFlip(ARCONTROLLER_Device_t *deviceController);
void loadWaypoints(void);
void displayWaypoint(void);

void onInputEvent (eIHM_INPUT_EVENT event, void *customData);



int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va);

double distance (double lat_0, double lon_0, double lat_1, double lon_1);
double heading(double lat_0, double lon_0, double lat_1, double lon_1);

int pointInPolygon(int nvert, double *vertx, double *verty, double testx, double testy);


#endif /* BEBOP_SAMPLE_H */
