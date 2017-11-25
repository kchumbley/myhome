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
/**
 * @file BebopSample.c
 * @brief This file contains sources about basic arsdk example sending commands to a bebop drone to pilot it,
 * receive its battery level and display the video stream.
 * @date 15/01/2015
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>
#include <curses.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>

#include "BebopSample.h"
#include "ihm.h"

#include "math.h"

#define PI 3.14159265358979323846
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

/*****************************************
 *
 *             define :
 *
 *****************************************/
#define TAG "BebopSample"

#define ERROR_STR_LENGTH 2048

#define BEBOP_IP_ADDRESS "10.202.0.1"
#define BEBOP_DISCOVERY_PORT 44444

#define DISPLAY_WITH_MPLAYER 0

#define FIFO_DIR_PATTERN "/tmp/arsdk_XXXXXX"
#define FIFO_NAME "arsdk_fifo"

#define IHM
/*****************************************
 *
 *             private header:
 *
 ****************************************/


/*****************************************
 *
 *             implementation :
 *
 *****************************************/

static char fifo_dir[] = FIFO_DIR_PATTERN;
static char fifo_name[128] = "";

int gIHMRun = 1;
char gErrorStr[ERROR_STR_LENGTH];
IHM_t *ihm = NULL;

FILE *videoOut = NULL;
int frameNb = 0;
ARSAL_Sem_t stateSem;
int isBebop2 = 0;

static void signal_handler(int signal) {
    gIHMRun = 0;
}

int main(int argc, char *argv[]) {
    // local declarations
    int failed = 0;
    ARDISCOVERY_Device_t *device = NULL;
    ARCONTROLLER_Device_t *deviceController = NULL;
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
    eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
    pid_t child = 0;

    /* Set signal handlers */
    struct sigaction sig_action = {
            .sa_handler = signal_handler,
    };

    int ret = sigaction(SIGINT, &sig_action, NULL);
    if (ret < 0) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Unable to set SIGINT handler : %d(%s)",
                    errno, strerror(errno));
        return 1;
    }
    ret = sigaction(SIGPIPE, &sig_action, NULL);
    if (ret < 0) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Unable to set SIGPIPE handler : %d(%s)",
                    errno, strerror(errno));
        return 1;
    }


    if (mkdtemp(fifo_dir) == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkdtemp failed.");
        return 1;
    }
    snprintf(fifo_name, sizeof(fifo_name), "%s/%s", fifo_dir, FIFO_NAME);

    if (mkfifo(fifo_name, 0666) < 0) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkfifo failed: %d, %s", errno, strerror(errno));
        return 1;
    }

    ARSAL_Sem_Init(&(stateSem), 0, 0);


/*****************************************
 * MODIFIED BY: KEV; CUSTOMIZED FOR: BEBOP V1;
 ****************************************/
//    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Select your Bebop : Bebop (1) ; Bebop2 (2)");
//    char answer = '1';
//    scanf(" %c", &answer);
//    if (answer == '2') {
//        isBebop2 = 1;
//    }
//
//    if (isBebop2) {
//        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Bebop 2 Sample --");
//    } else {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Bebop Sample --");
//    }

    if (!failed) {
        if (DISPLAY_WITH_MPLAYER) {
            // fork the process to launch mplayer
            if ((child = fork()) == 0) {
//                execlp("xterm", "xterm", "-e", "mplayer", "-demuxer",  "h264es", fifo_name, "-benchmark", "-really-quiet", NULL);
                execlp("xterm", "xterm", "-e", "mplayer", "-demuxer", "h264es", fifo_name, "-benchmark", NULL);
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                            "Missing mplayer, you will not see the video. Please install mplayer and xterm.");
                return -1;
            }
        }

        if (DISPLAY_WITH_MPLAYER) {
            videoOut = fopen(fifo_name, "w");
        }
    }

#ifdef IHM
    ihm = IHM_New(&onInputEvent);
    if (ihm != NULL) {
        gErrorStr[0] = '\0';
        ARSAL_Print_SetCallback(customPrintCallback); //use a custom callback to print, for not disturb ncurses IHM

        if (isBebop2) {
            IHM_PrintHeader(ihm, "-- Bebop 2 Sample --");
        } else {
            IHM_PrintHeader(ihm, "-- Bebop Sample --");
        }
    } else {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of IHM failed.");
        failed = 1;
    }
#endif

    // create a discovery device
    if (!failed) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovey device ... ");
        eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

        device = ARDISCOVERY_Device_New(&errorDiscovery);

        if (errorDiscovery == ARDISCOVERY_OK) {
            ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
            // create a Bebop drone discovery device (ARDISCOVERY_PRODUCT_ARDRONE)

            if (isBebop2) {
                errorDiscovery = ARDISCOVERY_Device_InitWifi(device, ARDISCOVERY_PRODUCT_BEBOP_2, "bebop2",
                                                             BEBOP_IP_ADDRESS, BEBOP_DISCOVERY_PORT);
            } else {
                errorDiscovery = ARDISCOVERY_Device_InitWifi(device, ARDISCOVERY_PRODUCT_ARDRONE, "bebop",
                                                             BEBOP_IP_ADDRESS, BEBOP_DISCOVERY_PORT);
            }

            if (errorDiscovery != ARDISCOVERY_OK) {
                failed = 1;
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
            }
        } else {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
            failed = 1;
        }
    }

    // create a device controller
    if (!failed) {
        deviceController = ARCONTROLLER_Device_New(device, &error);

        if (error != ARCONTROLLER_OK) {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
            failed = 1;
        } else {
            IHM_setCustomData(ihm, deviceController);
        }
    }

    if (!failed) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- delete discovey device ... ");
        ARDISCOVERY_Device_Delete(&device);
    }

    // add the state change callback to be informed when the device controller starts, stops...
    if (!failed) {
        error = ARCONTROLLER_Device_AddStateChangedCallback(deviceController, stateChanged, deviceController);

        if (error != ARCONTROLLER_OK) {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
            failed = 1;
        }
    }

    // add the command received callback to be informed when a command has been received from the device
    if (!failed) {
        error = ARCONTROLLER_Device_AddCommandReceivedCallback(deviceController, commandReceived, deviceController);

        if (error != ARCONTROLLER_OK) {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "add callback failed.");
            failed = 1;
        }
    }

    // add the frame received callback to be informed when a streaming frame has been received from the device
    if (!failed) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
        error = ARCONTROLLER_Device_SetVideoStreamCallbacks(deviceController, decoderConfigCallback,
                                                            didReceiveFrameCallback, NULL, NULL);

        if (error != ARCONTROLLER_OK) {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error: %s", ARCONTROLLER_Error_ToString(error));
        }
    }

    if (!failed) {
        IHM_PrintInfo(ihm, "Connecting ...");
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
        error = ARCONTROLLER_Device_Start(deviceController);

        if (error != ARCONTROLLER_OK) {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
        }
    }

    if (!failed) {
        // wait state update update
        ARSAL_Sem_Wait(&(stateSem));

        deviceState = ARCONTROLLER_Device_GetState(deviceController, &error);

        if ((error != ARCONTROLLER_OK) || (deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", deviceState);
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
        }
    }

    // send the command that tells to the Bebop to begin its streaming
    if (!failed) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
        error = deviceController->aRDrone3->sendMediaStreamingVideoEnable(deviceController->aRDrone3, 1);
        if (error != ARCONTROLLER_OK) {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
            failed = 1;
        }
    }

    if (!failed) {
        IHM_PrintInfo(ihm,
                      "Running ... ('t' to takeoff ; Spacebar to land ; 'e' for emergency ; Arrow keys and ('r','f','d','g') to move ; 'q' to quit; 'h' for go home; 'p' for plan, '-' for flat-trim)");

#ifdef IHM
        while (gIHMRun) {
            usleep(50);
        }
#else
        int i = 20;
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- sleep 20 ... ");
        while (gIHMRun && i--)
            sleep(1);
#endif
    }

#ifdef IHM
    IHM_Delete(&ihm);
#endif

    // we are here because of a disconnection or user has quit IHM, so safely delete everything
    if (deviceController != NULL) {


        deviceState = ARCONTROLLER_Device_GetState(deviceController, &error);
        if ((error == ARCONTROLLER_OK) && (deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED)) {
            IHM_PrintInfo(ihm, "Disconnecting ...");
            ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

            error = ARCONTROLLER_Device_Stop(deviceController);

            if (error == ARCONTROLLER_OK) {
                // wait state update update
                ARSAL_Sem_Wait(&(stateSem));
            }
        }

        IHM_PrintInfo(ihm, "");
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARCONTROLLER_Device_Delete ...");
        ARCONTROLLER_Device_Delete(&deviceController);

        if (DISPLAY_WITH_MPLAYER) {
            fflush(videoOut);
            fclose(videoOut);

            if (child > 0) {
                kill(child, SIGKILL);
            }
        }
    }

    ARSAL_Sem_Destroy(&(stateSem));

    unlink(fifo_name);
    rmdir(fifo_dir);

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");

    return EXIT_SUCCESS;
}

/*****************************************
 *
 *             private implementation:
 *
 ****************************************/

// called when the state of the device controller has changed
void stateChanged(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *customData) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....", newState);

    switch (newState) {
        case ARCONTROLLER_DEVICE_STATE_STOPPED:
            ARSAL_Sem_Post(&(stateSem));
            //stop
            gIHMRun = 0;

            break;

        case ARCONTROLLER_DEVICE_STATE_RUNNING:
            ARSAL_Sem_Post(&(stateSem));
            break;

        default:
            break;
    }
}

static void cmdBatteryStateChangedRcv(ARCONTROLLER_Device_t *deviceController,
                                      ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = NULL;

    if (elementDictionary == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
        return;
    }

    // get the command received in the device controller
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);

    if (singleElement == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is NULL");
        return;
    }

    // get the value
    HASH_FIND_STR(singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT,
                  arg);

    if (arg == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is NULL");
        return;
    }

    // update UI
    batteryStateChanged(arg->value.U8);
}

static void cmdSensorStateListChangedRcv(ARCONTROLLER_Device_t *deviceController,
                                         ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *dictElement = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *dictTmp = NULL;

    eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME sensorName = ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_MAX;
    int sensorState = 0;

    if (elementDictionary == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
        return;
    }

    // get the command received in the device controller
    HASH_ITER(hh, elementDictionary, dictElement, dictTmp)
    {
        // get the Name
        HASH_FIND_STR(dictElement->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME, arg);
        if (arg != NULL) {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg sensorName is NULL");
            continue;
        }

        sensorName = arg->value.I32;

        // get the state
        HASH_FIND_STR(dictElement->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORSTATE, arg);
        if (arg == NULL) {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg sensorState is NULL");
            continue;
        }

        sensorState = arg->value.U8;
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "sensorName %d ; sensorState: %d", sensorName, sensorState);
    }
}


void
onGPSCommandReceived(ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL) {
        double latitude = 0.0, longitude = 0.0, altitude = 0.0;
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_LATITUDE,
                      arg);
        if (arg != NULL) {
            latitude = arg->value.Double;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_LONGITUDE,
                      arg);
        if (arg != NULL) {
            longitude = arg->value.Double;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_ALTITUDE,
                      arg);
        if (arg != NULL) {
            altitude = arg->value.Double;
        }
        gpsStateChanged(latitude, longitude, altitude);
    }
}

void onAttitudeCommandReceived(ARCONTROLLER_Device_t *deviceController,
                               ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL) {
        float roll = 0.0, pitch = 0.0, yaw = 0.0;
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_ROLL, arg);
        if (arg != NULL) {
            roll = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_PITCH,
                      arg);
        if (arg != NULL) {
            pitch = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_YAW, arg);
        if (arg != NULL) {
            yaw = arg->value.Float;
        }
        attitudeStateChanged(roll, pitch, yaw);
    }
}

void
onSpeedCommandReceived(ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL) {
        float speedX, speedY, speedZ;
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDX, arg);
        if (arg != NULL) {
            speedX = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDY, arg);
        if (arg != NULL) {
            speedY = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDZ, arg);
        if (arg != NULL) {
            speedZ = arg->value.Float;
        }
        speedStateChanged(speedX, speedY, speedZ);
    }
}


void onMoveEndCommandReceived(ARCONTROLLER_Device_t *deviceController,
                              ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    float dX_end = 0.0, dY_end = 0.0, dZ_end = 0.0, dPsi_end = 0.0;
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL) {
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DX, arg);
        if (arg != NULL) {
            dX_end = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DY, arg);
        if (arg != NULL) {
            dY_end = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DZ, arg);
        if (arg != NULL) {
            dZ_end = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DPSI, arg);
        if (arg != NULL) {
            dPsi_end = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR, arg);
        if (arg != NULL) {
            eARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR error = arg->value.I32;
        }
    }
    char str[256];
    sprintf(str, "dX_end:%f m,dY_end:%f m,dZ_end:%f m, dPsi_end:%lf rd\n", dX_end, dY_end, dZ_end, dPsi_end);
    IHM_PrintInfo(ihm, str);
}

// called when a command has been received from the drone
void commandReceived(eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                     void *customData) {
    ARCONTROLLER_Device_t *deviceController = customData;

    if (deviceController == NULL)
        return;

    // if the command received is a battery state changed
    switch (commandKey) {
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED:
            cmdBatteryStateChangedRcv(deviceController, elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED:
            cmdSensorStateListChangedRcv(deviceController, elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED:
            onGPSCommandReceived(deviceController, elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED:
            onAttitudeCommandReceived(deviceController, elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED:
            onSpeedCommandReceived(deviceController, elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND:
            onMoveEndCommandReceived(deviceController, elementDictionary);
            break;
        default:
            break;
    }
}

void batteryStateChanged(uint8_t percent) {
    // callback of changing of battery level

    if (ihm != NULL) {
        IHM_PrintBattery(ihm, percent);
    }
}

static float cur_speedX, cur_speedY, cur_speedZ;

void speedStateChanged(float speedX, float speedY, float speedZ) {
    // callback of changing of attitude label

    if (ihm != NULL) {
        cur_speedX = speedX;
        cur_speedY = speedY;
        cur_speedZ = speedZ;
        IHM_PrintSpeed(ihm, speedX, speedY, speedZ);
    }
}

static float cur_roll, cur_pitch, cur_yaw;

void attitudeStateChanged(float roll, float pitch, float yaw) {
    // callback of changing of attitude label

    if (ihm != NULL) {
        cur_roll = roll;
        cur_pitch = pitch;
        cur_yaw = yaw;
        IHM_PrintAttitude(ihm, roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    }
}

static double cur_lat, cur_lon, cur_alt;

void gpsStateChanged(double latitude, double longitude, double altitude) {
    // callback of changing of gps label

    if (ihm != NULL) {
        cur_lat = latitude;
        cur_lon = longitude;
        cur_alt = altitude;
        IHM_PrintGPS(ihm, latitude, longitude, altitude);
        //FILE* file = fopen("/home/kevin/Documents/current_position.txt", "w");
        //fprintf(file,"lat: %f,lon: %f,alt: %f,", latitude, longitude, altitude);
        //fclose(file);
    }
}

eARCONTROLLER_ERROR decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void *customData) {
    if (videoOut != NULL) {
        if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_H264) {
            if (DISPLAY_WITH_MPLAYER) {
                fwrite(codec.parameters.h264parameters.spsBuffer, codec.parameters.h264parameters.spsSize, 1,
                       videoOut);
                fwrite(codec.parameters.h264parameters.ppsBuffer, codec.parameters.h264parameters.ppsSize, 1,
                       videoOut);

                fflush(videoOut);
            }
        }

    } else {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "videoOut is NULL.");
    }

    return ARCONTROLLER_OK;
}


eARCONTROLLER_ERROR didReceiveFrameCallback(ARCONTROLLER_Frame_t *frame, void *customData) {
    if (videoOut != NULL) {
        if (frame != NULL) {
            if (DISPLAY_WITH_MPLAYER) {
                fwrite(frame->data, frame->used, 1, videoOut);

                fflush(videoOut);
            }
        } else {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "frame is NULL.");
        }
    } else {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "videoOut is NULL.");
    }

    return ARCONTROLLER_OK;
}


// IHM callbacks:

void onInputEvent(eIHM_INPUT_EVENT event, void *customData) {
    // Manage IHM input events
    ARCONTROLLER_Device_t *deviceController = (ARCONTROLLER_Device_t *) customData;
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
    int maxVal = 100;//added by kev
    switch (event) {
        case IHM_INPUT_EVENT_EXIT:
            IHM_PrintInfo(ihm, "IHM_INPUT_EVENT_EXIT ...");
            gIHMRun = 0;
            break;
        case IHM_INPUT_EVENT_EMERGENCY:
            if (deviceController != NULL) {
                // send a Emergency command to the drone
                error = deviceController->aRDrone3->sendPilotingEmergency(deviceController->aRDrone3);
            }
            break;
        case IHM_INPUT_EVENT_LAND:
            if (deviceController != NULL) {
                // send a landing command to the drone
                error = deviceController->aRDrone3->sendPilotingLanding(deviceController->aRDrone3);
            }
            break;
        case IHM_INPUT_EVENT_TAKEOFF:
            if (deviceController != NULL) {
                //check status of nearby airspace flight restrictions
                FILE *file = fopen("/home/kevin/Documents/cur_advisory_color.txt", "r");
                char line[256];
                fscanf(file, "advisory_color: %s", line);
                fclose(file);
                //printf("%s", line);
                if (!strstr(line, "green")) {
                    //abort takeoff
                    IHM_PrintInfo(ihm, "Error advisory_color is not green, takeoff aborted\n\n");
                    break;
                } else {
                    //IHM_PrintInfo(ihm, "advisory_color is green\n\n");
                    // send a takeoff command to the drone
                    error = deviceController->aRDrone3->sendPilotingTakeOff(deviceController->aRDrone3);
                }
            }
            break;
        case IHM_INPUT_EVENT_UP:
            if (deviceController != NULL) {
                // set the flag and speed value of the piloting command
                error = deviceController->aRDrone3->setPilotingPCMDGaz(deviceController->aRDrone3, maxVal);
            }
            break;
        case IHM_INPUT_EVENT_DOWN:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->setPilotingPCMDGaz(deviceController->aRDrone3, -maxVal);
            }
            break;
        case IHM_INPUT_EVENT_RIGHT:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->setPilotingPCMDYaw(deviceController->aRDrone3, maxVal);
            }
            break;
        case IHM_INPUT_EVENT_LEFT:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->setPilotingPCMDYaw(deviceController->aRDrone3, -maxVal);
            }
            break;
        case IHM_INPUT_EVENT_FORWARD:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->setPilotingPCMDPitch(deviceController->aRDrone3, maxVal);
                error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
            }
            break;
        case IHM_INPUT_EVENT_BACK:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->setPilotingPCMDPitch(deviceController->aRDrone3, -maxVal);
                error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
            }
            break;
        case IHM_INPUT_EVENT_ROLL_LEFT:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->setPilotingPCMDRoll(deviceController->aRDrone3, -maxVal);
                error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
            }
            break;
        case IHM_INPUT_EVENT_ROLL_RIGHT:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->setPilotingPCMDRoll(deviceController->aRDrone3, maxVal);
                error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
            }
            break;
        case IHM_INPUT_EVENT_HOME:
            if (deviceController != NULL) {
                int start = 1;
                error = deviceController->aRDrone3->sendPilotingNavigateHome(deviceController->aRDrone3,
                                                                             (uint8_t) start);
            }
            break;
        case IHM_INPUT_EVENT_PLAN:
            if (deviceController != NULL) {
                //flips //FRONT, BACK, RIGHT, LEFT
                //eARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION direction = ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_BACK;
                //deviceController->aRDrone3->sendAnimationsFlip(deviceController->aRDrone3, direction);

                //float dX = 0.0, dY = 0.0, dZ = 0.0, dPsi = 0.0 - cur_yaw;
                /*FILE *file = fopen("/home/kevin/Documents/flightplan.mavlink.txt", "r");
                char lines[20][256];
                int i = 0;
                //fgets(lines[i], sizeof(lines[0]), file);
                int choice = 2;
                fscanf(file, "%s %s %s\n", lines[0], lines[1], lines[2]);
                //printf("%s %s %s\n", lines[0],lines[1],lines[2]);


                for (i = 0; i < 8; i++) {
                    char index[256], cur_wp[256], coord_frame[256], cmd[256], param1[256], param2[256], param3[256], param4[256];
                    double lat, lon, alt;
                    char autocontinue[256];
                    fscanf(file, "%s %s %s %s %s %s %s %s %lf %lf %lf %s\n", index, cur_wp, coord_frame, cmd, param1,
                           param2, param3, param4, &lon, &lat, &alt, autocontinue);
                    //fgets(lines[i], sizeof(lines[0]), file);
                    //printf("%s", lines[i]);
                    if (i == choice) {
                        double hypotenuse = distance(cur_lat, cur_lon, lat, lon);
                        dX = (float) distance(cur_lat, cur_lon, cur_lat, lon);
                        dY = (float) distance(cur_lat, cur_lon, lat, cur_lon);
                        dZ = 0;//(float) alt - cur_alt;
                        char str[256];
                        sprintf(str, "{cur_lat:%lf dg,cur_lon:%lf dg,cur_alt:%lf m},{lat:%lf dg,lon:%lf dg,alt:%lf m},{dX:%f m,dY:%f m,dZ:%f m, hypotenuse:%lf m}\n", cur_lat, cur_lon, cur_alt, lat, lon, alt, dX, dY, dZ, hypotenuse);
                        IHM_PrintInfo(ihm, str);
                    }

                }
                fclose(file);*/

                //relative move works
                /*dX = 0.0, dY = 0.0, dZ = 0.0, dPsi = 0.0 - cur_yaw;
                deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, dX, dY, dZ, dPsi);
                sleep(5);

                dX = 0.0, dY = 0.0, dZ = 0.0, dPsi = 360 * DEG2RAD;
                deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, dX, dY, dZ, dPsi);*/


                //TODO: FIX
                char filepath[256] = "flightplans/flightplan.mavlink";
                eARCOMMANDS_COMMON_MAVLINK_START_TYPE type = ARCOMMANDS_COMMON_MAVLINK_START_TYPE_FLIGHTPLAN;
                error = deviceController->common->sendMavlinkStart(deviceController->common, filepath, type);
            }
            break;
        case IHM_INPUT_EVENT_FLAT_TRIM:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->sendPilotingFlatTrim(deviceController->aRDrone3);
            }
            break;
        case IHM_INPUT_EVENT_NONE:
            if (deviceController != NULL) {
                bool rcDisabled = true;
                if (rcDisabled) {
                    error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0,
                                                                        0);
                } else {
                    FILE *file = fopen("/home/kevin/Documents/DJISamples/RCServer/cur_sticks.txt", "r");
                    //char line[256];
                    //fgets(line, sizeof(line), file);
                    int rh, rv, lv, lh;
                    fscanf(file, "cur_sticks: [%d,%d,%d,%d]", &rh, &rv, &lv, &lh);
                    fclose(file);
                    int8_t roll = lh / 7,
                            pitch = lv / 7,
                            yaw = rh / 7,
                            gaz = rv / 7;
                    uint8_t flag = 0;
                    if (roll == 0 && pitch == 0)
                        flag = 0;
                    else
                        flag = 1;
                    //long ms; // Milliseconds
                    //time_t s;  // Seconds
                    //struct timespec spec;
                    //clock_gettime(CLOCK_REALTIME, &spec);
                    //s  = spec.tv_sec;
                    //ms = round(spec.tv_nsec / 1.0e6);
                    uint32_t timestampAndSeqNum = 0;
                    //sprintf(line,"cur_sticks: [%d,%d,%d,%d], [roll:%d,pitch:%d,yaw:%d,gaz:%d]", rh, rv, lv, lh, roll, pitch, yaw, gaz);
                    //printf("%s", line);
                    //IHM_PrintSticks(ihm, line);
                    error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, flag, roll,
                                                                        pitch,
                                                                        yaw, gaz, timestampAndSeqNum);
                }
            }
            break;
        default:
            break;
    }

    // This should be improved, here it just displays that one error occured
    if (error != ARCONTROLLER_OK) {
        IHM_PrintInfo(ihm, "Error sending an event");
    }
}

int customPrintCallback(eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va) {
    // Custom callback used when ncurses is runing for not disturb the IHM

    if ((level == ARSAL_PRINT_ERROR) && (strcmp(TAG, tag) == 0)) {
        // Save the last Error
        vsnprintf(gErrorStr, (ERROR_STR_LENGTH - 1), format, va);
        gErrorStr[ERROR_STR_LENGTH - 1] = '\0';
    }

    return 1;
}

double distance(double lat_0, double lon_0, double lat_1, double lon_1) {
    double lat_0_rad = lat_0 * DEG2RAD, lon_0_rad = lon_0 * DEG2RAD, lat_1_rad = lat_1 * DEG2RAD, lon_1_rad =
            lon_1 * DEG2RAD;
    double central_angle_rad = acos(
            sin(lat_0_rad) * sin(lat_1_rad) + cos(lat_0_rad) * cos(lat_1_rad) * cos(lon_0_rad - lon_1_rad));
    double mean_radius = 6371; //earth, km
    return central_angle_rad * mean_radius * 1000; //meters
}

