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

char cur_advisory_color_filePathStr[100] = "/home/kevin/Documents/cur_advisory_color.txt";

bool rcDisabled = true;
char cur_sticks_filePathStr[100] = "/home/kevin/Documents/DJISamples/RCServer/cur_sticks.txt";

char flightplan_filePathStr[100] = "/home/kevin/Documents/flightplan_sphinx_ewu.mavlink";
char flightplan_remoteFTP_filePathStr[256] = "internal_000/flightplans/flightplan.mavlink";

static bool autoContinue = false;
const double max_dX = 1000;
const double endMove_dS = 5.0;

static int waypoint_choice = 0;
#define MAX_NUM_WAYPOINTS 8
static int numWaypoints = 0;
static double waypoints[MAX_NUM_WAYPOINTS][4];

#define MAX_NUM_REGIONS 1
#define MAX_NUM_POINTS 5

static int insideExclusionZone = 0;
static double exclusionZones[MAX_NUM_REGIONS][MAX_NUM_POINTS][2] = {
        {
                {47.490568, -117.585980},
                {47.490565, -117.585516},
                {47.490052, -117.585514},
                {47.490058, -117.585973},
                {47.490568, -117.585980},
        }};

static float cur_speedX, cur_speedY, cur_speedZ;
static float cur_roll, cur_pitch, cur_yaw;
static double cur_lat, cur_lon, cur_alt;

bool verboseModeDisabled = true;
bool boundingModeDisabled = true;

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
                      "Running ... ('t' to takeoff ; Spacebar to land ; 'e' for emergency ; Arrow keys and ('r','f','d','g') to move ; 'q' to quit; 'h' for go home; 'p' for plan, 0-9 for waypoints, '-' for flat-trim)");

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
    HASH_ITER(hh, elementDictionary, dictElement, dictTmp)  //TODO: fix syntax highlighting error reported in CLion
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
//    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
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

        cur_lat = latitude;
        cur_lon = longitude;
        cur_alt = altitude;
        if(! boundingModeDisabled) {
//            double maxLat = -90, minLat = 90, maxLon = -180, minLon = 180;
            double avgLat = 0, avgLon = 0;
            int cur_point_index = 0;
            for(cur_point_index = 0; cur_point_index < MAX_NUM_POINTS - 1; cur_point_index++) {
                double lat = exclusionZones[0][cur_point_index][0];
                double lon = exclusionZones[0][cur_point_index][1];
                avgLat += lat;
                avgLon += lon;

                /*if( lat > maxLat ) { maxLat = lat; }
                if( lat < minLat ) { minLat = lat; }
                if( lon > maxLon ) { maxLon = lon; }
                if( lon < minLon ) { minLon = lon; }*/
            }
            avgLat /= (double)(MAX_NUM_POINTS - 1);//TODO: Make global or dynamic var instead of hard-coding the value to equal 4.0
            avgLon /= (double)(MAX_NUM_POINTS - 1);


            double exclusionZones_0_center_lat = avgLat; //(maxLat - minLat)/2.0;
            double exclusionZones_0_center_lon = avgLon; //(maxLon - minLon)/2.0;
            double dS_0 = distance(cur_lat, cur_lon, exclusionZones_0_center_lat, exclusionZones_0_center_lon);
            double dPsi_0 = heading(cur_lat, cur_lon, exclusionZones_0_center_lat, exclusionZones_0_center_lon);

            double exclusionZones_0_lats[MAX_NUM_POINTS] = {
                    exclusionZones[0][0][0],
                    exclusionZones[0][1][0],
                    exclusionZones[0][2][0],
                    exclusionZones[0][3][0],
                    exclusionZones[0][4][0],
            };
            double exclusionZones_0_lons[MAX_NUM_POINTS] = {
                    exclusionZones[0][0][1],
                    exclusionZones[0][1][1],
                    exclusionZones[0][2][1],
                    exclusionZones[0][3][1],
                    exclusionZones[0][4][1],
            };

            insideExclusionZone = pointInPolygon(MAX_NUM_POINTS,exclusionZones_0_lats, exclusionZones_0_lons, cur_lat, cur_lon);

            if(insideExclusionZone){
                if(autoContinue){ autoContinue = false; }
//                error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
            }
            char str[256];
            if(!verboseModeDisabled)
                sprintf(str, "exclusionZones_0: { center:{ lat:%lf, lon:%lf, dS_0:%lf, dPsi_0:%lf }, inside: %s} \n",
                        exclusionZones_0_center_lat, exclusionZones_0_center_lon, dS_0, dPsi_0, insideExclusionZone ? "true" : "false");
            IHM_PrintInfo(ihm, str);
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
        float speedX = 0.0, speedY = 0.0, speedZ = 0.0;
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
            if(error>0) {
                char moveErrorStr[256];
                sprintf(moveErrorStr, "moveErrorStr:%i\n", error);
                IHM_PrintInfo(ihm, moveErrorStr);
            }
        }
    }
    /*char str[256];
    sprintf(str, "dX_end:%f m,dY_end:%f m,dZ_end:%f m, dPsi_end:%lf rd\n", dX_end, dY_end, dZ_end, dPsi_end);
    IHM_PrintInfo(ihm, str);*/
    double lat0 = cur_lat,lon0 = cur_lon,lat1 = waypoints[waypoint_choice][0],lon1 = waypoints[waypoint_choice][1];
    double tot_dX = distance(lat0, lon0, lat1, lon1);
    double dX = tot_dX < max_dX ? tot_dX : max_dX; //1000 meters max
    double dZ = cur_alt - waypoints[waypoint_choice][2];
    double dPsi = heading(lat0, lon0, lat1, lon1) - (double) cur_yaw;
    dPsi = dPsi>PI?(dPsi-(2.0*PI)):dPsi;
    if(autoContinue) {
        if(dX > 5.0 || abs(dZ) > 5.0) {//TODO: add dependence on gps, compass, imu and optical flow positioning accuracy
            if (abs(dPsi) > 0.5) {
                deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, 0.0, 0.0, 0.0, (float) dPsi);
            } else {
                deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, (float) dX, 0.0, (float) dZ, 0.0);
            }
        } else if (waypoint_choice < numWaypoints - 1) {
            waypoint_choice++;
            char waypoint_choice_str[256];
                sprintf(waypoint_choice_str, "waypoint_choice:%i\n", waypoint_choice);
            IHM_PrintInfo(ihm, waypoint_choice_str);
            performRelativeMove(deviceController);
        } else {
            autoContinue = false;
                IHM_PrintInfo(ihm, "PLAN FINISHED");
        }
    }
}

void onSetMaxPitchRollCommmandReceived(ARCONTROLLER_Device_t *deviceController,
                                       ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL) {
        float current = 0.0, min = 0.0, max = 0.0;
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED_CURRENT, arg);
        if (arg != NULL) {
            current = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED_MIN,
                      arg);
        if (arg != NULL) {
            min = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED_MAX,
                      arg);
        if (arg != NULL) {
            max = arg->value.Float;
        }
        if(!verboseModeDisabled) {
            char setMaxPitchRollStr[256];
            sprintf(setMaxPitchRollStr, "onSetMaxPitchRollCommmandReceived current:%f,min:%f,max:%f\n", current, min,
                    max);
            IHM_PrintInfo(ihm, setMaxPitchRollStr);
        }
    }
}

void onSetMaxVerticalSpeedCommmandReceived(ARCONTROLLER_Device_t *deviceController,
                                           ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    float current = 0.0, min = 0.0, max = 0.0;

    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL) {
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXVERTICALSPEEDCHANGED_CURRENT, arg);
        if (arg != NULL) {
            current = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXVERTICALSPEEDCHANGED_MIN, arg);
        if (arg != NULL) {
            min = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXVERTICALSPEEDCHANGED_MAX, arg);
        if (arg != NULL) {
            max = arg->value.Float;
        }
    }
    if(!verboseModeDisabled) {
        char setMaxVerticalSpeedStr[256];
        sprintf(setMaxVerticalSpeedStr, "onSetMaxVerticalSpeedCommmandReceived current:%f,min:%f,max:%f\n", current,
                min,
                max);
        IHM_PrintInfo(ihm, setMaxVerticalSpeedStr);
    }
}

void onSetMaxPitchRollRotationSpeedCommmandReceived(ARCONTROLLER_Device_t *deviceController,
                                                    ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    float current = 0.0, min = 0.0, max = 0.0;

    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL) {
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXPITCHROLLROTATIONSPEEDCHANGED_CURRENT,
                      arg);
        if (arg != NULL) {
            current = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXPITCHROLLROTATIONSPEEDCHANGED_MIN,
                      arg);
        if (arg != NULL) {
            min = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXPITCHROLLROTATIONSPEEDCHANGED_MAX,
                      arg);
        if (arg != NULL) {
            max = arg->value.Float;
        }
    }
    if(!verboseModeDisabled) {
        char setMaxPitchRollRotationSpeedStr[256];
        sprintf(setMaxPitchRollRotationSpeedStr,
                "onSetMaxPitchRollRotationSpeedCommmandReceived current:%f,min:%f,max:%f\n", current, min, max);
        IHM_PrintInfo(ihm, setMaxPitchRollRotationSpeedStr);
    }
}

void onSetMaxRotationSpeedCommmandReceived(ARCONTROLLER_Device_t *deviceController,
                                           ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary) {
    float current = 0.0, min = 0.0, max = 0.0;

    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL) {
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXROTATIONSPEEDCHANGED_CURRENT, arg);
        if (arg != NULL) {
            current = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXROTATIONSPEEDCHANGED_MIN, arg);
        if (arg != NULL) {
            min = arg->value.Float;
        }
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXROTATIONSPEEDCHANGED_MAX, arg);
        if (arg != NULL) {
            max = arg->value.Float;
        }
    }
    if(!verboseModeDisabled) {
        char setMaxRotationStr[256];
        sprintf(setMaxRotationStr, "onSetMaxRotationSpeedCommmandReceived current:%f,min:%f,max:%f\n", current, min,
                max);
        IHM_PrintInfo(ihm, setMaxRotationStr);
    }
}


// called when a command has been received from the drone
void commandReceived(eARCONTROLLER_DICTIONARY_KEY commandKey,
                     ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
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
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXVERTICALSPEEDCHANGED:
            onSetMaxVerticalSpeedCommmandReceived(deviceController, elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED:
            onSetMaxPitchRollCommmandReceived(deviceController, elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXPITCHROLLROTATIONSPEEDCHANGED:
            onSetMaxPitchRollRotationSpeedCommmandReceived(deviceController, elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXROTATIONSPEEDCHANGED:
            onSetMaxRotationSpeedCommmandReceived(deviceController, elementDictionary);
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


void speedStateChanged(float speedX, float speedY, float speedZ) {
    // callback of changing of speed label

    if (ihm != NULL) {
        cur_speedX = speedX;
        cur_speedY = speedY;
        cur_speedZ = speedZ;
        if(!verboseModeDisabled)
            IHM_PrintSpeed(ihm, speedX, speedY, speedZ);
    }
}


void attitudeStateChanged(float roll, float pitch, float yaw) {
    // callback of changing of attitude label

    if (ihm != NULL) {
        cur_roll = roll;
        cur_pitch = pitch;
        cur_yaw = yaw;
        if(!verboseModeDisabled)
            IHM_PrintAttitude(ihm, roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    }
}


void gpsStateChanged(double latitude, double longitude, double altitude) {
    // callback of changing of gps label
    if (ihm != NULL) {
        if(!verboseModeDisabled)
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

////TODO: FRONT, BACK, RIGHT, LEFT KEY BINDINGS
void performBackFlip(ARCONTROLLER_Device_t *deviceController) {
    eARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION direction = ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_BACK;
    int res = deviceController->aRDrone3->sendAnimationsFlip(deviceController->aRDrone3, direction);
    if (res < 0) {
        IHM_PrintInfo(ihm, "performBackFlip ERROR");
    }
}

void loadWaypoints() {
    //float dX = 0.0, dY = 0.0, dZ = 0.0, dPsi = 0.0;
    FILE *file = fopen(flightplan_filePathStr, "r");
    char lines[20][256];
    int i = 0;
    //fgets(lines[i], sizeof(lines[0]), file);
    int res = fscanf(file, "%s %s %s\n", lines[0], lines[1], lines[2]);
    if (res < 0) {
        IHM_PrintInfo(ihm, "loadWaypoints read header ERROR");
    }
    //printf("%s %s %s\n", lines[0],lines[1],lines[2]);

    int j = 0;//num found waypoints
    for (i = 0; i < MAX_NUM_WAYPOINTS; i++) {
        char index[256], cur_wp[256], coord_frame[256], cmd[256], param1[256], param2[256], param3[256];
        double yaw, lat, lon, alt;
        char autocontinue[256];
        int res = fscanf(file, "%s %s %s %s %s %s %s %lf %lf %lf %lf %s\n", index, cur_wp, coord_frame, cmd, param1,
                         param2, param3, &yaw, &lat, &lon, &alt, autocontinue);
        if (res < 0) {
            IHM_PrintInfo(ihm, "loadWaypoints read line ERROR");
        }
        if (strstr(cmd, "16")) {// check for waypoint command
            waypoints[j][0] = lat;
            waypoints[j][1] = lon;
            waypoints[j][2] = alt;
            yaw = yaw * DEG2RAD;
            waypoints[j][3] = yaw;
            j++;
        }
        //fgets(lines[i], sizeof(lines[0]), file);
        //printf("%s", lines[i]);

        /*if (i == waypoint_choice) { //TODO: test for broken after recent modifications
            double hypotenuse = distance(cur_lat, cur_lon, lat, lon);
            dX = (float) distance(cur_lat, cur_lon, cur_lat, lon);
            dY = (float) distance(cur_lat, cur_lon, lat, cur_lon);
            dZ = (float) 0.0; // (float) alt - cur_alt;
            dPsi = 0.0-yaw;
            char str[256];
            sprintf(str, "{cur_lat:%lf dg,cur_lon:%lf dg,cur_alt:%lf m,cur_yaw:%lf rd},{lat:%lf dg,lon:%lf dg,alt:%lf m,yaw:%lf rd},{dX:%f m,dY:%f m,dZ:%f m, hypotenuse:%lf m}\n", cur_lat, cur_lon, cur_alt, cur_yaw, lat, lon, alt, yaw, dX, dY, dZ, hypotenuse);
            IHM_PrintInfo(ihm, str);
        }*/

    }
    numWaypoints = j;
    fclose(file);
}

void performTimedFlightTest(ARCONTROLLER_Device_t *deviceController) {
    ////relative move works
    deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, 0.0, 0.0, 0.0, 0.0 - cur_yaw);
    IHM_PrintInfo(ihm, "HEADING NORTH SENT\n");
    sleep(5);

    deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, 1.0, 0.0, 0.0, 0.0);
    IHM_PrintInfo(ihm, "POSITION POSITIVE X SENT\n");
    sleep(5);
    deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, -1.0, 0.0, 0.0, 0.0);
    IHM_PrintInfo(ihm, "POSITION NEGATIVE X SENT\n");
    sleep(5);


    deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, 0.0, 1.0, 0.0, 0.0);
    IHM_PrintInfo(ihm, "POSITION POSITIVE Y SENT\n");
    sleep(5);
    deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, 0.0, -1.0, 0.0, 0.0);
    IHM_PrintInfo(ihm, "POSITION NEGATIVE Y SENT\n");
    sleep(5);

    deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, 0.0, 0.0, 1.0, 0.0);
    IHM_PrintInfo(ihm, "POSITION POSITIVE Z SENT\n");
    sleep(5);
    deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, 0.0, 0.0, -1.0, 0.0);
    IHM_PrintInfo(ihm, "POSITION NEGATIVE Z SENT\n");
    sleep(5);

    IHM_PrintInfo(ihm, "MOVES ALL DONE\n");
}

////TODO: listen for flightplan component state callbacks
void startFlightplan(ARCONTROLLER_Device_t *deviceController) {
    eARCOMMANDS_COMMON_MAVLINK_START_TYPE type = ARCOMMANDS_COMMON_MAVLINK_START_TYPE_FLIGHTPLAN;
    int error = deviceController->common->sendMavlinkStart(deviceController->common, flightplan_filePathStr, type);
    if (error < 0) {
        IHM_PrintInfo(ihm, "startFlightplan ERROR");
    }

}

void performRelativeMove(ARCONTROLLER_Device_t *deviceController) {
    double lat0 = cur_lat,lon0 = cur_lon,lat1 = waypoints[waypoint_choice][0],lon1 = waypoints[waypoint_choice][1];
    /*double tot_dX = distance(lat0, lon0, lat1, lon1);
    double dX = tot_dX < max_dX ? tot_dX : max_dX; //1000 meters max
    double dZ = cur_alt - waypoints[waypoint_choice][2];*/
    double dPsi = heading(lat0, lon0, lat1, lon1) - (double) cur_yaw;
    dPsi = dPsi>PI?(dPsi-(2.0*PI)):dPsi;
    /*char str[256];
    sprintf(str, "performRelativeMove UNUSED_dX:%lf m, UNUSED_dZ:%lf m, dPsi:%lf rd\n", dX, dZ, dPsi);
    IHM_PrintInfo(ihm, str);*/
    deviceController->aRDrone3->sendPilotingMoveBy(deviceController->aRDrone3, 0.0, 0.0, 0.0, (float)dPsi);
    autoContinue = true;////TODO: implement job queue
}

void displayWaypoint() {
    double dist = distance(cur_lat, cur_lon, waypoints[waypoint_choice][0], waypoints[waypoint_choice][1]);
    double head = heading(cur_lat, cur_lon, waypoints[waypoint_choice][0], waypoints[waypoint_choice][1]);
    double max_move_dist = dist < 1000.0 ? dist : 1000.0;

    char str[256];
    sprintf(str,
            "{WP_%i_LAT:%lf, WP_%i_LON:%lf, WP_%i_ALT:%lf, WP_%i_YAW:%lf, DISTANCE:%f, HEADING:%f} MAX_MOVE_DIST:%lf\n",
            waypoint_choice,
            waypoints[waypoint_choice][0],
            waypoint_choice,
            waypoints[waypoint_choice][1],
            waypoint_choice,
            waypoints[waypoint_choice][2],
            waypoint_choice,
            waypoints[waypoint_choice][3],
            (float) dist,
            (float) head,
            (float) max_move_dist
    );
    IHM_PrintInfo(ihm, str);
}

void performTakeoff(ARCONTROLLER_Device_t *deviceController){
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
    if(autoContinue){ autoContinue = false; }
    //TODO: check status of nearby airspace flight restrictions instead of just checking advisory color file
    FILE *file = fopen(cur_advisory_color_filePathStr, "r");
    char line[256];
    int res = fscanf(file, "advisory_color: %s", line);
    if (res < 0) {
        IHM_PrintInfo(ihm, "ERROR cur_advisory_color_filePathStr FAILED READ");
        break;
    }

    fclose(file);
    //printf("%s", line);
    if (!strstr(line, "green")) ////TODO: check waivers if not green
    {
        //abort takeoff
        IHM_PrintInfo(ihm, "ABORT TAKEOFF advisory_color NOT green\n");
        break;
    } else if(insideExclusionZone){
        //abort takeoff
        IHM_PrintInfo(ihm, "ABORT TAKEOFF insideExclusionZone: true\n");
        break;
    }else {
        IHM_PrintInfo(ihm, "advisory_color: green\n\n");
        //set max pitch and roll
        float currentMaxPitchRoll = 30.0;
        deviceController->aRDrone3->sendPilotingSettingsMaxTilt(deviceController->aRDrone3,
                                                                currentMaxPitchRoll);


        //set max vertical speed
        float currentMaxVerticalSpeed = 2.5;
        deviceController->aRDrone3->sendSpeedSettingsMaxVerticalSpeed(deviceController->aRDrone3,
                                                                      currentMaxVerticalSpeed);

        //set max pitch roll rotational speed
        float currentMaxPitchRollRotationSpeed = 300.0;
        deviceController->aRDrone3->sendSpeedSettingsMaxPitchRollRotationSpeed(deviceController->aRDrone3,
                                                                               currentMaxPitchRollRotationSpeed);

        //set max rotational speed
        float currentMaxRotationalSpeed = 200.0;
        deviceController->aRDrone3->sendSpeedSettingsMaxRotationSpeed(deviceController->aRDrone3,
                                                                      currentMaxRotationalSpeed);

        // send a takeoff command to the drone
        error = deviceController->aRDrone3->sendPilotingTakeOff(deviceController->aRDrone3);
    }
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
                if(autoContinue){ autoContinue = false; }
                // send a Emergency command to the drone
                error = deviceController->aRDrone3->sendPilotingEmergency(deviceController->aRDrone3);
            }
            break;
        case IHM_INPUT_EVENT_LAND:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                // send a landing command to the drone
                error = deviceController->aRDrone3->sendPilotingLanding(deviceController->aRDrone3);
            }
            break;
        case IHM_INPUT_EVENT_TAKEOFF:
            if (deviceController != NULL) {
                performTakeoff(deviceController);
            }
            break;
        case IHM_INPUT_EVENT_UP:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                // set the flag and speed value of the piloting command
                error = deviceController->aRDrone3->setPilotingPCMDGaz(deviceController->aRDrone3, maxVal);
            }
            break;
        case IHM_INPUT_EVENT_DOWN:
            if(autoContinue){ autoContinue = false; }
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->setPilotingPCMDGaz(deviceController->aRDrone3, -maxVal);
            }
            break;
        case IHM_INPUT_EVENT_RIGHT:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                error = deviceController->aRDrone3->setPilotingPCMDYaw(deviceController->aRDrone3, maxVal);
            }
            break;
        case IHM_INPUT_EVENT_LEFT:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                error = deviceController->aRDrone3->setPilotingPCMDYaw(deviceController->aRDrone3, -maxVal);
            }
            break;
        case IHM_INPUT_EVENT_FORWARD:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                error = deviceController->aRDrone3->setPilotingPCMDPitch(deviceController->aRDrone3, maxVal);
                error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
            }
            break;
        case IHM_INPUT_EVENT_BACK:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                error = deviceController->aRDrone3->setPilotingPCMDPitch(deviceController->aRDrone3, -maxVal);
                error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
            }
            break;
        case IHM_INPUT_EVENT_ROLL_LEFT:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                error = deviceController->aRDrone3->setPilotingPCMDRoll(deviceController->aRDrone3, -maxVal);
                error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
            }
            break;
        case IHM_INPUT_EVENT_ROLL_RIGHT:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                error = deviceController->aRDrone3->setPilotingPCMDRoll(deviceController->aRDrone3, maxVal);
                error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
            }
            break;
        case IHM_INPUT_EVENT_HOME:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                int start = 1;
                error = deviceController->aRDrone3->sendPilotingNavigateHome(deviceController->aRDrone3,
                                                                             (uint8_t) start);
            }
            break;
        case IHM_INPUT_EVENT_NUM:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                //waypoint_choice = abs((waypoint_choice - 1) % 3);
                int tmpNum = getNum();
                tmpNum = tmpNum < 0 ? 0 : tmpNum; //TODO: line not needed if num is already [0-9]
                tmpNum = tmpNum >= numWaypoints ? 0 : tmpNum;
                waypoint_choice = tmpNum;

                loadWaypoints();
                displayWaypoint();

            }
            break;
        case IHM_INPUT_EVENT_GOTO_CUR_WP:
            if (deviceController != NULL) {
                if(autoContinue){ autoContinue = false; }
                ////TODO:fix by extracting to method with synced io
                /*char yesNo[10];
                IHM_PrintInfo(ihm,"PROMPT goto previous waypoint?");
                scanf("%s", yesNo);
                if(!strstr(yesNo,"y")){
                    IHM_PrintInfo(ihm,"ABORT MOVE, yesNo NOT \"y\"");
                    break;
                }*/
                performRelativeMove(deviceController);
            }
            break;
        case IHM_INPUT_EVENT_FLAT_TRIM:
            if (deviceController != NULL) {
                error = deviceController->aRDrone3->sendPilotingFlatTrim(deviceController->aRDrone3);
            }
            break;
        case IHM_INPUT_EVENT_TOGGLE_STICKS:
            rcDisabled = !rcDisabled;
            char rcDisabledStr[256];
            sprintf(rcDisabledStr, "rcDisabled:%s", rcDisabled ? "true" : "false");
            IHM_PrintSticks(ihm, rcDisabledStr);
            break;
        case IHM_INPUT_EVENT_TOGGLE_VERBOSE_MODE:
            verboseModeDisabled = !verboseModeDisabled;
            char verboseModeDisabledStr[256];
            sprintf(verboseModeDisabledStr, "verboseModeDisabled:%s", verboseModeDisabled ? "true" : "false");
            IHM_PrintInfo(ihm, verboseModeDisabledStr);
            break;
        case IHM_INPUT_EVENT_TOGGLE_BOUNDING_MODE:
            boundingModeDisabled = !boundingModeDisabled;
            if(boundingModeDisabled) { insideExclusionZone = 0; }
            char boundingModeDisabledStr[256];
            sprintf(boundingModeDisabledStr, "boundingModeDisabled:%s", boundingModeDisabled ? "true" : "false");
            IHM_PrintInfo(ihm, boundingModeDisabledStr);
            break;
        case IHM_INPUT_EVENT_NONE: //TODO: REMOVE STICKS STUFF FROM INPUT LOOP AND TEST FOR OTHER RC BUTTONS
            if (deviceController != NULL) {

                if (rcDisabled) {
                    error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
                } else {
                    FILE *file = fopen(cur_sticks_filePathStr, "r");
                    int rh, rv, lv, lh;
                    int res = fscanf(file, "cur_sticks: [%d,%d,%d,%d]", &rh, &rv, &lv, &lh);
                    if (res < 0) {
                        if(!verboseModeDisabled)
                            IHM_PrintInfo(ihm, "cur_sticks_filePathStr failed read");
                        break;
                    }
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

                    if(!verboseModeDisabled) {
                        char line[256];
                        //fgets(line, sizeof(line), file);
                        sprintf(line, "cur_sticks: [%d,%d,%d,%d], [roll:%d,pitch:%d,yaw:%d,gaz:%d]", rh, rv, lv, lh,
                                roll, pitch, yaw, gaz);
                        //printf("%s", line);
                        IHM_PrintSticks(ihm, line);
                    }
                    /*if( roll || pitch || yaw || gaz ){
                        if(autoContinue){ autoContinue = false; }
                    }*/
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

double distance(double lat_0, double lon_0,
                double lat_1, double lon_1) {
    double lat_0_rad = lat_0 * DEG2RAD, lon_0_rad = lon_0 * DEG2RAD,
            lat_1_rad = lat_1 * DEG2RAD, lon_1_rad = lon_1 * DEG2RAD;
    double central_angle_rad = acos(
            sin(lat_0_rad) * sin(lat_1_rad) + cos(lat_0_rad) * cos(lat_1_rad) * cos(lon_0_rad - lon_1_rad));
    double mean_radius = 6371; //earth, km
    return central_angle_rad * mean_radius * 1000; //meters
}

////TODO: UPDATE: relative move is accurate enough for ~100m per move
////TODO: fix incorrect result by about 6deg, possibly add abs to to difference calc
double heading(double lat_0, double lon_0,
               double lat_1, double lon_1) {
    double dLon = (lon_1 * DEG2RAD - lon_0 * DEG2RAD);

    double y = sin(dLon) * cos(lat_1 * DEG2RAD);
    double x = cos(lat_0 * DEG2RAD) * sin(lat_1 * DEG2RAD) - sin(lat_0 * DEG2RAD) * cos(lat_1 * DEG2RAD) * cos(dLon);

    double heading = atan2(y, x);
    heading = fmod((heading + 2.0 * PI), 2.0 * PI);

    return heading;
}

/*

 Reference: https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html

 Arg            Desc
 nvert          Number of vertices in the polygon.
 vertx, verty   Arrays containing the x- and y-coordinates of the polygon's vertices.
 testx, testy   X- and y-coordinate of the test point.

 */

int pointInPolygon(int nvert, double *vertx, double *verty, double testx, double testy)
{
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i])) {
            c = !c;
        }
    }
    return c;
}