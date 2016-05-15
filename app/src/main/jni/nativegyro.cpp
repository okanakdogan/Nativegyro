//
// Created by Okan on 27.1.2016.
//
/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

//BEGIN_INCLUDE(all)
#include <jni.h>
#include <errno.h>

#include <EGL/egl.h>
#include <GLES/gl.h>

#include <android/sensor.h>
#include <android/log.h>
#include <android_native_app_glue.h>
#include <math.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "native-gyro", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "native-gyro", __VA_ARGS__))

#define EPSILON 0.000000001f
#define NS2S 1.0f / 1000000000.0f
#define TIME_CONSTANT 30
#define FILTER_COEFFICIENT 0.98f

/*Func. Prototypes*/

void matrixMultiplication(float A[], float B[], float res[]);
void getRotationMatrixFromOrientation(float o[],float resultMat[]);
void sensorManager_getOrientation(float R[],int sizeR,float values[]);
void sensorManager_getRotationMatrixFromVector(float R[],int sizeR, float rotationVector[],int sizeRV);
bool sensorManager_getRotationMatrix(float R[],int sizeR, float I[],int sizeI,
                                     float gravity[], float geomagnetic[]);
void gyroFunction(struct engine* engine,ASensorEvent event);
void calculateAccMagOrientation(struct engine* engine);
void calculateFusedOrientation(struct engine* engine);

/*  Some global variables*/
float accMagOrientation[3]={0};
bool accMagOrienttationInit=false;
bool initState=true;
int64_t timestamp;
// final orientation angles from sensor fusion
float fusedOrientation[3]={0};

/**
 * Our saved state data.
 */
struct saved_state {
    float angle;
    int32_t x;
    int32_t y;
};

/**
 * Shared state for our app.
 */
struct engine {
    struct android_app* app;

    ASensorManager* sensorManager;
    const ASensor* accelerometerSensor;
    const ASensor* gyroSensor;
    const ASensor* magSensor;
    ASensorEventQueue* sensorEventQueue;
    ASensorEventQueue* sensorEventQueueGyro;
    ASensorEventQueue* sensorEventQueueMag;

    // angular speeds from gyro
    float gyro[3];
    // rotation matrix from gyro data
    float gyroMatrix[9];
    float gyroOrientation[3];
    // accelerometer and magnetometer based rotation matrix
    float rotationMatrix[9];
    // magnetic field vector
    float magnet[3];
    // accelerometer vector
    float accel[3];

    int animating;
    EGLDisplay display;
    EGLSurface surface;
    EGLContext context;
    int32_t width;
    int32_t height;
    struct saved_state state;
};

/**
 * Initialize an EGL context for the current display.
 */
static int engine_init_display(struct engine* engine) {
    // initialize OpenGL ES and EGL

    /*
     * Here specify the attributes of the desired configuration.
     * Below, we select an EGLConfig with at least 8 bits per color
     * component compatible with on-screen windows
     */
    const EGLint attribs[] = {
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_NONE
    };
    EGLint w, h, dummy, format;
    EGLint numConfigs;
    EGLConfig config;
    EGLSurface surface;
    EGLContext context;

    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

    eglInitialize(display, 0, 0);

    /* Here, the application chooses the configuration it desires. In this
     * sample, we have a very simplified selection process, where we pick
     * the first EGLConfig that matches our criteria */
    eglChooseConfig(display, attribs, &config, 1, &numConfigs);

    /* EGL_NATIVE_VISUAL_ID is an attribute of the EGLConfig that is
     * guaranteed to be accepted by ANativeWindow_setBuffersGeometry().
     * As soon as we picked a EGLConfig, we can safely reconfigure the
     * ANativeWindow buffers to match, using EGL_NATIVE_VISUAL_ID. */
    eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);

    ANativeWindow_setBuffersGeometry(engine->app->window, 0, 0, format);

    surface = eglCreateWindowSurface(display, config, engine->app->window, NULL);
    context = eglCreateContext(display, config, NULL, NULL);

    if (eglMakeCurrent(display, surface, surface, context) == EGL_FALSE) {
        LOGW("Unable to eglMakeCurrent");
        return -1;
    }

    eglQuerySurface(display, surface, EGL_WIDTH, &w);
    eglQuerySurface(display, surface, EGL_HEIGHT, &h);

    engine->display = display;
    engine->context = context;
    engine->surface = surface;
    engine->width = w;
    engine->height = h;
    engine->state.angle = 0;

    // Initialize GL state.
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glDisable(GL_DEPTH_TEST);

    return 0;
}

/**
 * Just the current frame in the display.
 */
static void engine_draw_frame(struct engine* engine) {
    if (engine->display == NULL) {
        // No display.
        return;
    }

    // Just fill the screen with a color.
    glClearColor(((float)engine->state.x)/engine->width, engine->state.angle,
                 ((float)engine->state.y)/engine->height, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    eglSwapBuffers(engine->display, engine->surface);
}

/**
 * Tear down the EGL context currently associated with the display.
 */
static void engine_term_display(struct engine* engine) {
    if (engine->display != EGL_NO_DISPLAY) {
        eglMakeCurrent(engine->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (engine->context != EGL_NO_CONTEXT) {
            eglDestroyContext(engine->display, engine->context);
        }
        if (engine->surface != EGL_NO_SURFACE) {
            eglDestroySurface(engine->display, engine->surface);
        }
        eglTerminate(engine->display);
    }
    engine->animating = 0;
    engine->display = EGL_NO_DISPLAY;
    engine->context = EGL_NO_CONTEXT;
    engine->surface = EGL_NO_SURFACE;
}

/**
 * Process the next input event.
 */
static int32_t engine_handle_input(struct android_app* app, AInputEvent* event) {
    struct engine* engine = (struct engine*)app->userData;
    if (AInputEvent_getType(event) == AINPUT_EVENT_TYPE_MOTION) {
        engine->animating = 1;
        engine->state.x = AMotionEvent_getX(event, 0);
        engine->state.y = AMotionEvent_getY(event, 0);
        return 1;
    }
    return 0;
}

/**
 * Process the next main command.
 */
static void engine_handle_cmd(struct android_app* app, int32_t cmd) {
    struct engine* engine = (struct engine*)app->userData;
    switch (cmd) {
        case APP_CMD_SAVE_STATE:
            // The system has asked us to save our current state.  Do so.
            engine->app->savedState = malloc(sizeof(struct saved_state));
            *((struct saved_state*)engine->app->savedState) = engine->state;
            engine->app->savedStateSize = sizeof(struct saved_state);
            break;
        case APP_CMD_INIT_WINDOW:
            // The window is being shown, get it ready.
            if (engine->app->window != NULL) {
                engine_init_display(engine);
                engine_draw_frame(engine);
            }
            break;
        case APP_CMD_TERM_WINDOW:
            // The window is being hidden or closed, clean it up.
            engine_term_display(engine);
            break;
        case APP_CMD_GAINED_FOCUS:
            // When our app gains focus, we start monitoring the accelerometer.
            if (engine->accelerometerSensor != NULL) {
                ASensorEventQueue_enableSensor(engine->sensorEventQueue,
                                               engine->accelerometerSensor);

                // We'd like to get 60 events per second (in us).
               ASensorEventQueue_setEventRate(engine->sensorEventQueue,
                                               engine->accelerometerSensor, (1000L/60)*1000);

            }
            if(engine->gyroSensor !=NULL){
                ASensorEventQueue_enableSensor(engine->sensorEventQueueGyro,
                                               engine->gyroSensor);
                ASensorEventQueue_setEventRate(engine->sensorEventQueueGyro,
                                               engine->gyroSensor, (1000L/60)*1000);
            }
            if(engine->magSensor !=NULL){
                ASensorEventQueue_enableSensor(engine->sensorEventQueueMag,
                                               engine->magSensor);
                ASensorEventQueue_setEventRate(engine->sensorEventQueueMag,
                                               engine->magSensor, (1000L/60)*1000);
            }



            break;
        case APP_CMD_LOST_FOCUS:
            // When our app loses focus, we stop monitoring the accelerometer.
            // This is to avoid consuming battery while not being used.
            if (engine->accelerometerSensor != NULL) {
                ASensorEventQueue_disableSensor(engine->sensorEventQueue,
                                                engine->accelerometerSensor);
            }
            if(engine->gyroSensor !=NULL){
                ASensorEventQueue_disableSensor(engine->sensorEventQueueGyro,
                                                engine->gyroSensor);
            }
            if(engine->magSensor !=NULL){
                ASensorEventQueue_disableSensor(engine->sensorEventQueueMag,
                                                engine->gyroSensor);
            }
            // Also stop animating.
            engine->animating = 0;
            engine_draw_frame(engine);
            break;
    }
}

/**
 * This is the main entry point of a native application that is using
 * android_native_app_glue.  It runs in its own thread, with its own
 * event loop for receiving input events and doing other things.
 */
void android_main(struct android_app* state) {
    struct engine engine;

    // Make sure glue isn't stripped.
    app_dummy();

    memset(&engine, 0, sizeof(engine));
    state->userData = &engine;
    state->onAppCmd = engine_handle_cmd;
    state->onInputEvent = engine_handle_input;
    engine.app = state;

    // Prepare to monitor accelerometer
    engine.sensorManager = ASensorManager_getInstance();

    engine.accelerometerSensor = ASensorManager_getDefaultSensor(engine.sensorManager,
                                                                 ASENSOR_TYPE_ACCELEROMETER);
    engine.gyroSensor = ASensorManager_getDefaultSensor(engine.sensorManager,
                                                                 ASENSOR_TYPE_GYROSCOPE);
    engine.gyroSensor = ASensorManager_getDefaultSensor(engine.sensorManager,
                                                        ASENSOR_TYPE_MAGNETIC_FIELD);
    engine.sensorEventQueue = ASensorManager_createEventQueue(engine.sensorManager,
                                                              state->looper, LOOPER_ID_USER, NULL, NULL);
    engine.sensorEventQueueGyro = ASensorManager_createEventQueue(engine.sensorManager,
                                                              state->looper, LOOPER_ID_USER, NULL, NULL);
    engine.sensorEventQueueMag = ASensorManager_createEventQueue(engine.sensorManager,
                                                                  state->looper, LOOPER_ID_USER, NULL, NULL);

    //init gyro

    engine.gyroMatrix[0] = 1.0f; engine.gyroMatrix[1] = 0.0f; engine.gyroMatrix[2] = 0.0f;
    engine.gyroMatrix[3] = 0.0f; engine.gyroMatrix[4] = 1.0f; engine.gyroMatrix[5] = 0.0f;
    engine.gyroMatrix[6] = 0.0f; engine.gyroMatrix[7] = 0.0f; engine.gyroMatrix[8] = 1.0f;

    if (state->savedState != NULL) {
        // We are starting with a previous saved state; restore from it.
        engine.state = *(struct saved_state*)state->savedState;
    }

    // loop waiting for stuff to do.

    while (1) {
        // Read all pending events.
        int ident;
        int events;
        struct android_poll_source* source;

        // If not animating, we will block forever waiting for events.
        // If animating, we loop until all events are read, then continue
        // to draw the next frame of animation.
        while ((ident=ALooper_pollAll(engine.animating ? 0 : -1, NULL, &events,
                                      (void**)&source)) >= 0) {

            // Process this event.
            if (source != NULL) {
                source->process(state, source);
            }

            // If a sensor has data, process it now.
            if (ident == LOOPER_ID_USER) {

                if (engine.accelerometerSensor != NULL) {
                    ASensorEvent event;
                    while (ASensorEventQueue_getEvents(engine.sensorEventQueue,
                                                       &event, 1) > 0) {

                       /* LOGI("accelerometer: x=%f y=%f z=%f",
                             event.acceleration.x, event.acceleration.y,
                             event.acceleration.z);*/
                        engine.accel[0] = event.acceleration.x;
                        engine.accel[1] = event.acceleration.y;
                        engine.accel[2] = event.acceleration.z;

                        calculateAccMagOrientation(&engine);

                    }
                }
                if(engine.gyroSensor != NULL){
                    ASensorEvent event;

                    while(ASensorEventQueue_getEvents(engine.sensorEventQueueGyro,
                                                        &event,1) > 0){

                        gyroFunction(&engine,event);
                        calculateFusedOrientation(&engine);
                        // GyroOrientation buradan sonra hazır.
                        //üstteki iki fonksiyon sensor verisi ile çalışıp sonucu döndürür.

                        LOGI("gyro: x=%f y=%f z=%f",
                             engine.gyroOrientation[0]* 180/M_PI,
                             engine.gyroOrientation[1]* 180/M_PI,
                             engine.gyroOrientation[2]* 180/M_PI);

                      /*  LOGI("gyro: x=%f y=%f z=%f",
                             event.vector.x, event.vector.y,
                             event.vector.z);*/
                    }
                }
                if(engine.gyroSensor != NULL){
                    ASensorEvent event;

                    while(ASensorEventQueue_getEvents(engine.sensorEventQueueMag,
                                                      &event,1) > 0){

                        engine.magnet[0] = event.magnetic.x;
                        engine.magnet[1] = event.magnetic.y;
                        engine.magnet[2] = event.magnetic.z;

                    }
                }
            }

            // Check if we are exiting.
            if (state->destroyRequested != 0) {
                engine_term_display(&engine);
                return;
            }
        }

        if (engine.animating) {
            // Done with events; draw next animation frame.
            engine.state.angle += .01f;
            if (engine.state.angle > 1) {
                engine.state.angle = 0;
            }

            // Drawing is throttled to the screen update rate, so there
            // is no need to do timing here.
            engine_draw_frame(&engine);
        }
    }
}

//////!!!! BURADAN SONRASI GYRO JITTER EFEKTI DUZELTMEKE ICIN GEREKLI FONKLAR ICIN ///////////

// This function is borrowed from the Android reference
// at http://developer.android.com/reference/android/hardware/SensorEvent.html#values
// It calculates a rotation vector from the gyroscope angular speed values.

void getRotationVectorFromGyro(float gyroValues[],
                               float deltaRotationVector[],
                               float timeFactor) {
        float normValues[3];

        // Calculate the angular speed of the sample
        float omegaMagnitude = sqrtf(gyroValues[0] * gyroValues[0] +
                                     gyroValues[1] * gyroValues[1] +
                                     gyroValues[2] * gyroValues[2]);
        // Normalize the rotation vector if it's big enough to get the axis
        if(omegaMagnitude > EPSILON){
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }
        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = sinf(thetaOverTwo);
        float cosThetaOverTwo = cosf(thetaOverTwo);

        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;

}

void gyroFunction(struct engine* engine,ASensorEvent event) {
    // don't start until first accelerometer/magnetometer orientation has been acquired
/*
    if(!accMagOrienttationInit)
        return;
*/
    // initialisation of the gyroscope based rotation matrix
    if(initState) {
            float initMatrix[9];
            getRotationMatrixFromOrientation(accMagOrientation,initMatrix);
            float test[3];

            sensorManager_getOrientation(initMatrix,9, test);

            matrixMultiplication(engine->gyroMatrix, initMatrix,engine->gyroMatrix);

//            LOGI("initgyroMatrix: d0=%f d1=%f d2=%f d3=%f d4=%f d5=%f d6=%f d7=%f d8=%f",
//             engine->gyroMatrix[0],engine->gyroMatrix[1],engine->gyroMatrix[2],
//             engine->gyroMatrix[3],engine->gyroMatrix[4],engine->gyroMatrix[5],
//             engine->gyroMatrix[6],engine->gyroMatrix[7],engine->gyroMatrix[8]);

            initState = false;
        }

            // copy the new gyro values into the gyro array
            // convert the raw gyro data into a rotation vector
            float deltaVector[4];
            if(timestamp != 0) {
                const float dT = (event.timestamp - timestamp) * NS2S;

                engine->gyro[0] = event.vector.x;
                engine->gyro[1] = event.vector.y;
                engine->gyro[2] = event.vector.z;

                getRotationVectorFromGyro(engine->gyro, deltaVector, dT / 2.0f);
            }

            // measurement done, save current time for next interval
            timestamp = event.timestamp;

            // convert rotation vector into rotation matrix
            float deltaMatrix[9];

            sensorManager_getRotationMatrixFromVector(deltaMatrix,9,deltaVector,4);

//          LOGI("deltaMatrix: d0=%f d1=%f d2=%f d3=%f d4=%f d5=%f d6=%f d7=%f d8=%f",
//          deltaMatrix[0],deltaMatrix[1],deltaMatrix[2],deltaMatrix[3],deltaMatrix[4],
//          deltaMatrix[5],deltaMatrix[6],deltaMatrix[7],deltaMatrix[8]);

            // apply the new rotation interval on the gyroscope based rotation matrix
            matrixMultiplication(engine->gyroMatrix, deltaMatrix,engine->gyroMatrix);
           /* LOGI("gyroMatrix: d0=%f d1=%f d2=%f d3=%f d4=%f d5=%f d6=%f d7=%f d8=%f",
                 engine->gyroMatrix[0],engine->gyroMatrix[1],engine->gyroMatrix[2],
                 engine->gyroMatrix[3],engine->gyroMatrix[4],engine->gyroMatrix[5],
                 engine->gyroMatrix[6],engine->gyroMatrix[7],engine->gyroMatrix[8]);*/

            // get the gyroscope based orientation from the rotation matrix
            //SensorManager.getOrientation(engine->gyroMatrix, gyroOrientation);
            sensorManager_getOrientation(engine->gyroMatrix,9,engine->gyroOrientation);
        }


void getRotationMatrixFromOrientation(float o[],float resultMat[]) {
        float xM[9];
        float yM[9];
        float zM[9];

        float sinX = sinf(o[1]);
        float cosX = cosf(o[1]);
        float sinY = sinf(o[2]);
        float cosY = cosf(o[2]);
        float sinZ = sinf(o[0]);
        float cosZ = cosf(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float resultMatrix[9];
        matrixMultiplication(xM, yM,resultMatrix);
        matrixMultiplication(zM, resultMatrix,resultMatrix);
        memcpy(resultMat,resultMatrix, sizeof(resultMatrix));

    }

/*
 *  matrixMultiplication
 *
 *  it does multiplication for 3x3 matricies
 *
 *  INPUT:
 *   A: 3x3 matrix for first operand
 *   B: 3x3 matrix for second operand
 *
 *   OUTPUT:
 *   res: 3x3 matrix result of mulptiplication
 * */
void matrixMultiplication(float A[], float B[], float res[]) {
        float result[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        memcpy(res,result,sizeof(result));

    }

//accelerometer
/*
 * calculateAccMagOrientation
 *
 *  calculates orientation from rotation matrix that calculated with Accelerometer and magnatic field vectors
 *
 * INPUT:
 *  engine: strucut that contains accel and mag. field vectors
 *
 * */
void calculateAccMagOrientation(struct engine* engine) {

    if(sensorManager_getRotationMatrix(engine->rotationMatrix,9, NULL ,0, engine->accel,engine->magnet)) {
            sensorManager_getOrientation(engine->rotationMatrix,9,accMagOrientation);
            if(!accMagOrienttationInit)
                accMagOrienttationInit=true;
	    }

}

/*
 * calculateFusedOrientation
 *
 *  makes calculate for fix jitter effect.
 *
 * INPUT:
 *  engine: struct that contains gyroMatrix, gyroOrientation
 *
 * */

void calculateFusedOrientation(struct engine* engine){
    float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;

    /*
     * Fix for 179° <--> -179° transition problem:
     * Check whether one of the two orientation angles (gyro or accMag) is negative while the other one is positive.
     * If so, add 360° (2 * math.PI) to the negative value, perform the sensor fusion, and remove the 360° from the result
     * if it is greater than 180°. This stabilizes the output in positive-to-negative-transition cases.
     */

    // azimuth
    if (engine->gyroOrientation[0] < -0.5 * M_PI && accMagOrientation[0] > 0.0) {
        fusedOrientation[0] = (float) (FILTER_COEFFICIENT * (engine->gyroOrientation[0] + 2.0 * M_PI) + oneMinusCoeff * accMagOrientation[0]);
        fusedOrientation[0] -= (fusedOrientation[0] > M_PI) ? 2.0 * M_PI : 0;
    }
    else if (accMagOrientation[0] < -0.5 * M_PI && engine->gyroOrientation[0] > 0.0) {
        fusedOrientation[0] = (float) (FILTER_COEFFICIENT * engine->gyroOrientation[0] + oneMinusCoeff * (accMagOrientation[0] + 2.0 * M_PI));
        fusedOrientation[0] -= (fusedOrientation[0] > M_PI)? 2.0 * M_PI : 0;
    }
    else {
        fusedOrientation[0] = FILTER_COEFFICIENT * engine->gyroOrientation[0] + oneMinusCoeff * accMagOrientation[0];
    }

    // pitch
    if (engine->gyroOrientation[1] < -0.5 * M_PI && accMagOrientation[1] > 0.0) {
        fusedOrientation[1] = (float) (FILTER_COEFFICIENT * (engine->gyroOrientation[1] + 2.0 * M_PI) + oneMinusCoeff * accMagOrientation[1]);
        fusedOrientation[1] -= (fusedOrientation[1] > M_PI) ? 2.0 * M_PI : 0;
    }
    else if (accMagOrientation[1] < -0.5 * M_PI && engine->gyroOrientation[1] > 0.0) {
        fusedOrientation[1] = (float) (FILTER_COEFFICIENT * engine->gyroOrientation[1] + oneMinusCoeff * (accMagOrientation[1] + 2.0 * M_PI));
        fusedOrientation[1] -= (fusedOrientation[1] > M_PI)? 2.0 * M_PI : 0;
    }
    else {
        fusedOrientation[1] = FILTER_COEFFICIENT * engine->gyroOrientation[1] + oneMinusCoeff * accMagOrientation[1];
    }

    // roll
    if (engine->gyroOrientation[2] < -0.5 * M_PI && accMagOrientation[2] > 0.0) {
        fusedOrientation[2] = (float) (FILTER_COEFFICIENT * (engine->gyroOrientation[2] + 2.0 * M_PI) + oneMinusCoeff * accMagOrientation[2]);
        fusedOrientation[2] -= (fusedOrientation[2] > M_PI) ? 2.0 * M_PI : 0;
    }
    else if (accMagOrientation[2] < -0.5 * M_PI && engine->gyroOrientation[2] > 0.0) {
        fusedOrientation[2] = (float) (FILTER_COEFFICIENT * engine->gyroOrientation[2] + oneMinusCoeff * (accMagOrientation[2] + 2.0 * M_PI));
        fusedOrientation[2] -= (fusedOrientation[2] > M_PI)? 2.0 * M_PI : 0;
    }
    else {
        fusedOrientation[2] = FILTER_COEFFICIENT * engine->gyroOrientation[2] + oneMinusCoeff * accMagOrientation[2];
    }

    // overwrite gyro matrix and orientation with fused orientation
    // to comensate gyro drift
    //gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
    getRotationMatrixFromOrientation(fusedOrientation,engine->gyroMatrix);
    //System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);
    memcpy(engine->gyroOrientation,fusedOrientation,sizeof(fusedOrientation));

}



// Sensor Manager Functions that not avaible in sensor.h in NDK

// SensorManager getorientation func

void sensorManager_getOrientation(float R[],int sizeR,float values[]){

    /*
        * 4x4 (length=16) case:
        *   /  R[ 0]   R[ 1]   R[ 2]   0  \
        *   |  R[ 4]   R[ 5]   R[ 6]   0  |
        *   |  R[ 8]   R[ 9]   R[10]   0  |
        *   \      0       0       0   1  /
        *
        * 3x3 (length=9) case:
        *   /  R[ 0]   R[ 1]   R[ 2]  \
        *   |  R[ 3]   R[ 4]   R[ 5]  |
        *   \  R[ 6]   R[ 7]   R[ 8]  /
        *
        */
        if (sizeR == 9) {
            values[0] = atan2f(R[1], R[4]);
            values[1] = asinf(-R[7]);
            values[2] = atan2f(-R[6], R[8]);
        } else {
            values[0] = atan2f(R[1], R[5]);
            values[1] = asinf(-R[9]);
            values[2] = atan2f(-R[8], R[10]);
        }
}

void sensorManager_getRotationMatrixFromVector(float R[],int sizeR, float rotationVector[],int sizeRV) {
            float q0;
            float q1 = rotationVector[0];
            float q2 = rotationVector[1];
            float q3 = rotationVector[2];

         if (sizeRV == 4) {
                  q0 = rotationVector[3];
         } else {
               q0 = 1 - q1*q1 - q2*q2 - q3*q3;
                q0 = (q0 > 0) ? sqrtf(q0) : 0;
         }

        float sq_q1 = 2 * q1 * q1;
        float sq_q2 = 2 * q2 * q2;
        float sq_q3 = 2 * q3 * q3;
        float q1_q2 = 2 * q1 * q2;
        float q3_q0 = 2 * q3 * q0;
        float q1_q3 = 2 * q1 * q3;
        float q2_q0 = 2 * q2 * q0;
        float q2_q3 = 2 * q2 * q3;
        float q1_q0 = 2 * q1 * q0;

        if(sizeR == 9) {
            R[0] = 1 - sq_q2 - sq_q3;
            R[1] = q1_q2 - q3_q0;
            R[2] = q1_q3 + q2_q0;

            R[3] = q1_q2 + q3_q0;
            R[4] = 1 - sq_q1 - sq_q3;
            R[5] = q2_q3 - q1_q0;

            R[6] = q1_q3 - q2_q0;
            R[7] = q2_q3 + q1_q0;
            R[8] = 1 - sq_q1 - sq_q2;
        } else if (sizeR == 16) {
            R[0] = 1 - sq_q2 - sq_q3;
            R[1] = q1_q2 - q3_q0;
            R[2] = q1_q3 + q2_q0;
            R[3] = 0.0f;

            R[4] = q1_q2 + q3_q0;
            R[5] = 1 - sq_q1 - sq_q3;
            R[6] = q2_q3 - q1_q0;
            R[7] = 0.0f;

            R[8] = q1_q3 - q2_q0;
            R[9] = q2_q3 + q1_q0;
            R[10] = 1 - sq_q1 - sq_q2;
            R[11] = 0.0f;

            R[12] = R[13] = R[14] = 0.0f;
            R[15] = 1.0f;
        }
}

// Sensor Manager getRotationMatrix func

bool sensorManager_getRotationMatrix(float R[],int sizeR, float I[],int sizeI,
                                        float gravity[], float geomagnetic[]) {

    float Ax = gravity[0];
    float Ay = gravity[1];
    float Az = gravity[2];
    const float Ex = geomagnetic[0];
    const float Ey = geomagnetic[1];
    const float Ez = geomagnetic[2];
    float Hx = Ey*Az - Ez*Ay;
    float Hy = Ez*Ax - Ex*Az;
    float Hz = Ex*Ay - Ey*Ax;
    const float normH = sqrtf(Hx*Hx + Hy*Hy + Hz*Hz);
    if (normH < 0.1f) {
        // device is close to free fall (or in space?), or close to
        // magnetic north pole. Typical values are  > 100.
        return false;
    }
    const float invH = 1.0f / normH;
    Hx *= invH;
    Hy *= invH;
    Hz *= invH;
    const float invA = 1.0f / sqrtf(Ax*Ax + Ay*Ay + Az*Az);
    Ax *= invA;
    Ay *= invA;
    Az *= invA;
    const float Mx = Ay*Hz - Az*Hy;
    const float My = Az*Hx - Ax*Hz;
    const float Mz = Ax*Hy - Ay*Hx;
    if (R != NULL) {
        if (sizeR == 9) {
            R[0] = Hx;     R[1] = Hy;     R[2] = Hz;
            R[3] = Mx;     R[4] = My;     R[5] = Mz;
            R[6] = Ax;     R[7] = Ay;     R[8] = Az;
        } else if (sizeR == 16) {
            R[0]  = Hx;    R[1]  = Hy;    R[2]  = Hz;   R[3]  = 0;
            R[4]  = Mx;    R[5]  = My;    R[6]  = Mz;   R[7]  = 0;
            R[8]  = Ax;    R[9]  = Ay;    R[10] = Az;   R[11] = 0;
            R[12] = 0;     R[13] = 0;     R[14] = 0;    R[15] = 1;
        }
    }
    if (I != NULL) {
        // compute the inclination matrix by projecting the geomagnetic
        // vector onto the Z (gravity) and X (horizontal component
        // of geomagnetic vector) axes.
        const float invE = 1.0f / sqrtf(Ex*Ex + Ey*Ey + Ez*Ez);
        const float c = (Ex*Mx + Ey*My + Ez*Mz) * invE;
        const float s = (Ex*Ax + Ey*Ay + Ez*Az) * invE;
        if (sizeI == 9) {
            I[0] = 1;     I[1] = 0;     I[2] = 0;
            I[3] = 0;     I[4] = c;     I[5] = s;
            I[6] = 0;     I[7] =-s;     I[8] = c;
        } else if (sizeI == 16) {
            I[0] = 1;     I[1] = 0;     I[2] = 0;
            I[4] = 0;     I[5] = c;     I[6] = s;
            I[8] = 0;     I[9] =-s;     I[10]= c;
            I[3] = I[7] = I[11] = I[12] = I[13] = I[14] = 0;
            I[15] = 1;
        }
    }
    return true;
}

//END_INCLUDE(all)
