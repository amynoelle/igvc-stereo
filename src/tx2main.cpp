///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
// TODO publish image in ROS

/***************************************************************************************************
 ** This sample demonstrates how to grab images and depth map with the ZED SDK                    **
 ** and apply the result in a 3D view "point cloud style" with OpenGL /freeGLUT                   **
 ** Some of the functions of the ZED SDK are linked with a key press event		                  **
 ***************************************************************************************************/

// Standard includes
#include <stdio.h>
#include <string.h>

// OpenGL includes
#include <GL/glew.h>
#include <GL/freeglut.h> 
#include <GL/gl.h>
#include <GL/glut.h>

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
//#include "GLViewer.hpp"
//#include "SaveDepth.hpp"

//// Using std and sl namespaces
using namespace std;
using namespace sl;

//// Create ZED object (camera, callback, images)
sl::Camera zed;
sl::Mat point_cloud, left_image;
std::thread zed_callback;
bool quit;

//// Point Cloud visualizer
//GLViewer viewer;

//// Sample functions
void startZED();
void run();
void close();
void printHelp();

int main(int argc, char **argv) {


    // Setup configuration parameters for the ZED
    InitParameters initParameters;
    if (argc == 2) initParameters.svo_input_filename = argv[1];
    initParameters.camera_resolution = sl::RESOLUTION_HD720;
    initParameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE; //need quite a powerful graphic card in QUALITY
    initParameters.coordinate_units = sl::UNIT_METER; // set meter as the OpenGL world will be in meters
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed

    // Open the ZED
    ERROR_CODE err = zed.open(initParameters);
    if (err != SUCCESS) {
        //cout << errorCode2str(err) << endl;
        ROS_INFO("JETSON TK1 error code on zed open is: %s", errcode2str(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }
    ROS_INFO("JETSON TK1 initalized the ZED\n");

    // Print help in console
    printHelp();

    // Initialize point cloud viewer
    //viewer.init(zed.getResolution());

    //Start ZED 
    //startZED();
        while (!quit) {
            quit=false;
        try{
            //std::cout << " Here now 1" << std::endl;
            if (zed.grab() == SUCCESS) {
                //std::cout << " Here now 2" << std::endl;
                // Get depth as a displayable image (8bits) and display it with OpenCV
                zed.retrieveImage(left_image, sl::VIEW_LEFT); // For display purpose ONLY. To get real world depth values, use retrieveMeasure(mat, sl::MEASURE_DEPTH)
                //std::cout << " Here now 3" << std::endl;
                cv::imshow("Depth", cv::Mat(left_image.getHeight(), left_image.getWidth(), CV_8UC4, left_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
                //std::cout << " Here now 4" << std::endl;
                // Get XYZRGBA point cloud on GPU and send it to OpenGL
                zed.retrieveMeasure(point_cloud, sl::MEASURE_XYZRGBA, sl::MEM_GPU); // Actual metrics values
                //viewer.updatePointCloud(point_cloud);
                //std::cout << " Here now 5" << std::endl;
                // Handles keyboard event
                cv::waitKey(15);
                //processKeyEvent(zed, key);
            }
            else sl::sleep_ms(1);
        }
        catch (const std::exception& e){
                std::cout << "Exception caught grabbing from zed: "<< e.what() << std::endl;
        }
    } 

    /// Set GLUT callback
    //glutCloseFunc(close);
    //glutMainLoop();
    return 0;
}

/**
 *  This function frees and close the ZED, its callback(thread) and the viewer
 **/
void close() {
    quit = true;

    // Stop callback
    zed_callback.join();

    // Exit point cloud viewer
    //viewer.exit();

    // free buffer and close ZED
    left_image.free(MEM_CPU);
    point_cloud.free(MEM_GPU);
    zed.close();
}

/**
 * This function displays help in console
 **/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}
