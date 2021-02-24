#include "ardrone/ardrone.h"

#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <cstdio>
#include <vector>
#include <opencv2/aruco.hpp>


// Parameter for calibration pattern
#define PAT_ROWS   (6)                  // Rows of pattern
#define PAT_COLS   (9)                 // Columns of pattern
#define CHESS_SIZE (22.0)               // Size of a pattern [mm]
using namespace std;
using namespace cv;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // AR.Drone class
    ARDrone ardrone;



    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }
	//////////////////////////////////////

	//VideoCapture cap(1);
	Mat frame = ardrone.getImage();
	//Mat frame = cap.read();
	//cap >> frame;
	// Open XML file
	cout << frame.size() << endl;
	string filename("calibration.xml");

	FileStorage fs(filename, FileStorage::READ);
	fs.open(filename, cv::FileStorage::READ);
	// Load camera parameters
	cv::Mat cameraMatrix, distCoeffs;
	fs["intrinsic"] >> cameraMatrix;
	fs["distortion"] >> distCoeffs;
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	//lab05
	cv::Mat mapx, mapy;
	cout << frame.size() << endl;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
	
	while (1) {
		// Key input
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		// Get an image
		//cv::Mat image_raw = GP.getImage();
		//Mat image_raw = cap.read();
		Mat image_raw;
		//cap >> image_raw;
		image_raw = ardrone.getImage();
		// Undistort
		cv::Mat image;
		cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);

		// Display the image
		cv::imshow("camera", image);
		
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		std::vector<cv::Vec3d> rvecs, tvecs;
		if (ids.size() > 0) {
			aruco::drawDetectedMarkers(image, corners, ids);
			
			cv::aruco::estimatePoseSingleMarkers(corners, 0.075, cameraMatrix, distCoeffs, rvecs, tvecs);
			std::cout << tvecs[0] << endl;
			for (int i = 0; i<ids.size(); i++)
				cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.01);
			
		}
		cv::imshow("camera", image);
		
		//cv::aruco::estimatePoseSingleMarkers(corners, 0.01, cameraMatrix, distCoeffs, rvecs, tvecs);
		//cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
	}


	//////////////////////////////////////

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;

    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
        ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}