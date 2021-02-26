
#include "ardrone/ardrone.h"
#include <time.h>
#include "pid.hpp"
#include <cstring>
#include <opencv2/aruco.hpp>
#include <map>
#define searching_vr -0.25

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : I don't know when I would see these words in my program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

class xyzr { //four double members,x,y,z,r
public:
	double x;
	double y;
	double z;
	double r;
	xyzr() {
		x = 0.0;
		y = 0.0;
		z = 0.0;
		r = 0.0;
	}
	xyzr(double x_, double y_, double z_, double r_) {
		x = x_;
		y = y_;
		z = z_;
		r = r_;
	}
	xyzr& operator=(const xyzr& xyzr_) {
		x = xyzr_.x;
		y = xyzr_.y;
		z = xyzr_.z;
		r = xyzr_.r;
		return *this;
	}
};

void show_info(void);
void init_calibration(void);
xyzr basic_op(int);
void fixed_speed_moving(xyzr, unsigned int);
map<int, xyzr> detect_marker(cv::Mat &);
void fix_vr(double &, double);
void rotate_search(double, int);
xyzr estimate_v(xyzr, xyzr);
void trace_marker(int, xyzr, int);
void run(void);
void dodge(void);
void landing_on_marker(int);
bool detectAndDraw(Mat&, double);


ARDrone ardrone;
cv::Mat mapx, mapy;
bool working = false;
CascadeClassifier cascade;
cv::Mat cameraMatrix, distCoeffs;
int mode = 0;//camera mode

void show_info(void) {//show instruction of ardrone
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
	std::cout << "*    'Z'     -- working on/off        *" << std::endl;
	std::cout << "*    'R'     -- clear screen          *" << std::endl;
	std::cout << "*    'C'     -- Change camera         *" << std::endl;
	std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "***************************************" << std::endl;

	return;

}

void init_calibration(void) {//initialize mapx and mapy for calibration
	Mat frame = ardrone.getImage();
	string filename("calibration.xml");
	FileStorage fs(filename, FileStorage::READ);
	fs.open(filename, cv::FileStorage::READ);
	fs["intrinsic"] >> cameraMatrix;
	fs["distortion"] >> distCoeffs;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
	return;
}



xyzr basic_op(int key) { //read key to do basic operations, return vx,vy,vz,vr
						 // Take off / Landing 
	xyzr ret;
	if (key == ' ') {
		if (ardrone.onGround()) {
			ardrone.takeoff();
			fixed_speed_moving(xyzr(), 3000);
		}
		else
			ardrone.landing();
	}

	// Move
	if (key == 'i' || key == CV_VK_UP)    ret.x = 1.0;
	if (key == 'k' || key == CV_VK_DOWN)  ret.x = -1.0;
	if (key == 'u' || key == CV_VK_LEFT)  ret.r = 1.0;
	if (key == 'o' || key == CV_VK_RIGHT) ret.r = -1.0;
	if (key == 'j') ret.y = 1.0;
	if (key == 'l') ret.y = -1.0;
	if (key == 'q' || key == '+') ret.z = 1.0;
	if (key == 'a' || key == '-') ret.z = -1.0;

	// Change camera
	if (key == 'c') ardrone.setCamera(++mode % 4);
	if (key == 'z')
		working = !working;
	if (key == 'r') {
		system("CLS");
		show_info();
	}
	return ret;
}

void fixed_speed_moving(xyzr v, unsigned int t) {//ardrone moves with constant speed_v for time_t(ms)
	clock_t end = clock() + t * CLOCKS_PER_SEC / 1000;
	while (clock()<end) {
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		cv::Mat image_raw = ardrone.getImage();
		cv::Mat image;
		cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);		// calibration

		xyzr v;

		if (key != -1) {
			v = basic_op(key);
		}
		cv::imshow("camera", image);
		ardrone.move3D(v.x, v.y, v.z, v.r);
	}
	return;
}

map<int, xyzr> detect_marker(cv::Mat &image) {// detect markers on the image, then return their position in map form. 
	map<int, xyzr> marker_pos;
	std::vector<cv::Vec3d> rvecs, tvecs;

	static cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	//find and draw markers
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::detectMarkers(image, dictionary, corners, ids);
	aruco::drawDetectedMarkers(image, corners, ids);
	cv::aruco::estimatePoseSingleMarkers(corners, 0.094, cameraMatrix, distCoeffs, rvecs, tvecs);

	for (int i = 0; i < ids.size(); i++) {
		xyzr n(tvecs[i][2], tvecs[i][0], tvecs[i][1], rvecs[i][2]);
		if (rvecs[i][0] * n.r < 0)
			n.r = -abs(n.r);//different sign
		else
			n.r = abs(n.r);//same sign
		marker_pos[ids.at(i)] = n;
	}
	return marker_pos;
}

/**
* Fix the value of vr depending on previous posr (if abs(vr) > 0.3).
* All subsequent calls to this function will result in a positive/negative transformation of vr depending on previous results.
* @param double vr
*   The velocity in the r direction.
* @param double posr
*   Current position in the r direction.
* @return
*/
void fix_vr(double &vr, double posr) {
	static const int n = 4; //vr direction depends on last n vr and posr.
	static double last_vr[n] = { 0 };
	static double last_posr[n] = { 0 };
	static bool vr_must_pos = true;

	bool moving = true;
	for (int i = 0; i<n; i++) {
		if (abs(last_vr[i])<0.01) {
			moving = false;
			break;
		}
	}

	if (moving) {
		if (abs(last_posr[0])<abs(posr)) {
			vr_must_pos = !vr_must_pos;
			for (int i = 0; i<n; i++)
				last_vr[i] = 0.0;
		}
	}
	if (abs(vr)>0.3) {
		if (vr_must_pos)
			vr = abs(vr);
		else
			vr = -abs(vr);
	}
	else {
		if (vr_must_pos && vr<0)
			vr = 0;
		if (!vr_must_pos && vr>0)
			vr = 0;
	}

	for (int i = 0; i<n - 1; i++) {
		last_posr[i] = last_posr[i + 1];
		last_vr[i] = last_vr[i + 1];
	}
	last_posr[n - 1] = posr;
	last_vr[n - 1] = vr;
}

void rotate_search(double vr, int marker_id) {//rotate 1 sec , stop 1 sec
	cout << "rotating\n";
	int found_counter = 0;
	const int cycletime = 1000;
	bool rotate=false;
	for (;;) {
		clock_t cycle_end = clock() + cycletime * CLOCKS_PER_SEC / 1000;
		while (clock()<cycle_end) {
			cv::Mat image_raw = ardrone.getImage();
			cv::Mat image;
			cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);		// calibration
			int key = cv::waitKey(33);
			if (key == 0x1b) return;
			xyzr v;
			if (key != -1) {
				v = basic_op(key);
			}
			else {
				map<int, xyzr> marker_pos = detect_marker(image);
				if (marker_pos.find(marker_id) != marker_pos.end()) {
					found_counter++;
					ardrone.move3D(0.0, 0.0, 0.0, 0.0);
					if (found_counter > 5)
						return;
				}
				else
					ardrone.move3D(0.0, 0.0, 0.0, rotate?vr:0.0);
			}
			cv::imshow("camera", image);
		}
		rotate = !rotate;
	}
}

xyzr estimate_v(xyzr pos, xyzr tar) {	// Use current position and target position of the marker to calculate v
	static xyzr last_v;
	xyzr err, v;
	err.x = pos.x - tar.x;
	err.y = -(pos.y - tar.y);
	err.r = pos.r - tar.r;

	if (abs(err.x) < 0.1)
		err.x = 0;
	if (abs(err.y) < 0.1)
		err.y = 0;
	if (abs(err.r) < 0.2)
		err.r = 0;

	if (err.x>0.5)
		v.x = 0.5;
	else
		v.x = err.x;

	v.r = err.r*0.5;

	if (abs(v.r)<0.01)
		v.y = err.y*0.25;

	if (last_v.x*v.x<0)
		v.x *= 2;
	if (last_v.y*v.y<0)
		v.y *= 2;
	if (last_v.r*v.r<0)
		v.r *= 2;

	last_v = v;
	return v;
}


// Trace marker with specified id, until it is at the target position of image from camera
// If marker with next_id is found, jump out the loop 
void trace_marker(int id, xyzr tar, int next_id) {

	tar.r = 0.0;//r is difficult to fix, so...
	int found_counter = 0;
	for (;;) {
		cout << "tracing " << id << "\n";
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		cv::Mat image_raw = ardrone.getImage();
		cv::Mat image;
		cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);		// calibration

		int face_detect_counter = 18;

		xyzr v;

		if (key != -1) {
			v = basic_op(key);
		}
		else if (working) {
			map<int, xyzr> marker_pos = detect_marker(image);
			// To check if the marker has been recognized, use: marker_pos.find(id) != marker_pos.end().
			static xyzr last_pos;
			static int lost_counter = 0;//counter for loops that marker is lost
			static int vr_search_counter = 0;//counter for times searching marker with vr
			/*
			if (face_detect_counter == 18) {
				face_detect_counter = 0;
				if (detectAndDraw(image, 1)) {
					dodge();
					return;
				}
			}
			else {
				face_detect_counter++;
			}
			*/

			if (marker_pos.find(next_id) != marker_pos.end()) {
				found_counter++;
				if (found_counter>5)
					return;
			}
			else if (marker_pos.find(id) != marker_pos.end()) {//target marker recognized
				v = estimate_v(marker_pos[id], tar);
				cout << marker_pos[id].x << "  " << marker_pos[id].y << "  " << marker_pos[id].r << "\n";
				cout << v.x << "  " << v.y << "  " << v.r << "\n\n";
				fix_vr(v.r, marker_pos[id].r);

				last_pos = marker_pos[id];
				if (lost_counter>20) {
					if (vr_search_counter == 4)
						vr_search_counter = 0;
					else
						vr_search_counter++;
				}
				lost_counter = 0;


				if (abs(v.x) <= 0.01 && abs(v.y) <= 0.01 && abs(v.z) <= 0.01 && abs(v.r) <= 0.01) {
					found_counter++;
					if (found_counter>5)
						return;
				}
			}
			else {// Can't see target marker.

				lost_counter++;
				if (lost_counter>20) {
					if (last_pos.x < 0.5) {
						v.x = -0.25;
					}
					else {
						if (vr_search_counter == 4) {
							if (last_pos.y > 0) {
								v.y = -0.3;
							}
							else {
								v.y = 0.3;
							}
						}
						else {
							if (last_pos.y > 0)
								v.r = -0.25;
							else
								v.r = 0.25;
						}
					}
				}
				if (lost_counter>40) {
					if (last_pos.y > 0)
						v.r = -0.25;
					else
						v.r = 0.25;
				}
				if (lost_counter>1000) {
					cout << "I am lost,sorry.\n";
					return;
				}

			}
		}
		cout << v.x << "  " << v.y << "  " << v.r << "\n";
		ardrone.move3D(v.x, v.y, v.z, v.r);

		// Display the image
		cv::imshow("camera", image);
	}
}

void run(void) {
	while (!working) {
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		cv::Mat image_raw = ardrone.getImage();
		cv::Mat image;
		cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);		// calibration

		xyzr v;

		if (key != -1) {
			v = basic_op(key);
		}
		ardrone.move3D(v.x,v.y,v.z, v.r);
		cv::imshow("camera", image_raw);
	};
	rotate_search(-0.5, 11);
	trace_marker(11, xyzr(1.0, -0.6, 0.0, 0.0), 12);/*
	rotate_search(-0.25, 12);
	trace_marker(12, xyzr(1.0, -0.6, 0.0, 0.0), 13);
	rotate_search(0.25, 13);
	trace_marker(13, xyzr(1.0, -0.6, 0.0, 0.0), 1);
	rotate_search(-0.25, 1);
	trace_marker(1, xyzr(1.0, 0.0, 0.0, 0.0), 3);
	rotate_search(-0.25, 3);
	trace_marker(3, xyzr(1.0, 0.0, 0.0, 0.0), 21);
	rotate_search(-0.25, 21);
	trace_marker(21, xyzr(1.0, -0.6, 0.0, 0.0), 22);
	rotate_search(-0.25, 22);
	trace_marker(22, xyzr(1.0, -0.6, 0.0, 0.0), 4);
	rotate_search(0.25, 4);
	while (!ardrone.onGround()) {
		trace_marker(4, xyzr(1.0, 0.0, 0.0, 0.0), -1);
		ardrone.setCamera(++mode % 4);
		landing_on_marker(5);
		ardrone.setCamera(++mode % 4);
	}*/
	ardrone.landing();
	return;
}

void dodge(void) {
	fixed_speed_moving(xyzr(0.0, 0.0, 0.5, 0.0), 1000);
	fixed_speed_moving(xyzr(1.0, 0.0, 0.0, 0.0), 1000);
	fixed_speed_moving(xyzr(0.0, 0.0, -0.5, 0.0), 1000);
	return;
}

void landing_on_marker(int marker_id) {
	int counter = 0;
	cv::Mat image_raw = ardrone.getImage();
	cv::Mat image;
	cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);		// calibration
	map<int, xyzr> marker_pos = detect_marker(image);
	if (marker_pos.find(marker_id) != marker_pos.end()) {
		ardrone.landing();
		return;
	}
	else {
		counter++;
		if (counter>20)
			return;
	}

}

bool detectAndDraw(Mat& img, double scale = 1)
{
	bool too_close = false;
	double t = 0;
	int count_faces = 0;
	vector<Rect> faces, faces2;
	const static Scalar colors[] =
	{
		Scalar(255,0,0),
		Scalar(255,128,0),
		Scalar(255,255,0),
		Scalar(0,255,0),
		Scalar(0,128,255),
		Scalar(0,255,255),
		Scalar(0,0,255),
		Scalar(255,0,255)
	};
	Mat gray, smallImg;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	double fx = 1 / scale;
	//resize(gray, smallImg, Size(), fx, fx, INTER_LINEAR_EXACT);
	smallImg = gray;
	equalizeHist(smallImg, smallImg);

	cascade.detectMultiScale(smallImg, faces,
		1.1, 3, 0
		//|CASCADE_FIND_BIGGEST_OBJECT
		//|CASCADE_DO_ROUGH_SEARCH
		//| CASCADE_SCALE_IMAGE
		| CV_HAAR_SCALE_IMAGE,
		Size(0, 20));
	flip(smallImg, smallImg, 1);
	cascade.detectMultiScale(smallImg, faces2,
		1.1, 3, 0
		//|CASCADE_FIND_BIGGEST_OBJECT
		//|CASCADE_DO_ROUGH_SEARCH
		| CASCADE_SCALE_IMAGE,
		Size(0, 20));
	for (vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); ++r)
	{
		faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
	}

	//draw
	vector<Point> face_appeared;
	for (size_t i = 0; i < faces.size(); i++)
	{
		Rect r = faces[i];
		Mat smallImgROI;
		Point center;
		Scalar color = colors[i % 8];
		int radius;
		double aspect_ratio = (double)r.width / r.height;
		if (0.75 < aspect_ratio && aspect_ratio < 1.3)
		{
			center.x = cvRound((r.x + r.width*0.5)*scale);
			center.y = cvRound((r.y + r.height*0.5)*scale);
			radius = cvRound((r.width + r.height)*0.25*scale);
			std::cout << "circle" << count_faces + 1 << " : " << center.x << " , ";
			std::cout << center.y << " , radius:" << radius << endl;
			if (radius > 45)
				too_close = true;

			bool face_appeared_b = false;
			for (size_t j = 0; j < face_appeared.size(); j++)
			{
				if (abs(face_appeared.at(j).x - center.x) < 10 && abs(face_appeared.at(j).y - center.y) < 10)
				{//
					face_appeared_b = true;
					break;
				}
			}
			if (!face_appeared_b)
			{
				circle(img, center, radius, color, 3, 8, 0);
				count_faces++;
				face_appeared.push_back(center);
			}
		}
		else
			rectangle(img, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)),
				cvPoint(cvRound((r.x + r.width - 1)*scale), cvRound((r.y + r.height - 1)*scale)),
				color, 3, 8, 0);

	}
	//cout << "count_faces : " << count_faces<<endl;
	//namedWindow("result", 0);
	//imshow("result", img);
	//waitKey(0);
	return too_close;
}

int main(int argc, char *argv[])
{
	// Initialize
	if (!ardrone.open()) {
		std::cout << "ardrone.open() failed" << std::endl;
		return -1;
	}

	const String face_cascade_name = "D:\\tools\\openCV\\opencv\\sources\\data\\haarcascades\\haarcascade_frontalface_alt.xml";
	if (!cascade.load(face_cascade_name)) {
		std::cout << "face_cascade.load(face_cascade_name) failed" << std::endl;
		return -1;
	}

	init_calibration();
	show_info();

	run();

	ardrone.close();

	return 0;
}
