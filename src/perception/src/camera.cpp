#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

double img_size, img_center;
double left_m, right_m;
Point left_b, right_b;
bool left_detect = false, right_detect = false;
/*
double trap_bottom_width = 0.95;  //사다리꼴 아래쪽 가장자리 너비 계산을 위한 백분율
double trap_top_width = 0.95;     //사다리꼴 위쪽 가장자리 너비 계산을 위한 백분율
double trap_height = 0.45;         //사다리꼴 높이 계산을 위한 백분율
double trap_right_width = 0.5;
*/
double trap_bottom_width = 0.85;  //사다리꼴 아래쪽 가장자리 너비 계산을 위한 백분율
double trap_top_width = 0.07;     //사다리꼴 위쪽 가장자리 너비 계산을 위한 백분율
double trap_height = 0.4; 

Mat filter_colors(Mat img_frame){
    Mat output;
    UMat img_hsv, img_combine;
    UMat white_mask, white_image;
    UMat yellow_mask, yellow_image;
    img_frame.copyTo(output);

    //차선 색깔
    Scalar lower_white = Scalar(120, 120, 120); //흰색 차선 (RGB)
	Scalar upper_white = Scalar(180, 180, 180);
	Scalar lower_yellow = Scalar(10, 100, 100); //노란색 차선 (HSV)
	Scalar upper_yellow = Scalar(40, 255, 255);

    inRange(output, lower_white, upper_white, white_mask);
	bitwise_and(output, output, white_image, white_mask);

	cvtColor(output, img_hsv, COLOR_BGR2HSV);

	//노란색 필터링
	inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
	bitwise_and(output, output, yellow_image, yellow_mask);

	//두 영상을 합친다.
	addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, output);
	return output;
}

Mat limit_region(Mat img_edges) {
	/*
		관심 영역의 가장자리만 감지되도록 마스킹한다.
		관심 영역의 가장자리만 표시되는 이진 영상을 반환한다.
	*/
	int width = img_edges.cols;
	int height = img_edges.rows;

	Mat output;
	Mat mask = Mat::zeros(height, width, CV_8UC1);
	//관심 영역 정점 계산
	Point points[4]{
		Point((width * (trap_bottom_width)) / 2, height),
		Point((width * (trap_top_width)) / 2, height - height * trap_height),
		Point(width - (width * (1 - trap_top_width)) / 2, height - height * trap_height),
		Point(width - (width * (1 - trap_bottom_width)) / 2, height)
	};
	//정점으로 정의된 다각형 내부의 색상을 채워 그린다.
	fillConvexPoly(mask, points, 4, Scalar(255, 0, 0));

	//결과를 얻기 위해 edges 이미지와 mask를 곱한다.
	bitwise_and(img_edges, mask, output);
	return output;
}

vector<Vec4i> houghLines(Mat img_mask) {
	/*
		관심영역으로 마스킹 된 이미지에서 모든 선을 추출하여 반환
	*/
	vector<Vec4i> line;

	//확률적용 허프변환 직선 검출 함수 
	HoughLinesP(img_mask, line, 1,  CV_PI / 180, 40, 10, 20);
	return line;
}

vector<vector<Vec4i>> separateLine(Mat img_edges, vector<Vec4i> lines) {
	/*
		검출된 모든 허프변환 직선들을 기울기 별로 정렬한다.
		선을 기울기와 대략적인 위치에 따라 좌우로 분류한다.
	*/
	
	vector<vector<Vec4i>> output(2);
	Point ini, fini;
	vector<double> slopes;
	vector<Vec4i> selected_lines, left_lines, right_lines;
	double slope_thresh = 0.4;

	//검출된 직선들의 기울기를 계산
	for (int i = 0; i < lines.size(); i++) {
		Vec4i line = lines[i];
		ini = Point(line[0], line[1]);
		fini = Point(line[2], line[3]);

		double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) 
			/ (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

		//기울기가 너무 수평인 선은 제외
		if (abs(slope) > slope_thresh) {
			slopes.push_back(slope);
			selected_lines.push_back(line);
		}
	}

	//선들을 좌우 선으로 분류
	img_center = static_cast<double>((img_edges.cols / 2));
	for (int i = 0; i < selected_lines.size(); i++) {
		ini = Point(selected_lines[i][0], selected_lines[i][1]);
		fini = Point(selected_lines[i][2], selected_lines[i][3]);

		if (slopes[i] > 0 && fini.x > img_center && ini.x > img_center) {
			right_lines.push_back(selected_lines[i]);
			right_detect = true;
		}
		else if (slopes[i] < 0 && fini.x < img_center && ini.x < img_center) {
			left_lines.push_back(selected_lines[i]);
			left_detect = true;
		}
	}

	output[0] = right_lines;
	output[1] = left_lines;
	return output;
}

vector<Point> regression(vector<vector<Vec4i>> separatedLines, Mat img_input) {
	/*
		선형 회귀를 통해 좌우 차선 각각의 가장 적합한 선을 찾는다.
	*/
	vector<Point> output(4);
	Point ini, fini;
	Point ini2, fini2;
	Vec4d left_line, right_line;
	vector<Point> left_pts, right_pts;

	if (right_detect) {
		for (auto i : separatedLines[0]) {
			ini = Point(i[0], i[1]);
			fini = Point(i[2], i[3]);

			right_pts.push_back(ini);
			right_pts.push_back(fini);
		}

		if (right_pts.size() > 0) {
			//주어진 contour에 최적화된 직선 추출
			fitLine(right_pts, right_line, DIST_L2, 0, 0.01, 0.01);
			
			right_m = right_line[1] / right_line[0];  //기울기
			right_b = Point(right_line[2], right_line[3]);
		}
	}
	
	if (left_detect) {
		for (auto j : separatedLines[1]) {
			ini2 = Point(j[0], j[1]);
			fini2 = Point(j[2], j[3]);

			left_pts.push_back(ini2);
			left_pts.push_back(fini2);
		}

		if (left_pts.size() > 0) {
			//주어진 contour에 최적화된 직선 추출
			fitLine(left_pts, left_line, DIST_L2, 0, 0.01, 0.01);
			
			left_m = left_line[1] / left_line[0];  //기울기
			left_b = Point(left_line[2], left_line[3]); 
		}
	}

	//좌우 선 각각의 두 점을 계산한다.
	//y = m*x + b  --> x = (y-b) / m
	int ini_y = img_input.rows;
	int fin_y = 470;

	double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
	double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

	double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
	double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

	output[0] = Point(right_ini_x, ini_y);
	output[1] = Point(right_fin_x, fin_y);
	output[2] = Point(left_ini_x, ini_y);
	output[3] = Point(left_fin_x, fin_y);

	return output;
}

Mat drawLine(Mat img_input, vector<Point> lane) {
	/*
		좌우 차선을 경계로 하는 내부 다각형을 투명하게 색을 채운다.
		좌우 차선을 영상에 선으로 그린다.
		예측 진행 방향 텍스트를 영상에 출력한다.
	*/
	
	vector<Point> poly_points;
	Mat output;

	img_input.copyTo(output);
	poly_points.push_back(lane[2]);
	poly_points.push_back(lane[0]);
	poly_points.push_back(lane[1]);
	poly_points.push_back(lane[3]);

	fillConvexPoly(output, poly_points, Scalar(0,230, 30), LINE_AA, 0);  //다각형 색 채우기
	addWeighted(output, 0.3, img_input, 0.7, 0, img_input);  //영상 합하기

	//좌우 차선 선 그리기
	line(img_input, lane[0], lane[1], Scalar(0, 255, 255), 5, LINE_AA);
	line(img_input, lane[2], lane[3], Scalar(0, 255, 255), 5, LINE_AA);

	return img_input;
}

void image_cb(const sensor_msgs::ImageConstPtr& msg){

    vector<Vec4i> lines;
    vector<vector<Vec4i> > separated_lines;
	vector<Point> lane;
    String dir;
    Mat img_result;

    Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    // imshow("img", img);

    Mat img_color;
    img_color = filter_colors(img);
    // imshow("filter_color", img_color);

    Mat img_gray;
    cvtColor(img_color,img_gray,cv::COLOR_BGR2GRAY);
    // imshow("grayscale", img_gray);

    Mat img_edge;
    Canny(img_gray,img_edge, 50, 150);
    // imshow("img_edge", img_edge);

    Mat img_roi;
    img_roi = limit_region(img_edge);
    imshow("img_roi", img_roi);
    /*
    cvtColor(img,img_gray,cv::COLOR_BGR2GRAY);
    imshow("test", img_gray);
    Mat img_filtering;
    vector<cv::Point2f> srcRestCoord;
    vector<cv::Point2f> dstRestCoord;
    */
    
    lines = houghLines(img_roi);

    if (lines.size() > 0) {
			//7. 추출한 직선성분으로 좌우 차선에 있을 가능성이 있는 직선들만 따로 뽑아서 좌우 각각 직선을 계산한다. 
			// 선형 회귀를 하여 가장 적합한 선을 찾는다.
			separated_lines = separateLine(img_roi, lines);
			lane = regression(separated_lines, img);

			img_result = drawLine(img, lane);
		}
    
    // int width = img.cols; //width : 800
    // int height = img.rows; //height : 600

    imshow("result", img);
    waitKey(3);
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"camera");
    ros::NodeHandle nh;
    namedWindow("test");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("carla/ego_vehicle/rgb_front/image", 1, image_cb);
    ros::spin();
    destroyWindow("test");
}