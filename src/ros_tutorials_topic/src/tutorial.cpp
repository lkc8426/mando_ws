#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>


class tracking{

    ros::NodeHandle nh;

    ros::Publisher vehicle_control_pub;
    ros::Publisher waypoint_marker_pub;
    ros::Publisher waypoint_point_pub;
    ros::Subscriber odometry_sub;
    ros::Subscriber speedometer_sub;
    ros::Publisher car_pub;

    std::vector<std::vector<double>> double_vec_pointer;
    carla_msgs::CarlaEgoVehicleControl vehicle_control_cmd_msg;
    visualization_msgs::MarkerArray waypoint_path;
    geometry_msgs::Pose2D pose2d;
    visualization_msgs::Marker point_marker;
    visualization_msgs::MarkerArray car_path;
    visualization_msgs::Marker marker3;


    double distance_between_vehicle_point; //d
    double wheelbase; //L
    double steering; // theta

    double point_x;
    double point_y;
    double slope;
    double yaw_;
    double temp_x;
    double temp_y;
    double temp_distance;
    int count = 0;

    double Kp;
    double Ki;
    double Kd;
    double target_velocity;
    double current_velocity;
    double control_velocity;

    int index = 0;
    int error_count;
    double min = 10000;

public:
    tracking();

    void PublishControl();
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void PublishWayPoint();
    std::vector<std::string> split(std::string str,char delimeter);
    void WaypointVisualize();
    double to_double(std::string s);
    void WayPointCreate();
    void ControlCommand();
    void SelectPoint();
    void PidControl(double velocity);
    void SpeedCallback(const std_msgs::Float32::ConstPtr& msg);
    void Error();
    void CarWayVisualize();
};

tracking::tracking(){

    vehicle_control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd",100); // ROS Publisher 초기화
    odometry_sub = nh.subscribe("/carla/ego_vehicle/odometry",100, &tracking::OdometryCallback, this); // ROS 서브스크라이버 (Subscriber) 초기화, Odometry 토픽에 대한 콜백 함수를 등록
    speedometer_sub = nh.subscribe("/carla/ego_vehicle/speedometer",100, &tracking::SpeedCallback, this); // ROS 서브스크라이버 (Subscriber) 초기화, Speedometer 토픽에 대한 콜백 함수를 등록
    car_pub = nh.advertise<visualization_msgs::Marker>("car", 10); // ROS 퍼블리셔 (Publisher) 초기화, "car" 토픽에 대한 마커를 발행
}

void tracking::WayPointCreate(){
    // 파일에서 waypoint 좌표를 읽기 위한 ifstream 객체 생성
    std::ifstream in("/home/osh/mando_ws/src/ros_tutorials_topic/src/2ndway.txt");

    // 파일이 열리지 않았을 경우 에러 메시지 출력
    if(!in.is_open()){
        ROS_ERROR("No Such Files");
    }
    std::string s;

    // 파일에서 한 줄씩 읽어와 처리
    while(in){
        getline(in,s); // in이라는 문자열을 한줄씩 s에 저장한다. // 파일에서 한 줄을 문자열 s에 저장

        // 읽어온 문자열을 ','를 기준으로 나누어 벡터에 저장
        std::istringstream ss(s);
        std::vector<double> waypoint;
        std::vector<std::string> xy = split(s,',');

        // 나뉜 문자열을 double로 변환하여 waypoint 벡터에 추가
        for(std::vector<std::string>::iterator itr = xy.begin(); itr != xy.end(); ++itr){
            waypoint.push_back(to_double(*itr));
        }

        // waypoint 벡터를 double_vec_pointer 벡터에 추가
        double_vec_pointer.push_back(waypoint);
    }

    // ROS 퍼블리셔 초기화: waypoint를 시각화하기 위한 MarkerArray 퍼블리셔와, 개별 point를 시각화하기 위한 Marker 퍼블리셔
    waypoint_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("odometry",10);
    waypoint_point_pub = nh.advertise<visualization_msgs::Marker>("point", 10);
    
}

std::vector<std::string> tracking::split(std::string input,char delimeter){
    std::vector<std::string> point; // 결과를 저장할 벡터 생성
    std::stringstream ss(input); // 문자열 스트림을 생성하고 입력 문자열을 스트림에 넣음
    
    // 구분자를 사용하여 문자열을 나누고 결과를 벡터에 추가
    std::string temp; 
    while(getline(ss,temp,delimeter)){
        point.push_back(temp);
    }

    return point; // 나누어진 문자열이 담긴 벡터 반환
}

double tracking::to_double(std::string s) {
    std::istringstream ss(s); // 문자열 스트림 생성하고 입력 문자열을 스트림에 넣음
    double x; // double 변수 생성

    ss >> x; // 스트림에서 double로 변환하여 x에 저장
    return x; // 변환된 double 값 반환
}

void tracking::PublishWayPoint(){
    //waypoint_marker_pub는 visualization_msgs::MarkerArray 메시지 타입을 발행(publish)하는 ROS 퍼블리셔입니다. 이 퍼블리셔를 통해 메시지를 발행하면 해당 메시지가 ROS 시스템 상에서 등록된 모든 구독자(subscriber)들에게 전달됩니다.
    //waypoint_path는 visualization_msgs::MarkerArray 형식의 메시지로, 아마도 웨이포인트 경로를 표현하는 시각화 메시지일 것입니다.

    waypoint_marker_pub.publish(waypoint_path);
}

void tracking::CarWayVisualize(){    
    // 마커의 헤더 설정
    marker3.header.frame_id = "map";  // 마커의 좌표계 설정
    marker3.header.stamp = ros::Time::now();
    marker3.ns = "my_path";
    marker3.id = 0;
    marker3.type = visualization_msgs::Marker::LINE_STRIP;
    marker3.action = visualization_msgs::Marker::ADD;
    
    marker3.scale.x = 0.3; // 마커의 크기 설정

    // 마커의 색상 설정
    marker3.color.r = 1.0;
    marker3.color.g = 0.0;
    marker3.color.b = 0.0;
    marker3.color.a = 1.0;

    // 현재 자동차의 위치 설정
    geometry_msgs::Point car_point;

    // 만약 자동차의 위치가 (0, 0)이면 함수를 종료
    if(pose2d.x == 0. && pose2d.y == 0.){
        return;
    }
    // 그렇지 않으면, 현재 자동차의 위치를 설정하고 마커에 추가
    else{
        car_point.x = pose2d.x;
        car_point.y = pose2d.y;
        car_point.z = 0.05;
        marker3.points.push_back(car_point);
    }

    // 마커를 발행
    car_pub.publish(marker3);
    //이 함수는 현재 자동차의 위치를 시각화하기 위해 ROS의 visualization_msgs::Marker 메시지를 사용하고, 이를 특정 토픽에 발행하여 RViz 또는 다른 시각화 도구에서 확인할 수 있도록 합니다.
    
}

void tracking::WaypointVisualize(){
    // 벡터의 각 값에 대해 시각화 마커를 발행댐
    visualization_msgs::Marker marker;
    std::vector<double> temp;

    marker.header.frame_id = "map";  // 마커의 좌표계 설정
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
        
    marker.scale.x = 0.3;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;

    for(std::vector<std::vector<double>>::iterator itr = double_vec_pointer.begin(); itr != double_vec_pointer.end(); ++itr) {
        temp = *itr;

        geometry_msgs::Point point;
        point.x = temp[0];
        point.y = temp[1];
        point.z = 0;

        marker.points.push_back(point);
    }

    waypoint_path.markers.push_back(marker);
}

void tracking::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg2){
    pose2d.x = msg2->pose.pose.position.x;
    pose2d.y = msg2->pose.pose.position.y;

    tf::Quaternion q{
        msg2->pose.pose.orientation.x,
        msg2->pose.pose.orientation.y,
        msg2->pose.pose.orientation.z,
        msg2->pose.pose.orientation.w
    };
    
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    yaw_ = yaw;
}

void tracking::PublishControl(){
    vehicle_control_pub.publish(vehicle_control_cmd_msg);
}


void tracking::ControlCommand(){
    std::cout << "---------------------------------------------------------" << std::endl;
    slope = atan2((point_y-pose2d.y),(point_x - pose2d.x));
    distance_between_vehicle_point = sqrt(pow((point_x-pose2d.x),2) + pow((point_y-pose2d.y),2));
    wheelbase = 2.875;
    steering = atan2(2*wheelbase*sin(slope - yaw_), distance_between_vehicle_point);
    std::cout << "Now x : " << pose2d.x << " ";
    std::cout << "Now y : " << pose2d.y << std::endl;
    std::cout << "yaw :" << yaw_ << std::endl;
    std::cout << "distance_between_vehicle_point : " << distance_between_vehicle_point << std::endl;
    std::cout << "steering :" << steering << std::endl;
    std::cout << "current_velocity: " << current_velocity << std::endl;
    // std::cout << "count : " << count << std::endl;
    
    if(count == 3110){
        vehicle_control_cmd_msg.throttle = 0;
        vehicle_control_cmd_msg.brake = true;
    }

    vehicle_control_cmd_msg.steer = -steering;

    PublishControl();
}

void tracking::SelectPoint(){
    
    std::vector<double> temp;
    std::vector<std::vector<double>>::iterator itr = double_vec_pointer.begin() + count;

    temp = *itr;

    point_x = temp.at(0);
    point_y = temp.at(1);

    ControlCommand();

    while(distance_between_vehicle_point < 5){
        if(count >= 3110){
            break;
        }
        
        count++;
        itr = double_vec_pointer.begin() + count;
        temp = *itr;
        temp_x = temp.at(0);
        temp_y = temp.at(1);
        temp_distance = sqrt(pow((temp_x-pose2d.x),2) + pow((temp_y-pose2d.y),2));
        distance_between_vehicle_point = temp_distance;
    }

    // Error();
    

    point_x = temp_x;
    point_y = temp_y;

    point_marker.header.frame_id = "map";  // 마커의 좌표계 설정
    point_marker.header.stamp = ros::Time::now();
    point_marker.ns = "waypoint_specific_point";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;

    point_marker.scale.x = 1.0;
    point_marker.scale.y = 1.0;
    point_marker.scale.z = 1.0;

    point_marker.color.r = 1.0;
    point_marker.color.g = 0.0;
    point_marker.color.b = 0.0;
    point_marker.color.a = 1.0;


    point_marker.pose.position.x = point_x;
    point_marker.pose.position.y = point_y;

    waypoint_point_pub.publish(point_marker);

    ControlCommand();

}

void tracking::SpeedCallback(const std_msgs::Float32::ConstPtr& msg){
    current_velocity = 3.6 * (msg->data);
    
    PidControl(current_velocity);
}


void tracking::PidControl(double velocity){
   target_velocity = 35;
   Kp = 0.01;
   Ki = 0.005;
   Kd = 0.007;

   double error = target_velocity - velocity;
   double prev_error = 0;
   double time = 0.1;
   double i_term = 0;

   double p_term = Kp * error;
   i_term = i_term + Ki * (error * time);
   double d_term = Kd * ((error - prev_error) / time);
   control_velocity = p_term + i_term + d_term;
   prev_error = error;

    if(control_velocity > 1){
        control_velocity = 1;
    }

   vehicle_control_cmd_msg.throttle = control_velocity;

}
/*
void tracking::Error(){
    std::vector<double> error_waypoint;
    double error_x;
    double error_y;
    double current_error;
    error_count = count;

    std::cout << "error_count : " << error_count << std::endl;
    for(int i = 0; i < error_count; i++){
        std::vector<std::vector<double>>::iterator itr = double_vec_pointer.begin()+i;
        error_waypoint = *itr;

        error_x = pow((error_waypoint.at(0) - pose2d.x),2);
        error_y = pow((error_waypoint.at(1) - pose2d.y),2);
        current_error = sqrt(error_x + error_y);

        if(current_error < min){
            min = current_error;
            index = i;
        }
    
    }

    std::cout << "current_error : " << current_error << std::endl;
    std::cout << "index : " << index << std::endl;
    std::cout << "error : " << min << std::endl;
    */
    /*
   int index = 0;
   double min_x = -37.7037086486816;
   double min_y = 84.4505920410156;
   min = 10000;
   int interval = 200;
   double coordinate_x;
   double coordinate_y;
   double error_distance;

   for(int i = index; i <= interval + index ;i++){
    coordinate_x = min_x + (point_x - min_x) /200 * i;
    coordinate_y = (point_y - min_y)/(point_x - min_x) * (coordinate_x- min_x) + min_y;
    std::cout << "coordinate_x : " << coordinate_x << " " << "coordinate_y : " << coordinate_y << std::endl;
    error_distance  = sqrt((coordinate_x - pose2d.x) * (coordinate_x - pose2d.x) + (coordinate_y - pose2d.y) * (coordinate_y - pose2d.y));

    if(error_distance < min){
        index = i;
        min = error_distance;
        min_x = coordinate_x;
        min_y = coordinate_y;
    }
    
    
   }
    std::cout << "error : " << min << std::endl;
   
}
*/
int main(int argc,char ** argv){

    ros::init(argc, argv, "trajectory");
    tracking tr;

    tr.WayPointCreate();
    tr.WaypointVisualize();
    ros::Rate loop_rate(10);
    
    while(ros::ok){
        tr.PublishWayPoint();
        tr.SelectPoint();
        tr.CarWayVisualize();

        ros::spinOnce();
        loop_rate.sleep();
    }

    
    return 0;
}