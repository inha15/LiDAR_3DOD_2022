#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

void indexROI (Lidar_3DOD_2022::lidar_signal inputROI){
    ROI_xMin = inputROI.xMin;
    ROI_xMax = inputROI.xMax;
    ROI_yMin = inputROI.yMin;
    ROI_yMax = inputROI.yMax;    
    ROI_zMin = inputROI.zMin;
    ROI_zMax = inputROI.zMax;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "input_ROI");         //node name 
	ros::NodeHandle nh;                         //nodehandle

    nh.getParam("/ROI_node/switch_ROI", switch_ROI);
    nh.getParam("/ROI_node/ROI_xMin", ROI_xMin);
    nh.getParam("/ROI_node/ROI_xMax", ROI_xMax);
    nh.getParam("/ROI_node/ROI_yMin", ROI_yMin);
    nh.getParam("/ROI_node/ROI_yMax", ROI_yMax);
    nh.getParam("/ROI_node/ROI_zMin", ROI_zMin);
    nh.getParam("/ROI_node/ROI_zMax", ROI_zMax);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, ROI);
	ros::Subscriber sub_signal = nh.subscribe<Lidar_3DOD_2022::lidar_signal> ("/SIG_Delivery_Mission_ROI", 1, indexROI);
    pub_ROI = nh.advertise<sensor_msgs::PointCloud2> ("/1_velodyne_points_ROI", 1);
    pub_signal = nh.advertise<Lidar_3DOD_2022::lidar_signal> ("/estop_signal", 1);
    
	ros::spin();
}