#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

#define LidarZaxisPosition 0.93

inline float retSize(float a, float b) { return abs(a - b);}

void yesBongYZsizeFilter (vector<Lidar_3DOD_2022::object_msg>& retMsg){
    for (vector<Lidar_3DOD_2022::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (retSize(it->zMax, it->zMin) > 1.8 ||
            retSize(it->zMax, it->zMin) < 1.2 ||
            retSize(it->yMax, it->yMin) > 0.7 ||
            retSize(it->yMax, it->yMin) < 0.4 ) it = retMsg.erase(it);
        else {
            it->classes = "plate";
            it++;
        }
    }
}

void noBongYZsizeFilter (vector<Lidar_3DOD_2022::object_msg>& retMsg){
    for (vector<Lidar_3DOD_2022::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (retSize(it->zMax, it->zMin) > 0.7 ||
            retSize(it->zMax, it->zMin) < 0.4 ||
            retSize(it->yMax, it->yMin) > 0.7 ||
            retSize(it->yMax, it->yMin) < 0.4 ) it = retMsg.erase(it);
        else {
            it->classes = "plate";
            it++;
        }
    }
}

void yesBongYZpositionFilter (vector<Lidar_3DOD_2022::object_msg>& retMsg){
    for (vector<Lidar_3DOD_2022::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (it->zMax < 1.5 - LidarZaxisPosition ||
            it->zMax > 2.5 - LidarZaxisPosition) it = retMsg.erase(it);
        else {
            it++;
        }
    }
}

void noBongYZpositionFilter (vector<Lidar_3DOD_2022::object_msg>& retMsg){
    for (vector<Lidar_3DOD_2022::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (it->zMin < 0.5 - LidarZaxisPosition||
            it->zMax > 2.0 - LidarZaxisPosition) it = retMsg.erase(it);
        else {
            it++;
        }
    }
}




void delivery_filter_process(const Lidar_3DOD_2022::object_msg_arrConstPtr& objs){
    RT::start();

    vector<Lidar_3DOD_2022::object_msg> retMsgYesBong = objs->object_msg_arr;
    vector<Lidar_3DOD_2022::object_msg> retMsgNoBong = objs->object_msg_arr;
    cout << "initial size   " << retMsgYesBong.size() << endl;    

    yesBongYZsizeFilter     (retMsgYesBong);
    noBongYZsizeFilter      (retMsgNoBong);
    //yesBongYZpositionFilter (retMsgYesBong);
    //noBongYZpositionFilter  (retMsgNoBong);

    cout << "pole O size   " << retMsgYesBong.size(); 
    cout << "       pole X size   " << retMsgNoBong.size() << endl; 

    retMsgYesBong.insert(retMsgYesBong.end(), retMsgNoBong.begin(), retMsgNoBong.end());
    Lidar_3DOD_2022::object_msg_arr finMsg;
    finMsg.objc = retMsgYesBong.size();
    finMsg.object_msg_arr = retMsgYesBong;

    cout << "total filtered size   " << retMsgYesBong.size() << endl; 

    Lidar_3DOD_2022::obj_msg finMsg2;
    finMsg2.objc = retMsgYesBong.size();
    for(Lidar_3DOD_2022::object_msg msg : retMsgYesBong){
        finMsg2.x.push_back(msg.x);
        finMsg2.y.push_back(msg.y);
        finMsg2.z.push_back(msg.z);
        finMsg2.xMin.push_back(msg.xMin);
        finMsg2.yMin.push_back(msg.yMin);
        finMsg2.zMin.push_back(msg.zMin);
        finMsg2.xMax.push_back(msg.xMax);
        finMsg2.yMax.push_back(msg.yMax);
        finMsg2.zMax.push_back(msg.zMax);
    }

    //pub_object.publish(finMsg);
    pub_object.publish(finMsg2);

    RT::end_cal("filter");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "delivery filter");         //node name 
	ros::NodeHandle nh;                         //nodehandle

	ros::Subscriber sub = nh.subscribe<Lidar_3DOD_2022::object_msg_arr> ("/Lidar_object", 1, delivery_filter_process);
    //pub_object = nh.advertise<Lidar_3DOD_2022::object_msg_arr> ("/Lidar_delivery_filtered_object", 1);
    pub_object = nh.advertise<Lidar_3DOD_2022::obj_msg> ("/Lidar_delivery_filtered_object", 1);



    ros::spin();
}