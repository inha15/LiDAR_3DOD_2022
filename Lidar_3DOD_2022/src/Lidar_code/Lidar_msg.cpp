#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

string send_msg_minmax(float xMin,float xMax, float yMin, float yMax){
    string tmp_xMax = (xMax < 0) ? to_string(((int)(xMax * -100) / 2) * 2 + 1) : to_string(((int)(xMax * 100) / 2) * 2);
    string tmp_xMin = (xMin < 0) ? to_string(((int)(xMin * -100) / 2) * 2 + 1) : to_string(((int)(xMin * 100) / 2) * 2);
    string tmp_yMax = (yMax < 0) ? to_string(((int)(yMax * -100) / 2) * 2 + 1) : to_string(((int)(yMax * 100) / 2) * 2);
    string tmp_yMin = (yMin < 0) ? to_string(((int)(yMin * -100) / 2) * 2 + 1) : to_string(((int)(yMin * 100) / 2) * 2);
    string tmp, st;

    for(int i = 4; i > tmp_xMin.size(); i--){
        tmp = "0" + tmp;
    }
    st = tmp + tmp_xMin;
    tmp.clear();
    for(int i = 4; i > tmp_xMax.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_xMax;
    tmp.clear();
    for(int i = 4; i > tmp_yMin.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_yMin;
    tmp.clear();
    for(int i = 4; i > tmp_yMax.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_yMax;
    return st;
}

string send_msg_DXY(PXYZI obj) {
	string dist = to_string(((int)(cal_dist(obj.x, obj.y) * 100) / 2) * 2);
	string tmp_x = (obj.x < 0) ? to_string(((int)(obj.x * -100) / 2) * 2 + 1) : to_string(((int)(obj.x * 100) / 2) * 2);
	string tmp_y = (obj.y < 0) ? to_string(((int)(obj.y * -100) / 2) * 2 + 1) : to_string(((int)(obj.y * 100) / 2) * 2);
	string tmp, st;

	for (int i = 4; i > dist.size(); i--) {
		tmp = "0" + tmp;
	}
	st = tmp + dist;
	tmp.clear();
	for (int i = 4; i > tmp_x.size(); i--) {
		tmp = "0" + tmp;
	}
	st = st + tmp + tmp_x;
	tmp.clear();
	for (int i = 4; i > tmp_y.size(); i--) {
		tmp = "0" + tmp;
	}
	st = st + tmp + tmp_y;
	return st;
}

string send_msg_cnt(int sz){
    string st = to_string(sz);
    string tmp;
    for (int i = 4; i > st.size(); i--) {
		tmp = "0" + tmp;
	}
    tmp += st;
    return tmp;
}

void msg_process(vector<pair<PXYZI,string>>& sorted_OBJ){
    std_msgs::String Lidar_string;
    Lidar_string.data = send_msg_cnt(sorted_OBJ.size());
    for(int i = 0; i < sorted_OBJ.size(); i++){
        Lidar_string.data = Lidar_string.data + send_msg_DXY(sorted_OBJ[i].first) + sorted_OBJ[i].second;
        //Lidar_string.data +="/";
    }
    //cout << Lidar_string.data << endl;
    pub_msg.publish(Lidar_string);
}

void object_msg_process(const vector<struct objInfo>& objs){
    Lidar_3DOD_2022::object_msg_arr msg;
    Lidar_3DOD_2022::object_msg msgCpnt;

    vector<Lidar_3DOD_2022::object_msg> msgConvertVector;
    msg.objc = objs.size();
    for (objInfo obj : objs){
        msgCpnt.x = obj.x;
        msgCpnt.y = obj.y;
        msgCpnt.z = obj.z;
        msgCpnt.xMin = obj.xMin;
        msgCpnt.yMin = obj.yMin;
        msgCpnt.zMin = obj.zMin;
        msgCpnt.xMax = obj.xMax;
        msgCpnt.yMax = obj.yMax;
        msgCpnt.zMax = obj.zMax;
        msgCpnt.classes = obj.classes;
        msgCpnt.idx = obj.idx;
        msgConvertVector.push_back(msgCpnt);
    }
    msg.object_msg_arr = msgConvertVector;
    pub_object.publish(msg);

    {// /obj_msg 발행을 위한 임시 코드, /object_msg기반으로 생성
        Lidar_3DOD_2022::obj_msg finMsg2;
        finMsg2.objc = objs.size();
        for(Lidar_3DOD_2022::object_msg msg : msg.object_msg_arr){
            finMsg2.idx.push_back(msg.idx);
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
        pub_obj.publish(finMsg2);
    }

}
