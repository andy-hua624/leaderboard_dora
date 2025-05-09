extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <vector>
#include <cmath>
#include <sys/time.h>
#include <mutex>

#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <string.h>
#include <thread>
#include <chrono>

#include "ekf_msg.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace std;


// 创建一个空的 JSON 对象
json j_pose;
uint32_t count_1=0,count_2;

GNSS gnss_measure; //获取gnss输入数据
IMU imu_measure; //获取 imu 输入数据

void* EKF_fusion_pthread(void *dora_context)
{
  uint32_t count=0;
  const int rate = 50;   // 设定频率为 xx HZ
  const chrono::milliseconds interval((int)(1000/rate));
 
  while(true)
  {
      count++;
      struct timeval tv;
      gettimeofday(&tv, NULL);


      // cout << "The EKF_fusion pub time is: "  << tv.tv_sec <<","<< tv.tv_usec/1000.0f<<" ms " 
      //  << "count:" <<count << std::endl;
      //cout<<j_pose<<endl;


      // std::cout << "z: " << j["z"] << std::endl;
      // 将 JSON 对象序列化为字符串
      
      // std::cout <<"The EKF_fusion pub pose seq: "<<gnss_measure.header.seq
      //           <<" x: "<< gnss_measure.x
      //           <<" y: "<< gnss_measure.y
      //           <<" z: "<< gnss_measure.z<< std::endl;
                
      // j_pose["header"]["frame_id"] = gnss_measure.header.frame_id;
      // // j_pose["header"]["stamp"]["sec"] = gnss_measure.header.sec;
      // // j_pose["header"]["stamp"]["nanosec"] = gnss_measure.header.nanosec;
      j_pose["position"]["x"] = gnss_measure.x;
      j_pose["position"]["y"] = gnss_measure.y;
      // j_pose["position"]["z"] = gnss_measure.z;

      // //std::cout << "gnss_measure.x : " << gnss_measure.x << std::endl;   
      // //std::cout << "gnss_measure.y : " << gnss_measure.y << std::endl;   

      // j_pose["orientation"]["x"] = imu_measure.qx;
      // j_pose["orientation"]["y"] = imu_measure.qy;
      // j_pose["orientation"]["z"] = imu_measure.qz;
      // j_pose["orientation"]["w"] = imu_measure.qw;


      // j_pose["heading"] = ll;

      // Eigen::Quaterniond q;
      // q.w() = imu_measure.qw;
      // q.x() = imu_measure.qx;
      // q.y() = imu_measure.qy;
      // q.z() = imu_measure.qz;

      // // q.w() = 0.9586; //测试数据 EIGEN转换结果与MATLAB 和 python转换结果都不同
      // // q.x() = -0.02;  // 测试日期 20240528
      // // q.y() = -0.0377;
      // // q.z() = 0.0702;
      // Eigen::Matrix3d R = q.toRotationMatrix();
      // Eigen::Vector3d eulerAngle = R.eulerAngles(2,1,0);
      // cout << "yaw(z) roll(x) pitch(y) = " << eulerAngle.transpose() << endl<< std::endl;
      // j_pose["orientation"]["Heading"] = eulerAngle(0);
      //std::cout << "heading : " << eulerAngle(0) * 57.3 << std::endl;
      j_pose["orientation"]["Heading"]=imu_measure.yaw;

      // j_pose["orientation"]["Pitch"] = eulerAngle(1);
      // j_pose["orientation"]["Roll"] = eulerAngle(2);
      std::string json_string = j_pose.dump(4); // 参数 4 表示缩进宽度
      
      // 将字符串转换为 char* 类型
      char *c_json_string = new char[json_string.length() + 1];
      strcpy(c_json_string, json_string.c_str());
      std::string out_id = "DoraGnssPose";
      //std::cout<<json_string<<endl;
      std::cout << "准备发送数据: " << json_string << std::endl;
      int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
      if (result != 0)
      {
          std::cerr << "failed to send output" << std::endl;
      }
      //std::cout << "dora_send_output()" << std::endl;
      else
      {
          std::cout << "数据发送成功，输出 ID: " << out_id << std::endl;
      }
      this_thread::sleep_for(interval);
  }

}
int run(void *dora_context)
{
    std::mutex mtx_DoraNavSatFix;
    std::mutex mtx_DoraQuaternionStamped; // mtx.unlock();

    while(true)
    {
         
        void *event = dora_next_event(dora_context);
        
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);

            char *data;
            size_t data_len;
            read_dora_input_data(event, &data, &data_len);
            //std::cout << "id: " << id << std::endl;

            json j;
            // 将数据转化为字符串
            std::string data_str(data, data_len);
            if (data_str.empty()) {
                std::cerr << "输入 JSON 字符串为空！" << std::endl;
                return 0; // 或其他错误处理
            }
            try 
            {
              j = json::parse(data_str); // 解析 JSON 字符串               
            } 
            catch (const json::parse_error& e) 
            {
              std::cerr << "JSON 解析错误：" << e.what() << std::endl; // 处理解析失败的情况
              free_dora_event(event);
            }
            if (strcmp(id, "DoraNavSatFix") == 0)
            {
                count_1++;
                struct timeval tv;
                gettimeofday(&tv, NULL);
            
                // mtx_DoraNavSatFix.lock();
                // if (j.contains("seq") && j["seq"].is_number()) {
                //     gnss_measure.header.seq = j["seq"];
                // }
                // if (j.contains("sec") && j["sec"].is_number()) {
                //     gnss_measure.header.sec = j["sec"];
                // }
                // if (j.contains("nanosec") && j["nanosec"].is_number()) {
                //     gnss_measure.header.nanosec = j["nanosec"];
                // }
                // if (j.contains("x") && j["x"].is_number()) {
                //     gnss_measure.x = j["x"];
                // }
                // if (j.contains("y") && j["y"].is_number()) {
                //     gnss_measure.y = j["y"];
                // }
                // if (j.contains("z") && j["z"].is_number()) {
                //     gnss_measure.z = j["z"];
                // }
                // mtx_DoraNavSatFix.unlock();
            }
            else if(strcmp(id, "DoraQuaternionStamped") == 0)
            {
                count_2 ++;
                struct timeval tv;
                gettimeofday(&tv, NULL);
            
                // mtx_DoraQuaternionStamped.lock();
                // if (j.contains("seq") && j["seq"].is_number()) {
                //     imu_measure.header.seq = j["seq"];
                // }
                // if (j.contains("sec") && j["sec"].is_number()) {
                //     imu_measure.header.sec = j["sec"];
                // }
                // if (j.contains("nanosec") && j["nanosec"].is_number()) {
                //     imu_measure.header.nanosec = j["nanosec"];
                // }
                // if (j.contains("x") && j["x"].is_number()) {
                //     imu_measure.qx = j["x"];
                // }
                // if (j.contains("y") && j["y"].is_number()) {
                //     imu_measure.qy = j["y"];
                // }
                // if (j.contains("z") && j["z"].is_number()) {
                //     imu_measure.qz = j["z"];
                // }
                // if (j.contains("w") && j["w"].is_number()) {
                //     imu_measure.qw = j["w"];
                // }
                if (j.contains("heading_deg") && j["heading_deg"].is_number()) {
                    imu_measure.yaw = j["heading_deg"];
                }
                mtx_DoraQuaternionStamped.unlock();
            }
            
      }
      else if (ty == DoraEventType_Stop)
      {
          printf("[c node] received stop event\n");
      }
      else
      {
          printf("[c node] received unexpected event: %d\n", ty);
      }
      free_dora_event(event);

    }
    return 0;
}


int main()
{
  //std::cout << "ekf fusion for dora " << std::endl;
  auto dora_context = init_dora_context_from_env();

  // 从 JSON 数据中提取关键字
  j_pose["header"]["frame_id"] = "gnss";
  j_pose["header"]["stamp"]["sec"] = 0;
  j_pose["header"]["stamp"]["nanosec"] = 0.0;
  j_pose["position"]["x"] = 0.0;
  j_pose["position"]["y"] = 0.0;
  j_pose["position"]["z"] = 0.0;
  j_pose["position"]["vx"] = 0.0;
  j_pose["position"]["vy"] = 0.0;
  j_pose["position"]["vz"] = 0.0;

  j_pose["orientation"]["x"] = 0.0;
  j_pose["orientation"]["y"] = 0.0;
  j_pose["orientation"]["z"] = 0.0;
  j_pose["orientation"]["w"] = 0.0;
  j_pose["orientation"]["Roll"] = 0.0;
  j_pose["orientation"]["Pitch"] = 0.0;
  j_pose["orientation"]["Heading"] = 0.0;

  pthread_t id = 0;
  // 开启车辆状态线程
  if (pthread_create(&id, nullptr, EKF_fusion_pthread, dora_context) != 0)
  {
      std::cerr << "create EKF_fusion_pthread thread fail!" << std::endl;
      exit(-1);
  }

  auto ret = run(dora_context);
  free_dora_context(dora_context);

  std::cout << "exit ekf fusion node ..." << std::endl;
  return ret;
}