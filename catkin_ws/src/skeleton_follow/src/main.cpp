// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015-2017 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Be excellent to each other.
#include <SFML/Graphics.hpp>
#include <astra/astra.hpp>
#include <iostream>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <deque>
#include <unordered_map>
#include <chrono>
#include <astra_core/astra_core.hpp>
#include "LitDepthVisualizer.hpp"
#include <key_handler.h>

#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <skeleton_follow/Skeleton.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <string>


#include <python2.7/Python.h>
#include <iostream>
#include <string>

//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/opencv.hpp>
//#include <sensor_msgs/image_encodings.h>

#include <string.h>
#ifdef __cplusplus
extern "C" {               // 告诉编译器下列代码要以C链接约定的模式进行链接
#endif
#include <xdo.h>
#ifdef __cplusplus
}
#endif

using namespace std;

enum BTN_STATUS{
    PRESS,
    DOWN,
    UP
};

struct button_t
{
    BTN_STATUS last_state;
    int count;
    double last_change_time;
};

class window_control
{
public:
    window_control(){
        xdo = xdo_new(NULL);
        mouse_control_flag = 0;
        Py_Initialize();
        //导入环境变量
        PyRun_SimpleString("import sys");
        PyRun_SimpleString("import pyautogui");
        //python脚本路径
        PyRun_SimpleString("sys.path.append('./')");//放在cpp的同一路径下

        pFunc= PyObject_GetAttrString(pModule, "size");
        PyObject *pyValue = PyObject_CallObject(pFunc,NULL); //调用函数返回结果
        PyArg_ParseTuple(pyValue,"i|i",&screenWidth,&screenHeight);
        printf("window size: %d ,%d\n", screenWidth,screenHeight);
        Py_DECREF(pFunc);
        scaleX = screenWidth / 640;
        scaleY = screenHeight / 480;
        xdo_get_mouse_location2(xdo, &current_pos_x, &current_pos_y, &screen_num, &window);
        for(int i = 0; i < 10; i++)
        {
            xdo_move_mouse_relative(xdo, 1, 1);
        }
    }

    void process_body_data(astra::Frame& frame){
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>(); 
        if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
        {
            return;
        }
        const float jointScale = bodyFrame.info().width() / 120.f;
        const auto& bodies = bodyFrame.bodies();
        for (auto& body : bodies)
        {
            //printf("Processing frame #%d body %d left hand: %u\n",
            //    bodyFrame.frame_index(), body.id(), unsigned(body.hand_poses().left_hand()));
            // Pixels in the body mask with the same value as bodyId are
            // from the same body.
            auto bodyId = body.id();

            // bodyStatus is one of:
            // ASTRA_BODY_STATUS_NOT_TRACKING = 0,
            // ASTRA_BODY_STATUS_LOST = 1,
            // ASTRA_BODY_STATUS_TRACKING_STARTED = 2,
            // ASTRA_BODY_STATUS_TRACKING = 3,
            auto bodyStatus = body.status();

            if (bodyStatus == astra::BodyStatus::TrackingStarted)
            {
                //printf("Body Id: %d Status: Tracking started\n", bodyId);
            }
            if (bodyStatus == astra::BodyStatus::Tracking)
            {
                //printf("Body Id: %d Status: Tracking\n", bodyId);
            }

            if (bodyStatus == astra::BodyStatus::TrackingStarted ||
                bodyStatus == astra::BodyStatus::Tracking)
            {
                auto centerOfMass = body.center_of_mass();
                geometry_msgs::Vector3 bodycenter;
                bodycenter.x = centerOfMass.x;
                bodycenter.y = centerOfMass.y;
                bodycenter.z = centerOfMass.z;
                auto handPoses = body.hand_poses();
                // astra_handpose_t is one of:
                // ASTRA_HANDPOSE_UNKNOWN = 0
                // ASTRA_HANDPOSE_GRIP = 1
                astra::HandPose leftHandPose = handPoses.left_hand();
                astra::HandPose rightHandPose = handPoses.right_hand();

                //printf("Body %d Left hand pose: %d Right hand pose: %d\n",body->id,leftHandPose,rightHandPose);

                const bool jointTrackingEnabled       = body.joints_enabled();
                const bool handPoseRecognitionEnabled = body.hand_poses_enabled();

                //printf("Body %d CenterOfMass (%f, %f, %f)\n",bodyId,centerOfMass->x, centerOfMass->y, centerOfMass->z);
                for(auto& joint : body.joints())
                {
                    if(joint.type() == astra::JointType::Head)
                        ;//publishTransform(joint, fixed_frame, "head", g_skel);
                    if(joint.type() == astra::JointType::LeftHand){
                        auto jointStatus = joint.status();
                        auto worldPos = joint.world_position();
                        auto depthPos = joint.depth_position();
                        if(jointStatus == astra::JointStatus::Tracked ){
                            int diffx = depthPos.x - last_left_hand_pos_x;
                            int diffy = depthPos.y - last_left_hand_pos_y;
                            printf("x : %d - %d = %d = %d \n",depthPos.x,last_left_hand_pos_x,depthPos.x - last_left_hand_pos_x, (depthPos.x - last_left_hand_pos_x)/scaleX);
                            printf("y : %d - %d = %d = %d \n",depthPos.y,last_left_hand_pos_y,depthPos.y - last_left_hand_pos_y, (depthPos.y - last_left_hand_pos_y)/scaleY);
                            move_mouse(int(diffx/scaleX), int(diffy/scaleY));
                         }    
                    }
                    if(joint.type() == astra::JointType::RightHand)
                        ;//publishTransform(joint, fixed_frame, "right_hand", g_skel);

                    if(joint.type() == astra::JointType::Neck)
                        ;//publishTransform(joint, fixed_frame, "neck", g_skel);
                    if(joint.type() == astra::JointType::ShoulderSpine)
                        ;//publishTransform(joint, fixed_frame, "shoulderSpine", g_skel);
                    if(joint.type() == astra::JointType::MidSpine)
                        ;//publishTransform(joint, fixed_frame, "midSpine", g_skel);
                    if(joint.type() == astra::JointType::BaseSpine)
                        ;//publishTransform(joint, fixed_frame, "baseSpine", g_skel);

                    if(joint.type() == astra::JointType::RightShoulder)
                        ;//publishTransform(joint, fixed_frame, "right_shoulder", g_skel);
                    if(joint.type() == astra::JointType::RightElbow)
                        ;//publishTransform(joint, fixed_frame, "right_elbow", g_skel)
                    if(joint.type() == astra::JointType::RightHip)
                        ;//publishTransform(joint, fixed_frame, "right_hip", g_skel);
                    if(joint.type() == astra::JointType::RightWrist)
                        ;//publishTransform(joint, fixed_frame, "right_Wrist", g_skel);
                    if(joint.type() == astra::JointType::RightKnee)
                        ;//publishTransform(joint, fixed_frame, "right_knee", g_skel);
                    if(joint.type() == astra::JointType::RightFoot)
                        ;//publishTransform(joint, fixed_frame, "right_foot", g_skel);

                    if(joint.type() == astra::JointType::LeftShoulder)
                        ;//publishTransform(joint, fixed_frame, "left_shoulder", g_skel);
                    if(joint.type() == astra::JointType::LeftElbow)
                        ;//publishTransform(joint, fixed_frame, "left_elbow", g_skel);
                    if(joint.type() == astra::JointType::LeftHip)
                        ;//publishTransform(joint, fixed_frame, "left_hip", g_skel);
                    if(joint.type() == astra::JointType::LeftWrist)
                        ;//publishTransform(joint, fixed_frame, "left_Wrist", g_skel);
                    if(joint.type() == astra::JointType::LeftKnee)
                        ;//publishTransform(joint, fixed_frame, "left_knee", g_skel);
                    if(joint.type() == astra::JointType::LeftFoot)
                        ;//publishTransform(joint, fixed_frame, "left_foot", g_skel);
                }
            }
            else if (bodyStatus == astra::BodyStatus::TrackingLost)
            {
                printf("Body %u Status: Tracking lost.\n", bodyId);
            }
            else // bodyStatus == ASTRA_BODY_STATUS_NOT_TRACKING
            {
                printf("Body Id: %d Status: Not Tracking\n", bodyId);
            }
            break;
        }
    }

    void get_focused_window(){
        xdo_get_focused_window(xdo, &window);
        printf("focuse window id = %d\n", window);
        xdo_get_active_window(xdo, &window);
        printf("active window id = %d\n", window);
        xdo_get_window_name(xdo, window, &name, &name_len, &name_type);
        printf("window name:%s,len:%d,type:%d\n",name,name_len,name_type);
        xdo_get_window_size(xdo, window, &width, &height);
        printf("window_size:(width:%d , height:%d )\n", width,height);
        xdo_get_window_location(xdo, window, &position_x, &position_y, &screen);
        screen_num = XScreenNumberOfScreen(screen);
        printf("window position:(x:%d ,y:%d ,screen_num:%d)\n",position_x,position_y,screen_num);
        pid = xdo_get_pid_window(xdo, window);
        printf("window pid:%d\n",pid);        
    }

    void move_mouse(int x, int y){
        xdo_move_mouse_relative(xdo, x, y);
    }
private:
    int current_pos_x,current_pos_y;
    int screenWidth,screenHeight;
    int last_left_hand_pos_x,last_left_hand_pos_y;
    float scaleX,scaleY;
    xdo_t *xdo;
    long unsigned int window;
    unsigned char *name;
    int name_len;
    int name_type;
    int height,width;
    int position_x,position_y;
    Screen *screen;
    int screen_num;
    int pid;
    int mouse_control_flag;
    //python
    PyObject * pModule = NULL;
    PyObject * pFunc = NULL;
    PyObject * pClass = NULL;
    PyObject * pInstance = NULL;
};

class sfLine : public sf::Drawable
{
public:
    sfLine(const sf::Vector2f& point1, const sf::Vector2f& point2, sf::Color color, float thickness)
        : color_(color)
    {
        const sf::Vector2f direction = point2 - point1;
        const sf::Vector2f unitDirection = direction / std::sqrt(direction.x*direction.x + direction.y*direction.y);
        const sf::Vector2f normal(-unitDirection.y, unitDirection.x);

        const sf::Vector2f offset = (thickness / 2.f) * normal;

        vertices_[0].position = point1 + offset;
        vertices_[1].position = point2 + offset;
        vertices_[2].position = point2 - offset;
        vertices_[3].position = point1 - offset;

        for (int i = 0; i<4; ++i)
            vertices_[i].color = color;
    }

    void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(vertices_, 4, sf::Quads, states);
    }

private:
    sf::Vertex vertices_[4];
    sf::Color color_;
};

class SkeletonTracker
{
public:
  std::string fixed_frame;

  SkeletonTracker()
  {    
  }
  
  void init(ros::NodeHandle nh)
  {
    n = nh;
    int rate;
    n.param("tracking_rate", rate, 1);
    n.param("fixed_frame", fixed_frame, std::string("openni_depth_frame"));
    skeleton_pub_ = n.advertise<skeleton_follow::Skeleton>("/skeleton", rate);
    //image_transport负责订阅和发布
    //image_transport::ImageTransport it(n);
    //pub_image = it.advertise("/camera/rgb", 1);
  }

//   void process_colorframe(astra_colorframe_t colorFrame)
//   {
//       astra_image_metadata_t metadata;
//       astra_rgb_pixel_t* colorData_rgb;
//       uint8_t *data_ptr;
//       uint32_t colorByteLength;

// //          astra_colorframe_get_data_rgb_ptr(colorFrame, &colorData_rgb, &colorByteLength);
//       astra_colorframe_get_data_ptr(colorFrame,&data_ptr,&colorByteLength);
//       astra_colorframe_get_metadata(colorFrame, &metadata);

//       int width = metadata.width;
//       int height = metadata.height;
//       size_t index = ((width * (height / 2)) + (width / 2));
// //          cv::Mat (height, width, CV_8UC3,cv::Scalar(0,0,0));
//       //astra_rgb_pixel_t middle = colorData_rgb[index];
//       ROS_INFO("image: %d  : %d    : %d : %d\n", width, height, colorByteLength,sizeof(astra_rgb_pixel_t));
//       //char *ptr = new char(colorByteLength);
//       //astra_colorframe_copy_data(colorFrame,ptr);
//       //memcpy(image.data,(char *)colorData_rgb,colorByteLength);
//       //delete ptr;
// //         pub_image.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());

//       //cv::imshow("camera", image);
//       //cv::waitKey(3); // opencv刷新图像 3ms
//   }

  void publishTransform(const astra::Joint& joint, string const &frame_id, string const &child_frame_id, skeleton_follow::Skeleton &skeleton)
  {

    static tf::TransformBroadcaster br;
    // jointStatus is one of:
    // ASTRA_JOINT_STATUS_NOT_TRACKED = 0,
    // ASTRA_JOINT_STATUS_LOW_CONFIDENCE = 1,
    // ASTRA_JOINT_STATUS_TRACKED = 2,
    auto jointStatus = joint.status();
    auto worldPos = joint.world_position();
    auto depthPos = joint.depth_position();

    double x = worldPos.x;// / 1000.0;
    double y = worldPos.y;// / 1000.0;
    double z = worldPos.z;// / 1000.0;

    // orientation is a 3x3 rotation matrix where the column vectors also
    // represent the orthogonal basis vectors for the x, y, and z axes.
    const astra::Matrix3x3 joint_orientation = joint.orientation();
    const astra::Vector3f xAxis = joint_orientation.x_axis(); // same as orientation->m00, m10, m20
    const astra::Vector3f yAxis = joint_orientation.y_axis(); // same as orientation->m01, m11, m21
    const astra::Vector3f zAxis = joint_orientation.z_axis(); // same as orientation->m02, m12, m22
    KDL::Rotation rotation(xAxis.x, xAxis.y, xAxis.z,
               yAxis.x, yAxis.y, yAxis.z,
               zAxis.x, zAxis.y, zAxis.z);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    geometry_msgs::Vector3 position;
    geometry_msgs::Vector3 rpy;
    geometry_msgs::Pose2D  image_2d;
    geometry_msgs::Quaternion orientation;

    position.x = x;
    position.y = y;
    position.z = z;

    image_2d.x = depthPos.x;
    image_2d.y = depthPos.y;

    orientation.x = qx;
    orientation.y = qy;
    orientation.z = qz;
    orientation.w = qw;


    tf::Quaternion quat;
    tf::quaternionMsgToTF(orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    rpy.x = roll; rpy.y = pitch; rpy.z = yaw;
    //if(child_frame_id == "head")
    //    printf("%s : RPY = %f , %f , %f \n",child_frame_id.c_str(), roll, pitch,yaw);

    skeleton.name.push_back(child_frame_id);
    skeleton.position.push_back(position);
    skeleton.orientation.push_back(orientation);
    skeleton.confidence.push_back((double)jointStatus*0.5);
    skeleton.rpy.push_back(rpy);
    skeleton.image_2d.push_back(image_2d);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
  }

  void ros_process_Body(astra::Frame& frame)
  {

    astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();

    skeleton_follow::Skeleton g_skel;

    if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
    {
        return;
    }
    const float jointScale = bodyFrame.info().width() / 120.f;
    const auto& bodies = bodyFrame.bodies();
    for (auto& body : bodies)
    {
        //printf("Processing frame #%d body %d left hand: %u\n",
        //    bodyFrame.frame_index(), body.id(), unsigned(body.hand_poses().left_hand()));

        // Pixels in the body mask with the same value as bodyId are
        // from the same body.
        auto bodyId = body.id();

        // bodyStatus is one of:
        // ASTRA_BODY_STATUS_NOT_TRACKING = 0,
        // ASTRA_BODY_STATUS_LOST = 1,
        // ASTRA_BODY_STATUS_TRACKING_STARTED = 2,
        // ASTRA_BODY_STATUS_TRACKING = 3,
        auto bodyStatus = body.status();

        if (bodyStatus == astra::BodyStatus::TrackingStarted)
        {
            //printf("Body Id: %d Status: Tracking started\n", bodyId);
        }
        if (bodyStatus == astra::BodyStatus::Tracking)
        {
            //printf("Body Id: %d Status: Tracking\n", bodyId);
        }

        if (bodyStatus == astra::BodyStatus::TrackingStarted ||
            bodyStatus == astra::BodyStatus::Tracking)
        {
            auto centerOfMass = body.center_of_mass();
            geometry_msgs::Vector3 bodycenter;
            bodycenter.x = centerOfMass.x;
            bodycenter.y = centerOfMass.y;
            bodycenter.z = centerOfMass.z;
            g_skel.bodycenter.push_back(bodycenter);
            auto handPoses = body.hand_poses();
            // astra_handpose_t is one of:
            // ASTRA_HANDPOSE_UNKNOWN = 0
            // ASTRA_HANDPOSE_GRIP = 1
            astra::HandPose leftHandPose = handPoses.left_hand();
            astra::HandPose rightHandPose = handPoses.right_hand();
            g_skel.handpose.push_back((int)leftHandPose);
            g_skel.handpose.push_back((int)rightHandPose);
            //printf("Body %d Left hand pose: %d Right hand pose: %d\n",body->id,leftHandPose,rightHandPose);

            const bool jointTrackingEnabled       = body.joints_enabled();
            const bool handPoseRecognitionEnabled = body.hand_poses_enabled();

            //printf("Body %d CenterOfMass (%f, %f, %f)\n",bodyId,centerOfMass->x, centerOfMass->y, centerOfMass->z);
            for(auto& joint : body.joints())
            {
                if(joint.type() == astra::JointType::Head)
                    publishTransform(joint, fixed_frame, "head", g_skel);
                if(joint.type() == astra::JointType::Neck)
                    publishTransform(joint, fixed_frame, "neck", g_skel);
                if(joint.type() == astra::JointType::ShoulderSpine)
                    publishTransform(joint, fixed_frame, "shoulderSpine", g_skel);
                if(joint.type() == astra::JointType::MidSpine)
                    publishTransform(joint, fixed_frame, "midSpine", g_skel);
                if(joint.type() == astra::JointType::BaseSpine)
                    publishTransform(joint, fixed_frame, "baseSpine", g_skel);

                if(joint.type() == astra::JointType::RightShoulder)
                    publishTransform(joint, fixed_frame, "right_shoulder", g_skel);
                if(joint.type() == astra::JointType::RightElbow)
                    publishTransform(joint, fixed_frame, "right_elbow", g_skel);
                if(joint.type() == astra::JointType::RightHand)
                    publishTransform(joint, fixed_frame, "right_hand", g_skel);
                if(joint.type() == astra::JointType::RightHip)
                    publishTransform(joint, fixed_frame, "right_hip", g_skel);
                if(joint.type() == astra::JointType::RightWrist)
                    publishTransform(joint, fixed_frame, "right_Wrist", g_skel);
                if(joint.type() == astra::JointType::RightKnee)
                    publishTransform(joint, fixed_frame, "right_knee", g_skel);
                if(joint.type() == astra::JointType::RightFoot)
                    publishTransform(joint, fixed_frame, "right_foot", g_skel);

                if(joint.type() == astra::JointType::LeftShoulder)
                    publishTransform(joint, fixed_frame, "left_shoulder", g_skel);
                if(joint.type() == astra::JointType::LeftElbow)
                    publishTransform(joint, fixed_frame, "left_elbow", g_skel);
                if(joint.type() == astra::JointType::LeftHand)
                    publishTransform(joint, fixed_frame, "left_hand", g_skel);
                if(joint.type() == astra::JointType::LeftHip)
                    publishTransform(joint, fixed_frame, "left_hip", g_skel);
                if(joint.type() == astra::JointType::LeftWrist)
                    publishTransform(joint, fixed_frame, "left_Wrist", g_skel);
                if(joint.type() == astra::JointType::LeftKnee)
                    publishTransform(joint, fixed_frame, "left_knee", g_skel);
                if(joint.type() == astra::JointType::LeftFoot)
                    publishTransform(joint, fixed_frame, "left_foot", g_skel);
            }
            g_skel.user_id = bodyId;
            g_skel.header.stamp = ros::Time::now();
            g_skel.header.frame_id = fixed_frame;
            skeleton_pub_.publish(g_skel);
        }
        else if (bodyStatus == astra::BodyStatus::TrackingLost)
        {
            printf("Body %u Status: Tracking lost.\n", bodyId);
        }
        else // bodyStatus == ASTRA_BODY_STATUS_NOT_TRACKING
        {
            printf("Body Id: %d Status: Not Tracking\n", bodyId);
        }
        break;
    }
  }
     
private:
ros::NodeHandle n;
ros::Publisher  skeleton_pub_;
//image_transport::Publisher pub_image;    
};

class Visualizer : public astra::FrameListener
{
public:
    using PointList = std::deque<astra::Vector2i>;
    using PointMap = std::unordered_map<int, PointList>;

    Visualizer(std::string filepath)
    {
        font_.loadFromFile(filepath);
        prev_ = ClockType::now();
        g_skeleton_tracker = new SkeletonTracker();
        g_window = new window_control();
    }

    static sf::Color get_body_color(std::uint8_t bodyId)
    {
        if (bodyId == 0)
        {
            // Handle no body separately - transparent
            return sf::Color(0x00, 0x00, 0x00, 0x00);
        }
        // Case 0 below could mean bodyId == 25 or
        // above due to the "% 24".
        switch (bodyId % 24) {
        case 0:
            return sf::Color(0x00, 0x88, 0x00, 0xFF);
        case 1:
            return sf::Color(0x00, 0x00, 0xFF, 0xFF);
        case 2:
            return sf::Color(0x88, 0x00, 0x00, 0xFF);
        case 3:
            return sf::Color(0x00, 0xFF, 0x00, 0xFF);
        case 4:
            return sf::Color(0x00, 0x00, 0x88, 0xFF);
        case 5:
            return sf::Color(0xFF, 0x00, 0x00, 0xFF);

        case 6:
            return sf::Color(0xFF, 0x88, 0x00, 0xFF);
        case 7:
            return sf::Color(0xFF, 0x00, 0xFF, 0xFF);
        case 8:
            return sf::Color(0x88, 0x00, 0xFF, 0xFF);
        case 9:
            return sf::Color(0x00, 0xFF, 0xFF, 0xFF);
        case 10:
            return sf::Color(0x00, 0xFF, 0x88, 0xFF);
        case 11:
            return sf::Color(0xFF, 0xFF, 0x00, 0xFF);

        case 12:
            return sf::Color(0x00, 0x88, 0x88, 0xFF);
        case 13:
            return sf::Color(0x00, 0x88, 0xFF, 0xFF);
        case 14:
            return sf::Color(0x88, 0x88, 0x00, 0xFF);
        case 15:
            return sf::Color(0x88, 0xFF, 0x00, 0xFF);
        case 16:
            return sf::Color(0x88, 0x00, 0x88, 0xFF);
        case 17:
            return sf::Color(0xFF, 0x00, 0x88, 0xFF);

        case 18:
            return sf::Color(0xFF, 0x88, 0x88, 0xFF);
        case 19:
            return sf::Color(0xFF, 0x88, 0xFF, 0xFF);
        case 20:
            return sf::Color(0x88, 0x88, 0xFF, 0xFF);
        case 21:
            return sf::Color(0x88, 0xFF, 0xFF, 0xFF);
        case 22:
            return sf::Color(0x88, 0xFF, 0x88, 0xFF);
        case 23:
            return sf::Color(0xFF, 0xFF, 0x88, 0xFF);
        default:
            return sf::Color(0xAA, 0xAA, 0xAA, 0xFF);
        }
    }

    void init_depth_texture(int width, int height)
    {
        if (displayBuffer_ == nullptr || width != depthWidth_ || height != depthHeight_)
        {
            depthWidth_ = width;
            depthHeight_ = height;
            int byteLength = depthWidth_ * depthHeight_ * 4;

            displayBuffer_ = BufferPtr(new uint8_t[byteLength]);
            std::memset(displayBuffer_.get(), 0, byteLength);

            texture_.create(depthWidth_, depthHeight_);
            sprite_.setTexture(texture_, true);
            sprite_.setPosition(0, 0);
        }
    }

    void init_overlay_texture(int width, int height)
    {
        if (overlayBuffer_ == nullptr || width != overlayWidth_ || height != overlayHeight_)
        {
            overlayWidth_ = width;
            overlayHeight_ = height;
            int byteLength = overlayWidth_ * overlayHeight_ * 4;

            overlayBuffer_ = BufferPtr(new uint8_t[byteLength]);
            std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);

            overlayTexture_.create(overlayWidth_, overlayHeight_);
            overlaySprite_.setTexture(overlayTexture_, true);
            overlaySprite_.setPosition(0, 0);
        }
    }

    void check_fps()
    {
        double fpsFactor = 0.02;

        std::clock_t newTimepoint= std::clock();
        long double frameDuration = (newTimepoint - lastTimepoint_) / static_cast<long double>(CLOCKS_PER_SEC);

        frameDuration_ = frameDuration * fpsFactor + frameDuration_ * (1 - fpsFactor);
        lastTimepoint_ = newTimepoint;
        double fps = 1.0 / frameDuration_;

        printf("FPS: %3.1f (%3.4Lf ms)\n", fps, frameDuration_ * 1000);
    }

    void processPoints(astra::Frame& frame)
    {
        const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

        const int width = pointFrame.width();
        const int height = pointFrame.height();

        init_depth_texture(width, height);

        visualizer_.update(pointFrame);
        const astra::RgbPixel* vizBuffer = visualizer_.get_output();

        for (int i = 0; i < width * height; i++)
        {
            const int rgbaOffset = i * 4;
            displayBuffer_[rgbaOffset] = vizBuffer[i].r;
            displayBuffer_[rgbaOffset + 1] = vizBuffer[i].b;
            displayBuffer_[rgbaOffset + 2] = vizBuffer[i].g;
            displayBuffer_[rgbaOffset + 3] = 255;
        }

        texture_.update(displayBuffer_.get());
    }

    void processDepth(astra::Frame& frame)
    {
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

        if (!depthFrame.is_valid()) { return; }

        int width = depthFrame.width();
        int height = depthFrame.height();

        init_depth_texture(width, height);

        const int16_t* depthPtr = depthFrame.data();
        for(int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                int index = (x + y * width);
                int index4 = index * 4;

                int16_t depth = depthPtr[index];
                uint8_t value = depth % 255;

                displayBuffer_[index4] = value;
                displayBuffer_[index4 + 1] = value;
                displayBuffer_[index4 + 2] = value;
                displayBuffer_[index4 + 3] = 255;
            }
        }

        texture_.update(displayBuffer_.get());
    }

    void processColorRGB(astra::Frame& frame)
    {
        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

        int width = colorFrame.width();
        int height = colorFrame.height();

        init_depth_texture(width, height);

        const astra::RgbPixel* colorData = colorFrame.data();

        for (int i = 0; i < width * height; i++)
        {
            int rgbaOffset = i * 4;
            displayBuffer_[rgbaOffset] = colorData[i].r;
            displayBuffer_[rgbaOffset + 1] = colorData[i].g;
            displayBuffer_[rgbaOffset + 2] = colorData[i].b;
            displayBuffer_[rgbaOffset + 3] = 255;
        }

        texture_.update(displayBuffer_.get());
    }

    void processBodies(astra::Frame& frame)
    {
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();

        jointPositions_.clear();
        circles_.clear();
        circleShadows_.clear();
        boneLines_.clear();
        boneShadows_.clear();

        if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
        {
            clear_overlay();
            return;
        }

        const float jointScale = bodyFrame.info().width() / 120.f;

        const auto& bodies = bodyFrame.bodies();

        for (auto& body : bodies)
        {
            //printf("Processing frame #%d body %d left hand: %u\n",
            //    bodyFrame.frame_index(), body.id(), unsigned(body.hand_poses().left_hand()));
            for(auto& joint : body.joints())
            {
                jointPositions_.push_back(joint.depth_position());
            }

            update_body(body, jointScale);
            break;
        }

        const auto& floor = bodyFrame.floor_info(); //floor
        if (floor.floor_detected())
        {
            const auto& p = floor.floor_plane();
            //std::cout << "Floor plane: ["
             //   << p.a() << ", " << p.b() << ", " << p.c() << ", " << p.d()
             //   << "]" << std::endl;

        }

        const auto& bodyMask = bodyFrame.body_mask();
        const auto& floorMask = floor.floor_mask();

        update_overlay(bodyMask, floorMask);
    }

    void process_hand_frame(astra::Frame& frame)
    {
        const astra::HandFrame handFrame = frame.get<astra::HandFrame>();

        handPoints_ = handFrame.handpoints();

        shorten_hand_traces();
        for (const auto& handPoint : handPoints_)
        {
            if (handPoint.status() == HAND_STATUS_TRACKING)
            {
                update_hand_trace(handPoint.tracking_id(), handPoint.depth_position());
            }
        }
    }

    void update_hand_trace(int trackingId, const astra::Vector2i& position)
    {
        auto it = pointMap_.find(trackingId);
        if (it == pointMap_.end())
        {
            PointList list;
            for (int i = 0; i < maxTraceLength_; i++)
            {
                list.push_back(position);
            }
            pointMap_.insert({trackingId, list});
        }
        else
        {
            PointList& list = it->second;
            while (list.size() < maxTraceLength_)
            {
                list.push_back(position);
            }
        }
    }

    void shorten_hand_traces()
    {
        auto it = pointMap_.begin();

        while (it != pointMap_.end())
        {
            PointList& list = it->second;
            if (list.size() > 1)
            {
                list.pop_front();
                ++it;
            }
            else
            {
                it = pointMap_.erase(it);
            }
        }
    }

    void update_body(astra::Body body,
                     const float jointScale)
    {
        const auto& joints = body.joints();

        if (joints.empty())
        {
            return;
        }

        for (const auto& joint : joints)
        {
            astra::JointType type = joint.type();
            const auto& pos = joint.depth_position();

            if (joint.status() == astra::JointStatus::NotTracked)
            {
                continue;
            }

            auto radius = jointRadius_ * jointScale; // pixels
            sf::Color circleShadowColor(0, 0, 0, 255);

             auto color = sf::Color(0x00, 0xFF, 0x00, 0xFF);

            if ((type == astra::JointType::LeftHand && astra::HandPose::Grip==body.hand_poses().left_hand()) ||
                (type == astra::JointType::RightHand &&  astra::HandPose::Grip==body.hand_poses().right_hand()))
            {
                radius *= 1.5f;
                circleShadowColor = sf::Color(255, 0, 0, 255);
                color = sf::Color(0x00, 0xAA, 0xFF, 0xFF);
            }

            const auto shadowRadius = radius + shadowRadius_ * jointScale;
            const auto radiusDelta = shadowRadius - radius;

            sf::CircleShape circle(radius);

            circle.setFillColor(sf::Color(color.r, color.g, color.b, 255));
            circle.setPosition(pos.x - radius, pos.y - radius);
            circles_.push_back(circle);

            sf::CircleShape shadow(shadowRadius);
            shadow.setFillColor(circleShadowColor);
            shadow.setPosition(circle.getPosition() - sf::Vector2f(radiusDelta, radiusDelta));
            circleShadows_.push_back(shadow);
        }

        update_bone(joints, jointScale, astra::JointType::Head, astra::JointType::Neck);
        update_bone(joints, jointScale, astra::JointType::Neck, astra::JointType::ShoulderSpine);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::LeftShoulder);
        update_bone(joints, jointScale, astra::JointType::LeftShoulder, astra::JointType::LeftElbow);
        update_bone(joints, jointScale, astra::JointType::LeftElbow, astra::JointType::LeftWrist);
        update_bone(joints, jointScale, astra::JointType::LeftWrist, astra::JointType::LeftHand);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::RightShoulder);
        update_bone(joints, jointScale, astra::JointType::RightShoulder, astra::JointType::RightElbow);
        update_bone(joints, jointScale, astra::JointType::RightElbow, astra::JointType::RightWrist);
        update_bone(joints, jointScale, astra::JointType::RightWrist, astra::JointType::RightHand);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::MidSpine);
        update_bone(joints, jointScale, astra::JointType::MidSpine, astra::JointType::BaseSpine);

        update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::LeftHip);
        update_bone(joints, jointScale, astra::JointType::LeftHip, astra::JointType::LeftKnee);
        update_bone(joints, jointScale, astra::JointType::LeftKnee, astra::JointType::LeftFoot);

        update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::RightHip);
        update_bone(joints, jointScale, astra::JointType::RightHip, astra::JointType::RightKnee);
        update_bone(joints, jointScale, astra::JointType::RightKnee, astra::JointType::RightFoot);
    }

    void update_bone(const astra::JointList& joints,
                     const float jointScale,astra::JointType j1,
                     astra::JointType j2)
    {
        const auto& joint1 = joints[int(j1)];
        const auto& joint2 = joints[int(j2)];

        if (joint1.status() == astra::JointStatus::NotTracked ||
            joint2.status() == astra::JointStatus::NotTracked)
        {
            //don't render bones between untracked joints
            return;
        }

        //actually depth position, not world position
        const auto& jp1 = joint1.depth_position();
        const auto& jp2 = joint2.depth_position();

        auto p1 = sf::Vector2f(jp1.x, jp1.y);
        auto p2 = sf::Vector2f(jp2.x, jp2.y);

        sf::Color color(255, 255, 255, 255);
        float thickness = lineThickness_ * jointScale;
        if (joint1.status() == astra::JointStatus::LowConfidence ||
            joint2.status() == astra::JointStatus::LowConfidence)
        {
            color = sf::Color(128, 128, 128, 255);
            thickness *= 0.5f;
        }

        boneLines_.push_back(sfLine(p1,
            p2,
            color,
            thickness));
        const float shadowLineThickness = thickness + shadowRadius_ * jointScale * 2.f;
        boneShadows_.push_back(sfLine(p1,
            p2,
            sf::Color(0, 0, 0, 255),
            shadowLineThickness));
    }

    void update_overlay(const astra::BodyMask& bodyMask,
                        const astra::FloorMask& floorMask)
    {
        const auto* bodyData = bodyMask.data();
        const auto* floorData = floorMask.data();
        const int width = bodyMask.width();
        const int height = bodyMask.height();

        init_overlay_texture(width, height);

        const int length = width * height;

        for (int i = 0; i < length; i++)
        {
            const auto bodyId = bodyData[i];
            const auto isFloor = floorData[i];

            sf::Color color(0x0, 0x0, 0x0, 0x0);

            if (bodyId != 0)
            {
                color = get_body_color(bodyId);
            }
            else if (isFloor != 0)
            {
                color = sf::Color(0x0, 0x0, 0xFF, 0x88);
            }

            const int rgbaOffset = i * 4;
            overlayBuffer_[rgbaOffset] = color.r;
            overlayBuffer_[rgbaOffset + 1] = color.g;
            overlayBuffer_[rgbaOffset + 2] = color.b;
            overlayBuffer_[rgbaOffset + 3] = color.a;
        }

        overlayTexture_.update(overlayBuffer_.get());
    }

    void clear_overlay()
    {
        int byteLength = overlayWidth_ * overlayHeight_ * 4;
        std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);

        overlayTexture_.update(overlayBuffer_.get());
    }

    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    {
        //processDepth(frame);
        processColorRGB(frame);
        processBodies(frame);
        g_skeleton_tracker->ros_process_Body(frame);
        process_hand_frame(frame);
        //  check_fps();
    }

    void draw_bodies(sf::RenderWindow& window)
    {
        const float scaleX = window.getView().getSize().x / overlayWidth_;
        const float scaleY = window.getView().getSize().y / overlayHeight_;

        sf::RenderStates states;
        sf::Transform transform;
        transform.scale(scaleX, scaleY);
        states.transform *= transform;

        for (const auto& bone : boneShadows_)
            window.draw(bone, states);

        for (const auto& c : circleShadows_)
            window.draw(c, states);

        for (const auto& bone : boneLines_)
            window.draw(bone, states);

        for (auto& c : circles_)
            window.draw(c, states);

    }

    void draw_circle(sf::RenderWindow& window, float radius, float x, float y, sf::Color color)
    {
        sf::CircleShape shape(radius);
        shape.setFillColor(color);
        shape.setOrigin(radius, radius);
        shape.setPosition(x, y);

        window.draw(shape);
    }

    void draw_shadow_text(sf::RenderWindow& window, sf::Text& text, sf::Color color, int x, int y)
    {
        text.setColor(sf::Color::Black);
        text.setPosition(x + 5, y + 5);
        window.draw(text);

        text.setColor(color);
        text.setPosition(x, y);

        window.draw(text);
    }

    void draw_hand_label(sf::RenderWindow& window,
                         const float radius,
                         const float x,
                         const float y,
                         const astra::HandPoint& handPoint)
    {
        const auto trackingId = handPoint.tracking_id();

        std::stringstream str;
        str << trackingId;
        if (handPoint.status() == HAND_STATUS_LOST) { str << "Lost"; }

        sf::Text label(str.str(), font_);
        const int characterSize = 60;
        label.setCharacterSize(characterSize);

        const auto bounds = label.getLocalBounds();
        label.setOrigin(bounds.left + bounds.width / 2.f, characterSize);
        draw_shadow_text(window, label, sf::Color::White, x, y - radius - 10);
    }

    void draw_hand_position(sf::RenderWindow& window,
                            const float radius,
                            const float x,
                            const float y,
                            const astra::HandPoint& handPoint)
    {
        const auto worldPosition = handPoint.world_position();

        std::stringstream str;
        str << std::fixed
            << std::setprecision(0)
            << worldPosition.x << ", "
            << worldPosition.y << ", "
            << worldPosition.z;
        // std::cout << std::fixed
        //     << std::setprecision(0)
        //     << worldPosition.x << ", "
        //     << worldPosition.y << ", "
        //     << worldPosition.z << std::endl;
        sf::Text label(str.str(), font_);
        const int characterSize = 20;
        label.setCharacterSize(characterSize);

        const auto bounds = label.getLocalBounds();
        label.setOrigin(bounds.left + bounds.width / 2.f, 0);
        draw_shadow_text(window, label, sf::Color::White, x, y + radius + 10);
    }

    void draw_hand_trace(sf::RenderWindow& window,
                         const PointList& pointList,
                         const sf::Color& color,
                         const float depthScale)
    {
        if (pointList.size() < 2) { return; }

        const float thickness = 4;
        auto it = pointList.begin();
        astra::Vector2i previousPoint = *it;

        while (it != pointList.end())
        {
            const astra::Vector2i currentPoint = *it;
            ++it;

            const sf::Vector2f p1((previousPoint.x + .5f) * depthScale,
                                  (previousPoint.y + .5f) * depthScale);
            const sf::Vector2f p2((currentPoint.x + .5f) * depthScale,
                                  (currentPoint.y + .5f) * depthScale);
            previousPoint = currentPoint;

            window.draw(sfLine(p1, p2, color, thickness));
        }
    }

    void draw_hand_points(sf::RenderWindow& window, const float depthScale)
    {
        const float radius = 16;
        const sf::Color candidateColor(255, 255, 0);
        const sf::Color lostColor(255, 0, 0);
        const sf::Color trackingColor(0, 139, 69);

        for (const auto& handPoint : handPoints_)
        {
            sf::Color color;

            switch(handPoint.status())
            {
            case HAND_STATUS_LOST:
                color = lostColor;
            case HAND_STATUS_CANDIDATE:
                color = candidateColor;
            default:
                color = trackingColor;
            }

            const astra::Vector2i& p = handPoint.depth_position();

            const float circleX = (p.x + .5f) * depthScale;
            const float circleY = (p.y + .5f) * depthScale;

            draw_circle(window, radius, circleX, circleY, color);
            draw_hand_label(window, radius, circleX, circleY, handPoint);

            if (handPoint.status() == HAND_STATUS_TRACKING)
            {
                draw_hand_position(window, radius, circleX, circleY, handPoint);
            }
        }

        const sf::Color lineColor(0, 0, 255);
        for (const auto& kvp : pointMap_)
        {
            const PointList& list = kvp.second;
            draw_hand_trace(window, list, lineColor, depthScale);
        }
    }

    void draw_to(sf::RenderWindow& window)
    {
        if (displayBuffer_ != nullptr)
        {
            const float scaleX = window.getView().getSize().x / depthWidth_;
            const float scaleY = window.getView().getSize().y / depthHeight_;
            sprite_.setScale(scaleX, scaleY);

            window.draw(sprite_); // depth
        }

        if (overlayBuffer_ != nullptr)
        {
            const float scaleX = window.getView().getSize().x / overlayWidth_;
            const float scaleY = window.getView().getSize().y / overlayHeight_;
            overlaySprite_.setScale(scaleX, scaleY);
            window.draw(overlaySprite_); //bodymask and floormask
        }
        
        if (displayBuffer_)
        {
            const float depthScale = window.getView().getSize().x / depthWidth_;
            sprite_.setScale(depthScale, depthScale);
            window.draw(sprite_);
            draw_hand_points(window, depthScale);
        }

        draw_bodies(window);
    }

private:
    samples::common::LitDepthVisualizer visualizer_;
    long double frameDuration_{ 0 };
    std::clock_t lastTimepoint_ { 0 };
    sf::Texture texture_;
    sf::Sprite sprite_;

    using BufferPtr = std::unique_ptr < uint8_t[] >;
    BufferPtr displayBuffer_{ nullptr };

    std::vector<astra::Vector2f> jointPositions_;

    int depthWidth_{0};
    int depthHeight_{0};
    int overlayWidth_{0};
    int overlayHeight_{0};

    std::vector<sfLine> boneLines_;
    std::vector<sfLine> boneShadows_;
    std::vector<sf::CircleShape> circles_;
    std::vector<sf::CircleShape> circleShadows_;

    float lineThickness_{ 0.5f }; // pixels
    float jointRadius_{ 1.0f };   // pixels
    float shadowRadius_{ 0.5f };  // pixels

    BufferPtr overlayBuffer_{ nullptr };
    sf::Texture overlayTexture_;
    sf::Sprite overlaySprite_;
    //handproces
    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;

    ClockType::time_point prev_;
    float elapsedMillis_{.0f};

    sf::Font font_;

    std::vector<astra::HandPoint> handPoints_;

    PointMap pointMap_;

    int maxTraceLength_{15};
public:
    std::string word_filepath;
    SkeletonTracker *g_skeleton_tracker;
    window_control *g_window;
};

astra::DepthStream configure_depth(astra::StreamReader& reader)
{
    auto depthStream = reader.stream<astra::DepthStream>();

    //We don't have to set the mode to start the stream, but if you want to here is how:
    astra::ImageStreamMode depthMode;

    depthMode.set_width(640);
    depthMode.set_height(480);
    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
    depthMode.set_fps(30);

    depthStream.set_mode(depthMode);

    return depthStream;
}

astra::ColorStream configure_color(astra::StreamReader& reader)
{
    auto colorStream = reader.stream<astra::ColorStream>();

    auto oldMode = colorStream.mode();

    astra::ImageStreamMode colorMode;
    colorMode.set_width(640);
    colorMode.set_height(480);
    colorMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
    colorMode.set_fps(30);

    colorStream.set_mode(colorMode);

    auto newMode = colorStream.mode();
    printf("Changed color mode: %dx%d @ %d -> %dx%d @ %d\n",
        oldMode.width(), oldMode.height(), oldMode.fps(),
        newMode.width(), newMode.height(), newMode.fps());

    return colorStream;
}

astra::InfraredStream configure_ir(astra::StreamReader& reader, bool useRGB)
{
    auto irStream = reader.stream<astra::InfraredStream>();

    auto oldMode = irStream.mode();

    astra::ImageStreamMode irMode;
    irMode.set_width(640);
    irMode.set_height(480);

    if (useRGB)
    {
        irMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
    }
    else
    {
        irMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_GRAY16);
    }

    irMode.set_fps(30);
    irStream.set_mode(irMode);

    auto newMode = irStream.mode();
    printf("Changed IR mode: %dx%d @ %d -> %dx%d @ %d\n",
        oldMode.width(), oldMode.height(), oldMode.fps(),
        newMode.width(), newMode.height(), newMode.fps());

    return irStream;
}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "skeleton_tracker",ros::init_options::NoSigintHandler);
    ros::NodeHandle np("~");

    string filepath;
    bool is_a_recording;
    np.getParam("load_filepath", filepath); 
    np.param<bool>("load_recording", is_a_recording, false);      
    string word_filepath;
    np.getParam("word_filepath", word_filepath);

    astra::initialize();
    
    if (argc == 2)
    {
        FILE *fp = fopen(argv[1], "rb");
        char licenseString[1024] = { 0 };
        fread(licenseString, 1, 1024, fp);
        orbbec_body_tracking_set_license(licenseString);

        fclose(fp);
    }
    else
    {
        const char* licenseString = "<INSERT LICENSE KEY HERE>";
        orbbec_body_tracking_set_license(licenseString);
    }

    sf::RenderWindow window(sf::VideoMode(640, 480), "Simple Body Viewer");

#ifdef _WIN32
    auto fullscreenStyle = sf::Style::None;
#else
    auto fullscreenStyle = sf::Style::Fullscreen;
#endif

    const sf::VideoMode fullScreenMode = sf::VideoMode::getFullscreenModes()[0];
    const sf::VideoMode windowedMode(640, 480);
    bool isFullScreen = false;

    astra::StreamSet sensor;
    astra::StreamReader reader = sensor.create_reader();

    Visualizer listener(word_filepath);
    listener.g_skeleton_tracker->init(np);

    auto depthStream = configure_depth(reader);
    depthStream.start();
    
    auto colorRGBStream = configure_color(reader);
    colorRGBStream.start();

    auto bodyStream = reader.stream<astra::BodyStream>();
    bodyStream.start();

    auto pointStream = reader.stream<astra::PointStream>();
    pointStream.start();

    auto handStream = reader.stream<astra::HandStream>();
    handStream.start();

    reader.add_listener(listener);

    astra::SkeletonProfile profile = bodyStream.get_skeleton_profile();

    // HandPoses includes Joints and Segmentation
    astra::BodyTrackingFeatureFlags features = astra::BodyTrackingFeatureFlags::HandPoses;

    while (window.isOpen())
    {
        astra_update();

        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;
            case sf::Event::KeyPressed:
            {
                if (event.key.code == sf::Keyboard::C && event.key.control)
                {
                    window.close();
                }
                switch (event.key.code)
                {
                case sf::Keyboard::D:
                {
                    auto oldMode = depthStream.mode();
                    astra::ImageStreamMode depthMode;

                    depthMode.set_width(640);
                    depthMode.set_height(400);
                    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
                    depthMode.set_fps(30);

                    depthStream.set_mode(depthMode);
                    auto newMode = depthStream.mode();
                    printf("Changed depth mode: %dx%d @ %d -> %dx%d @ %d\n",
                           oldMode.width(), oldMode.height(), oldMode.fps(),
                           newMode.width(), newMode.height(), newMode.fps());
                    break;
                }
                case sf::Keyboard::Q:
                    window.close();
                    break;
                case sf::Keyboard::F:
                    if (isFullScreen)
                    {
                        window.create(windowedMode, "Simple Body Viewer", sf::Style::Default);
                    }
                    else
                    {
                        window.create(fullScreenMode, "Simple Body Viewer", fullscreenStyle);
                    }
                    isFullScreen = !isFullScreen;
                    break;
                case sf::Keyboard::R:
                    depthStream.enable_registration(!depthStream.registration_enabled());
                    break;
                case sf::Keyboard::M:
                    depthStream.enable_mirroring(!depthStream.mirroring_enabled());
                    colorRGBStream.enable_mirroring(!colorRGBStream.mirroring_enabled());
                    break;
                case sf::Keyboard::P:
                    if (profile == astra::SkeletonProfile::Full)
                    {
                        profile = astra::SkeletonProfile::Basic;
                        printf("Skeleton Profile: basic\n");
                    }
                    else
                    {
                        profile = astra::SkeletonProfile::Full;
                        printf("Skeleton Profile: full\n");
                    }
                    bodyStream.set_skeleton_profile(profile);
                    break;
                case sf::Keyboard::T:
                    if (features == astra::BodyTrackingFeatureFlags::Segmentation)
                    {
                        // Joints includes Segmentation
                        features = astra::BodyTrackingFeatureFlags::Joints;
                        printf("Default Body Features: Seg+Body\n");
                    }
                    else if (features == astra::BodyTrackingFeatureFlags::Joints)
                    {
                        // HandPoses includes Joints and Segmentation
                        features = astra::BodyTrackingFeatureFlags::HandPoses;
                        printf("Default Body Features: Seg+Body+Hand\n");
                    }
                    else
                    {
                        // HandPoses includes Joints and Segmentation
                        features = astra::BodyTrackingFeatureFlags::Segmentation;
                        printf("Default Body Features: Seg\n");
                    }
                    bodyStream.set_default_body_features(features);
                    break;
                default:
                    break;
                }
                break;
            }
            default:
                break;
            }
        }

        // clear the window with black color
        window.clear(sf::Color::Black);

        listener.draw_to(window);
        window.display();
    }

    astra::terminate();

    return 0;
}
