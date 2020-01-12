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
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <skeleton_markers/Skeleton.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

#include <string.h>
#ifdef __cplusplus
extern "C" {               // 告诉编译器下列代码要以C链接约定的模式进行链接
#endif
#include <xdo.h>

#ifdef __cplusplus
}
#endif

using namespace std;

int mouse_keyboard()
{
    Window *list = NULL;
    xdo_t *my_xdo = xdo_new(NULL);
    xdo_search_t search;
    unsigned int nwindows;
    unsigned int i;

    memset(&search, 0, sizeof(xdo_search_t));

    /* setup search */
    search.require = 0;//SEARCH_ANY;
    search.max_depth = -1;
    search.searchmask |= (SEARCH_CLASSNAME | SEARCH_ONLYVISIBLE);
    search.only_visible = True;
    char *winname = "gedit";
    search.winclassname= winname;

    /* do search */
    xdo_search_windows(my_xdo,&search,&list,&nwindows);

    /* print results */
    printf("number of windows: %d\n", nwindows);
    unsigned char * name;
    int size;
    int type;
    for(int i=0; i<nwindows; i++ ){
        xdo_get_window_name(my_xdo,list[i],&name,&size,&type);
        printf("%d: %s\n",i, name);
    }

    /* Send keypresses to first gedit in list */


    for(int i=0; i<1000; i++) {
        xdo_activate_window(my_xdo, list[0]);
        //xdo_move_mouse(my_xdo, i, i, screen);
        xdo_move_mouse_relative(my_xdo, i, i);
        sleep(1);
        // xdo_enter_text_window(my_xdo,CURRENTWINDOW,"HELLO",1000);
        // sleep(1);
        // xdo_enter_text_window(my_xdo,CURRENTWINDOW,"WORLD",1000);
        // sleep(1);
        // xdo_send_keysequence_window(my_xdo,CURRENTWINDOW,"ctrl+a",1000);
        // sleep(1);
        // xdo_send_keysequence_window(my_xdo,CURRENTWINDOW,"Delete",1000);
        // sleep(3);
    }
    return 0;
}

namespace skeleton_tracker
{
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
        skeleton_pub_ = n.advertise<skeleton_markers::Skeleton>("/skeleton", rate);
                //image_transport负责订阅和发布
        //image_transport::ImageTransport it(n);
        //pub_image = it.advertise("/camera/rgb", 1);
      }

      void process_colorframe(astra_colorframe_t colorFrame)
      {
          astra_image_metadata_t metadata;
          astra_rgb_pixel_t* colorData_rgb;
          uint8_t *data_ptr;
          uint32_t colorByteLength;

//          astra_colorframe_get_data_rgb_ptr(colorFrame, &colorData_rgb, &colorByteLength);
          astra_colorframe_get_data_ptr(colorFrame,&data_ptr,&colorByteLength);
          astra_colorframe_get_metadata(colorFrame, &metadata);

          int width = metadata.width;
          int height = metadata.height;
          size_t index = ((width * (height / 2)) + (width / 2));
//          cv::Mat (height, width, CV_8UC3,cv::Scalar(0,0,0));
          //astra_rgb_pixel_t middle = colorData_rgb[index];
          ROS_INFO("image: %d  : %d    : %d : %d\n", width, height, colorByteLength,sizeof(astra_rgb_pixel_t));
          //char *ptr = new char(colorByteLength);
          //astra_colorframe_copy_data(colorFrame,ptr);
          //memcpy(image.data,(char *)colorData_rgb,colorByteLength);
          //delete ptr;
//         pub_image.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
  
          //cv::imshow("camera", image);
          //cv::waitKey(3); // opencv刷新图像 3ms
      }

      void publishTransform(const astra_joint_t* joint, string const &frame_id, string const &child_frame_id, skeleton_markers::Skeleton &skeleton)
      {

        static tf::TransformBroadcaster br;
        // jointStatus is one of:
        // ASTRA_JOINT_STATUS_NOT_TRACKED = 0,
        // ASTRA_JOINT_STATUS_LOW_CONFIDENCE = 1,
        // ASTRA_JOINT_STATUS_TRACKED = 2,
        const astra_joint_status_t jointStatus = joint->status;
        const astra_vector3f_t* worldPos = &joint->worldPosition;
        const astra_vector2f_t* depthPos = &joint->depthPosition;

        double x = worldPos->x;// / 1000.0;
        double y = worldPos->y;// / 1000.0;
        double z = worldPos->z;// / 1000.0;

        // orientation is a 3x3 rotation matrix where the column vectors also
        // represent the orthogonal basis vectors for the x, y, and z axes.
        const astra_matrix3x3_t* joint_orientation = &joint->orientation;
        const astra_vector3f_t* xAxis = &joint_orientation->xAxis; // same as orientation->m00, m10, m20
        const astra_vector3f_t* yAxis = &joint_orientation->yAxis; // same as orientation->m01, m11, m21
        const astra_vector3f_t* zAxis = &joint_orientation->zAxis; // same as orientation->m02, m12, m22
        KDL::Rotation rotation(xAxis->x, xAxis->y, xAxis->z,
                   yAxis->x, yAxis->y, yAxis->z,
                   zAxis->x, zAxis->y, zAxis->z);
        double qx, qy, qz, qw;
        rotation.GetQuaternion(qx, qy, qz, qw);

        geometry_msgs::Vector3 position;
        geometry_msgs::Vector3 rpy;
        geometry_msgs::Pose2D  image_2d;
        geometry_msgs::Quaternion orientation;

        position.x = x;
        position.y = y;
        position.z = z;

        image_2d.x = depthPos->x;
        image_2d.y = depthPos->y;

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
        skeleton.confidence.push_back(jointStatus*0.5);
        skeleton.rpy.push_back(rpy);
        skeleton.image_2d.push_back(image_2d);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
      }
    void processKinect(astra_bodyframe_t bodyFrame)
      {

        skeleton_markers::Skeleton g_skel;

        astra_body_list_t bodyList;
        const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
        if (rc != ASTRA_STATUS_SUCCESS)
        {
            printf("Error %d in astra_bodyframe_body_list()\n", rc);
            return;
        }

        for(int i = 0; i < bodyList.count; ++i)
        {
            astra_body_t* body = &bodyList.bodies[i];

            // Pixels in the body mask with the same value as bodyId are
            // from the same body.
            astra_body_id_t bodyId = body->id;

            // bodyStatus is one of:
            // ASTRA_BODY_STATUS_NOT_TRACKING = 0,
            // ASTRA_BODY_STATUS_LOST = 1,
            // ASTRA_BODY_STATUS_TRACKING_STARTED = 2,
            // ASTRA_BODY_STATUS_TRACKING = 3,
            astra_body_status_t bodyStatus = body->status;

            if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED)
            {
                //printf("Body Id: %d Status: Tracking started\n", bodyId);
            }
            if (bodyStatus == ASTRA_BODY_STATUS_TRACKING)
            {
                //printf("Body Id: %d Status: Tracking\n", bodyId);
            }

            if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED ||
                bodyStatus == ASTRA_BODY_STATUS_TRACKING)
            {
                const astra_vector3f_t* centerOfMass = &body->centerOfMass;
                geometry_msgs::Vector3 bodycenter;
                bodycenter.x = centerOfMass->x;
                bodycenter.y = centerOfMass->y;
                bodycenter.z = centerOfMass->z;
                g_skel.bodycenter.push_back(bodycenter);
                const astra_handpose_info_t* handPoses = &body->handPoses;
                // astra_handpose_t is one of:
                // ASTRA_HANDPOSE_UNKNOWN = 0
                // ASTRA_HANDPOSE_GRIP = 1
                const astra_handpose_t leftHandPose = handPoses->leftHand;
                const astra_handpose_t rightHandPose = handPoses->rightHand;
                g_skel.handpose.push_back(leftHandPose);
                g_skel.handpose.push_back(rightHandPose);
                //printf("Body %d Left hand pose: %d Right hand pose: %d\n",body->id,leftHandPose,rightHandPose);
                const astra_body_tracking_feature_flags_t features = body->features;
                const bool jointTrackingEnabled       = (features & ASTRA_BODY_TRACKING_JOINTS)     == ASTRA_BODY_TRACKING_JOINTS;
                const bool handPoseRecognitionEnabled = (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

                //printf("Body %d CenterOfMass (%f, %f, %f)\n",bodyId,centerOfMass->x, centerOfMass->y, centerOfMass->z);

                astra_joint_t* joint = NULL;
                joint = &body->joints[ASTRA_JOINT_HEAD];
                publishTransform(joint, fixed_frame, "head", g_skel);
                joint = &body->joints[ASTRA_JOINT_NECK];
                publishTransform(joint, fixed_frame, "neck", g_skel);
                joint = &body->joints[ASTRA_JOINT_SHOULDER_SPINE];
                publishTransform(joint, fixed_frame, "shoulderSpine", g_skel);
                joint = &body->joints[ASTRA_JOINT_MID_SPINE];
                publishTransform(joint, fixed_frame, "midSpine", g_skel);
                joint = &body->joints[ASTRA_JOINT_BASE_SPINE];
                publishTransform(joint, fixed_frame, "baseSpine", g_skel);

                joint = &body->joints[ASTRA_JOINT_RIGHT_SHOULDER];
                publishTransform(joint, fixed_frame, "right_shoulder", g_skel);
                joint = &body->joints[ASTRA_JOINT_RIGHT_ELBOW];
                publishTransform(joint, fixed_frame, "right_elbow", g_skel);
                joint = &body->joints[ASTRA_JOINT_RIGHT_HAND];
                publishTransform(joint, fixed_frame, "right_hand", g_skel);
                joint = &body->joints[ASTRA_JOINT_RIGHT_HIP];
                publishTransform(joint, fixed_frame, "right_hip", g_skel);
                joint = &body->joints[ASTRA_JOINT_RIGHT_WRIST];
                publishTransform(joint, fixed_frame, "right_Wrist", g_skel);
                joint = &body->joints[ASTRA_JOINT_RIGHT_KNEE];
                publishTransform(joint, fixed_frame, "right_knee", g_skel);
                joint = &body->joints[ASTRA_JOINT_RIGHT_FOOT];
                publishTransform(joint, fixed_frame, "right_foot", g_skel);

                joint = &body->joints[ASTRA_JOINT_LEFT_SHOULDER];
                publishTransform(joint, fixed_frame, "left_shoulder", g_skel);
                joint = &body->joints[ASTRA_JOINT_LEFT_ELBOW];
                publishTransform(joint, fixed_frame, "left_elbow", g_skel);
                joint = &body->joints[ASTRA_JOINT_LEFT_HAND];
                publishTransform(joint, fixed_frame, "left_hand", g_skel);
                joint = &body->joints[ASTRA_JOINT_LEFT_HIP];
                publishTransform(joint, fixed_frame, "left_hip", g_skel);
                joint = &body->joints[ASTRA_JOINT_LEFT_WRIST];
                publishTransform(joint, fixed_frame, "left_Wrist", g_skel);
                joint = &body->joints[ASTRA_JOINT_LEFT_KNEE];
                publishTransform(joint, fixed_frame, "left_knee", g_skel);
                joint = &body->joints[ASTRA_JOINT_LEFT_FOOT];
                publishTransform(joint, fixed_frame, "left_foot", g_skel);

                g_skel.user_id = bodyId;
                g_skel.header.stamp = ros::Time::now();
                g_skel.header.frame_id = fixed_frame;
                skeleton_pub_.publish(g_skel);
            }
            else if (bodyStatus == ASTRA_BODY_STATUS_LOST)
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

}

void output_floor(astra_bodyframe_t bodyFrame)
{
    astra_floor_info_t floorInfo;

    astra_status_t rc = astra_bodyframe_floor_info(bodyFrame, &floorInfo);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_floor_info()\n", rc);
        return;
    }

    const astra_bool_t floorDetected = floorInfo.floorDetected;
    const astra_plane_t* floorPlane = &floorInfo.floorPlane;
    const astra_floormask_t* floorMask = &floorInfo.floorMask;

    if (floorDetected != ASTRA_FALSE)
    {
        printf("Floor plane: [%f, %f, %f, %f]\n",
               floorPlane->a,
               floorPlane->b,
               floorPlane->c,
               floorPlane->d);

        const int32_t bottomCenterIndex = floorMask->width / 2 + floorMask->width * (floorMask->height - 1);
        printf("Floor mask: width: %d height: %d bottom center value: %d\n",
            floorMask->width,
            floorMask->height,
            floorMask->data[bottomCenterIndex]);
    }
}

void output_body_mask(astra_bodyframe_t bodyFrame)
{
    astra_bodymask_t bodyMask;

    const astra_status_t rc = astra_bodyframe_bodymask(bodyFrame, &bodyMask);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_bodymask()\n", rc);
        return;
    }

    const int32_t centerIndex = bodyMask.width / 2 + bodyMask.width * bodyMask.height / 2;
    printf("Body mask: width: %d height: %d center value: %d\n",
        bodyMask.width,
        bodyMask.height,
        bodyMask.data[centerIndex]);
}

void output_bodyframe_info(astra_bodyframe_t bodyFrame)
{
    astra_bodyframe_info_t info;

    const astra_status_t rc = astra_bodyframe_info(bodyFrame, &info);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_info()\n", rc);
        return;
    }

    // width and height of floor mask, body mask, and the size of depth image
    // that joint depth position is relative to.
    const int32_t width = info.width;
    const int32_t height = info.height;

    printf("BodyFrame info: Width: %d Height: %d\n",
        width,
        height);
}

void output_joint(const int32_t bodyId, const astra_joint_t* joint)
{
    // jointType is one of ASTRA_JOINT_* which exists for each joint type
    const astra_joint_type_t jointType = joint->type;

    // jointStatus is one of:
    // ASTRA_JOINT_STATUS_NOT_TRACKED = 0,
    // ASTRA_JOINT_STATUS_LOW_CONFIDENCE = 1,
    // ASTRA_JOINT_STATUS_TRACKED = 2,
    const astra_joint_status_t jointStatus = joint->status;

    const astra_vector3f_t* worldPos = &joint->worldPosition;

    // depthPosition is in pixels from 0 to width and 0 to height
    // where width and height are member of astra_bodyframe_info_t
    // which is obtained from astra_bodyframe_info().
    const astra_vector2f_t* depthPos = &joint->depthPosition;

    printf("Body %u Joint %d status %d @ world (%.1f, %.1f, %.1f) depth (%.1f, %.1f)\n",
           bodyId,
           jointType,
           jointStatus,
           worldPos->x,
           worldPos->y,
           worldPos->z,
           depthPos->x,
           depthPos->y);

    // orientation is a 3x3 rotation matrix where the column vectors also
    // represent the orthogonal basis vectors for the x, y, and z axes.
    const astra_matrix3x3_t* gorientation = &joint->orientation;
    const astra_vector3f_t* xAxis = &gorientation->xAxis; // same as orientation->m00, m10, m20
    const astra_vector3f_t* yAxis = &gorientation->yAxis; // same as orientation->m01, m11, m21
    const astra_vector3f_t* zAxis = &gorientation->zAxis; // same as orientation->m02, m12, m22

    //printf("Head orientation x: [%f %f %f]\n", xAxis->x, xAxis->y, xAxis->z);
    //printf("Head orientation y: [%f %f %f]\n", yAxis->x, yAxis->y, yAxis->z);
    //printf("Head orientation z: [%f %f %f]\n", zAxis->x, zAxis->y, zAxis->z);
    KDL::Rotation rotation(xAxis->x, xAxis->y, xAxis->z,
           yAxis->x, yAxis->y, yAxis->z,
           zAxis->x, zAxis->y, zAxis->z);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    geometry_msgs::Vector3 position;
    geometry_msgs::Quaternion orientation;

    orientation.x = qx;
    orientation.y = qy;
    orientation.z = qz;
    orientation.w = qw;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    ROS_INFO("RPY = %f , %f , %f \n", roll, pitch,yaw);
}

void output_hand_poses(const astra_body_t* body)
{
    const astra_handpose_info_t* handPoses = &body->handPoses;

    // astra_handpose_t is one of:
    // ASTRA_HANDPOSE_UNKNOWN = 0
    // ASTRA_HANDPOSE_GRIP = 1
    const astra_handpose_t leftHandPose = handPoses->leftHand;
    const astra_handpose_t rightHandPose = handPoses->rightHand;

    printf("Body %d Left hand pose: %d Right hand pose: %d\n",
        body->id,
        leftHandPose,
        rightHandPose);
}

void output_bodies(astra_bodyframe_t bodyFrame)
{
    int i;
    astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_body_list()\n", rc);
        return;
    }

    for(i = 0; i < bodyList.count; ++i)
    {
        astra_body_t* body = &bodyList.bodies[i];

        // Pixels in the body mask with the same value as bodyId are
        // from the same body.
        astra_body_id_t bodyId = body->id;

        // bodyStatus is one of:
        // ASTRA_BODY_STATUS_NOT_TRACKING = 0,
        // ASTRA_BODY_STATUS_LOST = 1,
        // ASTRA_BODY_STATUS_TRACKING_STARTED = 2,
        // ASTRA_BODY_STATUS_TRACKING = 3,
        astra_body_status_t bodyStatus = body->status;

        if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED)
        {
            printf("Body Id: %d Status: Tracking started\n", bodyId);
        }
        if (bodyStatus == ASTRA_BODY_STATUS_TRACKING)
        {
            printf("Body Id: %d Status: Tracking\n", bodyId);
        }

        if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED ||
            bodyStatus == ASTRA_BODY_STATUS_TRACKING)
        {
            const astra_vector3f_t* centerOfMass = &body->centerOfMass;

            const astra_body_tracking_feature_flags_t features = body->features;
            const bool jointTrackingEnabled       = (features & ASTRA_BODY_TRACKING_JOINTS)     == ASTRA_BODY_TRACKING_JOINTS;
            const bool handPoseRecognitionEnabled = (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

            printf("Body %d CenterOfMass (%f, %f, %f) Joint Tracking Enabled: %s Hand Pose Recognition Enabled: %s\n",
                bodyId,
                centerOfMass->x, centerOfMass->y, centerOfMass->z,
                jointTrackingEnabled       ? "True" : "False",
                handPoseRecognitionEnabled ? "True" : "False");

            const astra_joint_t* joint = &body->joints[ASTRA_JOINT_HEAD];

            output_joint(bodyId, joint);

            output_hand_poses(body);
        }
        else if (bodyStatus == ASTRA_BODY_STATUS_LOST)
        {
            printf("Body %u Status: Tracking lost.\n", bodyId);
        }
        else // bodyStatus == ASTRA_BODY_STATUS_NOT_TRACKING
        {
            printf("Body Id: %d Status: Not Tracking\n", bodyId);
        }
    }
}

skeleton_tracker::SkeletonTracker *g_skeleton_tracker;

void output_bodyframe(astra_bodyframe_t bodyFrame)
{
    output_floor(bodyFrame);

    output_body_mask(bodyFrame);

    output_bodyframe_info(bodyFrame);

    output_bodies(bodyFrame);
}

astra_reader_t reader;
astra_streamsetconnection_t sensor;

void mysig(void *)
{
    printf("------------exit----------------\n");
    sleep(1);
    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);
    astra_terminate();
    exit(0);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "skeleton_tracker",ros::init_options::NoSigintHandler);
    ros::NodeHandle np("~");
    signal(SIGINT,mysig);
    g_skeleton_tracker = new skeleton_tracker::SkeletonTracker();
    g_skeleton_tracker->init(np);
    string filepath;
    bool is_a_recording;
    np.getParam("load_filepath", filepath); 
    np.param<bool>("load_recording", is_a_recording, false);      


    set_key_handler();

    astra_initialize();

    const char* licenseString = "<INSERT LICENSE KEY HERE>";
    orbbec_body_tracking_set_license(licenseString);

    astra_streamset_open("device/default", &sensor);

    astra_reader_create(sensor, &reader);

    astra_bodystream_t bodyStream;
    astra_reader_get_bodystream(reader, &bodyStream);
    astra_stream_start(bodyStream);
    
    //astra_depthstream_t colorStream;
    //astra_reader_get_colorstream(reader, &colorStream);
    //astra_stream_start(colorStream);

    do
    {
        astra_update();

        astra_reader_frame_t frame;
        astra_status_t rc = astra_reader_open_frame(reader, 0, &frame);

        if (rc == ASTRA_STATUS_SUCCESS)
        {
            astra_bodyframe_t bodyFrame;
            astra_frame_get_bodyframe(frame, &bodyFrame);

            //output_bodyframe(bodyFrame);
            g_skeleton_tracker->processKinect(bodyFrame);
            
            //astra_colorframe_t colorFrame;
            //astra_frame_get_colorframe(frame, &colorFrame);
            //g_skeleton_tracker->process_colorframe(colorFrame);

            astra_reader_close_frame(&frame);
        }

    } while (ros::ok());

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);

    astra_terminate();
}