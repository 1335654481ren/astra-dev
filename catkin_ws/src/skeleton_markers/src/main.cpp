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
#include <GL/glut.h>
#include <string>
#include "KinectController.h"
#include "KinectDisplay.h"


using std::string;

#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

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

        double x = worldPos->x / 1000.0;
        double y = worldPos->y / 1000.0;
        double z = worldPos->z / 1000.0;

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
            geometry_msgs::Quaternion orientation;

            position.x = x;
            position.y = y;
            position.z = z;

            orientation.x = qx;
            orientation.y = qy;
            orientation.z = qz;
            orientation.w = qw;

            skeleton.name.push_back(child_frame_id);
            skeleton.position.push_back(position);
            skeleton.orientation.push_back(orientation);
            skeleton.confidence.push_back(jointStatus*0.5);

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
        }
      }
         
  private:
    ros::NodeHandle n;
    ros::Publisher  skeleton_pub_;    
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
    const astra_matrix3x3_t* orientation = &joint->orientation;
    const astra_vector3f_t* xAxis = &orientation->xAxis; // same as orientation->m00, m10, m20
    const astra_vector3f_t* yAxis = &orientation->yAxis; // same as orientation->m01, m11, m21
    const astra_vector3f_t* zAxis = &orientation->zAxis; // same as orientation->m02, m12, m22

    printf("Head orientation x: [%f %f %f]\n", xAxis->x, xAxis->y, xAxis->z);
    printf("Head orientation y: [%f %f %f]\n", yAxis->x, yAxis->y, yAxis->z);
    printf("Head orientation z: [%f %f %f]\n", zAxis->x, zAxis->y, zAxis->z);
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

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480
KinectController g_kinect_controller;
skeleton_tracker::SkeletonTracker *g_skeleton_tracker;
astra_reader_t reader;

void output_bodyframe(astra_bodyframe_t bodyFrame)
{
    output_floor(bodyFrame);

    output_body_mask(bodyFrame);

    output_bodyframe_info(bodyFrame);

    output_bodies(bodyFrame);
}


void glutIdle (void)
{
    glutPostRedisplay();
}

void glutDisplay (void)
{

        // g_kinect_controller.getContext().WaitAndUpdateAll();
    astra_update();

    astra_reader_frame_t frame;
    astra_status_t rc = astra_reader_open_frame(reader, 0, &frame);

    if (rc == ASTRA_STATUS_SUCCESS)
    {

        astra_bodyframe_t bodyFrame;
        astra_frame_get_bodyframe(frame, &bodyFrame);
        astra_frame_index_t frameIndex;
        astra_bodyframe_get_frameindex(bodyFrame, &frameIndex);
        printf("Frame index: %d\n", frameIndex);

        xn::SceneMetaData sceneMD;
        xn::DepthMetaData depthMD;

        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Setup the OpenGL viewpoint
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();

        glOrtho(0, 640, 480, 0, -1.0, 1.0);

        glDisable(GL_TEXTURE_2D);

        output_bodyframe(bodyFrame);
        g_skeleton_tracker->processKinect(bodyFrame);

    //    kinect_display_drawDepthMapGL(depthMD, sceneMD);
    //    kinect_display_drawSkeletonGL(g_kinect_controller.getUserGenerator(),g_kinect_controller.getDepthGenerator());

        printf("----------------------------\n");

        astra_reader_close_frame(&frame);
    }

    glutSwapBuffers();
}

void glutKeyboard (unsigned char key, int x, int y)
{
    switch (key)
    {
    case 27:
    exit(1);
    break;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "skeleton_tracker");
    ros::NodeHandle np("~");

    g_skeleton_tracker = new skeleton_tracker::SkeletonTracker();
    g_skeleton_tracker->init(np);
    string filepath;
    bool is_a_recording;
    np.getParam("load_filepath", filepath); 
    np.param<bool>("load_recording", is_a_recording, false);      

    //  g_skeleton_tracker.init();
    //  g_kinect_controller.init(filepath.c_str(), is_a_recording);
    set_key_handler();
    astra_initialize();
    const char* licenseString = "<INSERT LICENSE KEY HERE>";
    orbbec_body_tracking_set_license(licenseString);
    astra_streamsetconnection_t sensor;

    astra_streamset_open("device/default", &sensor);

    astra_reader_create(sensor, &reader);
    astra_bodystream_t bodyStream;
    astra_reader_get_bodystream(reader, &bodyStream);
    astra_stream_start(bodyStream);
    

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glutCreateWindow ("NITE Skeleton Tracker");

    glutKeyboardFunc(glutKeyboard);
    glutDisplayFunc(glutDisplay);
    glutIdleFunc(glutIdle);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);  

    glutMainLoop();
  
    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);
    astra_terminate();
}
