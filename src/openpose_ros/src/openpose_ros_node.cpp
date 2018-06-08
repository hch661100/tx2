#define USE_CAFFE
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>

#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <cstdio> // sscanf
#include <string> // std::string

#include <opencv2/core/core.hpp> // cv::Mat & cv::Size

#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <glog/logging.h> // google::InitGoogleLogging, CHECK, CHECK_EQ, LOG, VLOG, ...

//#include <openpose/headers.hpp>
// OpenPose dependencies
#include <openpose/core/headers.hpp>
#include <openpose/core/array.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>

#include "openpose_ros/backward.hpp"
#include "openpose_ros/transport_util.h"

#include "image_recognition_msgs/PersonDetection.h"
#include "image_recognition_msgs/BodypartDetection.h"

#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;

namespace backward{
    backward::SignalHandling sh;
}
static const std::string OPENCV_WINDOW = "Openpose Window";
//static const std::string image_topic = "/videofile/image_raw";//"/usb_cam/image_raw";

// Gflags in the command line terminal. Check all the options by adding the flag `--help`, e.g. `openpose.bin --help`.
// Note: This command will show you flags for several files. Check only the flags for the file you are checking. E.g. for `openpose.bin`, look for `Flags from examples/openpose/openpose.cpp:`.
// Debugging
DEFINE_int32(logging_level,             4,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while 255 will not output any."
                                                        " Current OpenPose library messages are in the range 0-4: 1 for low priority messages and 4 for important ones.");
// OpenPose
DEFINE_string(model_pose,               "COCO",        "Model to be used (e.g. COCO, MPI, MPI_4_layers).");
DEFINE_string(model_folder,             "/home/nvidia/openpose/models/",      "Folder where the pose models (COCO and MPI) are located.");
DEFINE_string(net_resolution,           "304x176",      "Multiples of 16.");
DEFINE_string(resolution,               "304x176",     "The image resolution (display). Use \"-1x-1\" to force the program to use the default images resolution.");
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless num_scales>1. Initial scale is always 1. If you want to change the initial scale, "
                                                        "you actually want to multiply the `net_resolution` by your desired initial scale.");
DEFINE_int32(num_scales,                1,              "Number of scales to average.");
// OpenPose Rendering
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will hide it.");

DEFINE_int32(render_pose,               1,              "Set to 0 for no rendering, 1 for CPU rendering (slightly faster), and 2 for GPU rendering (slower but greater functionality, e.g. `alpha_X` flags). If rendering is enabled, it will render both `outputData` and `cvOutputData` with the original image and desired body part to be shown (i.e. keypoints, heat maps or PAFs).");

cv::Mat inputImage_l;
cv::Mat inputImage_r;
image_recognition_msgs::PersonDetection person_detection_;
image_transport::Publisher img_pub, debug_pub;
ros::Publisher  pose_depth_pub;
ros::Publisher  sub_pub;
ros::Publisher  cvMatPose_pub;
ros::Publisher  delay_odom_pub;


Eigen::Vector3d v;
Eigen::Quaterniond q;   
Eigen::Matrix3d R;

vector<double> sub_pos, sub_rect;

vector<nav_msgs::Odometry> v_odom;

float f_x, f_y, c_x, c_y;
float scale_;
float fx_l, fy_l, cx_l, cy_l;
float fx_r, fy_r, cx_r, cy_r;
float baseLine;
float ref_depth;
cv::Mat imageDepth_;
cv::Mat debugDepth_;
int num_person = 0;
int person_id = 0;
/*
struct buffer
{
   time timestamp;
   Eigen::Vector3d v;
   Eigen::Quaterniond q;   
   Eigen::Matrix3d R;

   buffer(time t, Eigen::Vector3d _v, Eigen::Quaterniond _q, Eigen::Matrix3d _R):timestamp(t), v(_v),q(_q),R(_R){}
};

vector<buffer> v_buffer;
*/
class OpenPoseNode
{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport* it;
        ros::Subscriber image_sub_l;
        ros::Subscriber sub_odom;
        ros::Subscriber image_sub_depth;

    public:
       
        bool img_l_rcv = false; 
        bool img_depth_rcv = false;
        ros::Publisher pose_pub_l, pose_pub_r;


        cv::Mat inputImage_l;
        cv::Mat inputImage_depth;

        OpenPoseNode()
        {

 	scale_ = 1.0;
	baseLine = 0.120;

	// [677.2421875, 0.0, 651.271484375, 0.0, 677.2421875, 374.57452392578125, 0.0, 0.0, 1.0]
 	f_x = fx_l = 349.763 * scale_;
	f_y = fy_l = 349.763 * scale_;
	
	fx_r = 349.763 * scale_;
	fy_r = 349.763 * scale_;

	c_x = cx_l = cx_r = 356.07 * scale_;
	c_y = cy_l = cy_r = 187.899 * scale_;

        image_sub_l = nh_.subscribe("image_left", 1, &OpenPoseNode::convert_image_l, this);
        image_sub_depth = nh_.subscribe("image_depth", 1, &OpenPoseNode::convert_image_depth, this);
        sub_odom = nh_.subscribe("odom", 1, &OpenPoseNode::odom_callback, this);
        sub_pub = nh_.advertise<std_msgs::Float64MultiArray>("sub_screen_pos", 100);

	pose_depth_pub   = nh_.advertise<visualization_msgs::MarkerArray>( "pose_3d_vis", 1 );
        pose_pub_l  = nh_.advertise<image_recognition_msgs::PersonDetection>("pose_detected_l", 50);
        cvMatPose_pub = nh_.advertise<std_msgs::Float64MultiArray>("/cvMatPose", 1);
        delay_odom_pub = nh_.advertise<nav_msgs::Odometry>("/delayodom", 1);
            
            it = new image_transport::ImageTransport(nh_);
            img_pub = it->advertise("/skeleton_img", 1);
            debug_pub = it->advertise("/debug_img", 1);
        }

        ~OpenPoseNode(){}

        void convert_image_l(const sensor_msgs::Image msg)
        {   
	    //cout<<"left"<<endl;
            inputImage_l = matFromImage( msg ).clone();
            img_l_rcv = true;            
        }

        void convert_image_depth(const sensor_msgs::Image msg)
        {   
	   //cout<<"right"<<endl;
            imageDepth_ = matFromImage( msg ).clone();
            img_depth_rcv = true;
            debugDepth_ = matFromImage( msg ).clone();
        }

        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            
                        
            //if(!img_l_rcv && !img_depth_rcv) return;


            // ros::Duration(0.18).sleep();
	     v.x() = msg->pose.pose.position.x;
	     v.y() = msg->pose.pose.position.y;
     	     v.z() = msg->pose.pose.position.z;
     

             q.x() = msg->pose.pose.orientation.x;  
             q.y() = msg->pose.pose.orientation.y;  
             q.z() = msg->pose.pose.orientation.z;  
             q.w() = msg->pose.pose.orientation.w;  

             R = q.toRotationMatrix();

	}


        cv::Mat get_r_image()
        {
            return inputImage_r;
        }

        cv::Mat get_l_image()
        {
            return inputImage_l;
        }
};

op::PoseModel gflagToPoseModel(const std::string& poseModeString)
{
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    if (poseModeString == "COCO")
        return op::PoseModel::COCO_18;
    else if (poseModeString == "MPI")
        return op::PoseModel::MPI_15;
    else if (poseModeString == "MPI_4_layers")
        return op::PoseModel::MPI_15_4;
    else
    {
        op::error("String does not correspond to any model (COCO, MPI, MPI_4_layers)", __LINE__, __FUNCTION__, __FILE__);
        return op::PoseModel::COCO_18;
    }
}

// Google flags into program variables
std::tuple<cv::Size, cv::Size, cv::Size, op::PoseModel> gflagsToOpParameters()
{
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // outputSize
    cv::Size outputSize;
    auto nRead = sscanf(FLAGS_resolution.c_str(), "%dx%d", &outputSize.width, &outputSize.height);
    op::checkE(nRead, 2, "Error, resolution format (" +  FLAGS_resolution + ") invalid, should be e.g., 960x540 ", __LINE__, __FUNCTION__, __FILE__);
    // netInputSize
    cv::Size netInputSize;
    nRead = sscanf(FLAGS_net_resolution.c_str(), "%dx%d", &netInputSize.width, &netInputSize.height);
    op::checkE(nRead, 2, "Error, net resolution format (" +  FLAGS_net_resolution + ") invalid, should be e.g., 656x368 (multiples of 16)", __LINE__, __FUNCTION__, __FILE__);
    // netOutputSize
    const auto netOutputSize = netInputSize;
    // poseModel
    const auto poseModel = gflagToPoseModel(FLAGS_model_pose);
    // Check no contradictory flags enabled
    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (FLAGS_scale_gap <= 0. && FLAGS_num_scales > 1)
        op::error("Uncompatible flag configuration: scale_gap must be greater than 0 or num_scales = 1.", __LINE__, __FUNCTION__, __FILE__);
    // Logging and return result
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    return std::make_tuple(outputSize, netInputSize, netOutputSize, poseModel);
}

// Wrapping the body-part-detection for publishing a ros message: Only for COCO 18 parts model now
image_recognition_msgs::PersonDetection pose_ros_wrapper(cv::Mat cvMatPoses, int id)
{
     image_recognition_msgs::PersonDetection detect_msgs;
     detect_msgs.header.stamp = ros::Time::now();
     detect_msgs.id = id;
     detect_msgs.nose.x = cvMatPoses.at<float>(id,0,0);
     detect_msgs.nose.y = cvMatPoses.at<float>(id,0,1);
     detect_msgs.nose.confidence = cvMatPoses.at<float>(id,0,2);
     //cout<<"x: "<<cvMatPoses.at<float>(id,0,0)<<" , y: "<<cvMatPoses.at<float>(id,0,1)<<endl;

     detect_msgs.neck.x = cvMatPoses.at<float>(id,1,0);
     detect_msgs.neck.y = cvMatPoses.at<float>(id,1,1);
     detect_msgs.neck.confidence = cvMatPoses.at<float>(id,1,2);

     detect_msgs.right_shoulder.x = cvMatPoses.at<float>(id,2,0);
     detect_msgs.right_shoulder.y = cvMatPoses.at<float>(id,2,1);
     detect_msgs.right_shoulder.confidence = cvMatPoses.at<float>(id,2,2);

     detect_msgs.right_elbow.x = cvMatPoses.at<float>(id,3,0);
     detect_msgs.right_elbow.y = cvMatPoses.at<float>(id,3,1);
     detect_msgs.right_elbow.confidence = cvMatPoses.at<float>(id,3,2);

     detect_msgs.right_wrist.x = cvMatPoses.at<float>(id,4,0);
     detect_msgs.right_wrist.y = cvMatPoses.at<float>(id,4,1);
     detect_msgs.right_wrist.confidence = cvMatPoses.at<float>(id,4,2);

     detect_msgs.left_shoulder.x = cvMatPoses.at<float>(id,5,0);
     detect_msgs.left_shoulder.y = cvMatPoses.at<float>(id,5,1);
     detect_msgs.left_shoulder.confidence = cvMatPoses.at<float>(id,5,2);

     detect_msgs.left_elbow.x = cvMatPoses.at<float>(id,6,0);
     detect_msgs.left_elbow.y = cvMatPoses.at<float>(id,6,1);
     detect_msgs.left_elbow.confidence = cvMatPoses.at<float>(id,6,2);
 
     detect_msgs.left_wrist.x = cvMatPoses.at<float>(id,7,0);
     detect_msgs.left_wrist.y = cvMatPoses.at<float>(id,7,1);
     detect_msgs.left_wrist.confidence = cvMatPoses.at<float>(id,7,2);

     detect_msgs.right_hip.x = cvMatPoses.at<float>(id,8,0);
     detect_msgs.right_hip.y = cvMatPoses.at<float>(id,8,1);
     detect_msgs.right_hip.confidence = cvMatPoses.at<float>(id,8,2);

     detect_msgs.right_knee.x = cvMatPoses.at<float>(id,9,0);
     detect_msgs.right_knee.y = cvMatPoses.at<float>(id,9,1);
     detect_msgs.right_knee.confidence = cvMatPoses.at<float>(id,9,2);

     detect_msgs.right_ankle.x = cvMatPoses.at<float>(id,10,0);
     detect_msgs.right_ankle.y = cvMatPoses.at<float>(id,10,1);
     detect_msgs.right_ankle.confidence = cvMatPoses.at<float>(id,10,2);

     detect_msgs.left_hip.x = cvMatPoses.at<float>(id,11,0);
     detect_msgs.left_hip.y = cvMatPoses.at<float>(id,11,1);
     detect_msgs.left_hip.confidence = cvMatPoses.at<float>(id,11,2);

     detect_msgs.left_knee.x = cvMatPoses.at<float>(id,12,0);
     detect_msgs.left_knee.y = cvMatPoses.at<float>(id,12,1);
     detect_msgs.left_knee.confidence = cvMatPoses.at<float>(id,12,2);

     detect_msgs.left_ankle.x = cvMatPoses.at<float>(id,13,0);
     detect_msgs.left_ankle.y = cvMatPoses.at<float>(id,13,1);
     detect_msgs.left_ankle.confidence = cvMatPoses.at<float>(id,13,2);

     detect_msgs.right_eye.x = cvMatPoses.at<float>(id,14,0);
     detect_msgs.right_eye.y = cvMatPoses.at<float>(id,14,1);
     detect_msgs.right_eye.confidence = cvMatPoses.at<float>(id,14,2);

     detect_msgs.left_eye.x = cvMatPoses.at<float>(id,15,0);
     detect_msgs.left_eye.y = cvMatPoses.at<float>(id,15,1);
     detect_msgs.left_eye.confidence = cvMatPoses.at<float>(id,15,2);

     detect_msgs.right_ear.x = cvMatPoses.at<float>(id,16,0);
     detect_msgs.right_ear.y = cvMatPoses.at<float>(id,16,1);
     detect_msgs.right_ear.confidence = cvMatPoses.at<float>(id,16,2);

     detect_msgs.left_ear.x = cvMatPoses.at<float>(id,17,0);
     detect_msgs.left_ear.y = cvMatPoses.at<float>(id,17,1);
     detect_msgs.left_ear.confidence = cvMatPoses.at<float>(id,17,2);
/*
    ROS_INFO("%f", 
    sqrt(pow(detect_msgs.left_shoulder.x - detect_msgs.right_shoulder.x,2)+pow(detect_msgs.left_shoulder.y - detect_msgs.right_shoulder.y,2))/sqrt(pow(detect_msgs.left_hip.y - detect_msgs.neck.y,2)+pow(detect_msgs.left_hip.x - detect_msgs.neck.x,2))
    );
*/
    return detect_msgs;
}


void localize_subject(cv::Mat cvMatPoses, int id){
     float sub_x1=9999, sub_x2=0, sub_y1=9999, sub_y2=0;
     for(int i=0; i<18; i++){
        if(cvMatPoses.at<float>(id,i,2) < 0.3) continue;
        sub_x1 = cvMatPoses.at<float>(id,i,0) < sub_x1? cvMatPoses.at<float>(id,i,0): sub_x1;
        sub_x2 = cvMatPoses.at<float>(id,i,0) > sub_x2? cvMatPoses.at<float>(id,i,0): sub_x2;
        sub_y1 = cvMatPoses.at<float>(id,i,1) < sub_y1? cvMatPoses.at<float>(id,i,1): sub_y1;
        sub_y2 = cvMatPoses.at<float>(id,i,1) > sub_y2? cvMatPoses.at<float>(id,i,1): sub_y2;
     }
     sub_pos.push_back((sub_x1+sub_x2)/2.0);
     sub_pos.push_back((sub_y1+sub_y2)/2.0);
     sub_pos.push_back((sub_x2-sub_x1)*(sub_y2-sub_y1));

//     ROS_INFO("%f %f %f %f", sub_x1, sub_y1, sub_x2, sub_y2);
     sub_rect.push_back(sub_x1);
     sub_rect.push_back(sub_y1);
     sub_rect.push_back(sub_x2);
     sub_rect.push_back(sub_y2);
}
Eigen::Vector3d triangulationFromDepth( float u_1, float v_1 ) // left --- right
{

	float X, Y, Z;

	u_1 = u_1/304.0 * 672.0;
	v_1 = v_1/170.1 * 376.0;
    //v_1 = v_1/153.6*480.0;
    circle(debugDepth_, cv::Point(u_1,v_1), 5, cv::Scalar( 0, 0, 0 ), 2);

    Z = ref_depth;

    if(v_1 >= 376) {
      return Eigen::Vector3d(NAN, NAN, NAN); 
    }else{
      Z = imageDepth_.at<float>(v_1, u_1);

    }


    //if( isnan(Z) ){
      //std::cout<<"nan"<<std::endl;
    //}

	X = (u_1 - cx_l) * Z / fx_l;
	Y = (v_1 - cy_l) * Z / fy_l;

	return Eigen::Vector3d(X, Y, Z);
}


visualization_msgs::MarkerArray points_vis;
void vis3dPose(std::vector<Eigen::Vector3d> ptsList)
{	
/*	for (auto & mk: points_vis.markers) 
		 mk.action = visualization_msgs::Marker::DELETE;
	
	pose_depth_pub.publish(points_vis);

    points_vis.markers.clear();
*/
    visualization_msgs::Marker mk;
    mk.header.frame_id = "camera";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
	  
	mk.ns = "pose_3d/body_points";
	mk.color.a = 1.0;
	mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;
/*
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";
    odom_msg.pose.pose.position.x = v.x();
    odom_msg.pose.pose.position.y = v.y();
    odom_msg.pose.pose.position.z = v.z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    delay_odom_pub.publish(odom_msg);
*/
    Eigen::Vector3d p_, p;

	for(int i = 0; i < int(ptsList.size()); i++)
	{
	      mk.id = i + person_id * 13;
              //std::cout<<mk.id<<" "<<i<<" "<<num_person<<std::endl;
/*
	      mk.pose.position.x = ptsList[i](0); 
	      mk.pose.position.y = ptsList[i](1); 
	      mk.pose.position.z = ptsList[i](2);
*/
              p_.y() = -(ptsList[i](0));
              p_.x() = (ptsList[i](2));
              p_.z() = -(ptsList[i](1));

              p = R*p_ + v;
	      mk.pose.position.x = p.x(); 
	      mk.pose.position.y = p.y(); 
	      mk.pose.position.z = p.z();

	      points_vis.markers.push_back(mk);
        }

	//pose_depth_pub.publish(points_vis);


       
}

void calculate_3D_skeleton(image_recognition_msgs::PersonDetection pose_left_)
{
    Eigen::Vector3d nose_3d, neck_3d, right_shoulder_3d, right_elbow_3d, right_wrist_3d, left_shoulder_3d,
					left_elbow_3d, left_wrist_3d, right_hip_3d, right_knee_3d, right_ankle_3d, left_hip_3d, 
					left_knee_3d, left_ankle_3d, right_eye_3d, left_eye_3d, right_ear_3d, left_ear_3d;
	
	std::vector<Eigen::Vector3d> pose_3d_list;
	std::vector<Eigen::Vector3d> hands_3d_list, foot_3d_list, head_3d_list;

       
        if( pose_left_.left_hip.x  != 0 && pose_left_.left_hip.y != 0 && pose_left_.right_hip.x  != 0 && pose_left_.right_hip.y != 0 && pose_left_.left_hip.confidence > 0.5 && pose_left_.right_hip.confidence > 0.5)
	{	
		left_hip_3d = triangulationFromDepth( pose_left_.left_hip.x, pose_left_.left_hip.y );
		right_hip_3d = triangulationFromDepth( pose_left_.right_hip.x, pose_left_.right_hip.y );
                //ref_depth = (left_hip_3d(2) + right_hip_3d(2))/2;
		pose_3d_list.push_back((left_hip_3d + right_hip_3d) / 2);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

        if( pose_left_.neck.x  != 0 && pose_left_.neck.y != 0 && pose_left_.neck.confidence > 0.5)
	{	
		neck_3d = triangulationFromDepth( pose_left_.neck.x, pose_left_.neck.y);
		pose_3d_list.push_back(neck_3d);
                ref_depth = neck_3d(2);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

	if( pose_left_.nose.x  != 0 && pose_left_.nose.y != 0 && pose_left_.nose.confidence > 0.5)
	{	
		nose_3d = triangulationFromDepth( pose_left_.nose.x, pose_left_.nose.y);
		pose_3d_list.push_back(nose_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }


	if( pose_left_.right_shoulder.x  != 0 && pose_left_.right_shoulder.y != 0 && pose_left_.right_shoulder.confidence > 0.5)
	{	
		right_shoulder_3d = triangulationFromDepth( pose_left_.right_shoulder.x, pose_left_.right_shoulder.y );
		pose_3d_list.push_back(right_shoulder_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

	if( pose_left_.right_elbow.x  != 0 && pose_left_.right_elbow.y != 0 && pose_left_.right_elbow.confidence > 0.5)
	{	
		right_elbow_3d = triangulationFromDepth( pose_left_.right_elbow.x, pose_left_.right_elbow.y );
		pose_3d_list.push_back(right_elbow_3d);
	}

	if( pose_left_.right_wrist.x  != 0 && pose_left_.right_wrist.y != 0 && pose_left_.right_wrist.confidence > 0.5)
	{	
		right_wrist_3d = triangulationFromDepth( pose_left_.right_wrist.x, pose_left_.right_wrist.y );
		pose_3d_list.push_back(right_wrist_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }


	if( pose_left_.left_shoulder.x  != 0 && pose_left_.left_shoulder.y != 0 && pose_left_.left_shoulder.confidence > 0.5)
	{	
		left_shoulder_3d = triangulationFromDepth( pose_left_.left_shoulder.x, pose_left_.left_shoulder.y );
		pose_3d_list.push_back(left_shoulder_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

	if( pose_left_.left_elbow.x  != 0 && pose_left_.left_elbow.y != 0 && pose_left_.left_elbow.confidence > 0.5)
	{	
		left_elbow_3d = triangulationFromDepth( pose_left_.left_elbow.x, pose_left_.left_elbow.y );
		pose_3d_list.push_back(left_elbow_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

	if( pose_left_.left_wrist.x  != 0 && pose_left_.left_wrist.y != 0 && pose_left_.left_wrist.confidence > 0.5)
	{	
		left_wrist_3d = triangulationFromDepth( pose_left_.left_wrist.x, pose_left_.left_wrist.y );
		pose_3d_list.push_back(left_wrist_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }


	if( pose_left_.right_knee.x  != 0 && pose_left_.right_knee.y != 0 && pose_left_.right_knee.confidence > 0.5)
	{	
		right_knee_3d = triangulationFromDepth( pose_left_.right_knee.x, pose_left_.right_knee.y );
		pose_3d_list.push_back(right_knee_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

	if( pose_left_.right_ankle.x  != 0 && pose_left_.right_ankle.y != 0 && pose_left_.right_ankle.confidence > 0.5)
	{	
		right_ankle_3d = triangulationFromDepth( pose_left_.right_ankle.x, pose_left_.right_ankle.y );
		pose_3d_list.push_back(right_ankle_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

	if( pose_left_.left_knee.x  != 0 && pose_left_.left_knee.y != 0 && pose_left_.left_knee.confidence > 0.5)
	{	
		left_knee_3d = triangulationFromDepth( pose_left_.left_knee.x, pose_left_.left_knee.y );
		pose_3d_list.push_back(left_knee_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

	if( pose_left_.left_ankle.x  != 0 && pose_left_.left_ankle.y != 0 && pose_left_.left_ankle.confidence > 0.5)
	{	
		left_ankle_3d = triangulationFromDepth( pose_left_.left_ankle.x, pose_left_.left_ankle.y );
		pose_3d_list.push_back(left_ankle_3d);
	}else{
                pose_3d_list.push_back(Eigen::Vector3d(NAN, NAN, NAN));
        }

        //imshow("debugDepth_", debugDepth_);
        //cv::waitKey(10);

	vis3dPose(pose_3d_list); 

}

int opRealTimeProcessing()
{

    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);

    // Step 1 - Read Google flags (user defined configuration)
    cv::Size outputSize;
    cv::Size netInputSize;
    cv::Size netOutputSize;
    op::PoseModel poseModel;
    std::tie(outputSize, netInputSize, netOutputSize, poseModel) = gflagsToOpParameters();

    // Step 2 - Initialize all required classes
    op::CvMatToOpInput cvMatToOpInput{netInputSize, FLAGS_num_scales, (float)FLAGS_scale_gap};
    op::CvMatToOpOutput cvMatToOpOutput{outputSize};

    op::PoseExtractorCaffe poseExtractorCaffe{netInputSize, netOutputSize, outputSize, FLAGS_num_scales, (float)FLAGS_scale_gap, poseModel,
                                              FLAGS_model_folder, FLAGS_num_gpu_start};

    op::PoseRenderer poseRenderer{netOutputSize, outputSize, poseModel, nullptr, (float)FLAGS_alpha_pose};
    op::OpOutputToCvMat opOutputToCvMat{outputSize};

    // Step 3 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)

    poseExtractorCaffe.initializationOnThread();

    poseRenderer.initializationOnThread();

    // Step 4 - Initialize the image subscriber
    OpenPoseNode opn;

    //int count = 0;
    //const auto timerBegin = std::chrono::high_resolution_clock::now();
    ros::spinOnce();
    // Step 5 - Continuously process images from image subscriber
    while (ros::ok())
    {   

    const auto timerBegin = std::chrono::high_resolution_clock::now();
        // Step 6 - Get cv_image ptr and check that it is not null
        if( opn.img_l_rcv && opn.img_depth_rcv)
        {   
            cv::Mat img_l = opn.get_l_image();

            
            opn.img_l_rcv = false;
            opn.img_depth_rcv = false;
            //cv::cvtColor(img_l, img_l, CV_GRAY2BGR);
            //continue;
            // Step 7 - Format Input and Output Image
            const auto netInputArray_l = cvMatToOpInput.format(img_l);
            double scaleInputToOutput;
            op::Array<float> outputArray;
            std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput.format(img_l);

            poseExtractorCaffe.forwardPass(netInputArray_l, img_l.size());

            const auto poseKeyPoints_l = poseExtractorCaffe.getPoseKeyPoints();

            auto person_num = poseKeyPoints_l.getSize(0);
            auto pose_num   = poseKeyPoints_l.getSize(1);
            //std::cout<<"person number: "<<person_num<<"pose number: "<<pose_num<<std::endl;
    
            if(person_num > 0){
               cv::Mat cvMatPoses = poseKeyPoints_l.getConstCvMat();



               std_msgs::Float64MultiArray cvMat;
               cvMat.data.clear();
               for(int i=0; i<person_num; i++){
                  for(int j=0; j<18; j++){
                       cvMat.data.push_back(cvMatPoses.at<float>(i,j,0));
                       cvMat.data.push_back(cvMatPoses.at<float>(i,j,1));
                       cvMat.data.push_back(cvMatPoses.at<float>(i,j,2));
                  }
               }               

                               points_vis.markers.clear();



			        for(int i=0; i<person_num; i++){
                                   person_id = i;
                                   person_detection_ = pose_ros_wrapper(cvMatPoses, i);
                                   localize_subject(cvMatPoses, i);
              // Step X - Calculate 3D skeleton by fusing 2D skeleton and stereo depth
                                   
				   calculate_3D_skeleton(person_detection_);



    /*ROS_INFO("%lf", 
    sqrt(pow(person_detection_.left_shoulder.x - person_detection_.right_shoulder.x,2)+pow(person_detection_.left_shoulder.y - person_detection_.right_shoulder.y,2))
    / sqrt(pow(person_detection_.left_hip.y - person_detection_.neck.y,2)+pow(person_detection_.left_hip.x - person_detection_.neck.x,2))
    );*/


				}
	                        pose_depth_pub.publish(points_vis);

                                opn.pose_pub_l.publish(person_detection_);  

                                if(sub_pos.size() == 6 && sub_pos[0] > sub_pos[3]){
                                   double tmp;
                                   tmp = sub_pos[0]; sub_pos[0] = sub_pos[3]; sub_pos[3] = tmp;
                                   tmp = sub_pos[1]; sub_pos[1] = sub_pos[4]; sub_pos[4] = tmp;
                                   tmp = sub_pos[2]; sub_pos[2] = sub_pos[5]; sub_pos[5] = tmp;
                                }

                                std_msgs::Float64MultiArray array;
                                array.data.clear();
                                for(unsigned int i=0; i<sub_pos.size(); i++){
                                    array.data.push_back(sub_pos[i]);
                                }
                                sub_pub.publish(array);
            

            // Step 9 - Render the pose 
            poseRenderer.renderPose(outputArray, poseKeyPoints_l);
            // Step 10 - OpenPose output format to cv::Mat
            auto outputImage_l = opOutputToCvMat.formatToCvMat(outputArray);

            cvMatPose_pub.publish(cvMat); 


            for(unsigned int i=0; i<sub_rect.size()/4;i++){
                 cv::rectangle(outputImage_l, cv::Point(sub_rect[4*i], sub_rect[4*i+1]), cv::Point(sub_rect[4*i+2], sub_rect[4*i+3]), cv::Scalar(0,0,i*100), -1, 0);
            }
            }

            // Step 9 - Render the pose 
            poseRenderer.renderPose(outputArray, poseKeyPoints_l);
            // Step 10 - OpenPose output format to cv::Mat
            auto outputImage_l = opOutputToCvMat.formatToCvMat(outputArray);

            sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, debugDepth_).toImageMsg();
            debug_pub.publish(msg_img);

            sensor_msgs::ImagePtr msg_img2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage_l).toImageMsg();
            img_pub.publish(msg_img2);

/*            
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";
    odom_msg.pose.pose.position.x = v.x();
    odom_msg.pose.pose.position.y = v.y();
    odom_msg.pose.pose.position.z = v.z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    delay_odom_pub.publish(odom_msg);
*/
            //opn.img_l_rcv = false;
            //opn.img_depth_rcv = false;

        }
           
        ros::spinOnce();
        const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-timerBegin).count() * 1e-9;
        cout<<"Runtime: "<<totalTimeSec<<endl;
    }

    // Measuring total time
//    const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-timerBegin).count() * 1e-9;
//    cout<<"Runtime: "<<totalTimeSec<<endl;
//    const auto message = "Real-time pose estimation demo successfully finished. Total time: " + std::to_string(totalTimeSec) + " seconds. " 
//                         + std::to_string(count) + " frames processed. Average FPS is " + std::to_string(count/totalTimeSec);
//    op::log(message, op::Priority::Max);

    return 0;
}

ros::Subscriber image_sub_;
int main(int argc, char** argv)
{
    //google::InitGoogleLogging("openpose_ros_node");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "openpose_ros_node");
  
    return opRealTimeProcessing();
}
