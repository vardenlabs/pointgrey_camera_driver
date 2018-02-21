#include <ros/ros.h>

#include <string>
#include <iostream>
#include <math.h>
#include <map>
#include <cassert>
#include <vector>
#include <string>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <time.h>
#include <boost/program_options.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

/**
* Node that applies a transformation to images.  Currently the transformation is used to rotate images 180 degrees.
**/

using namespace boost::program_options;
namespace enc = sensor_msgs::image_encodings;
typedef boost::function<void (sensor_msgs::Image::ConstPtr&)> PublishFunction_t;

struct CameraTransform{
    std::string topic;
    std::string transformed_topic;
};

class CameraTransformerNode {

public:
    std::vector<ros::Publisher> _publishers;
    std::vector<ros::Subscriber> _subscribers;
    std::vector<boost::function<void (const sensor_msgs::Image::ConstPtr&)>> _subscriber_callbacks;
    std::vector<CameraTransform> _transforms;
    /**
    *   Constructor for Object detector node
    *
    *   nh  -   ros node handle
    *   model_file  -   location of the object detectors model file
    *   transformation_file     -   location of the transformation file containing
    *                               the transformation from image to applanix
    *   cameras     -   list of cameras to subscribe to ("1", "2", etc)
    **/
    CameraTransformerNode(ros::NodeHandle* nh,
                          std::vector<CameraTransform>& transforms):
    _transforms(transforms)
    {
        // load parameters from the parameter server


        // initialize publishers publishers and subscribe topics
        
        for(int i =0; i < _transforms.size(); i++){
            const CameraTransform transform = _transforms[i];
            const ros::Publisher publisher = nh->advertise<sensor_msgs::Image>(transform.transformed_topic, 5);
            _publishers.push_back(publisher);
            
            boost::function<void (const sensor_msgs::Image::ConstPtr&)> callback = boost::bind(&CameraTransformerNode::transform_image, 
                                                                                                        this, 
                                                                                                        i, 
                                                                                                        _1);    

            _subscriber_callbacks.push_back(callback);
            const ros::Subscriber subscriber = nh->subscribe(transform.topic,
                                            10,
                                            _subscriber_callbacks[i]);            
            _subscribers.push_back(subscriber);
            std::cout << "topic: " << transform.topic << " transformed_topic: " << transform.transformed_topic << std::endl;
        }

        // intialize the camera synchronizer to the camera topics and bind the publish call back
    }


    /**
    *   gets a cvImagePtr from a compressed image message
    **/
    boost::optional<cv::Mat> getImageFromMessage(const sensor_msgs::Image::ConstPtr& image_msg){
        // create a cv_ptr sharing image data
        cv_bridge::CvImageConstPtr cv_ptr;

        try{
          cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& e){
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return boost::none;
        }
        // extract image and segment lanes
        cv::Mat image = cv_ptr->image;
        cv::Mat rgb;
        cv::cvtColor(image, rgb, CV_BGR2RGB);
        boost::optional<cv::Mat> image_op = rgb;
        return image_op;
    }

    sensor_msgs::Image createImageMessage(const sensor_msgs::Image::ConstPtr& image_msg, cv::Mat transformed){
        cv_bridge::CvImage transformed_bridge;
        sensor_msgs::Image transformed_msg; // >> message to be sent

        std_msgs::Header header; // empty header
        header.seq = image_msg->header.seq; // user defined counter
        header.stamp = image_msg->header.stamp; // time
        transformed_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, transformed);
        transformed_bridge.toImageMsg(transformed_msg); // from cv_bridge to sensor_msgs::Image
        
        return transformed_msg;
    }    

    /**
    *   Transforms the image message according to the transform at index.
    *   Currently each image is rotated 180 degrees.  But the CameraTransform struct can
    *   be updated to supported more complicated transformations.
    **/
    void transform_image(int i, const sensor_msgs::Image::ConstPtr& image_msg){
        // std::cout << "got image : " << i << std::endl;
        const CameraTransform transform = _transforms[i];
        boost::optional<cv::Mat> image_op;
        try{
           image_op = this->getImageFromMessage(image_msg);
        }catch (cv_bridge::Exception& e){
           ROS_ERROR("cv_bridge exception: %s", e.what());
           return;          
        }
        cv::Mat image = *image_op;

        // rotate the image 180 degreee
        flip(image, image, -1);

        sensor_msgs::Image transformed_msg = createImageMessage(image_msg, image);
        _publishers[i].publish(transformed_msg);
    }
};
int main (int argc, char **argv)
{
    variables_map vm;
    try
    {
        options_description desc{"Options"};
        desc.add_options()
        ("help,h", "help")
        ("camera", value<std::vector<std::string>>()->multitoken(), "repeated argument.  should be in the format <image topic> <transformed image topic> <rotation>");
        parsed_options parsed_options = parse_command_line(argc, argv, desc);


       std::vector<std::vector<std::string>> camera_args;
       for (const option& o : parsed_options.options) {
          if (o.string_key == "camera"){
             camera_args.push_back(o.value);
          }
       }

        store(parsed_options, vm);
        notify(vm);

        if (vm.count("help")) {
            std::cout << desc << '\n';
            exit(1);
        } 

        std::vector<CameraTransform> camera_transforms;
        for(std::vector<std::string> arg_list : camera_args){
            if(arg_list.size() < 2){
                printf("Camera arg must have at least 1 values\n");
                exit(1);
            }
            std::string image_topic = arg_list[0];
            std::string transform_topic = arg_list[1];
            CameraTransform transform = {image_topic, transform_topic};
            camera_transforms.push_back(transform);
        }

        // initialize ros
        ros::init(argc, argv, "camera_transformer_node");
        ros::NodeHandle node("");

        CameraTransformerNode object_detector_node(&node,
                                                   camera_transforms);


        ros::spin();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

  return 0;
}
