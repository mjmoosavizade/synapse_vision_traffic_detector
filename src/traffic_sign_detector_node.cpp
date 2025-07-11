// synapse_vision_traffic_detector/src/traffic_sign_detector_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp" // For receiving image data
#include "cv_bridge/cv_bridge.h"     // For converting ROS Image to OpenCV Mat
#include "opencv2/opencv.hpp"        // Core OpenCV functions
#include "opencv2/dnn.hpp"           // OpenCV's Deep Neural Network module

// --- FIX: Corrected the include path to match the package name ---
#include "synapse_vision_traffic_detector/msg/traffic_sign.hpp" // Our custom message

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm> // For std::max and std::min

using namespace std::chrono_literals;

/*
 * Detection Struct:
 * A simple structure to hold the processed information for a single detected object.
 */
struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
};

/*
 * TrafficSignDetector Class:
 * This ROS 2 C++ node subscribes to image topics, performs traffic sign detection
 * using a YOLOv8 ONNX model via OpenCV's DNN module, and publishes the results
 * as custom TrafficSign messages.
 */
class TrafficSignDetector : public rclcpp::Node
{
public:
    /*
     * Constructor:
     * Initializes the ROS 2 node, declares and loads parameters,
     * loads the ONNX model, and sets up publishers and subscribers.
     */
    TrafficSignDetector()
    : Node("traffic_sign_detector_node")
    {
        // 1. Declare ROS 2 parameters for configuration
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<std::vector<std::string>>("class_names", std::vector<std::string>());
        this->declare_parameter<double>("confidence_threshold", 0.25);
        this->declare_parameter<double>("nms_threshold", 0.45);
        this->declare_parameter<int>("input_width", 640);
        this->declare_parameter<int>("input_height", 640);

        // 2. Load parameters from params.yaml
        model_path_ = this->get_parameter("model_path").as_string();
        class_names_ = this->get_parameter("class_names").as_string_array();
        confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
        nms_threshold_ = this->get_parameter("nms_threshold").as_double();
        input_width_ = this->get_parameter("input_width").as_int();
        input_height_ = this->get_parameter("input_height").as_int();

        // Log loaded parameters for verification
        RCLCPP_INFO(this->get_logger(), "Node parameters loaded:");
        RCLCPP_INFO(this->get_logger(), " - Model Path: %s", model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), " - Confidence Threshold: %.2f", confidence_threshold_);
        RCLCPP_INFO(this->get_logger(), " - NMS Threshold: %.2f", nms_threshold_);
        RCLCPP_INFO(this->get_logger(), " - Input Size: %dx%d", input_width_, input_height_);
        RCLCPP_INFO(this->get_logger(), " - Number of Classes: %zu", class_names_.size());
        if (class_names_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Class names list is empty! Detections will show generic IDs.");
        }

        // 3. Load the ONNX model using OpenCV DNN
        if (model_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Model path is empty. Please set 'model_path' parameter.");
            rclcpp::shutdown();
            return;
        }

        try {
            net_ = cv::dnn::readNetFromONNX(model_path_);
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            RCLCPP_INFO(this->get_logger(), "YOLOv8 ONNX model loaded successfully from %s", model_path_.c_str());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 4. Create a subscriber for incoming image messages
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&TrafficSignDetector::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: /camera/image_raw");

        // 5. Create a publisher for our custom TrafficSign detection messages
        publisher_ = this->create_publisher<synapse_vision_traffic_detector::msg::TrafficSign>(
            "traffic_sign_detection/output",
            10);
        RCLCPP_INFO(this->get_logger(), "Publishing detection results to topic: /traffic_sign_detection/output");
    }

private:
    cv::dnn::Net net_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<synapse_vision_traffic_detector::msg::TrafficSign>::SharedPtr publisher_;

    std::string model_path_;
    std::vector<std::string> class_names_;
    double confidence_threshold_;
    double nms_threshold_;
    int input_width_;
    int input_height_;

    /*
     * image_callback:
     * Callback function executed when a new image message is received.
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image frame.");
            return;
        }

        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(input_width_, input_height_),
                               cv::Scalar(), true, false);

        net_.setInput(blob);

        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());

        std::vector<Detection> detections = postprocess(frame, outputs);

        for (const auto& det : detections)
        {
            auto traffic_sign_msg = synapse_vision_traffic_detector::msg::TrafficSign();
            traffic_sign_msg.header = msg->header;

            if (det.class_id >= 0 && det.class_id < static_cast<int>(class_names_.size())) {
                traffic_sign_msg.sign_type = class_names_[det.class_id];
            } else {
                traffic_sign_msg.sign_type = "Unknown_Class_ID_" + std::to_string(det.class_id);
            }
            traffic_sign_msg.confidence = det.confidence;
            traffic_sign_msg.bbox_x_min = det.box.x;
            traffic_sign_msg.bbox_y_min = det.box.y;
            traffic_sign_msg.bbox_width = det.box.width;
            traffic_sign_msg.bbox_height = det.box.height;
            traffic_sign_msg.model_name = "YOLOv8_Reyga_Traffic_Signs";

            publisher_->publish(traffic_sign_msg);
        }
    }

    /*
     * postprocess:
     * Processes the raw output from the YOLOv8 ONNX model.
     */
    std::vector<Detection> postprocess(const cv::Mat& frame, const std::vector<cv::Mat>& outputs)
    {
        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        if (outputs.empty() || outputs[0].empty()) {
            RCLCPP_ERROR(this->get_logger(), "DNN output is empty.");
            return {};
        }

        // The output from the model has a shape of (1, 38, 8400).
        // We need to transpose this to (8400, 38) to process each of the 8400 detection proposals.
        cv::Mat output_data = outputs[0].reshape(1, outputs[0].size[1]);
        cv::transpose(output_data, output_data); // Transpose from (38, 8400) to (8400, 38)

        int num_boxes = output_data.rows;
        int num_features = output_data.cols;

        if (num_features != (4 + class_names_.size())) {
            RCLCPP_ERROR(this->get_logger(), "YOLOv8 output feature count mismatch! Expected %zu, but got %d.",
                         (4 + class_names_.size()), num_features);
            return {};
        }

        float x_factor = static_cast<float>(frame.cols) / input_width_;
        float y_factor = static_cast<float>(frame.rows) / input_height_;

        for (int i = 0; i < num_boxes; ++i)
        {
            cv::Mat current_proposal = output_data.row(i);
            cv::Mat class_scores_row = current_proposal.colRange(4, num_features);
            double max_conf;
            cv::Point class_id_point;
            cv::minMaxLoc(class_scores_row, nullptr, &max_conf, nullptr, &class_id_point);

            if (max_conf > confidence_threshold_)
            {
                float center_x = current_proposal.at<float>(0);
                float center_y = current_proposal.at<float>(1);
                float box_width = current_proposal.at<float>(2);
                float box_height = current_proposal.at<float>(3);

                int x_min = static_cast<int>((center_x - box_width / 2.0f) * x_factor);
                int y_min = static_cast<int>((center_y - box_height / 2.0f) * y_factor);
                int width = static_cast<int>(box_width * x_factor);
                int height = static_cast<int>(box_height * y_factor);

                class_ids.push_back(class_id_point.x);
                confidences.push_back(static_cast<float>(max_conf));
                boxes.push_back(cv::Rect(x_min, y_min, width, height));
            }
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);

        std::vector<Detection> final_detections;
        final_detections.reserve(indices.size());
        for (int idx : indices)
        {
            Detection det;
            det.class_id = class_ids[idx];
            det.confidence = confidences[idx];
            det.box = boxes[idx];

            det.box.x = std::max(0, det.box.x);
            det.box.y = std::max(0, det.box.y);
            det.box.width = std::min(frame.cols - det.box.x, det.box.width);
            det.box.height = std::min(frame.rows - det.box.y, det.box.height);

            final_detections.push_back(det);
        }

        return final_detections;
    }
};

/*
 * main function:
 * Entry point for the ROS 2 node.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrafficSignDetector>();
    RCLCPP_INFO(node->get_logger(), "TrafficSignDetector Node Starting...");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "TrafficSignDetector Node Shutting Down.");
    rclcpp::shutdown();
    return 0;
}
