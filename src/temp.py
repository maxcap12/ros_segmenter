import rospy



def init_params():
    rospy.set_param(
        "~params", 
        {
            "image_params": 
            {
                "image_size": 640
            },
            "ros_topics": 
            {
                "raw_image_topic": "/camera/color/image_raw",
                "segmented_image_topic": "/camera/color/image_segment",
                "depth_image_topic": "/camera/aligned_depth_to_color/image_raw",
                "mapped_images_topic": "/mapped_images",
                "position_topic": "/position"
            },
            "model_params":
            {
                "model_name": "FastSAM",
                "model_path": "~/catkin_ws/src/scene_segment_ros2/src/fastsam/FastSAM/include/FastSAM-s.pt",
                "point_prompt": [[0, 0]],
                "box_prompt": [[0, 0, 0, 0]],
                "text_prompt": "",
                "point_label": [0],

                "iou": 0.9,
                "conf": 0.4,
                "contour": False
            },
            "interpretor_params": 
            {
                "accepted_objects": []
            }
        })
    