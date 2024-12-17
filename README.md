# semantic_inference

<div align="center">
   <img src="docs/media/ade20k_segmentation_efficientvit.png"/>
   <img src="docs/media/ade20k_segmentation_recolor_efficientvit.png"/>
</div>s

This repository provides code for running inference on images with pre-trained models to provide both closed and open-set semantics.
Closed-set and open-set segmentation are implemented as follows:
  - Inference using dense 2D closed-set semantic segmentation models is implemented in c++ using TensorRT
  - Inference using open-set segmentation models and language features is implemented in python

Both kinds of semantic segmentation have a ROS interface associated with them, split between c++ and python as appropriate.

## Getting started
### Requirements
This package has been tested on ubuntu 20.04 with ros noetic, cuda-12.1 and tensrort-8.6.1.

### Making a workspace
First, make sure rosdep is setup:
```bash
# Initialize necessary tools for working with ROS and catkin
sudo apt install python3-catkin-tools python3-rosdep
sudo rosdep init
rosdep update
```
To start, clone this repository into your catkin workspace and run rosdep to get any missing dependencies.
This usually looks like the following:
```bash
mkdir -p ~/semantic_inference_ws/src
cd semantic_inference_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
cd ~/semantic_inference_ws/src
git clone git@github.com:ArghyaChatterjee/semantic_inference.git
rosdep install --from-paths . --ignore-src -r -y
catkin build
```

Semantic Inference package depends on [config_utilities](https://github.com/MIT-SPARK/config_utilities) package. You need to clone that package inside your `~/semantic_inference_ws/src` before building the `semantic_inference` package.

Once you've added this repository to your workspace, follow one (or both) of the following setup-guides as necessary:
- [Closed-Set](docs/closed_set.md#setting-up)
- [Open-Set](docs/open_set.md#setting-up)

## Usage

More details about including these 2 sets in bigger project can be found in the [closed-set](docs/closed_set.md#using-closed-set-segmentation-online) and [open-set](docs/open_set.md#using-open-set-segmentation-online) documentation.

Now, launch the semantic segmentation inference node in the following way:
```
roslaunch semantic_inference_ros semantic_inference.launch
```
Since in the launch file, the subscribed topic is already remapped from `/semantic_inference/color/image_raw` to `/tesse/left_cam/rgb/image_raw`. Play the rosbag:
```
rosbag play ~/uHumans2_office_s1_00h.bag --clock 
```

If not, here is how you can play the rosbag while remapping:
```
rosbag play ~/uHumans2_office_s1_00h.bag --clock /tesse/left_cam/rgb/image_raw:=/semantic_inference/color/image_raw
```

> **Note** </br>
> This usage (remapping the rosbag output topic) is a little bit backwards from how remappings from ROS are normally specified and is because launch files are unable to take remappings from the command line.

List of ros topics being published:
```
/semantic_inference/semantic/image_raw
...
/semantic_inference/semantic_color/image_raw
...
/semantic_inference/semantic_overlay/image_raw
...
/semantic_inference/color/image_raw
```
List of ros nodes being published:
```
/nodelet_manager
/rosout
/semantic_inference
```

List of topics `/semantic_inference` node is publishing:
```
 * /rosout [rosgraph_msgs/Log]
```
List of topics `/semantic_inference` node is subscribing:
```
Subscriptions: None
```
List of topics `/nodelet_manager` node is subscribing:
``` 
 * /semantic_inference/color/image_raw [unknown type]
```
List of topics `/nodelet_manager` node is publishing:
```
 * /rosout [rosgraph_msgs/Log]
 * /semantic_inference/semantic/image_raw [sensor_msgs/Image]
 * /semantic_inference/semantic/image_raw/compressed [sensor_msgs/CompressedImage]
 * /semantic_inference/semantic/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /semantic_inference/semantic/image_raw/compressedDepth [sensor_msgs/CompressedImage]
 * /semantic_inference/semantic/image_raw/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic/image_raw/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /semantic_inference/semantic/image_raw/theora [theora_image_transport/Packet]
 * /semantic_inference/semantic/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /semantic_inference/semantic_color/image_raw [sensor_msgs/Image]
 * /semantic_inference/semantic_color/image_raw/compressed [sensor_msgs/CompressedImage]
 * /semantic_inference/semantic_color/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic_color/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /semantic_inference/semantic_color/image_raw/compressedDepth [sensor_msgs/CompressedImage]
 * /semantic_inference/semantic_color/image_raw/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic_color/image_raw/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /semantic_inference/semantic_color/image_raw/theora [theora_image_transport/Packet]
 * /semantic_inference/semantic_color/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic_color/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /semantic_inference/semantic_overlay/image_raw [sensor_msgs/Image]
 * /semantic_inference/semantic_overlay/image_raw/compressed [sensor_msgs/CompressedImage]
 * /semantic_inference/semantic_overlay/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic_overlay/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /semantic_inference/semantic_overlay/image_raw/compressedDepth [sensor_msgs/CompressedImage]
 * /semantic_inference/semantic_overlay/image_raw/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic_overlay/image_raw/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /semantic_inference/semantic_overlay/image_raw/theora [theora_image_transport/Packet]
 * /semantic_inference/semantic_overlay/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /semantic_inference/semantic_overlay/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
```



