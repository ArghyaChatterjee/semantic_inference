# semantic_inference

<div align="center">
   <img src="docs/media/demo_segmentation.png"/>
</div>

This repository provides code for running inference on images with pre-trained models to provide both closed and open-set semantics.
Closed-set and open-set segmentation are implemented as follows:
  - Inference using dense 2D closed-set semantic segmentation models is implemented in c++ using TensorRT
  - Inference using open-set segmentation models and language features is implemented in python

Both kinds of semantic segmentation have a ROS interface associated with them, split between c++ and python as appropriate.

## Getting started
To start, clone this repository into your catkin workspace and run rosdep to get any missing dependencies.
This usually looks like the following:
```bash
cd ~/semantic_inference_ws/src
git clone git@github.com:ArghyaChatterjee/semantic_inference.git
rosdep install --from-paths . --ignore-src -r -y
```

Semantic Inference package depends on `config_utilities`[https://github.com/MIT-SPARK/config_utilities] package. You need to clone that package inside your `~/semantic_inference_ws/src` before building the `semantic_inference` package.

# Making a workspace
First, make sure rosdep is setup:
```bash
# Initialize necessary tools for working with ROS and catkin
sudo apt install python3-catkin-tools python3-rosdep
sudo rosdep init
rosdep update
```

Then, make the workspace and initialize it:
```bash
# Setup the workspace
mkdir -p ~/semantic_inference_ws/src
cd semantic_inference_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

</details>

Once you've added this repository to your workspace, follow one (or both) of the following setup-guides as necessary:
- [Closed-Set](docs/closed_set.md#setting-up)
- [Open-Set](docs/open_set.md#setting-up)

## Usage

`semantic_inference` is not intended for standalone usage. Instead, the intention is for the launch files in `semantic_inference` to be used in a larger project.

More details about including them can be found in the [closed-set](docs/closed_set.md#using-closed-set-segmentation-online) and [open-set](docs/open_set.md#using-open-set-segmentation-online) documentation.

However, launch the semantic segmentation inference node in the following way:
```
roslaunch semantic_inference_ros semantic_inference.launch
```
and then
```
rosbag play path/to/rosbag /some/color/image/topic:=/semantic_inference/color/image_raw
```
in a separate terminal to quickly test a particular segmentation model.

> **Note** </br>
> This usage (remapping the rosbag output topic) is a little bit backwards from how remappings from ROS are normally specified and is because launch files are unable to take remappings from the command line.
