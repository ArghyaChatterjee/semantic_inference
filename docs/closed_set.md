# Closed-Set Segmentation

## Setting up

### Getting Dependencies

Using dense 2D (closed-set) semantic-segmentation models requires CUDA and TensorRT.
The process for installing CUDA and TensorRT varies by system.
Most users should follow NVIDIA's documentation [here](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html).

> **Note**<br>
> The packages in this repository will compile without TensorRT and CUDA, and some functionality (e.g., remapping label images and visualizing the results) is available without TensorRT and CUDA.
> If you do not have a NVIDIA GPU, you can skip straight to [building](#building).

## TensorRT with Cuda Installation:
Thisi repo was built with tensorrt- 8.6.1 and cuda-12.1. Download TensorRT-8.6.1 tar file from the website, extract it and set the path like this in your `.bashrc` file.
```
export TENSORRT_INCLUDE_DIR=/home/user/TensorRT-8.6.1.6-cuda-12.1/TensorRT-8.6.1.6/include
export TENSORRT_LIBRARY_DIR=/home/user/TensorRT-8.6.1.6-cuda-12.1/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib
export LD_LIBRARY_PATH=${TENSORRT_LIBRARY_DIR}:${LD_LIBRARY_PATH}
```
The cuda path looks like the following:
```
export PATH=/usr/local/cuda-12.1/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
### Building

Once the necessary dependencies are installed and this repository has been placed in a workspace, run the following:
```
catkin build
```

You can run the following to make sure everything is working:
```
catkin test semantic_inference
```

### Models

Running dense 2D semantic segmentation requires obtaining a pre-trained model.
Several pre-exported models live [here](https://drive.google.com/drive/folders/1GrmgFDFCssDxKe_Nyx8PPTK1pRMA0gEO?usp=sharing).
By default, the code uses [this](https://drive.google.com/file/d/1XRcsyLSvqqhqNIaOI_vmqpUpmBT6gk9-/view?usp=drive_link) model.

> **Note** <br>
> We recommend using models within the [dense2d](https://drive.google.com/drive/folders/17p_ZZIxI9jI_3GjjtbMijC2WFnc9Bz-a?usp=sharing) folder, which are named corresponding to the labelspace they output to.
> The top-level models are deprecated as they do not follow this naming scheme (they all output to the ade20k label space).

All models should be placed in the `models` directory of the `semantic_inference` package in local clone of the repository.
To use a specific downloaded model, use the argument `model_name:=MODEL_NAME` when running the appropriate launch file (where `MODEL_NAME` is the filename of the model to use minus the extension).

Note that the pipeline as implemented works with any pre-trained model exported to [onnx](https://onnx.ai/) as long as the model takes 32-bit float 3-channel tensors as input and outputs labels in a single-channel tensor as integers.
The pipeline can optionally rescale and offset the input tensor.
See [here](exporting.md) for details on exporting a new model.

### Python utilities

You may find it useful to set up some of the included model utilities. From the top-level of this repository, run:
```
python3 -m virtualenv /path/to/new/environment
source /path/to/new/environment/bin/activate
pip install ./semantic_inference
```

## Using closed-set segmentation online

To use the open-set segmentation as part of a larger system, include [semantic_inference.launch](../semantic_inference_ros/launch/semantic_inference.launch) in your launch file. Often this will look like this:
```xml
<launch>

    <!-- ... rest of launch file ... -->

    <remap from="semantic_inference/color/image_raw" to="YOUR_INPUT_TOPIC_HERE"/>
    <include file="$(find semantic_inference_ros)/launch/semantic_inference.launch"/>

</launch>
```

## Adding New Datasets

To adapt to a new dataset (or to make a new grouping of labels), you will have to:

  - Make a new grouping config (see [this](config/label_groupings/ade150_outdoor.yaml) or [this](config/label_groupings/ade150_indoor.yaml) for examples)
  - Pass in the appropriate arguments to the launch file (`labelspace_name`)

You can view the groupings for a particular labelspace by running `semantic_inference labelspace compare`.
For a grouping of the ade20k labelspace:
```bash
source ~/path/to/environment/bin/activate
cd path/to/semantic_inference
semantic_inference labelspace compare resources/ade20k_categories.csv config/label_groupings/ade150_indoor.yaml
```
