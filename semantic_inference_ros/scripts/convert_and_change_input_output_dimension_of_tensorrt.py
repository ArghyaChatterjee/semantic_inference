import tensorrt as trt
import os

def build_engine(onnx_file_path, engine_file_path, input_shape=None):
    # Logger
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

    # Create builder, network, and config
    with trt.Builder(TRT_LOGGER) as builder:
        with builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)) as network:
            with trt.OnnxParser(network, TRT_LOGGER) as parser:
                with builder.create_builder_config() as config:
                    # Parse ONNX model
                    print(f"Loading ONNX file from path {onnx_file_path}")
                    with open(onnx_file_path, "rb") as model:
                        if not parser.parse(model.read()):
                            for error in range(parser.num_errors):
                                print(f"ERROR: {parser.get_error(error)}")
                            return None

                    # Input shape (optional: override dynamic input shape)
                    if input_shape:
                        input_tensor = network.get_input(0)
                        input_tensor.shape = input_shape  # e.g., [1, 3, 1080, 1920]

                    # Optimization profile for dynamic input (if applicable)
                    config.max_workspace_size = 1 << 30  # 1GB workspace
                    config.set_flag(trt.BuilderFlag.FP16)  # Enable FP16 if supported

                    # Build and serialize engine
                    print("Building engine...")
                    engine = builder.build_engine(network, config)
                    with open(engine_file_path, "wb") as f:
                        f.write(engine.serialize())
                    print(f"Serialized engine saved at {engine_file_path}")
                    return engine

# Paths
onnx_path = "/home/arghya/semantic_overlay_ws/src/semantic_inference/semantic_inference/models/ade20k-efficientvit_seg_l2_in_1080p_out_1080p.onnx"
trt_engine_path = "/home/arghya/semantic_overlay_ws/src/semantic_inference/semantic_inference/engines/ade20k-efficientvit_seg_l2_in_1080p_out_1080p.trt"

# Convert ONNX to TensorRT
build_engine(onnx_path, trt_engine_path, input_shape=[1, 3, 1080, 1920])
