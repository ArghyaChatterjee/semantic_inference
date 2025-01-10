import onnx

# Load the ONNX model
# model_path = "/home/arghya/semantic_overlay_ws/src/semantic_inference/semantic_inference/models/ade20k-efficientvit_seg_l2_in_1080p_out_1080p.onnx"
model_path = "/home/arghya/semantic_overlay_ws/src/semantic_inference/semantic_inference/models/ade20k-efficientvit_seg_l2.onnx"
model = onnx.load(model_path)

# Inspect inputs
graph = model.graph
print("Inputs:")
for input_tensor in graph.input:
    dims = [dim.dim_value for dim in input_tensor.type.tensor_type.shape.dim]
    print(f"  Name: {input_tensor.name}")
    print(f"  Shape: {dims}")

# Inspect outputs
print("\nOutputs:")
for output_tensor in graph.output:
    dims = [dim.dim_value for dim in output_tensor.type.tensor_type.shape.dim]
    print(f"  Name: {output_tensor.name}")
    print(f"  Shape: {dims}")
