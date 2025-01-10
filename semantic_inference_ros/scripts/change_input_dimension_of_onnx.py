import onnx
from onnx import helper, shape_inference

# Load the model
model_path = "/home/arghya/semantic_overlay_ws/src/semantic_inference/semantic_inference/models/ade20k-efficientvit_seg_l2.onnx"
model = onnx.load(model_path)

# Update input shape
input_name = "input"  # Replace with your model's actual input name
for input_tensor in model.graph.input:
    if input_tensor.name == input_name:
        input_tensor.type.tensor_type.shape.dim[2].dim_value = 1080  # Height
        input_tensor.type.tensor_type.shape.dim[3].dim_value = 1920  # Width

# Save the updated model
updated_model_path = "/home/arghya/semantic_overlay_ws/src/semantic_inference/semantic_inference/models/ade20k-efficientvit_seg_l2_1080_1920.onnx"
onnx.save(model, updated_model_path)

# Optional: Run shape inference to validate
inferred_model = shape_inference.infer_shapes(onnx.load(updated_model_path))
onnx.save(inferred_model, updated_model_path)
