import tensorrt as trt

def inspect_trt_model(trt_file_path):
    with open(trt_file_path, 'rb') as f, trt.Runtime(trt.Logger()) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())
        for binding in engine:
            if engine.binding_is_input(binding):
                print(f"Input Name: {binding}")
                print(f"Input Shape: {engine.get_binding_shape(binding)}")
            else:
                print(f"Output Name: {binding}")
                print(f"Output Shape: {engine.get_binding_shape(binding)}")

trt_file = "/home/arghya/semantic_overlay_ws/src/semantic_inference/semantic_inference/engines/ade20k-efficientvit_seg_l2_in_1080p_out_1080p.trt"
inspect_trt_model(trt_file)
