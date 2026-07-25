model_transform.py \
--model_name yolov8_num_detect_v2 \
--model_def ./export.onnx \
--input_shapes [[1,3,224,320]] \
--mean "0,0,0" \
--scale "0.00392156862745098,0.00392156862745098,0.00392156862745098" \
--keep_aspect_ratio \
--pixel_format rgb \
--channel_format nchw \
--output_names "/model.22/dfl/conv/Conv_output_0,/model.22/Sigmoid_output_0" \
--test_input ./test.jpg \
--test_result yolov8_num_detect_v2_top_outputs.npz \
--tolerance 0.99,0.99 \
--mlir yolov8_num_detect_v2.mlir

