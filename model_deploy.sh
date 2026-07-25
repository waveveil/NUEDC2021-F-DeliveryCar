model_deploy.py \
--mlir yolov8_num_detect_v2.mlir \
--quantize INT8 \--quant_input \
--calibration_table yolov8_num_detect_v2_table \
--processor cv181x \
--test_input yolov8_num_detect_v2_in_f32.npz \
--test_reference yolov8_num_detect_v2_top_outputs.npz \
--tolerance 0.7,0.5 \
--model yolov8_num_detect_v2.cvimodel

