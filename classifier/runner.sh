
### simulator
PATH_TO_YOUR_PIPELINE_CONFIG=/home/ks/robo4-carnd-capstone/classifier/models/ssd_mobilenet_v1_coco_simulator/ssd_mobilenet_v1_coco_simulator.config
PATH_TO_TRAIN_DIR=/home/ks/robo4-carnd-capstone/classifier/models/ssd_mobilenet_v1_coco_simulator/train
PATH_TO_EVAL_DIR=/home/ks/robo4-carnd-capstone/classifier/models/ssd_mobilenet_v1_coco_simulator/eval
PATH_TO_MODEL_DIRECTORY=/home/ks/robo4-carnd-capstone/classifier/models/ssd_mobilenet_v1_coco_simulator


# From the tensorflow/models/research/ directory
python object_detection/train.py \
    --logtostderr \
    --pipeline_config_path=${PATH_TO_YOUR_PIPELINE_CONFIG} \
    --train_dir=${PATH_TO_TRAIN_DIR}

# From the tensorflow/models/research/ directory
python object_detection/eval.py \
    --logtostderr \
    --pipeline_config_path=${PATH_TO_YOUR_PIPELINE_CONFIG} \
    --checkpoint_dir=${PATH_TO_TRAIN_DIR} \
    --eval_dir=${PATH_TO_EVAL_DIR}


tensorboard --logdir=${PATH_TO_MODEL_DIRECTORY}


python export_inference_graph.py \
    --input_type image_tensor \
    --pipeline_config_path ${PATH_TO_YOUR_PIPELINE_CONFIG} \
    --trained_checkpoint_prefix ${PATH_TO_TRAIN_DIR}/model.ckpt-69495 \
    --output_directory ${PATH_TO_MODEL_DIRECTORY}/exported_model_dir





