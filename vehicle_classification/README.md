Running the vehicle_classification.py code will
  1) Load the YOLO v7 tiny model (yolo_weights.pt) trained on the given vehicle dataset
  2) Read the input video file 'video1_final.mp4'
  3) Run inference on each frame using the trained model
  4) Draw bounding boxes, class names and confidence score
  5) Save the output as a video named 'video1_output_detections.mp4'

- Please run the vehicle_classification.py code from within the yolov7 folder
-Use the below code inside the yolov7 folder to install the requirements for yolov7 
  !pip install -q -r requirements.txt
