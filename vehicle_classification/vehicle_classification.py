from pathlib import Path

import torch
import random
import numpy as np
import cv2


from yolov7.models.yolo import Model
from utils.general import check_requirements, set_logging
from utils.google_utils import attempt_download
from utils.torch_utils import select_device

# dependencies = ['torch', 'yaml']
# check_requirements(Path("/content/yolov7/requirements.txt"), exclude=('pycocotools', 'thop'))

def custom(path_or_model='path/to/model.pt', autoshape=True):

    model = torch.load(path_or_model, map_location=torch.device('cpu')) if isinstance(path_or_model, str) else path_or_model  # load checkpoint
    if isinstance(model, dict):
        model = model['ema' if model.get('ema') else 'model']  # load model

    hub_model = Model(model.yaml).to(next(model.parameters()).device)  # create
    hub_model.load_state_dict(model.float().state_dict())  # load state_dict
    hub_model.names = model.names  # class names
    if autoshape:
        hub_model = hub_model.autoshape()  
    device = select_device('0' if torch.cuda.is_available() else 'cpu')  # default to GPU if available
    return hub_model.to(device)

model = custom(path_or_model='../yolo_weights.pt')  # custom example

# Name of the classes according to class indices.
names= ['Truck', 'Motorcycle', 'Car', 'Bus', 'Ambulance']


# Creating random colors for bounding box visualization.
colors = {
    name: [random.randint(0, 255) for _ in range(3)] for i, name in enumerate(names)
}

def plot_detections(frame, df_prediction):

  for i, row in enumerate(df_prediction[0].iterrows()):

    row_index, row_data = row
    (xmin, ymin, xmax, ymax, score, cls_id, cls_name) = row_data.to_list()

    if score < 0.3:
      continue
    
    start_point = (int(xmin), int(ymin))
    end_point = (int(xmax), int(ymax))

    cv2.rectangle(frame, start_point, end_point, color=(0,255,0), thickness=2)
    cv2.putText(frame, str(cls_name + str(np.round(score, 2))), (start_point[0]-3, start_point[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.75, [225, 255, 255], thickness=2,)

  return frame


cap = cv2.VideoCapture('../video1_final.mp4')
detections_frames_list = []

if (cap.isOpened()== False): 
  print("Error opening video stream or file")

while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True: 
    # frames_list.append(frame)
    dim = (640, 640)
    input_img = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA) # resize image

    results = model(input_img)  # batched inference
    results.print()
    # results.save()
    df_prediction = results.pandas().xyxy
    df_prediction

    detections_frames_list.append(plot_detections(frame = input_img, df_prediction= df_prediction))

  else: 
    break
cap.release()
# frames = [detections_frames_list[i] for i in range(0,len(detections_frames_list),1)]
# print(len(detections_frames_list))

# import numpy as np
# import cv2 as cv
cap = cv2.VideoCapture("../video1_final.mp4")
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))   # float `width`
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'MP4V') #codec
out = cv2.VideoWriter('../video1_output_detections.mp4', fourcc, 20.0, (640, 640))
counter=0
for frame in detections_frames_list:
    # ret, frame = cap.read()

    counter+=1
    out.write(frame)
    if cv2.waitKey(1) == ord('q'):
        break
print(counter)
cap.release()
out.release()
cv2.destroyAllWindows()
 

