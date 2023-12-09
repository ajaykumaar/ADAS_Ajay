import matplotlib.pyplot as plt
# import seaborn as sns
import numpy as np
import cv2
import pickle

################################# define params   ##############################

#steering angle from video using compass reading
steering= np.array([331, 331, 331, 331, 331, 331, 331, 331, 331, 331, 331, 331, 331, 331, 331, 331, 331, 332, 332, 332, 332, 332, 332, 332, 332, 332, 333, 333, 333, 333, 333, 333, 334, 334, 334, 334, 335, 335, 335, 335, 336, 336, 336, 337, 337, 338, 338, 338, 339, 339, 339, 340, 340, 341, 341, 341, 342, 342, 343, 343, 343, 344, 344, 344, 345, 345, 346, 346, 346, 347, 347, 348, 348, 348, 349, 349, 350, 350, 351, 351, 351, 352, 352, 353, 353, 354, 354, 355, 355, 355, 356, 356, 357, 357, 357, 358, 358, 359, 359, 359, 360, 360, 361, 361, 361, 362, 362, 362, 362, 363, 363, 363, 363, 364, 364, 364, 364, 365, 365, 365, 365, 365, 365, 365, 365, 364, 364, 364, 364, 363, 363, 362, 362, 361, 361, 361, 360, 359, 359, 358, 358, 357, 357, 356, 356, 355, 355, 354, 354, 353, 353, 352, 351, 351, 350, 349, 349, 348, 347, 347, 346, 345, 345, 344, 343, 343, 342, 342, 341, 340, 340, 339, 338, 338, 337, 336, 336, 335, 335, 334, 333, 333, 332, 332, 331, 330, 330, 329, 328, 328, 327, 326, 326, 325, 324, 324, 323, 323, 322, 321, 321, 320, 320, 319, 319, 318 ,318, 317, 317, 316, 316, 315, 315, 315, 314, 314, 314, 313, 313, 313, 312, 312, 312, 311, 311, 311, 311, 311, 310, 310, 310, 310, 310, 310, 310, 310, 310, 310, 310, 310, 310, 311, 311, 311, 312, 312, 312, 312, 313, 313, 314, 314, 315, 316, 316, 317, 318, 318, 319, 320, 320, 321, 321, 322, 323, 323, 324, 324, 325, 325, 326, 326, 327, 328, 328, 328, 329, 329, 330, 330, 330, 331, 331, 332, 332, 332, 333, 333, 333, 333, 334, 334, 334, 334, 334, 334, 334, 335, 335, 335, 335, 335, 335, 335, 335, 335, 335, 335, 335])
steering_angle_array = steering - 340 #setting 340 as 0 deg

steering_angle_array = [ np.deg2rad(s_angle) for s_angle in steering_angle_array]

dt= 0.1 
sim_length= 4 #seconds
time_stamps = np.arange(0, sim_length, dt)
L = 2.69 #m #BMW X1
v = 1.0 #m/s
track_width = 1.58 #m #BMW X1
max_steering_angle = np.deg2rad(35)
min_steering_angle = np.deg2rad(-35)
tire_radius = 0.43/2  #
tire_radius *= 10  

theta_y = np.deg2rad(40)
R_along_y = np.array(([np.cos(theta_y), 0, np.sin(theta_y)], [0, 1, 0], [-np.sin(theta_y), 0, np.cos(theta_y)]))

#camera params
with open('calib_result_1.pickle', 'rb') as handle:
    calib_result=pickle.load(handle)

matrix_0 = calib_result['intr_mat']
distortion_0 = calib_result['dist']

rvec= np.array([0., 0., 0.])
tvec= np.array([3., 1.5, 0.])

############################# functions definition   ###############################

#final equations
def get_rear_wheel_traj(steering_angle):

  delta_xy_dict = {}
  x_traj=[]
  y_traj=[]
  theta_traj=[]
  R_list=[]
  translation_list =[]

  for i in range(len(time_stamps)):

    t = time_stamps[i]
    # steering_angle = steering_angle_array[i]

    if steering_angle != 0:

      theta = (v*t*np.tan(steering_angle))/L
      theta_traj.append(np.rad2deg(theta))

      x = (L * np.sin(theta))/np.tan(steering_angle)
      y = (L * np.cos(theta))/np.tan(steering_angle)

      R= L/np.tan(steering_angle) #radius

    elif steering_angle == 0:
    
      max_theta = (v*t*np.tan(max_steering_angle))/L
      # min_theta = (v*t*np.tan(min_steering_angle))/L

      # theta_traj.append(np.rad2deg(90))

      x = (L * np.sin(max_theta))/np.tan(max_steering_angle)
      y = (L * np.cos(max_theta))/np.tan(max_steering_angle)

      R= L/np.tan(max_steering_angle) #radius

    x_traj.append(x)
    y_traj.append(y)

    R_list.append(R)

    t_O_to_axle = np.array([0, - R])
    translation_list.append(t_O_to_axle)

  #translating from axle center to rear wheels
  axle_x_traj=[]
  axle_y_traj=[]

  for x_pt,y_pt in zip(x_traj,y_traj):
    xy_pt = np.array([x_pt,y_pt])

    axle_xy_pt = xy_pt + t_O_to_axle

    axle_x_traj.append(axle_xy_pt[0])
    axle_y_traj.append(axle_xy_pt[1])

  if steering_angle == 0:

    left_wheel_x_traj = [ (x_pt) for x_pt in axle_x_traj]
    left_wheel_y_traj = [ -(y_pt + (track_width/2)) for y_pt in axle_y_traj]

  else:
    left_wheel_x_traj =  axle_x_traj
    left_wheel_y_traj = [ y_pt - (track_width/2) for y_pt in axle_y_traj]


  right_wheel_x_traj = [ x_pt for x_pt in axle_x_traj]
  right_wheel_y_traj = [ y_pt + (track_width/2) for y_pt in axle_y_traj]

  delta_xy_dict["axle_x_traj"] = axle_x_traj
  delta_xy_dict["axle_y_traj"] = axle_y_traj
  delta_xy_dict["left_wheel_x_traj"] = left_wheel_x_traj
  delta_xy_dict["left_wheel_y_traj"] = left_wheel_y_traj
  delta_xy_dict["right_wheel_x_traj"] = right_wheel_x_traj
  delta_xy_dict["right_wheel_y_traj"] = right_wheel_y_traj

  return delta_xy_dict

#######################################  main code  ##################################



delta_xy_full_dict={}

for steering_angle in steering_angle_array:

  delta_xy_dict = get_rear_wheel_traj(steering_angle = steering_angle)
  delta_xy_full_dict[steering_angle] = delta_xy_dict 

cap = cv2.VideoCapture('video.mp4')

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
# width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))   
# height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# fourcc = cv2.VideoWriter_fourcc(*'MP4V') #codec
# out = cv2.VideoWriter('output_overlay_test_final.mp4', fourcc, 20.0, (700,500))

frame_counter=0
while(cap.isOpened()):
    
    ret, frame = cap.read()
    # frame = cv2.resize(frame, (1200,1599), interpolation = cv2.INTER_AREA)

    # print(frame.shape)
    
    frame = cv2.resize(frame, (1200,1599), interpolation = cv2.INTER_AREA)
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    plot_rear_wheel_dict = delta_xy_full_dict[steering_angle_array[frame_counter]]

    left_xy_wrt_axle = [ [x,y] for [x,y] in zip(plot_rear_wheel_dict["left_wheel_x_traj"],plot_rear_wheel_dict["left_wheel_y_traj"])]
    right_xy_wrt_axle = [ [x,y] for [x,y] in zip(plot_rear_wheel_dict["right_wheel_x_traj"],plot_rear_wheel_dict["right_wheel_y_traj"])]

    left_xyz_wrt_axle = np.array([xy + [-tire_radius] for xy in left_xy_wrt_axle])
    right_xyz_wrt_axle = np.array([xy + [-tire_radius] for xy in right_xy_wrt_axle])

    left_xyz_wrt_cam = np.array([ xyz.reshape((3,1)) for xyz in left_xyz_wrt_axle])
    right_xyz_wrt_cam = np.array([ xyz.reshape((3,1)) for xyz in right_xyz_wrt_axle])

    #unit conversion
    left_xyz_wrt_cam = left_xyz_wrt_cam * 10
    right_xyz_wrt_cam = right_xyz_wrt_cam * 10

    # #translate along z axis
    left_xyz_wrt_cam = left_xyz_wrt_cam.reshape((40,3))
    left_xyz_wrt_cam[:,2] =  left_xyz_wrt_cam[:,2]*0 + 20  ### setting a constant z

    right_xyz_wrt_cam = right_xyz_wrt_cam.reshape((40,3))
    right_xyz_wrt_cam[:,2] = right_xyz_wrt_cam[:,2]*0 + 20 

    ## change the sign of x
    left_xyz_wrt_cam[:,0] =  left_xyz_wrt_cam[:,0]*-1
    right_xyz_wrt_cam[:,0] =  right_xyz_wrt_cam[:,0]*-1

    left_xyz_wrt_cam = np.array([ np.matmul(R_along_y,xyz) for xyz in left_xyz_wrt_cam])
    right_xyz_wrt_cam = np.array([ np.matmul(R_along_y,xyz) for xyz in right_xyz_wrt_cam])

    left_imagePoints,_ = cv2.projectPoints(left_xyz_wrt_cam, rvec, tvec, matrix_0, distortion_0)
    left_imagePoints = np.array(left_imagePoints).reshape((len(left_imagePoints),2))

    right_imagePoints,_ = cv2.projectPoints(right_xyz_wrt_cam, rvec, tvec, matrix_0, distortion_0)
    right_imagePoints = np.array(right_imagePoints).reshape((len(right_imagePoints),2))

    ##### draw points on the image
    for i in range(1, len(left_imagePoints)):
      l_x = int(left_imagePoints[i, 0])
      l_y = int(left_imagePoints[i, 1])

      r_x = int(right_imagePoints[i, 0])
      r_y = int(right_imagePoints[i, 1])
    
      cv2.circle(frame, (l_y,l_x), radius=5, color=(0, 0, 255), thickness=-1)
      cv2.circle(frame, (r_y, r_x), radius=5, color=(0, 255, 0), thickness=-1)


    frame_counter+=1


    # plt.close()
    frame = cv2.resize(frame, (700,500), interpolation = cv2.INTER_AREA)
    # out.write(frame)
    cv2.imshow("frame", frame)

    if cv2.waitKey(1) == ord('q'):
      break

# print(frame_counter)
cap.release()
# out.release()
cv2.destroyAllWindows()
  

