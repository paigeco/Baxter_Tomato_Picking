from tomato_detector import DetectTomatos
import cv2
import pyrealsense2 as rs
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
global picked

picked = False
def findTomato(_):
    print("Recieved detection job")
    tomato_position = Pose()

    # cap = cv2.VideoCapture(0)
    dt = DetectTomatos()

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)

    N = 100
    t_stor = np.zeros((3,N))
    counter = 0

    #input("Press enter when ready to begin!     ")

    while(1):
        if counter > (N-1):
            counter = 0
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        if not aligned_depth_frame or not color_frame:
            continue


        #M = cv2.moments(mask)
        #if M["m00"] != 0:
        targets, img = dt.find_tomatos(color_image)
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', img)
        
        for target in targets:
            #cv2.imshow('area', target['area'])
            center_xyz = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [target['center'][1], target['center'][0]],
                                                        depth_image[int(target['center'][1]), int(target['center'][0])])
            if not( np.any(np.array(center_xyz) == 0)):
                t_stor[:,counter] = np.array(center_xyz)
            
            #Less than 1% error
            if np.max(np.std(np.array(t_stor).T,axis=0)/(np.mean(np.array(t_stor).T,axis=0))+0.00000000001) < 0.1:
                cv2.destroyAllWindows()
                mean_position = np.mean(np.array(t_stor).T,axis=0)
                tomato_position.position.x = mean_position[2]/1000
                tomato_position.position.y = -mean_position[1]/1000
                tomato_position.position.z = -mean_position[0]/1000
                tomato_position_cmd.publish(tomato_position)
                print("Tomato found at: ", tomato_position)
                picked = True
                return

            
        
        counter +=1
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    print("This is the tomato detector window: ")
    rospy.init_node("tomato_finder", anonymous=True)
    tomato_position_cmd = rospy.Publisher("/tomato_position", Pose, queue_size=1)
    rospy.Subscriber("/find_tomato_flag", String, findTomato)
    while not rospy.is_shutdown() and (not picked):
        rospy.sleep(0.1)
