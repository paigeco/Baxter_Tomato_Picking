# Standard imports
import cv2
import numpy as np

class DetectTomatos:
    def __init__(self):
        self.snipping_offset = 5

        self.fruit_mask = [
            np.array([0,30,40])*2.55,
            np.array([5,90,90])*2.55,
            np.array([95,30,40])*2.55,
            np.array([100,90,90])*2.55
        ]
        self.kernel = np.ones((10, 10), np.uint8)
        h_params = self.hole_patch_params()
        t_params = self.tomato_params()

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            self.hole_detector = cv2.SimpleBlobDetector(h_params)
            self.tomato_detector = cv2.SimpleBlobDetector(t_params)
        else : 
            self.hole_detector = cv2.SimpleBlobDetector_create(h_params)
            self.tomato_detector = cv2.SimpleBlobDetector_create(t_params)

    def tomato_params(self):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()# Set the threshold
        params.minThreshold = 10
        params.maxThreshold = 200

        # Set the area filter
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 100000

        # Set the circularity filter
        params.filterByCircularity = True
        params.minCircularity = 0.1
        params.maxCircularity = 1

        # Set the convexity filter
        params.filterByConvexity = True
        params.minConvexity = 0.01
        params.maxConvexity = 1

        # Set the inertia filter
        params.filterByInertia = True
        params.minInertiaRatio = 0.01
        params.maxInertiaRatio = 1
        return params
    
    def hole_patch_params(self):
        params = cv2.SimpleBlobDetector_Params()

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1500
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.01
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.01
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01
        
        return params

    def fill_holes(self, img):
        mask = self.mask(img)
        #M = cv2.moments(mask)
        mask = cv2.dilate(mask, self.kernel, iterations=2)
        mask = cv2.erode(mask, self.kernel, iterations=2)
        #if M["m00"] != 0:
        #    print(int(M["m10"] / (M["m00"])+1), int(M["m01"] / (M["m00"]+1)))
        #gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    
        # Detect blobs.
        keypoints = self.tomato_detector.detect(mask)
        ff = mask.copy()

        for key in keypoints:
            ff=cv2.circle(ff, (int(key.pt[0]),int(key.pt[1])), radius=(int(key.size/2)), color=(255), thickness=-1)
        
        mask = cv2.dilate(ff, self.kernel, iterations=2)
        mask = cv2.erode(mask, self.kernel, iterations=2)
        return ff

    def mask(self, img):
        img = cv2.blur(img,ksize=(20,20))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        maskl = cv2.inRange(hsv, self.fruit_mask[0], self.fruit_mask[1])
        maskh = cv2.inRange(hsv, self.fruit_mask[2], self.fruit_mask[3])
        
        return cv2.bitwise_or(maskl, maskh)

    def find_tomatos(self, img):
        filled_mask = 255-self.fill_holes(img)
        keypoints = self.tomato_detector.detect(filled_mask)
        im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        im = im_with_keypoints.copy()
        targets = []
        for key in keypoints:
            tgt = {'size':key.size,'center':np.array(key.pt).astype(float),'2Dtarget':(np.array(key.pt[0]).astype(int),np.array(key.pt[1]-((key.size*0.5)+self.snipping_offset)).astype(int))}
            xli = np.array([key.pt[0]-key.size*0.5, key.pt[0]+key.size*0.5]).astype(int)
            yli = np.array([key.pt[1]-key.size*0.5, key.pt[1]+key.size*0.5]).astype(int)
            tgt['area'] = np.array(filled_mask[yli[0]:yli[1],xli[0]:xli[1]])
            targets.append(tgt)
            #print(tgt['center'])
            im=cv2.circle(im, ([int(tgt['center'][0]),int(tgt['center'][1])]), radius=10, color=(255, 192, 203), thickness=-1)
        return targets, im


