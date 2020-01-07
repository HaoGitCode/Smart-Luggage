# import all the library
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import socket
import time

# Global variables
global clothcolor, peoplenum, position, finalarea, distance
peoplenum = 0
clothcolor = 0
position = [0,0]
finalarea = 0
distance = 0

def camera():
    global clothcolor, peoplenum, position, finalarea, distance
    targetarea = 0
    firstframe = 0
    people = False
    targetx = 0
    targety = 0
    area = 0  # area of target people

    # depth camera
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # Start streaming
    pipeline.start(config)
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
            images = depth_colormap

            # color image
            img = cv2.VideoCapture(4)
            ret, img = img.read()
            img = cv2.resize(img, None, fx=1, fy=1)
            height, width, channels = img.shape

            # Detecting objects
            blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)

            # Showing informations on the screen
            class_ids = []
            confidences = []
            boxes = []
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        # Object detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            font = cv2.FONT_HERSHEY_PLAIN
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    color = colors[i]
                    if label == "person":
                        people = True
                        peoplenum = peoplenum + 1
                        color = colors[i]
                        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                        cv2.putText(img, label, (x, y + 30), font, 3, color, 3)

                        # get the cloth color
                        area = w * h
                        if targetarea < area:
                            targetarea = area
                            targetx = int((x + w) * 0.5)
                            targety = int((y + h) * 0.5)

            if peoplenum == 0:
                targetx = 0
                targety = 0

            if firstframe == 0 and people == True:
                firstframe = 1
                clothcolor = img[targetx, targety]

            # get the average distance from depth camera on a selected area
            tempnum = 0
            distance = 0
            for i in range(targetx-5,targetx+5):
                for j in range(targety-5,targety+5):
                    if depth_image[i][j] == 0:
                        continue
                    tempnum = tempnum + 1
                    distance = distance + depth_image[i][j]
            if tempnum != 0:
                distance = distance/tempnum
            else:
                distance = 0

            # The centroid of the target people
            position = [targetx, targety]
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.imshow("Image", img)
            peoplenum = 0
            finalarea = targetarea
            targetarea = 0
            cv2.waitKey(1)

    finally:

        # Stop streaming
        pipeline.stop()

# send the control signal to nano from laptop
def control():
    global clothcolor, peoplenum, position, finalarea, distance
    # IP address for the host
    host = "10.7.88.88"
    port = 9000
    s = socket.socket()
    s.connect((host, port))
    
    #Note: the command sent to the nano consists of two characters, the first one is responsible for controlling forward. The second one is responsible for rotating.
    
    # when there is no people in the scene
    while 1:
        time.sleep(1)
        pos = position[0]
        if pos == 0:
            last_message = 'ss'
            print(last_message)
            s.send(last_message.encode('utf-8'))
            continue

        # when the distance is far, 'f' is sent to the nano which represent forward.
        if distance > 2000:
            print("forward")
            last_message1 = 'f'
        else:
        # 's' means no instruction would be executed.
            print("stay")
            last_message1 = 's'

        # when the centroid of target would be placed on the right side of the image, 'l' is sent and the robot would turn left. On the other hand, 'r' is sent and the robot would turn right. Otherwise, 's' is sent and robot will do nothing.
        if pos > 250:
            print("rotateleft")
            last_message2 = 'l'
        elif pos < 190:
            print("rotateright")
            last_message2 = 'r'
        else:
            print("center")
            last_message2 = 's'

        # The final command is consist of the first and the second command
        last_message = last_message1 + last_message2
        print(last_message)
        s.send(last_message.encode('utf-8'))
    s.close()


if __name__ == "__main__":
    # Load Yolo
    net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    # start the threads
    t1 = threading.Thread(target=camera)
    t2 = threading.Thread(target=control)
    t1.start()
    t2.start()
