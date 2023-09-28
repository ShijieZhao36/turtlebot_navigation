import numpy as np
import cv2
import os

class Detection:
    def process_frame(self,image):
        Width = image.shape[1]
        Height = image.shape[0]
        scale = 0.00392

        classes = None

        print(os.listdir('.'))
        class_file = 'src/yolov/yolov3.txt'
        weights = 'src/yolov/yolo-fastest-1.1-xl.weights'
        config = 'src/yolov/yolo-fastest-1.1-xl.cfg'

        with open(class_file, 'r') as f:
            classes = [line.strip() for line in f.readlines()]

        COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

        net = cv2.dnn.readNet(weights, config)

        blob = cv2.dnn.blobFromImage(image, scale, (320,320), (0,0,0), True, crop=False)

        net.setInput(blob)

        outs = net.forward(self.get_output_layers(net))

        class_ids = []
        confidences = []
        boxes = []
        conf_threshold = 0.5
        nms_threshold = 0.4


        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                label = str(classes[class_id])
                if confidence > 0.5 and label == 'cell phone':
                    center_x = int(detection[0] * Width)
                    center_y = int(detection[1] * Height)
                    w = int(detection[2] * Width)
                    h = int(detection[3] * Height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    print(center_x,center_y)
                    boxes.append([x, y, w, h])


        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

        for i in indices:
            try:
                box = boxes[i]
            except:
                i = i[0]
                box = boxes[i]
            
            x = box[0]
            y = box[1]
            w = box[2]
            h = box[3]
            label = str(classes[class_ids[i]])
            color = COLORS[class_ids[i]]
            self.draw_prediction(image, label, color, round(x), round(y), round(x+w), round(y+h))
        return image
    
    def get_output_layers(self,net):
    
        layer_names = net.getLayerNames()
        try:
            output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
        except:
            output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        return output_layers
    
    def draw_prediction(self,img,label, color, x, y, x_plus_w, y_plus_h):
        cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)
        cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)