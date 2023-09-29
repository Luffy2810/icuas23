import argparse
import time
from pathlib import Path
import numpy as np
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import sys
sys.path.append('.')
from PIL import Image
from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages,letterbox
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel    
from torchvision import transforms


class mlcv:
    def __init__(self):
        self.weights='/root/yolov7/best.pt'
        self.weights_1='/root/yolov7/classify.pt'
        self.model_1 = torch.load(self.weights_1)
        
        self.view_img=True
        self.image_size=448
        self.trace=True
        self.device='0'
        self.device = select_device(self.device)
        self.augment=False
        self.conf_thres=0.25
        self.iou_thres=0.40
        self.classes=None
        self.agnostic_nms=False
        self.img_size=448
        self.half = self.device.type != 'cpu'

        self.model = attempt_load(self.weights, map_location=self.device) 
        self.stride = int(self.model.stride.max()) 
        imgsz = check_img_size(self.img_size, s=self.stride)  

        if self.trace:
            self.model = TracedModel(self.model, self.device, self.img_size)

        if self.half:
            self.model.half()    
            
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))
        self.model.eval()
        self.model_1.eval().to(self.device)
    def classify(self,img):
        with torch.no_grad():
            t=transforms.Compose([
            transforms.Resize(224),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])
            try:
                img = Image.fromarray(np.uint8(img)).convert('RGB')
                img=t(img)
                img=torch.unsqueeze(img, 0)
                img=img.float().to(self.device)
                _, pred =torch.max(self.model_1(img), 1)

            # if output==1:
            #     cv2.imshow('img', img)
            #     cv2.waitKey(1000) 
                return pred.cpu().numpy()[0]
            except:
                return None
    def detect(self,source):
        t_0=time.time()
        weights, view_img, imgsz, trace = self.weights, self.view_img, self.img_size, self.trace

        # print (self.device)
        # time.sleep(10)
          # run once
        old_img_w = old_img_h = imgsz
        old_img_b = 1

        t0 = time.time()
        img0=source
        img = letterbox(img0, 448, stride=self.stride)[0]
        img = img[:, :, ::-1].transpose(2,0 ,1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)



        t_1=time.time()
        t1 = time_synchronized()
        with torch.no_grad():   
            pred = self.model(img, augment=self.augment)[0]
        t_2=time.time()    
        t2 = time_synchronized()
        img0_copy=img0.copy()

        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)
        t_3=time.time()
        t3 = time_synchronized()
        lst=[]
        coord_lst=[]
        for i, det in enumerate(pred):  
            gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh   
            if len(det):
                # Rescale boxes from img_size to img0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
                # print (img0.shape)
                # time.sleep(5)
                
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum() 
                for *xyxy, conf, cls in reversed(det):
                    x=xyxy
                    c1, c2 = [int(x[0]), int(x[1])], [int(x[2]), int(x[3])]
                    l=c2[0]-c1[0]
                    w=c2[1]-c1[1]
                    if l<0.6*w:
                        l=int(0.6*w)
                    if w<0.6*l:
                        w=0.6*l
                    x_mid=int((c2[0]+c1[0])/2)
                    
                    y_mid=int((c2[1]+c1[1])/2)

                    c1[0]=x_mid-int(0.75*l)
                    c1[0]=max(c1[0],0)
                    c2[0]=c2[0]+int(0.75*l)
                    c2[0]=min(c2[0],img0.shape[0])
                    c1[1]=y_mid-int(0.75*w)
                    c1[1]=max(c1[1],0)
                    c2[1]=y_mid+int(0.75*w)
                    c2[1]=min(c2[1],img0.shape[1])
                    x=[c1[0],c1[1],c2[0],c2[1]]

                    cropped_img=img0[c1[1]:c2[1],c1[0]:c2[0],:]
                    ta=time.time()
                    out=self.classify(cropped_img)
                    tb=time.time()
                    #print (1/(tb-ta))
                    # print (out)
                    # out=1
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    if out==0: 
                        lst.append(cropped_img) # Add bbox to image
                        coord_lst.append([int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])])
                        # print ("drawing")
                        # cv2.putText(img0_copy, str(out), (10,450), font, 3, (0, 255, 0), 2, cv2.LINE_AA)
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        plot_one_box(xyxy, img0_copy, label=label, color=self.colors[int(cls)], line_thickness=2)
                        # plot_one_box(x, img0_copy, label=label, color=self.colors[int(cls)], line_thickness=2)
                    else:
                        # lst.append(cropped_img) # Add bbox to image
                        # cv2.putText(img0_copy, str(out), (10,450), font, 3, (0, 255, 0), 2, cv2.LINE_AA)
                        # print ("drawing")
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        # plot_one_box(xyxy, img0_copy, label=label, color=self.colors[int(cls)], line_thickness=2)
                        # plot_one_box(x, img0_copy, label=label, color=self.colors[int(cls)], line_thickness=2)
            t_4=time.time()
            #print(1/(t_4-t_0),1/(t_3-t_0),1/(t_2-t_0),1/(t_1-t_0))

        if self.view_img:
            # # for i in lst:
            
            cv2.imshow('img', img0_copy)
            cv2.waitKey(1)  # 1 millisecond
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            # pass
        # print (coord_lst)
        return lst,coord_lst,img0_copy


if __name__ == '__main__':
    mlcv=mlcv()
    img=cv2.imread('/home/sahil/icuas23_competition/classify/output/train/non_crack/2_1a.jpeg')
    mlcv.detect(img)
