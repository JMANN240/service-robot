import os
import sys
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn

import rospy
from std_msgs.msg import String
pub = rospy.Publisher('stop_sign', String, queue_size=10)

#from sensor_msgs.msg import Image

#from cv_bridge import CvBridge

print("Starting")

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

print(1)
print(ROOT)

from models.common import DetectMultiBackend
from utils.datasets import LoadStreams
from utils.general import (LOGGER, check_img_size, check_imshow, non_max_suppression, scale_coords)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, time_sync


@torch.no_grad()
def run(weights='../resource/best.pt', source=0):
    source = str(source)

    # Load model
    device = select_device()
    model = DetectMultiBackend(weights, device=device, dnn=False)
    stride, names, pt, jit = model.stride, model.names, model.pt, model.jit
    imgsz = check_img_size([640,480], s=stride)  # check image size

    #imgsz = [img_data.width, img_data.height]
    
    # Dataloader
    view_img = check_imshow()
    #view_img = CvBridge().imgmsg_to_cv2(img_data,"bgr8")
    cudnn.benchmark = True  # set True to speed up constant image size inference
    dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt and not jit)

    # Run inference
    if pt and device.type != 'cpu':
        print(imgsz,type(imgsz))
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.model.parameters())))  # warmup
    dt, seen = [0.0, 0.0, 0.0], 0
    for path, im, im0s, vid_cap, s in dataset:
        t1 = time_sync()
        im = torch.from_numpy(im).to(device)
        im = im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        visualize = False
        pred = model(im, augment=False, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, 0.25, 0.45, None, False, max_det=1000)
        dt[2] += time_sync() - t3

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            p, im0 = path[i], im0s[i].copy()
            s += f'{i}: '

            p = Path(p)  # to Path
            s += '%gx%g ' % im.shape[2:]  # print string
            annotator = Annotator(im0, line_width=3, example=str(names))
            go_bool = True
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                #go_bool = True
                for *xyxy, conf, cls in reversed(det):
                    
                    if view_img:  # Add bbox to image
                        pub.publish("STOP!")
                        go_bool = False
                        #print("Hello")
                        c = int(cls)  # integer class
                        label = f'{names[c]} {conf:.2f}'
                        annotator.box_label(xyxy, label, color=colors(c, True))
            if go_bool:
                pub.publish("GO!")
                        
            # Print time (inference-only)
            LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

            # Stream results
            im0 = annotator.result()
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)


def func(img_data):
    run(img_data)
    

if __name__ == "__main__":
    rospy.init_node('stop_sign_detect',anonymous=True)
    #rospy.Subscriber('/usb_cam/image_raw',Image,func)

    run()

