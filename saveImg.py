import cv2
import time

cap = cv2.VideoCapture(0)

i = 0

while True:
    i = i + 1
    success, frame = cap.read()
    ti = time.localtime()
    path = '/home/zerone/Desktop/dev_ws/Img/img_'+str(ti.tm_hour)+str(ti.tm_min)+str(ti.tm_sec)+str(i)+'.png'
    print(path)
    print(cv2.imwrite(path,frame))
    #cv2.imshow("",frame)
    #cv2.waitKey(1)