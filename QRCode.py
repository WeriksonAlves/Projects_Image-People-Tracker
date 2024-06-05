# Importing Libraries
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import urllib.request
 
url='http://192.168.0.192/cam-hi.jpg'

cv2.namedWindow("QR Code: WRM", cv2.WINDOW_AUTOSIZE)
 
prev=""
pres=""
while True:
    img_resp = urllib.request.urlopen(url)
    imgnp = np.array(bytearray(img_resp.read()),dtype=np.uint8)
    frame = cv2.imdecode(imgnp,-1)
    decodedObjects = pyzbar.decode(frame)
    for obj in decodedObjects:
        pres = obj.data
        if prev == pres:
            pass
        else:
            data = str(obj.data, 'utf-8')
            print("Type:",obj.type)
            print("Data: ",data)
            prev=pres
        cv2.putText(frame, str(data), (50, 50), cv2.FONT_HERSHEY_PLAIN, 2,(0, 255, 0), 3)
 
    cv2.imshow("live transmission", frame)
 
    key = cv2.waitKey(1)
    if key == 27:
        break
 
cv2.destroyAllWindows()