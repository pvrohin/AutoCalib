import numpy as np
import cv2
import os

def get_corners(images,rows,cols,square_size):
    objp=np.zeros((rows*cols,3),np.float32)
    objp[:,:2]=np.mgrid[0:rows,0:cols].T.reshape(-1,2)*square_size

    objpoints=[]
    imgpoints=[]

    for image in images:
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        ret,corners=cv2.findChessboardCorners(gray,(rows,cols),None)

        if ret==True:
            criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
            corners2=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)

    imgpoints=np.array(imgpoints)
    objpoints=np.array(objpoints)

    imgpoints=imgpoints.reshape(imgpoints.shape[0],imgpoints.shape[1],2)

    return objpoints,imgpoints

def main():
    image_path='./Calibration_Imgs'
    image_files=os.listdir(image_path)
    image_files.sort()

    images=[]
    for image_file in image_files:
        image=cv2.imread(os.path.join(image_path,image_file))
        images.append(image)

    rows=9
    cols=6

    square_size=21.5

    if not os.path.exists('./Results'):
        os.makedirs('./Results')


    images_cp=images[:]
    objpoints,imgpoints=get_corners(images,rows,cols,square_size)

    print(objpoints.shape)
    print(imgpoints.shape)

if __name__=='__main__':
    main()