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

def find_Homography(src_points,dst_points):
    A = []
    for i in range(len(src_points)):
        x = src_points[i][0]
        y = src_points[i][1]
        xp = dst_points[i][0]
        yp = dst_points[i][1]
        A.append([-x, -y, -1, 0, 0, 0, x * xp, y * xp, xp])
        A.append([0, 0, 0, -x, -y, -1, x * yp, y * yp, yp])

    A = np.array(A)
    print(A.shape)
    _, _, V=np.linalg.svd(A)
    H = V[-1].reshape(3,3)
    return H

def get_camera_matrix(objpoints,imgpoints,shape):
    H_list=[]
    for i in range(len(objpoints)):
        H = find_Homography(objpoints[i],imgpoints[i])
        H_list.append(H)

    H_list = np.array(H_list)

    V = np.zeros((2*len(objpoints),6))

    for i in range(len(objpoints)):
        H=H_list[i]
        V[2*i]=[H[0,0]*H[0,1],H[0,0]*H[1,1]+H[1,0]*H[0,1],H[1,0]*H[1,1],H[2,0]*H[0,1]+H[0,0]*H[2,1],H[2,0]*H[1,1]+H[1,0]*H[2,1],H[2,0]*H[2,1]]
        V[2*i+1]=[H[0,0]*H[0,0]-H[0,1]*H[0,1],H[0,0]*H[1,0]+H[1,0]*H[0,0],H[1,0]*H[1,0]-H[1,1]*H[1,1],H[2,0]*H[0,0]+H[0,0]*H[2,0],H[2,0]*H[1,0]+H[1,0]*H[2,0],H[2,0]*H[2,0]-H[2,1]*H[2,1]]

    _,_,Vt=np.linalg.svd(V)
    b=Vt[-1]

    
    v0=(b[1]*b[3]-b[0]*b[4])/(b[0]*b[2]-b[1]**2)
    lam=b[5]-(b[3]**2+v0*(b[1]*b[3]-b[0]*b[4]))/b[0]
    alpha=np.sqrt(lam/b[0])
    beta=np.sqrt(lam*b[0]/(b[0]*b[2]-b[1]**2))
    gamma=-b[1]*alpha**2*beta/lam
    u0=gamma*v0/beta-b[3]*alpha**2/lam

    K=np.zeros((3,3))
    K[0,0]=alpha
    K[0,1]=gamma
    K[0,2]=u0
    K[1,1]=beta
    K[1,2]=v0
    K[2,2]=1

    return K,H_list

    

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

    init_K, H_list = get_camera_matrix(objpoints,imgpoints,images[0].shape)
    print(H_list.shape)

if __name__=='__main__':
    main()