import numpy as np
import cv2
import os
from scipy.optimize import least_squares

def get_corners(images,rows,cols,square_size):
    objp=np.zeros((rows*cols,3),np.float32)
    objp[:,:2]=np.mgrid[0:rows,0:cols].T.reshape(-1,2)*square_size

    objpoints=[]
    imgpoints=[]

    for idx, image in enumerate(images):
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

def get_R_t(K,H_list):
    R_t_list=[]
    for i in range(len(H_list)):
        H=H_list[i]
        h1=H[:,0]
        h2=H[:,1]
        h3=H[:,2]
        a=1/(np.linalg.norm(np.dot(np.linalg.inv(K),h1)))
        r1 = a*np.dot(np.linalg.inv(K),h1)
        r2 = a*np.dot(np.linalg.inv(K),h2)
        r3 = np.cross(r1,r2)
        t = a*np.dot(np.linalg.inv(K),h3)
        R_t_list.append(np.column_stack([r1,r2,r3,t]))
    return R_t_list

def get_projected_points(K,k,R_t_list,objpoints,imgpoints):
    projected_points=[]
    error=[]
    for i in range(len(objpoints)):
        R_t=R_t_list[i]
        obj=objpoints[i]
        img=imgpoints[i]
        R_T_dash=np.transpose([R_t[:,0],R_t[:,1],R_t[:,3]]).reshape(3,3)
        P=np.dot(K,R_T_dash)
        e=0
        points=[]
        for j in range(len(obj)):

            uv=np.dot(P,np.array([obj[j,0],obj[j,1],1]).reshape(3,1))
            u=uv[0]/uv[2]
            v=uv[1]/uv[2]

            xy=np.dot(R_t,np.array([obj[j,0],obj[j,1],0,1]).reshape(4,1))
            x=xy[0]/xy[2]
            y=xy[1]/xy[2]

            r2=x**2+y**2

            u_dash=u+(u-K[0,2])*(k[0]*r2+k[1]*r2**2)
            v_dash=v+(v-K[1,2])*(k[0]*r2+k[1]*r2**2)
            points.append([u_dash,v_dash])
            m=np.array([u_dash,v_dash]).reshape(2,)
            mij=img[j]
            e+=np.linalg.norm(m-mij)
        error.append(e)
        projected_points.append(points)

    return projected_points,error 

def get_geometric_error(x0,R_t_list,objpoints,imgpoints):

    alpha,gamma,u0,beta,v0,k1,k2=x0
    K=np.array([[alpha,gamma,u0],[0,beta,v0],[0,0,1]])


    error=[]
    for i in range(len(objpoints)):
        R_t=R_t_list[i]
        obj=objpoints[i]
        img=imgpoints[i]
        R_T_dash=np.transpose([R_t[:,0],R_t[:,1],R_t[:,3]]).reshape(3,3)
        P=np.dot(K,R_T_dash)
        e=0
        for j in range(len(obj)):

            uv=np.dot(P,np.array([obj[j,0],obj[j,1],1]).reshape(3,1))
            u=uv[0]/uv[2]
            v=uv[1]/uv[2]

            xy=np.dot(R_t,np.array([obj[j,0],obj[j,1],0,1]).reshape(4,1))
            x=xy[0]/xy[2]
            y=xy[1]/xy[2]

            r2=x**2+y**2

            u_dash=u+(u-u0)*(k1*r2+k2*r2**2)
            v_dash=v+(v-v0)*(k1*r2+k2*r2**2)
            m=np.array([u_dash,v_dash]).reshape(2,)
            mij=img[j]
            

            e+=np.linalg.norm(m-mij)
        error.append(e)

    return error
   


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

    R_t_list = get_R_t(init_K,H_list)

    init_k = [0,0]

    projected_points,error=get_projected_points(init_K,init_k,R_t_list,objpoints,imgpoints)

    mean_error=np.sum(error)/(len(objpoints)*len(objpoints[0]))
    print('Average error before optimization:',mean_error)

    res=least_squares(get_geometric_error,[init_K[0,0],init_K[0,1],init_K[0,2],init_K[1,1],init_K[1,2],init_k[0],init_k[1]],method='lm',args=(R_t_list,objpoints,imgpoints))
    
    K=np.array([[res.x[0],res.x[1],res.x[2]],[0,res.x[3],res.x[4]],[0,0,1]])
    print('Final K:',K)
    print('Final k:',res.x[5],res.x[6])

    projected_points,error=get_projected_points(K,[res.x[5],res.x[6]],R_t_list,objpoints,imgpoints)

    mean_error=np.sum(error)/(len(objpoints)*len(objpoints[0]))
    print('Average error after optimization:',mean_error)

    dist=np.array([res.x[5],res.x[6],0,0])

    for i in range(len(images_cp)):
        img=cv2.undistort(images_cp[i],K,dist)
        points=projected_points[i]
        for point in points:
            img=cv2.circle(img,(int(point[0]),int(point[1])),5,(0,0,255),-1)
        cv2.imwrite('./Results/'+'Reprojected_'+str(i)+'.jpg',img)

if __name__=='__main__':
    main()