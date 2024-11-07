import numpy as np
import cv2
import sys
from sklearn.cluster import KMeans
from scipy.cluster.hierarchy import linkage, fcluster
import math

def from_lsd(lines_lsd):
    """
    lines_lsd : the weird output of cv line segment detector.
    returns : [[x1, y1, x2, y2, a, b, c, L],
               [...                       ],
               ...
              ]

              where 
                - [(x1, y1), (x2, y2)] is the segment (in image coordinates).
                - ax + by + c = 0 is the line equation
                - (a, b) is the normal vector of the line
                - |(a, b)| = 1, b >= 0
                - L is the length of the segment.
    """
    if lines_lsd is None :
        return np.array([]).reshape((0,8))

    nlines = lines_lsd.shape[0]
    segments = None

    if nlines == 0 :
        return np.array([]).reshape((0,8))

    if nlines == 1 :
        segments = lines_lsd.squeeze().reshape((1,4))
    else :
        segments = lines_lsd.squeeze()

    x1 = segments[:, 0]
    y1 = segments[:, 1]
    x2 = segments[:, 2]
    y2 = segments[:, 3]
    b = x1 - x2
    a = y2 - y1
    c = -(a * x1 + b * y1)

    # Ensure that the normal vectors are always
    # pointing toward positive y
    where_b_is_neg = b < 0
    c[where_b_is_neg] *= -1.0
    b[where_b_is_neg] *= -1.0
    a[where_b_is_neg] *= -1.0

    # The length of the segment
    norm = np.sqrt(a**2 + b**2)
    return np.stack([x1, y1, x2, y2, a/norm, b/norm, c/norm, norm], axis=1)


def draw_segments(img, segments, color, thickness):
    '''
        img : cv2 image
        segments : np.array(num_lines, 8)
        color  : 3 element tuple
        thickness : int
    '''
    for s in segments:
        cv2.line(img, (s[0], s[1]), (s[2], s[3]), color, thickness)


def draw_lines(img, segments, color, thickness):
    '''
        img : cv2 image
        segments : np.array(num_lines, 8)
        color  : 3 element tuple
        thickness : int
    '''

    for s in segments:
        a, b, c = s[4:7]
        # If the line is horizontal
        if a == 0:
            cv2.line(img, (0, -int(c/b)), (img.shape[1]-1, -int((c+a*(img.shape[1]-1))/b)), color, thickness)
        # If the line is vertical
        elif b == 0:
            cv2.line(img, (-int(c/a), 0), (-int(c/a), img.shape[0] - 1), color, thickness)
        else:
            # We look for the intersection point with the horizontal axis y=0 and y=height-1
            y = 0
            x = -c/a   #-(c - b*y)/a
            if x < 0:
                x = 0
                y = -(c + a * x) / b
            elif x >= img.shape[1]:
                x = img.shape[1] - 1
                y = -(c + a * x) / b
            first_point = (int(x), int(y))

            y = img.shape[0] - 1
            x = -(c+b*y)/a
            if x < 0:
                x = 0
                y = -(c + a * x) / b
            elif x >= img.shape[1]:
                x = img.shape[1] - 1
                y = -(c + a * x) / b
            second_point = (int(x), int(y))

            cv2.line(img, first_point, second_point, color, thickness)

def intersections(segments):
    intersections = []
    for i, si in enumerate(segments):
        for sj in segments[i+1:]:
            cross_product = np.cross(si[4:6], sj[4:6]) # [a1,b1] ^ [a2, b2]
            if cross_product != 0:
                coeff = 1.0 / cross_product

                intersections.append([coeff * np.cross(si[5:7]   , sj[5:7]), # [b1, c1] ^ [b2, c2]
                                      coeff * np.cross(sj[[4, 6]], si[[4, 6]])]) # -[a1, c1] ^ [a2, c2]
    return np.array(intersections)


def hierarchical_clustering(normal_vectors):
    Z = linkage(normal_vectors, method='ward')  # 'ward' minimizes variance, use other methods if needed
    return fcluster(Z, 2, criterion='maxclust')

def vanishing_lines(lines, normal_vectors):
    # Perform hierarchical/agglomerative clustering
    # Extract two largest clusters
    labels = hierarchical_clustering(normal_vectors)
    lines = np.concatenate((lines[labels == 1], lines[labels == 2]),axis=0)
    normal_vectors = np.concatenate((normal_vectors[labels == 1], normal_vectors[labels == 2]),axis=0)

    kmeans = KMeans(n_clusters=2, random_state=0).fit(normal_vectors)
    labels = kmeans.labels_
    # Séparer les lignes en fonction des clusters
    first = lines[labels == 0]
    #Verifier non vide
    #Tous les éléments ont cette forme (xa,ya,xb,yb,a,b,c,l)
    #Prendre premier élement de first, save signe de a
    
    #Meme chose pour labels==1
    #Si Signe a meme signe alors regarder labels==2 ...
    return sort_vp_lines(lines[labels == 0], lines[labels == 1])

def sort_vp_lines(lines1, lines2):
    lines1 = lines1.reshape(-1,8)
    lines2 = lines2.reshape(-1,8)
            
    mean1 = np.mean(lines1,axis=0)
    mean2 = np.mean(lines2,axis=0)
            
    a1 = mean1[4]
    a2 = mean2[4]
    if a1<0 and a2>0:
        left = mean1
        right = mean2
        return left, right
    elif a1>0 and a2<0:
        left = mean2
        right = mean1
        return left, right
    else:
        return None

def vp_point(left, right):
    lines = np.stack((left,right))
    point = intersections(lines)
    x = int(point[0][0])
    y = int(point[0][1])
    return (x,y)
def horizontal_misplacement(frame, vanishing_point):

   ans=vanishing_point[0]/frame.shape[1]-0.5
   return ans

def angle_indicator(line1, line2): #Ratio between left and right angles
   agl1 = np.pi/2 - np.arccos(abs(line1[0]))
   agl2 = np.pi/2 - np.arccos(abs(line2[0]))
   # Return the ratio of the two angles
#    return agl1/agl2
   agl=agl1/(agl1+agl2) - 0.5
   
   return agl




