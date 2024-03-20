import numpy as np
def lerp(A,B,t):
    return A+(B-A)*t

def getIntersection(A,B,C,D): 
    tTop=(D[0]-C[0])*(A[1]-C[1])-(D[1]-C[1])*(A[0]-C[0])
    uTop=(C[1]-A[1])*(A[0]-B[0])-(C[0]-A[0])*(A[1]-B[1])
    bottom=(D[1]-C[1])*(B[0]-A[0])-(D[0]-C[0])*(B[1]-A[1])
    
    if(bottom!=0):
         t=tTop/bottom
         u=uTop/bottom
         if t>=0 and t<=1 and u>=0 and u<=1:
            x=lerp(A[0],B[0],t)
            y=lerp(A[1],B[1],t)
            print("t",t)
            return [x,y]
         return None

def calculate_normal_vector(p1, p2):
    dx = p2[0] - p1[0] #p2.x - p1.x
    dy = p2[1] - p1[1] #y-y
    length = np.sqrt(dx**2 + dy**2)
    # print(length,np.hypot(dx,dy));
    # be perpendicular alternate the vector  x=y ,y=-x
    # and  unit vector by length to normal vector
    nx = dy / length  # Reverse direction for inside offset 
    ny = -dx / length  # Reverse direction for inside offset 
    return nx, ny

def offset_polygon(polygon, distance):
    offset_vertices1 = []
    offset_vertices2 = []
    offset_intersection1 =[]
    offset_intersection2 =[]
    num_points = len(polygon)
    for i in range(num_points):
        p1 = polygon[i] # [0,0]
        p2 = polygon[(i + 1) % num_points] # [10,3]
        nx, ny = calculate_normal_vector(p1, p2)
        offset_vertices1.append([p1[0] + nx * distance, p1[1] + ny * distance])
        offset_vertices1.append([p2[0] + nx * distance, p2[1] + ny * distance])
        offset_vertices2.append([p1[0] + nx * -distance, p1[1] + ny * -distance])
        offset_vertices2.append([p2[0] + nx * -distance, p2[1] + ny * -distance])
    num_vertices1 = len(offset_vertices1)
    for j in range(0,num_vertices1,2): 
        s1 = offset_vertices1[j]
        s2 = offset_vertices1[(j+1)%num_vertices1]
        s3 = offset_vertices1[(j+2)%num_vertices1]
        s4 = offset_vertices1[(j+3)%num_vertices1]
        point_intersection = getIntersection(s1,s2,s3,s4) #[x,y]
        if point_intersection is not None:
           offset_intersection1.append(point_intersection)
        else:
           offset_intersection1.append(s2)
           offset_intersection1.append(s3)

        #################################################
        s1 = offset_vertices2[j]
        s2 = offset_vertices2[(j+1)%num_vertices1]
        s3 = offset_vertices2[(j+2)%num_vertices1]
        s4 = offset_vertices2[(j+3)%num_vertices1 ]
        point_intersection = getIntersection(s1,s2,s3,s4) #[x,y]
        if point_intersection is not None:
           offset_intersection2.append(point_intersection)
        else:
           offset_intersection2.append(s2)
           offset_intersection2.append(s3)

    if len(offset_intersection1)<len(offset_intersection2):
        return np.array(offset_intersection1)

    return np.array(offset_intersection2)

def polysIntersect(poly1, poly2):
    for i in range(len(poly1)):
        for j in range(len(poly2)):
                touch=getIntersection(
                poly1[i],
                poly1[(i+1)%poly1.length],
                poly2[j],
                poly2[(j+1)%poly2.length])
                if(touch):
                  return True
                
    return False

                