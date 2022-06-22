import cv2 as cv
# import the numpy library
import numpy as np
import math
import message

map13 = cv.imread('src/assignment/map2.pgm')
map3 = cv.cvtColor(map13,cv.COLOR_BGR2GRAY)
# Thresholding:
_,map2 = cv.threshold(map3, 200,255,cv.THRESH_BINARY) # pixels below 127 are marked black and above till 255 are marked white
# Display image
# cv.imshow('map', map2)

# defining initial position and the goal
initial = (210, 300)
goal = (470,470)

# finding neighbouring node
def neighbour(v_node, img34):
    # next_node list to store accessed neighbours from current node u
    next_node = []
    x,y = v_node
    for i in range(-1,2):
        for j in range(-1,2):
            if i == 0 and  j ==0:
                continue
            distance = float(((i**2+j**2)**0.5))
            
            pixel = img34[x+i, y+j]
            if pixel == 255:
                next_node.append((x+i, y+j,distance))
            elif pixel == 0:
                distance = 10000000000
                next_node.append((x+i, y+j,distance))
               
    return next_node



def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
    return path


def distance_heuristic(state_key, goal_k):
    a,b = state_key
    c,d = goal_k
    d = float(((c-a)**2 + (d-b)**2 )**0.5)
    return d

def a_star_search(origin_key, goal_key, img):
    open_queue = message.priority_dict({})
    # to store processed vertices
    closed_dict = {}
    # dictionary of predecessors for each vertex
    predecessors = {}
    
    predecessors[origin_key]= 0
    costs = {}
    costs[origin_key] = 0.0
    open_queue[origin_key] = distance_heuristic(origin_key,goal_key)

   
    goal_found = False
    while (open_queue):
        # print(len(predecessors))
        u,l= open_queue.pop_smallest()
        # print(u)
        ucost = costs[u]
        
        # by popping if we get required goal
        if u == goal_key:
            goal_found = True
            break
        
        # For outgoing edges of current vertex
        for dist in neighbour(u,img):
            v = (dist[0],dist[1])
            # to get distance to the next vertex
            uv_cost = float(dist[2]) 
            # if node already processed
            if v in closed_dict:
                continue

            if v in open_queue:
                # if we get node by lower cost than previous
                if ucost + uv_cost +distance_heuristic(v, goal_key) < open_queue[v]:
                    open_queue[v] = ucost + uv_cost + distance_heuristic(v, goal_key)
                    costs[v]= ucost+uv_cost
                    predecessors[v] = u
            else:
                open_queue[v] = ucost + uv_cost + distance_heuristic(v, goal_key)
                costs[v] = ucost + uv_cost
                predecessors[v] = u
        closed_dict[u] = ucost

    if not goal_found:
        raise ValueError("Goal not found in search.")
    return get_path(origin_key, goal_key, predecessors)

path = a_star_search(initial, goal, map2)
def plot(image,path):
    for i in path:
        x,y = i
        image[x,y]=0
    cv.imshow('map',image)
    cv.waitKey(0)
plot(map2,path)