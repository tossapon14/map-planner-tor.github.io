"""
Grid based sweep planner

author: Atsushi Sakai
"""

import math
from enum import IntEnum
import numpy as np
import matplotlib.pyplot as plt
import sys

from Mapping.grid_map_lib import GridMap, FloatGrid
from Mapping.utils import offset_polygon,getIntersection

do_animation = False
fig, ax = None, None
_movedbackward = False
gpointX=[]
gpointY=[]
polyX = []
polyY = []

class SweepSearcher:
    class SweepDirection(IntEnum):
        UP = 1
        DOWN = -1

    class MovingDirection(IntEnum):
        RIGHT = 1
        LEFT = -1

    def __init__(self,
                 moving_direction, sweep_direction, x_inds_goal_y, goal_y):
        self.moving_direction = moving_direction
        self.sweep_direction = sweep_direction
        self.turing_window = []
        self.direction = 0
        self.update_turning_window()
        self.x_indexes_goal_y = x_inds_goal_y
        self.goal_y = goal_y

    def find_point_edge(self,ix,iy,grid_map):  # @
        global polyY,polyX,gpointX,gpointY  
        x_pos, y_pos =grid_map.calc_grid_central_xy_position_from_xy_index(ix,iy)
        x_pos2,y_pos2 =grid_map.calc_grid_central_xy_position_from_xy_index(ix+self.moving_direction,iy)
        s1 =[x_pos,y_pos]
        s2 =[x_pos2,y_pos2]
        length = len(polyX)
        plt.plot(x_pos, y_pos, "*k")
        plt.plot(x_pos2, y_pos2, "*m")
        plt.plot(polyX,polyY,"-y")
        print(polyX,polyY)
        for i in range(length):
            s3 = [polyX[i],polyY[i]]
            s4 = [polyX[(i+1)%length],polyY[(i+1)%length]]
            intersec = getIntersection(s1,s2,s3,s4)
            print("intersec  ",intersec)
            if intersec is not None:
                print("find_point_edge")
                gpointX.append(intersec[0])
                gpointY.append(intersec[1])
                return intersec
        print("not find")
        return None

    def move_target_grid(self, c_x_index, c_y_index, grid_map):#c_x,c_y index start
        n_x_index = self.moving_direction + c_x_index  #m_d right = +1
        n_y_index = c_y_index
        # found safe grid use not find FloatGrid = 0.0 (inside)
        if not self.check_occupied(n_x_index, n_y_index, grid_map):
            return n_x_index, n_y_index
        else:  # occupied FloatGrid = 1.0 (outside)
            # print("find turning because find border")
            next_c_x_index, next_c_y_index = self.find_safe_turning_grid(
                c_x_index, c_y_index, grid_map)
            # gpointX.pop()
            # gpointY.pop()
            # print("check ",self.check_occupied(c_x_index, c_y_index, grid_map))
            self.find_point_edge(c_x_index,c_y_index,grid_map) 
            self.find_point_edge(c_x_index,c_y_index+self.sweep_direction,grid_map)
            """return None because never find free grid"""
            if (_movedbackward)and(next_c_x_index is None) and (next_c_y_index is None): 
                # moving backward 
                next_c_x_index = -self.moving_direction + c_x_index
                next_c_y_index = c_y_index
                if self.check_occupied(next_c_x_index, next_c_y_index, grid_map, FloatGrid(1.0)):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            elif c_y_index>=self.goal_y:
                    return None,None
            elif (next_c_x_index is None) and (next_c_y_index is None):
                next_c_y_index =  c_y_index + self.sweep_direction # *
                next_c_x_index = c_x_index                         # @
                
                while self.check_occupied(next_c_x_index, next_c_y_index, grid_map):
                  next_c_x_index = -self.moving_direction + next_c_x_index

                if self.check_occupied(next_c_x_index, next_c_y_index, grid_map, FloatGrid(1.0)):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None

            self.swap_moving_direction()
            return next_c_x_index, next_c_y_index

    @staticmethod
    def check_occupied(c_x_index, c_y_index, grid_map, occupied_val=FloatGrid(0.5)):
        return grid_map.check_occupied_from_xy_index(c_x_index, c_y_index, occupied_val)

    def find_safe_turning_grid(self, c_x_index, c_y_index, grid_map):
        # print(self.turing_window) => [(<MovingDirection.RIGHT: 1>, 0), (<MovingDirection.RIGHT: 1>, <SweepDirection.UP: 1>), (0, <SweepDirection.UP: 1>), (-1, <SweepDirection.UP: 1>)]
        for (d_x_ind, d_y_ind) in self.turing_window: 
            #   ***                                          
            #    @*
            next_x_ind = d_x_ind + c_x_index
            next_y_ind = d_y_ind + c_y_index

            # found safe grid
            if not self.check_occupied(next_x_ind, next_y_ind, grid_map):
                return next_x_ind, next_y_ind
        """return None because naver find free grid"""
        return None, None

    def is_search_done(self, grid_map):   ## only check index y goal
        # print("list x on Y",self.x_indexes_goal_y)
        for ix in self.x_indexes_goal_y: # x_indexes_goal_y is line of
            # print("get data grid x",ix,grid_map.get_value_from_xy_index(ix,self.goal_y).data)
            if not self.check_occupied(ix, self.goal_y, grid_map):
                return False

        # all lower grid is occupied
        return True

    def update_turning_window(self):
        # turning window definition
        # robot can move grid based on it.
        self.turing_window = [
            (self.moving_direction, 0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_moving_direction(self):
        self.moving_direction *= -1
        self.update_turning_window()

    def search_start_grid(self, grid_map):  #return x,y start
        x_inds = []
        y_ind = 0
        if self.sweep_direction == self.SweepDirection.DOWN:
            x_inds, y_ind = search_free_grid_index_at_edge_y(     #find starting point, sweep UP.Starting point must start from bottom 
                grid_map, from_upper=True) #
        elif self.sweep_direction == self.SweepDirection.UP:
            x_inds, y_ind = search_free_grid_index_at_edge_y(
                grid_map, from_upper=False)

        if self.moving_direction == self.MovingDirection.RIGHT:
            return min(x_inds), y_ind
        elif self.moving_direction == self.MovingDirection.LEFT:
            return max(x_inds), y_ind

        raise ValueError("self.moving direction is invalid ")        # print(ox[i + 1] , ox[i]," = ",ox[i + 1] - ox[i])


def find_max_right(ox,oy):
    max_right =0.0
    vec = [0.0,0.0]
    for i in range(len(ox)):
        d= ox[i]
        if d>max_right:
           max_right = d
           vec = [ox[i],oy[i]]
    return vec,vec
    
def find_sweep_direction_and_start_position(ox, oy):  # vec = vector direction [5i,7j]  
    #sweep_start_pos = start from point longest vector [x,y]
    # find sweep_direction
    max_dist = 0.0
    vec = [0.0, 0.0]
    sweep_start_pos = [0.0, 0.0]
    for i in range(len(ox) - 1):
        dx = ox[i + 1] - ox[i]  # find vector x
        dy = oy[i + 1] - oy[i]  # find vector y  
        d = np.hypot(dx, dy)   # find distance of vector

        if d > max_dist:   
            max_dist = d
            vec = [dx, dy]
            sweep_start_pos = [ox[i], oy[i]]

    return vec, sweep_start_pos


def convert_grid_coordinate(ox, oy, sweep_start_position,radian=0.0): #sweep_vector use cal radian
    # transform rotation for longest vector go to vector horizontal and
    # find max width maxheight of area
    # tx ,ty is translate all point start form (0,0)
    # transform axis is (0,0) of plt()
    # if th (+) rotation clockwise  th(-) couterclockwise
    tx = [ix - sweep_start_position[0] for ix in ox] #  point x (corner area) -  origin vector start  
    ty = [iy - sweep_start_position[1] for iy in oy]
    th = radian     #math.atan(y from 0,x from 0) find arctan of vector
    # print("x",sweep_start_position[0],"y",sweep_start_position[1],"th", th)
    # plt.plot(tx,ty, "-k")
    rot_mat = np.array([[np.cos(th), -np.sin(th)],
                     [np.sin(th), np.cos(th)]])
    converted_xy = np.stack([tx, ty]).T @ rot_mat
    # plt.plot(converted_xy[:, 0], converted_xy[:, 1], "-r")
    print("polygon ",converted_xy[:, 0], converted_xy[:, 1])
    return converted_xy[:, 0], converted_xy[:, 1]


def convert_global_coordinate(x, y, sweep_start_position,radian):
    th =-radian
    rot_mat = np.array([[np.cos(th), -np.sin(th)],
                     [np.sin(th), np.cos(th)]])
    converted_xy = np.stack([x, y]).T @ rot_mat
    rx = [ix + sweep_start_position[0] for ix in converted_xy[:, 0]]
    ry = [iy + sweep_start_position[1] for iy in converted_xy[:, 1]]
    return rx, ry

def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
    y_index = None
    x_indexes = []

    if from_upper:
        y_range = range(grid_map.height)[::-1]
        x_range = range(grid_map.width)[::-1]
    else:
        y_range = range(grid_map.height)
        x_range = range(grid_map.width)
    l_x_pos = []
    l_y_pos =[] 
    
    for iy in y_range:
        for ix in x_range:  #sweep axis horizontal
            if not SweepSearcher.check_occupied(ix, iy, grid_map): # from grid_map_lib return true if grid have FloatGrid ready
              # use false fine grid FloatDrid.data == 0.0
            #    print("search grid index ------------- ")
            #    print(f"y {iy} x {ix}")
               x_pos, y_pos = grid_map.calc_grid_central_xy_position_from_xy_index(ix, iy)
               l_x_pos.append(x_pos)
               l_y_pos.append(y_pos)
               y_index = iy
               x_indexes.append(ix)
        if y_index:
            break
    return x_indexes, y_index


def setup_grid_map(ox, oy, resolutionH,resolutionV, sweep_direction, offset_grid=5):
    if offset_grid%2 == 0:
        offset_grid = offset_grid+1
    width = math.floor(max(ox)-min(ox)) / resolutionH +offset_grid
    height = math.floor(max(oy) - min(oy)) / resolutionV +offset_grid
    center_x = (np.max(ox) + np.min(ox)) / 2.0
    center_y = (np.max(oy) + np.min(oy)) / 2.0
    
    plt.plot(center_x,center_y,".m")
    left_lower_x = center_x - width / 2.0 *  resolutionH
    left_lower_y = center_y - height / 2.0 *  resolutionV
    # print("center y",center_x,"center y",center_y," lower x",left_lower_x," => ","width ",width,"  devid  ",width / 2.0 *  resolutionH)
    # plt.plot(left_lower_x,left_lower_y,".r")
    right_higher_x = center_x + width / 2.0 *  resolutionH
    right_higher_y = center_y + height / 2.0 *  resolutionV
    # plt.plot(right_higher_x,right_higher_y,".g")
    # print(width,height)
    grid_map = GridMap(width, height, resolutionH,resolutionV, center_x, center_y)
    # grid_map.print_grid_map_info()
    # x_pos,y_pos = grid_map.set_value_from_polygon(ox, oy, FloatGrid(1.0), inside=False)
    grid_map.set_value_from_polygon(ox, oy, FloatGrid(1.0), inside=False)

    # plt.plot(x_pos,y_pos,"-")
    # x_pos1,y_pos1 =  grid_map.expand_grid ()
    # plt.plot(x_pos1,y_pos1,".g")
    # plt.plot(x_pos1,y_pos1,"-m")
    x_inds_goal_y = []
    goal_y = 0
    if sweep_direction == SweepSearcher.SweepDirection.UP:     #find goal line if UP => goal must start from top from_upper =true
        x_inds_goal_y, goal_y = search_free_grid_index_at_edge_y(
            grid_map, from_upper=True)
    elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
        x_inds_goal_y, goal_y = search_free_grid_index_at_edge_y(
            grid_map, from_upper=False)
    print("end setup grid ****************")
    return grid_map, x_inds_goal_y, goal_y # return class grid_map,final goal line =>  x index[] along the line y ,y
    # return [],[],[] 


def sweep_path_search(sweep_searcher, grid_map):
    global gpointX,gpointY
    # search start grid
    st_x_index, st_y_index = sweep_searcher.search_start_grid(grid_map)
    if not grid_map.set_value_from_xy_index(st_x_index, st_y_index, FloatGrid(0.5)):  
        print("Cannot find start grid")
        return [], []

    x, y = grid_map.calc_grid_central_xy_position_from_xy_index(st_x_index,
                                                                st_y_index) # return x_pos,y_pos from lowerleft
    gpointX, gpointY = [x], [y] # add start x_pos,y_pos

    while True:
        st_x_index, st_y_index = sweep_searcher.move_target_grid(st_x_index,st_y_index,grid_map)# index x, y
        if sweep_searcher.is_search_done(grid_map) or (st_x_index is None or st_y_index is None):
            #is_search  only check index y goal
            print("Done")
            break

        x, y = grid_map.calc_grid_central_xy_position_from_xy_index(
            st_x_index, st_y_index) ## return x_pos,y_pos from lowerleft

        gpointX.append(x)
        gpointY.append(y)
        grid_map.set_value_from_xy_index(st_x_index, st_y_index, FloatGrid(0.5)) #if index pass index x,y .I will set this grid value = 0.5

    return gpointX, gpointY


def planning(ox, oy, resolutionH,resolutionV,radian =0.0,
             moving_direction=SweepSearcher.MovingDirection.LEFT,
             sweeping_direction=SweepSearcher.SweepDirection.UP,
             ):
      global polyX,polyY
      print("-------------------------------------------------------------------")
      print("moving_direction { Right }  sweeping_direction { from UP }")
      sweep_vec, sweep_start_position = find_max_right(ox, oy)
      
      rox, roy = convert_grid_coordinate(ox, oy,sweep_start_position,radian)#sweep_vec = vector (5i,8j)use cal radian, sweep_start = point [x,y]
      polyX=rox
      polyY=roy
      grid_map, x_inds_goal_y, goal_y = setup_grid_map(rox,roy , resolutionH,resolutionV,
                                                     sweeping_direction) 
      #class grid_map,x index[] along the line y ,y

      sweep_searcher = SweepSearcher(moving_direction, sweeping_direction,
                                   x_inds_goal_y, goal_y)

      px, py = sweep_path_search(sweep_searcher, grid_map)# pass class,class
    # plt.plot(px, py, "r")
    # plt.plot(px, py, ".")
      plt.plot(px[0], py[0], "*")
 
      rx, ry = convert_global_coordinate(px, py,
                                       sweep_start_position,radian)


      return rx, ry


def planning_animation(ox, oy, resolutionH,resolutionV,radian):  # pragma: no cover
    px, py = planning(ox, oy, resolutionH,resolutionV,radian)

    # animation
    if do_animation:
        list_pathX,list_pathY=[],[]
        for ipx, ipy in zip(px, py):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            list_pathX.append(ipx)
            list_pathY.append(ipy)
            plt.plot(ox,oy, 'b') # draw polygon area
            plt.plot(ox,oy, '.r') # plot dot of area
            plt.plot(list_pathX, list_pathY, "r")
            plt.plot(list_pathX, list_pathY, ".")
            plt.plot(px[0], py[0], "or")

            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.05)

        plt.cla()
    
    plt.plot(ox,oy, 'b')
    plt.plot(ox,oy, '.r')
    plt.plot(px, py, "r")
    plt.plot(px, py, ".")
    plt.plot(px[0], py[0], "or")
    plt.plot(px[-1], py[-1], "ob")
    plt.axis("equal")
    plt.grid(True)
    plt.draw()
        
    
def main():  # pragma: no cover
    global fig,ax
    print("start!!")
    offset_distance = 1
    resolutionH = 1
    resolutionV = 1
    radian = math.radians(0)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.grid(True)
    ox = []
    oy =[]
 
    def onclick(event):
    #    print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
    #       (event.button, event.x, event.y, event.xdata, event.ydata))
       fig.clear()
       ax = fig.add_subplot(111)
       ax.set_xlim([-10, 10])
       ax.set_ylim([-10, 10])
       plt.grid(True)
       if int(event.button) is 1:
           if len(ox) is 2:
             ox.append(event.xdata)
             oy.append(event.ydata)
             ox.append(ox[0])
             oy.append(oy[0])
             
           elif len(ox) < 2:
             ox.append(event.xdata)
             oy.append(event.ydata)
           elif len(ox)>2:
             ox.insert(-1,event.xdata)
             oy.insert(-1,event.ydata)
             
       elif int(event.button) is 3 and len(ox) is not 0:
           if len(ox) is 4 :
                ox.pop()
                oy.pop() 
                ox.pop()
                oy.pop()
           elif len(ox) < 4:
                ox.pop()
                oy.pop()
           elif len(ox)>4:
                ox.pop(-2)
                oy.pop(-2)
       elif int(event.button) is 2:
           fig.clear()
           ax = fig.add_subplot(111)
           ax.set_xlim([-10, 10])
           ax.set_ylim([-10, 10])
           plt.grid(True)
           ox.clear()
           oy.clear()
            
        
    #    print(ox)
    #    print(oy)
       plt.plot(ox,oy, 'b')
       plt.plot(ox,oy, '.r')
       plt.draw()
       
    def on_press(event):
        sys.stdout.flush()
        if event.key == 'enter':
            ox1 = ox
            oy1 =oy
            ox1.pop()
            oy1.pop()
            polygon = np.array([[x,y] for x,y in zip(ox1,oy1)])
            offset = offset_polygon(polygon, offset_distance)
            plt.plot(offset[:, 0], offset[:, 1], label='Offset Polygon (Inside)', color='red')
            plt.draw()
            ox2 = offset[:,0]
            oy2 = offset[:,1]
            planning_animation(ox2, oy2, resolutionH,resolutionV,radian)

   
    plt.gcf().canvas.mpl_connect('button_press_event', onclick)
    plt.gcf().canvas.mpl_connect('key_press_event', on_press)
   
   
   
    # ox = [-5.0, 10, 12,20]
    # oy = [0.0, 10,5.0, -5.0]
    # oxring = [-5.0, 10, 12,20, -5.0]
    # oyring = [0.0, 10,5.0, -5.0, 0.0]
    # fig.canvas.mpl_connect(
    #             'key_release_event',
    #             lambda event: [exit(0) if event.key == 'escape' else None])
    # plt.plot(oxring,oyring, 'b')
    # plt.plot(oxring,oyring, '.r')
    # plt.draw()
    # polygon = np.array([[x,y] for x,y in zip(ox,oy)])
    # offset = offset_polygon(polygon, offset_distance)
    # plt.plot(offset[:, 0], offset[:, 1], label='Offset Polygon (Inside)', color='red')
    # ox = offset[:,0]
    # oy =offset[:,1]
    # planning_animation(ox, oy, resolutionH,resolutionV,radian)
   
    
    plt.show()

if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    sys.exit(0)