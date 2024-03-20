"""

Grid map library in python

author: Atsushi Sakai

"""
import matplotlib.pyplot as plt
import numpy as np


class FloatGrid:

    def __init__(self, init_val=0.0):
        self.data = init_val

    def get_float_data(self):
        return self.data

    def __eq__(self, other):
        if not isinstance(other, FloatGrid):
            return NotImplemented
        return self.get_float_data() == other.get_float_data()

    def __lt__(self, other):
        if not isinstance(other, FloatGrid):
            return NotImplemented
        return self.get_float_data() < other.get_float_data()


class GridMap:
    """
    GridMap class
    """

    def __init__(self, width, height, resolutionH,resolutionV,
                 center_x, center_y, init_val=FloatGrid(0.0)):
        """__init__

        :param width: number of grid for width
        :param height: number of grid for height
        :param resolution: grid resolution [m]
        :param center_x: center x position  [m]
        :param center_y: center y position [m]
        :param init_val: initial value for all grid
        """
        self.width = int(width)
        self.height = int(height)
        self.resolutionH = resolutionH
        self.resolutionV = resolutionV
        self.center_x = center_x
        self.center_y = center_y

        self.left_lower_x = self.center_x - self.width / 2.0 * self.resolutionH
        self.left_lower_y = self.center_y - self.height / 2.0 * self.resolutionV
        # self.left_lower_y = self.center_y - self.height / 2.0

        self.n_data = self.width * self.height
        self.data = [init_val] * int(width*height)
        self.data_type = type(init_val)
        
    def get_value_from_xy_index(self, x_ind, y_ind):
        """get_value_from_xy_index

        when the index is out of grid map area, return None

        :param x_ind: x index
        :param y_ind: y index
        """

        grid_ind = self.calc_grid_index_from_xy_index(x_ind, y_ind)#grid_ind = int(y_ind * self.width + x_ind)

        if 0 <= grid_ind < self.n_data:
            return self.data[grid_ind]
        else:
            return None

    def get_xy_index_from_xy_pos(self, x_pos, y_pos):
        """get_xy_index_from_xy_pos

        :param x_pos: x position [m]
        :param y_pos: y position [m]
        """
        x_ind = self.calc_xy_index_from_position(
            x_pos, self.left_lower_x, self.width)
        y_ind = self.calc_xy_index_from_position(
            y_pos, self.left_lower_y, self.height)

        return x_ind, y_ind

    def set_value_from_xy_pos(self, x_pos, y_pos, val):
        """set_value_from_xy_pos

        return bool flag, which means setting value is succeeded or not

        :param x_pos: x position [m]
        :param y_pos: y position [m]
        :param val: grid value
        """

        x_ind, y_ind = self.get_xy_index_from_xy_pos(x_pos, y_pos)

        if (not x_ind) or (not y_ind):
            return False  # NG

        flag = self.set_value_from_xy_index(x_ind, y_ind, val)

        return flag

    def set_value_from_xy_index(self, x_ind, y_ind, val):
        """set_value_from_xy_index

        return bool flag, which means setting value is succeeded or not

        :param x_ind: x index
        :param y_ind: y index
        :param val: grid value FloatGrid(1.0)
        """
        # print("x {} y {}".format(x_ind,y_ind))
        if (x_ind is None) or (y_ind is None):
            return False

        grid_ind = int(y_ind * self.width + x_ind) # find pixel index in map
        if 0 <= grid_ind < self.n_data and isinstance(val, self.data_type):
            self.data[grid_ind] = val
            return True  # OK
        else:
            return False  # NG

    def set_value_from_polygon(self, pol_x, pol_y, val, inside=True):
        """set_value_from_polygon

        Setting value inside or outside polygon

        :param pol_x: x position list for a polygon
        :param pol_y: y position list for a polygon
        :param val: grid value FloatGrid(1.0)
        :param inside: setting data inside or outside
        """
        #pass inside = false => focus point outbound area 


        # making ring polygon
        if (pol_x[0] != pol_x[-1]) or (pol_y[0] != pol_y[-1]):
            pol_x =np.append(pol_x,pol_x[0])
            pol_y = np.append(pol_y,pol_y[0])
            print("make ring")
        # setting value for all grid
        l_x_pos = []
        l_y_pos =[] 
        for x_ind in range(self.width):
            for y_ind in range(self.height):
                x_pos, y_pos = self.calc_grid_central_xy_position_from_xy_index(
                    x_ind, y_ind)
                # l_x_pos.append(x_pos)
                # l_y_pos.append(y_pos)
                # print("x {} y {}".format(x_ind,y_ind))
                flag = self.check_inside_polygon(x_pos, y_pos, pol_x, pol_y)
                if flag is inside:
                   self.set_value_from_xy_index(x_ind, y_ind, val) # set data on grid
                #    if(y_pos == 0):
                    # print("use flag FALSE  x {}  y {}".format(x_pos,y_pos))

                   l_x_pos.append(x_pos)
                   l_y_pos.append(y_pos)
        return l_x_pos,l_y_pos
    def calc_grid_index_from_xy_index(self, x_ind, y_ind):
        grid_ind = int(y_ind * self.width + x_ind)
        return grid_ind

    def calc_xy_index_from_grid_index(self, grid_ind):
        y_ind, x_ind = divmod(grid_ind, self.width)
        return x_ind, y_ind

    def calc_grid_index_from_xy_pos(self, x_pos, y_pos):
        """get_xy_index_from_xy_pos

        :param x_pos: x position [m]
        :param y_pos: y position [m]
        """
        x_ind = self.calc_xy_index_from_position(
            x_pos, self.left_lower_x, self.width)
        y_ind = self.calc_xy_index_from_position(
            y_pos, self.left_lower_y, self.height)

        return self.calc_grid_index_from_xy_index(x_ind, y_ind)

    def calc_grid_central_xy_position_from_grid_index(self, grid_ind):
        x_ind, y_ind = self.calc_xy_index_from_grid_index(grid_ind)
        return self.calc_grid_central_xy_position_from_xy_index(x_ind, y_ind)

    def calc_grid_central_xy_position_from_xy_index(self, x_ind, y_ind):
        x_pos = self.calc_grid_central_xy_position_from_index(
            x_ind, self.left_lower_x,self.resolutionH)
        y_pos = self.calc_grid_central_xy_position_from_index(
            y_ind, self.left_lower_y,self.resolutionV)
        return x_pos, y_pos

    def calc_grid_central_xy_position_from_index(self, index, lower_pos,resolution):
        return lower_pos + index * resolution + resolution / 2.0 # (+)resolution/2.0 for point plot via (0.0,0.0)

    # def calc_xy_index_from_position(self, pos, lower_pos, max_index):
    #     ind = int(np.floor((pos - lower_pos) / self.resolution))
    #     if 0 <= ind <= max_index:
    #         return ind
    #     else:
    #         return None

    def check_occupied_from_xy_index(self, x_ind, y_ind, occupied_val):
        #occupied_val pass default value FloatGrid().data =1
        val = self.get_value_from_xy_index(x_ind, y_ind)  ## return FloatGrid()
        #val is data[grid_index] from you set before
        if val is None or val.data >= occupied_val.data:# if outside val have value 1.0 occupied_val = 0.5
        
            return True
        else: # floatgrid.data == 0.0
            # print("false y {}  x {} = val grig {}".format(y_ind,x_ind,val.data))
            return False

    def expand_grid(self, occupied_val=FloatGrid(1.0)):
        x_inds, y_inds, values = [], [], []
        l_x_pos = []
        l_y_pos =[] 
        #this function assign value point around this current point
        #this func focus point outbound area 
        # print("expand ----------------------------")
        for ix in range(self.width):
            for iy in range(self.height):
                if self.check_occupied_from_xy_index(ix, iy, occupied_val):
                    x_inds.append(ix)
                    y_inds.append(iy)
                    x_pos, y_pos = self.calc_grid_central_xy_position_from_xy_index(
                    ix, iy)
                    l_x_pos.append(x_pos)
                    l_y_pos.append(y_pos)
                    values.append(self.get_value_from_xy_index(ix, iy))
        # print("end ----------------------------")

        for (ix, iy, value) in zip(x_inds, y_inds, values):
            self.set_value_from_xy_index(ix + 1, iy, val=value)
            self.set_value_from_xy_index(ix, iy + 1, val=value)
            # self.set_value_from_xy_index(ix + 1, iy + 1, val=value)
            self.set_value_from_xy_index(ix - 1, iy, val=value)
            self.set_value_from_xy_index(ix, iy - 1, val=value)
            # self.set_value_from_xy_index(ix - 1, iy - 1, val=value)
        return l_x_pos,l_y_pos

    @staticmethod
    def check_inside_polygon(iox, ioy, x, y): #iox, ioy is point\\ x,y is list area

        n_point = len(x) - 1
        inside = False
        for i1 in range(n_point):
            i2 = (i1 + 1) % (n_point + 1)

            if x[i1] >= x[i2]:
                min_x, max_x = x[i2], x[i1]
            else:
                min_x, max_x = x[i1], x[i2]
            if not min_x <= iox < max_x:  # point iox inside point between is false
                continue                  # go to next check
            if x[i2] == x[i1]:
                continue            
            tmp1 = (y[i2] - y[i1]) / (x[i2] - x[i1]) # m
          
            if (y[i1] + tmp1 * (iox - x[i1]) - ioy) > 0.0: # y[i1] is new origin # find y from m*vector x
                inside = not inside
                
        return inside

    def print_grid_map_info(self):
        print("width:", self.width)
        print("height:", self.height)
        print("resolution vertical:", self.resolutionV)
        print("resolution Horizontal:", self.resolutionH)
        print("center_x:", self.center_x)
        print("center_y:", self.center_y)
        print("left_lower_x:", self.left_lower_x)
        print("left_lower_y:", self.left_lower_y)
        print("n_data:", self.n_data)

    def plot_grid_map(self, ax=None):
        float_data_array = np.array([d.get_float_data() for d in self.data])
        grid_data = np.reshape(float_data_array, (self.height, self.width))
        if not ax:
            fig, ax = plt.subplots()
        heat_map = ax.pcolor(grid_data, cmap="Blues", vmin=0.0, vmax=1.0)
        plt.axis("equal")

        return heat_map
