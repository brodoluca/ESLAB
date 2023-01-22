import numpy as np
import  cv2
import time
import itertools
SHOW_ASTAR = False
ROOM_OVER_OBSTACLE = 30
WIDTH_OBSTACLE = 42
HEIGHT_OBSTACLE = 90.0
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    def __str__(self) -> str:
        return str(str(self.position[0])+","+str(self.position[1]))
    def __eq__(self, other):
        return self.position == other.position

def intersect(p1, p2, p3, p4):
    x1,y1 = p1
    x2,y2 = p2
    x3,y3 = p3
    x4,y4 = p4
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
    if denom == 0: # parallel
        return None
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
    if ua < 0 or ua > 1: # out of range
        return None
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom
    if ub < 0 or ub > 1: # out of range
        return None
    x = x1 + ua * (x2-x1)
    y = y1 + ua * (y2-y1)
    return (x,y)


class PathFinder():
    def __init__(self, maze,start,end_points, obstacles):
        self.__maze = maze
        self.start = start
        self.end_points = end_points
        self.__solver = self.astar
        self.__obsta = obstacles
        self.solved = False
        #print(self.__obsta)
    def __map_to_real_world(self,value):
        return float(value)*(1.30) / float(480)

    def __get_ordered_list(self,points, x, y, reverse = False):
        points.sort(key = lambda p: (p[0] - x)**2 + (p[1] - y)**2, reverse = reverse)
        return points

    def set_solver(self,func):
        self.__solver = func
    def optimize_commands(self,direction_points):
        outcome = []
        path_number = 0
        for path in self.paths:
             if(path_number == 0):
                 start_point = self.start
             else:
                 start_point = self.end_points[path_number-1]
             for direction_point in direction_points[path_number]:
                        distance = np.sqrt(np.sum(np.square(np.asarray(start_point) - np.asarray(direction_point ))))
                        distance = self.__map_to_real_world(distance)
                        angle = np.arctan2((start_point[1] - direction_point[1]) ,( start_point[0] -direction_point[0])) #we are using x and y inverted, so we have to invert them again for real word
                        angle = (angle+np.pi/2)* 180 / np.pi
                        angle = angle if angle>=0 else 360 + angle
                        outcome.append((str(distance), str(angle)))
                        start_point= direction_point
             path_number+=1 
        return outcome
    def __solve(self, start,end_points):
        if(start is None or end_points is None):
            self.__path = self.__solver(self.__maze,self.start,self.end_points)
        else:
            self.__path = self.__solver(self.__maze,start,end_points)
        return self.__path
    def solve(self):
        self.paths = []
        start = self.start
        end_points = self.end_points.copy()
        new_end_points = []
        while(len(end_points)>0):
            #print(end_points)
            #print(new_end_points)
            self.paths.append(self.__solve(start,end_points))
            #print(solver.end_points)
            #print(self.paths[-1][-1])
            new_end_points.append(self.paths[-1][-1])
            end_points.remove(self.paths[-1][-1])
            #print(solver.end_points)
            #print(self.paths[-1][-1])
            start = self.paths[-1][-1]
            #print(start)
        self.end_points = new_end_points # we update the list, so we have them in order of path
        self.solved = True
        return self.paths

    #gets a path as input
#returns a list of tuples in the form
# [(direction, distance), ]
#direction and distance are strings
#direction is angle to turn to (careful, not the amount to turn, but the direction)
# if we are at -45, for example, and the angle is 0, we turn 45,i.e. until reaching 0.
#time is for how much (at constant speed)
    def __get_euclidean_norms(self,path):
        sums = []
        for i in range(1,len(path)):
            sums.append( np.linalg.norm(np.array(path[i]) - np.array(path[i-1])))
        return sums
    

    def determine_rover_commands(self, start = None):
        if(start is not None):
            self.start = start
        if(not self.solved):
            self.solve()
        outcome = []
        direction_points = {}
        path_number = 0
        for path in self.paths:
            temp = self.__get_euclidean_norms(path)
            b_points = []
            for i in range(1,len(temp)):
                if(temp[i] != temp[i-1] ):
                    b_points.append(path[i])
            b_points.append(self.end_points[path_number])
            direction_points[path_number] = b_points
            #print(direction_points)
            if(path_number == 0): # if we are considering the first path, then we take the start point
                start_point = self.start 
            else: #otherwise we take the end point from the last path
                start_point = self.end_points[path_number-1]
            #print("Path n.: " + str(path_number+1))
            for direction_point in direction_points[path_number]:
                #print(start_point)
                #print(direction_point)
                distance = np.sqrt(np.sum(np.square(np.asarray(start_point) - np.asarray(direction_point ))))
                distance = self.__map_to_real_world(distance)
                angle = np.arctan2((start_point[1] - direction_point[1]) ,( start_point[0] -direction_point[0])) #we are using x and y inverted, so we have to invert them again for real word
                #print(distance)
                #print((angle+np.pi/2)* 180 / np.pi)
                angle = (angle+np.pi/2)* 180 / np.pi
                angle = angle if angle>=0 else 360 + angle
                outcome.append((str(distance), str(angle)))
                start_point= direction_point
            path_number+=1 
        #print(direction_point)
        self.outcome = outcome
        #(outcome)
        return outcome, direction_points



    def optimize_path(self,maze, p,starting_point,end_points):
        ep = end_points.copy()
        obstacles = maze[4]
        orientation_obstacles = maze[5]
        for a,path in zip(p,p.values()): 
                f = 1
                b=0
                point = path[-f]
                save_old_status = []

                continu = True


                while(continu ):
                    no_obstacle = True

                    for (orientation,obstacle) in zip(orientation_obstacles,obstacles):

                        if(orientation == 1):
                            first_point = (int(obstacle[1] -8 - WIDTH_OBSTACLE/2  - ROOM_OVER_OBSTACLE+5),int(obstacle[0]+(8)-HEIGHT_OBSTACLE/2  - ROOM_OVER_OBSTACLE+5))
                            second_point = (int(obstacle[1] -8 + WIDTH_OBSTACLE/2 + ROOM_OVER_OBSTACLE-5),int(obstacle[0]+(8)-HEIGHT_OBSTACLE/2  - ROOM_OVER_OBSTACLE+5))
                            third_point = (int(obstacle[1] -8 - WIDTH_OBSTACLE/2  - ROOM_OVER_OBSTACLE+ 5),int(obstacle[0]+(8)+HEIGHT_OBSTACLE/2  + ROOM_OVER_OBSTACLE-5))
                            fourth_point = (int(obstacle[1] -8 + WIDTH_OBSTACLE/2  + ROOM_OVER_OBSTACLE-5),int(obstacle[0]+(8)+HEIGHT_OBSTACLE/2  + ROOM_OVER_OBSTACLE-5))
                        else:
                            first_point = (int(obstacle[1] -8 - HEIGHT_OBSTACLE/2  - ROOM_OVER_OBSTACLE+5),int(obstacle[0]+(8)-WIDTH_OBSTACLE/2  - ROOM_OVER_OBSTACLE+5))
                            second_point = (int(obstacle[1] -8 + HEIGHT_OBSTACLE/2 + ROOM_OVER_OBSTACLE-5),int(obstacle[0]+(8)-WIDTH_OBSTACLE/2  - ROOM_OVER_OBSTACLE+5))
                            third_point = (int(obstacle[1] -8 - HEIGHT_OBSTACLE/2  - ROOM_OVER_OBSTACLE+ 5),int(obstacle[0]+(8)+WIDTH_OBSTACLE/2  + ROOM_OVER_OBSTACLE-5))
                            fourth_point = (int(obstacle[1] -8 + HEIGHT_OBSTACLE/2  + ROOM_OVER_OBSTACLE-5),int(obstacle[0]+(8)+WIDTH_OBSTACLE/2  + ROOM_OVER_OBSTACLE-5))
                    
                        
                        rectangle = [first_point,second_point,third_point,fourth_point]
                        for permutation in itertools.permutations(rectangle):
                            if intersect(starting_point,point,permutation[0],permutation[1]) is not None:
                                no_obstacle = False
                                break
                    if(no_obstacle):
                                    save_old_status.append(point)
                                    if(point in ep):
                                        ep.remove(point)
                                    continu = False
                                    
                
                    if(no_obstacle and point not in end_points):
                            starting_point = path[-f]
                            f=1
                            continu = True
                    else:
                        f+=1
                        if(f>len(path)-1):
                          save_old_status = p[a]
                          continu = False
                        
                    point = path[-f]
                p[a] = save_old_status
                starting_point = end_points[a]
                
        return p
   

    def astar(self,maze, start, ends):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""
        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0

        #convert to list
        end_nodes = []
        sorted_ends = self.__get_ordered_list(ends, start[0],start[1], True)
        
        for end in sorted_ends:
            end_node = Node(None, end)
            end_node.g = end_node.h = end_node.f = 0
            end_nodes.append(end_node)

        #end_node = end_nodes[0]

        #print(end_node)
        # Initialize both open and closed list
        open_list = []
        closed_list = []
        #print(start_node)
        # Add the start node
        open_list.append(start_node)
        #cv2.imshow("Maze",np.array(maze))
        #cv2.waitKey(0)
        # Loop until you find the end
        while len(open_list) > 0:
            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)



            # Found the goal
            for end_n in end_nodes:
                if current_node == end_n:
                    path = []
                    current = current_node
                    while current is not None:
                        path.append(current.position)
                        current = current.parent
                    return path[::-1] # Return reversed path

            # Generate children
            children = []
            steps_dimensions = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
            #for obstacle in self.__obsta:
            dist = np.linalg.norm(np.array(current_node.position) - np.array(end_node.position))
            #print(dist)
            
            #if(dist>5):
             #   a = 5
              #  steps_dimensions = [(0, -a), (0, a), (-a, 0), (a, 0), (-a, -a), (-a, a), (a, -a), (a, a)]
            if(dist > 25):
                    a = 25
                    steps_dimensions = [(0, -a), (0, a), (-a, 0), (a, 0), (-a, -a), (-a, a), (a, -a), (a, a)]

            for new_position in steps_dimensions:
                                #  (0, -2), (0, 2), (-2, 0), (2, 0), (-2, -2), (-3, 3), (2, -2), (2, 2),
                                # (0, -3), (0, 3), (-3, 0), (3, 0), (-3, -3), (-4, 4), (3, -3), (3, 3),
                                    #(0, -4), (0, 4), (-4, 0), (4, 0), (-4, -4), (-5, 5), (4, -4), (4, 4),
                                    #(0, -5), (0, 5), (-5, 0), (5, 0), (-5, -5), (-2, 2), (5, -5), (5, 5),
                                    #(0, -6), (0, 6), (-6, 0), (6, 0), (-6, -6), (-1, 1), (1, -1), (1, 1),
                                    #(0, -7), (0, 7), (-7, 0), (7, 0), (-7, -7), (-3, 3), (2, -2), (2, 2),
                                    #(0, -8), (0, 8), (-8, 0), (8, 0), (-8, -8), (-4, 4), (3, -3), (3, 3),
                                    #(0, -9), (0, 9), (-9, 0), (9, 0), (-9, -9), (-5, 5), (4, -4), (4, 4),
                                    #(0, -10), (0, 10), (-10, 0), (10, 0), (-10, -10), (-10, 10), (10, -10), (10, 10),

                                    #(0, -20), (0, 20), (-20, 0), (20, 0), (-20, -20), (-20, 10), (20, -20), (20, 20),
                                    
                                    #(0, -5), (0, 5), (-5, 0), (5, 0), (-5, -5), (-2, 2), (5, -5), (5, 5),
                                    #(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),
                                    
                                    #]: # Adjacent squares
                #print("times")
                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                far_from_border = 0
                if node_position[0] > (len(maze) - 1+far_from_border) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1)-far_from_border or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if maze[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                #
                #child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.h = np.linalg.norm(np.array(child.position) - np.array(end_node.position))
                #print(self.__obsta)
                temp = self.__get_ordered_list(self.__obsta, current_node.position[0], current_node.position[1], False)
                

                #child.h = abs(child.position[0]-end_node.position[0]) + abs(child.position[1]-end_node.position[1]) 
                #print(child.h)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)
                #open_list = __get_ordered_list(open_list,end_node.position[0],end_node.position[1])
                #cv2.imshow("Test",image_scenario)
                maze[current_node.position[0]][current_node.position[1]]=1
                if(SHOW_ASTAR):
                    #maze[current_node.position[0]][current_node.position[1]]=1
                    cv2.imshow("Maze",np.array(maze))
                    #time.sleep(0.001)
                    cv2.waitKey(1)
                    #cv2.destroyAllWindows()

    
