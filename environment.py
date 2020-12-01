import random
import math
Dirt, Obs, Corral = '■', '×', 'C'

#Constants
Directions = [(-1,0), (0,1), (1,0), (0,-1), (-1,1), (1,1), (1,-1), (-1,-1)] # N, E, S, W, NE, SE, SW, NW

#Utils
def is_in_range(pos, n, m):
    return pos[0] >= 0 and pos[0] < n and pos[1] >= 0 and pos[1] < m    

def print_env(env, robot):
    s = ''
    space = '  '
    for i in range(env.n):
        s += '\n'
        for j in range(env.m):
            if env.kids[i][j] != 0:
                if (robot.x, robot.y) == (i, j):
                    if len(repr(env.kids[i][j])) == 2:
                        s += 'R' + repr(env.kids[i][j])
                    else:
                        s += 'R' + repr(env.kids[i][j]) + ' '   
                else:
                    if len(repr(env.kids[i][j])) == 2:
                        s += repr(env.kids[i][j]) + ' ' 
                    else:
                        s += repr(env.kids[i][j]) + space     
            elif (robot.x, robot.y) == (i, j):
                s += 'R' + space
            elif env.environment[i][j] == None:
                s+='-' + space
            else:
                s+=env.environment[i][j] + space   
    print(s)

def robot_finished(env, kids, robot):
    return robot.all_kids_captured(env, kids) and env.dirt_cells == 0
   
def robot_failed(env):
    return env.dirt_invasion()

def robot_succeded(env, kids, robot):
    return robot_finished(env, kids, robot) or not robot_failed(env)

class Environment:
    def __init__(self, n, m, dirt_percent, obs_percent, count_kids):
        self.n = n
        self.m = m
        self.obs_percent = obs_percent
        self.count_kids = count_kids
        self.dirt_cells = 0
        self.environment = [[None for i in range(0,m)] for j in range(0,n)]
        self.kids = [[0 for i in range(0,m)] for j in range(0,n)]
        self.robot = None
        self.generate_environment(dirt_percent, obs_percent, count_kids)
        
    def dirt_invasion(self):
        obs = math.floor((self.obs_percent * self.n * self.m) / 100)
        dirt = (self.dirt_cells * 100) / (self.n*self.m - self.count_kids - obs)
        return dirt >= 60     

    def __repr__(self):
        s = ''
        for i in range(self.n):
            s += '\n'
            for j in range(self.m):
                if self.kids[i][j] != 0:
                    s+=repr(self.kids[i][j]) + ' '
                elif self.environment[i][j] == None:
                    s+='- '
                else:
                    s+=self.environment[i][j] + ' '   
        return s

    def generate_robot_start(self):
        while True:
           pos = (random.randint(0, self.n-1), random.randint(0, self.m-1))
           if self.environment[pos[0]][pos[1]] == Obs or self.kids[pos[0]][pos[1]] != 0: continue
           else:
               self.robot = pos 
               return pos
        
    def generate_kids(self, count):
        kids = {}
        id = 1
        while count > 0:
            pos = (random.randint(0, self.n-1), random.randint(0, self.m-1))
            if self.is_empty(pos):
                self.kids[pos[0]][pos[1]] = id
                current_kid = Kid(id, pos[0], pos[1])
                kids[id]=current_kid
                count -= 1
                id += 1
        return kids

    def generate_environment(self, dirt_percent, obs_percent, count_kids):
        kids_percent = count_kids * 100 / self.n*self.m
        assert dirt_percent >= 60 or dirt_percent + obs_percent + kids_percent*2 > 99, Exception("Invalid initialization values")
        self.generate_corral(count_kids)
        self.generate_dirt(dirt_percent)
        self.generate_obstacles(obs_percent)   
        return

    def generate_corral(self, count):
        pos = (random.randint(0, self.n-1), random.randint(0, self.m-1))
        self.environment[pos[0]][pos[1]]= Corral
        count -= 1
        queue = [pos]
        while True:
            pos = queue[0]
            queue.remove(queue[0])
            for i in range(0,4):
                d = Directions[i]
                nextpos = (pos[0]+d[0], pos[1]+d[1])
                if is_in_range(nextpos, self.n, self.m):    
                    if self.environment[nextpos[0]][nextpos[1]] == None:
                        self.environment[nextpos[0]][nextpos[1]] = Corral
                        count -= 1
                        if count == 0: return
                        queue.append(nextpos)

    def generate_dirt(self, dirt_percent):
        count = math.floor((dirt_percent * self.n * self.m) / 100)
        while count > 0:
            pos = (random.randint(0, self.n-1), random.randint(0, self.m-1))
            if self.environment[pos[0]][pos[1]] == None:
                self.environment[pos[0]][pos[1]]= Dirt
                count-=1
                self.dirt_cells += 1
        return        

    def generate_obstacles(self, obs_percent):
        while True:
            for i in range(self.n):
                for j in range(self.m):
                    if self.environment[i][j]==Obs:
                        self.environment[i][j]=None
            count = math.floor((obs_percent * self.n * self.m) / 100)
            while count > 0:
                pos = (random.randint(0, self.n-1), random.randint(0, self.m-1))
                if self.is_empty(pos):
                    self.environment[pos[0]][pos[1]]= Obs
                    count -= 1
            if self.validate_obstacles(obs_percent): return     

    def validate_obstacles(self, obs_percent):
        count = self.n * self.m - math.floor((obs_percent * self.n * self.m) / 100)
        pos = (random.randint(0, self.n-1), random.randint(0, self.m-1))
        while self.environment[pos[0]][pos[1]] == Obs:
            pos = (random.randint(0, self.n-1), random.randint(0, self.m-1))

        tree = [[False for i in range(0,self.m)] for j in range(0,self.n)]
        queue = [pos]
        tree[pos[0]][pos[1]] = True
        while len(queue) > 0:
            pos = queue[0]
            queue.remove(queue[0])
            count -= 1
            for i in range(0,4):
                d = Directions[i]
                nextpos = (pos[0]+d[0], pos[1]+d[1])
                if is_in_range(nextpos, self.n, self.m):     
                    if self.environment[nextpos[0]][nextpos[1]] != Obs and not tree[nextpos[0]][nextpos[1]]:
                        queue.append(nextpos)
                        tree[nextpos[0]][nextpos[1]]=True
        if count == 0: return True
        return False       

    def is_empty(self, pos):
        return self.environment[pos[0]][pos[1]] == None and self.kids[pos[0]][pos[1]] == 0 and pos != self.robot

    def min_reachable_corral(self):
        reachability = 0
        corral = ()
        for i in range(self.n):
            for j in range(self.m):
                elem = self.environment[i][j]
                if (elem == Corral and self.kids[i][j] == 0) or (elem == Corral and self.kids[i][j] != 0 and self.robot == (i,j)):
                    current = self.calculate_reachability((i,j))
                    if current > reachability:
                        reachability = current
                        corral = (i,j)
        return corral

    def calculate_reachability(self, corral):
        count = 0
        for i in range(0, 4):
            d = Directions[i]
            pos = (corral[0]+d[0], corral[1]+d[1])
            if not is_in_range(pos, self.n, self.m): 
                count += 1.1
            elif self.environment[pos[0]][pos[1]] == Corral:
                 count += 1
            elif self.environment[pos[0]][pos[1]] == Obs:
                count += 1.1     
        return count

class Agent:
    def __init__(self, id, pos_x, pos_y):
        self.id = id
        self.x = pos_x
        self.y = pos_y

class Kid(Agent):
    def __init__(self, id, pos_x, pos_y):
        super().__init__(id, pos_x, pos_y)
        self.enable = True

    def action(self, env, showlog=False):
        if not self.enable: return
        i = random.randint(0, 3)
        d = Directions[i]
        nextpos = (self.x+d[0], self.y+d[1])
        if is_in_range(nextpos, env.n, env.m):
            elem = env.environment[nextpos[0]][nextpos[1]]
            if elem == Dirt or elem == Corral or env.kids[nextpos[0]][nextpos[1]] != 0 or nextpos == env.robot:
                if showlog == True:
                    print("Kid {0} in {1} cannot move to {2}".format(self.id, repr((self.x, self.y)), repr(nextpos)))
            elif elem == Obs:
                if self.move_obstacles(d, env):
                    if showlog == True:
                        print("Kid {0} in {1} move to {2}".format(self.id, repr((self.x, self.y)), repr(nextpos)))
                    self.mess_and_move(d, env)                
                else:
                    if showlog == True:
                        print("Kid {0} in {1} cannot move to {2}".format(self.id, repr((self.x, self.y)), repr(nextpos)))
            else:
                if showlog == True:
                    print("Kid {0} in {1} move to {2}".format(self.id, repr((self.x, self.y)), repr(nextpos)))
                self.mess_and_move(d, env)
        else:
            if showlog == True:
                print("Kid {0} in {1} cannot move to {2}".format(self.id, repr((self.x, self.y)), repr(nextpos)))
        return        

    def move_obstacles(self, direction, env):
        pos = (self.x + direction[0], self.y + direction[1])
        if self.move_obstacles_rec(pos, direction, env):
            env.environment[pos[0]][pos[1]] = None
            return True
        return False    

    def move_obstacles_rec(self, pos, direction, env):
        nextpos = (pos[0] + direction[0], pos[1] + direction[1])
        if not is_in_range(nextpos, env.n, env.m): return False
        elif env.environment[nextpos[0]][nextpos[1]] == Obs:
            return self.move_obstacles_rec(nextpos, direction, env)
        elif env.is_empty(nextpos):
            env.environment[nextpos[0]][nextpos[1]] = Obs
            return True
        else: return False

    def mess_and_move(self, direction, env):
        density = self.calculate_density(env)
        dirty_cells = 0
        env.kids[self.x][self.y] = 0
        if env.robot != (self.x, self.y):
            env.environment[self.x][self.y] = Dirt
            env.dirt_cells += 1
            dirty_cells += 1
            if density == 0:
                self.x += direction[0]
                self.y += direction[1]
                env.kids[self.x][self.y]=self.id
                return
        for d in Directions:
            pos = (self.x+d[0], self.y+d[1])
            if is_in_range(pos, env.n, env.m) and d != direction:
                if env.is_empty(pos):
                    env.environment[pos[0]][pos[1]] = Dirt
                    env.dirt_cells += 1
                    dirty_cells += 1
                    if (density == 1 and dirty_cells == 3) or (density == 0 and dirty_cells == 1): break
        self.x += direction[0]
        self.y += direction[1]
        env.kids[self.x][self.y]=self.id
        return               

    def calculate_density(self, env):
        density = 0
        for d in Directions:
            pos = (self.x+d[0], self.y+d[1])
            if is_in_range(pos, env.n, env.m):
                if env.kids[pos[0]][pos[1]] != 0 and env.environment[pos[0]][pos[1]] != Corral:
                    density += 1
        return density

class Robot(Agent):
    def __init__(self, id, pos_x, pos_y):
        super().__init__(id, pos_x, pos_y)
        self.kid_id = None

    def action(self, env, kids):
        # The robot is not carrying a kid
        if self.kid_id == None:
            if env.environment[self.x][self.y] == Dirt:
                env.environment[self.x][self.y] = None
                env.dirt_cells -= 1
                return
            elif self.all_kids_captured(env, kids):
                path = self.find_path_to_dirt(env)
                if path == None or len(path) == 0: return
                next_pos = path[-1]
                self.move_alone(next_pos, env)
            else:
                path = self.find_path_to_kid(env, kids)
                if path == None or len(path) == 0: return
                next_pos = path[-1]
                # If there is a kid in the current cell of the robot
                if env.kids[self.x][self.y] != 0:
                    # If there is a kid at distance 1 
                    if env.kids[next_pos[0]][next_pos[1]] != 0:
                        if env.environment[self.x][self.y] != Corral:
                            kids[env.kids[self.x][self.y]].enable = True
                        self.move_and_take_kid(next_pos, env, kids)    
                    # The closest kid is in the current cell    
                    else:
                        # If the current cell is a not corral then take the kid
                        if env.environment[self.x][self.y] != Corral:
                            captured_kid = env.kids[self.x][self.y]
                            kids[captured_kid].enable = False
                            self.kid_id = captured_kid
                        else:
                            self.move_alone(next_pos, env)    
                # Elif there is a kid in the next cell of the minimum path        
                elif env.kids[next_pos[0]][next_pos[1]] != 0:
                    self.move_and_take_kid(next_pos, env, kids) 
                # There is no kid in the next cell of the minimum path
                else:
                    self.move_alone(next_pos, env)
        # The robot is carrying a kid
        else:
            path = self.find_path_to_corral(env)
            if path == None or len(path) == 0: return
            next_pos = path[-1]
            # If the robot is in the corral already
            if next_pos == (self.x, self.y):
                self.kid_id = None
            # There is a kid in the next cell of the path to the corral   
            elif env.kids[next_pos[0]][next_pos[1]] != 0:
                if env.environment[self.x][self.y] == Dirt:
                    for i in range(0,4):
                        d = Directions[i]
                        next_pos = (self.x + d[0], self.y + d[1])
                        if is_in_range(next_pos, env.n, env.m):
                            if env.environment[next_pos[0]][next_pos[1]] != Obs and env.kids[next_pos[0]][next_pos[1]] == 0:
                                self.move_with_kid(next_pos, env, kids)                           
                else:
                    self.kid_id = None

            # There is no kid in the next cell of the path to the corral    
            else:
                if len(path) == 1:
                    self.move_with_kid(next_pos, env, kids)  
                else:
                    sec_pos = path[-2]
                    if env.environment[sec_pos[0]][sec_pos[1]] != Obs and env.kids[sec_pos[0]][sec_pos[1]] == 0:
                        self.move_with_kid(sec_pos, env, kids)                   
                    else:
                        self.move_with_kid(next_pos, env, kids)                
        return

    def move_and_take_kid(self, next_pos, env, kids):
        captured_kid = env.kids[next_pos[0]][next_pos[1]]
        kids[captured_kid].enable = False
        self.x = next_pos[0]
        self.y = next_pos[1]
        self.kid_id = captured_kid
        env.robot = next_pos

    def move_alone(self, next_pos, env):
        self.x = next_pos[0]
        self.y = next_pos[1]
        env.robot = next_pos

    def move_with_kid(self, next_pos, env, kids):
        kid_id = self.kid_id
        env.kids[self.x][self.y] = 0
        env.kids[next_pos[0]][next_pos[1]] = kid_id
        kids[kid_id].x = next_pos[0]
        kids[kid_id].y = next_pos[1]
        self.x = next_pos[0]
        self.y = next_pos[1]
        env.robot = next_pos

    def find_path_to_corral(self, env):
        corral = env.min_reachable_corral()
        success = False
        if corral[0] == self.x and corral[1] == self.y:
            return [corral]
        tree = [[None for i in range(0,env.m)] for j in range(0,env.n)]
        trace = [[False for i in range(0,env.m)] for j in range(0,env.n)]
        pos = (self.x, self.y)
        queue = [pos]
        tree[pos[0]][pos[1]] = None
        trace[pos[0]][pos[1]] = True 
        while len(queue) > 0:
            pos = queue[0]
            queue.remove(queue[0])
            for i in range(0,4):
                d = Directions[i]
                nextpos = (pos[0]+d[0], pos[1]+d[1])
                if is_in_range(nextpos, env.n, env.m):
                    if nextpos == corral:
                        tree[nextpos[0]][nextpos[1]] = pos
                        queue = []
                        success = True
                        break
                    elem = env.environment[nextpos[0]][nextpos[1]]
                    if (env.kids[nextpos[0]][nextpos[1]] != 0 and elem == Corral) or elem == Obs or trace[nextpos[0]][nextpos[1]]:
                        continue
                    else:
                        queue.append(nextpos)
                        tree[nextpos[0]][nextpos[1]] = pos
                        trace[nextpos[0]][nextpos[1]] = True
        if not success: return None
        return self.build_path(corral, tree)

    def find_path_to_dirt(self, env):
        tree = [[None for i in range(0,env.m)] for j in range(0,env.n)]
        trace = [[False for i in range(0,env.m)] for j in range(0,env.n)]
        closest_dirt = ()
        pos = (self.x, self.y)
        queue = [pos]
        tree[pos[0]][pos[1]] = None
        trace[pos[0]][pos[1]] = True
        while len(queue) > 0:
            pos = queue[0]
            queue.remove(queue[0])
            for i in range(0,4):
                d = Directions[i]
                nextpos = (pos[0]+d[0], pos[1]+d[1])
                if is_in_range(nextpos, env.n, env.m):
                    if env.environment[nextpos[0]][nextpos[1]] == Dirt and trace[nextpos[0]][nextpos[1]] == False:
                        closest_dirt = nextpos
                        tree[nextpos[0]][nextpos[1]] = pos
                        queue = []
                        break
                    if env.environment[nextpos[0]][nextpos[1]] == None and trace[nextpos[0]][nextpos[1]] == False:
                        queue.append(nextpos)
                        tree[nextpos[0]][nextpos[1]] = pos
                        trace[nextpos[0]][nextpos[1]] = True
        if closest_dirt == (): return None
        return self.build_path(closest_dirt, tree)

    def build_path(self, end, tree):
        reverse_path = [end]
        next_cell = tree[end[0]][end[1]]
        while True:
            if next_cell != None:
                reverse_path.append(next_cell)
                next_cell = tree[next_cell[0]][next_cell[1]]
            else: break
        return reverse_path[:-1]

    def all_kids_captured(self, env, kids):
        if self.kid_id != None: return False
        for kid in kids.values():
            if env.environment[kid.x][kid.y] != Corral: return False
        return True

# Search for closest kid
class RobotA(Robot):
    def find_path_to_kid(self, env, kids):  
        tree = [[None for i in range(0,env.m)] for j in range(0,env.n)]
        trace = [[False for i in range(0,env.m)] for j in range(0,env.n)]
        closest_kid = ()
        pos = (self.x, self.y)
        queue = [pos]
        tree[pos[0]][pos[1]] = None
        trace[pos[0]][pos[1]] = True
        while len(queue) > 0:
            pos = queue[0]
            queue.remove(queue[0])
            for i in range(0,4):
                d = Directions[i]
                nextpos = (pos[0]+d[0], pos[1]+d[1])
                if is_in_range(nextpos, env.n, env.m):
                    if env.kids[nextpos[0]][nextpos[1]] != 0 and env.environment[nextpos[0]][nextpos[1]] != Corral and trace[nextpos[0]][nextpos[1]] == False:
                        closest_kid = nextpos
                        tree[nextpos[0]][nextpos[1]] = pos
                        queue = []
                        break
                    if env.environment[nextpos[0]][nextpos[1]] != Obs and trace[nextpos[0]][nextpos[1]] == False:
                        queue.append(nextpos)
                        tree[nextpos[0]][nextpos[1]] = pos
                        trace[nextpos[0]][nextpos[1]] = True
        if closest_kid == (): return None
        return self.build_path(closest_kid, tree)

# Search for kid with most kids around
class RobotB(Robot):
    def find_path_to_kid(self, env, kids):
        denser_kid = self.find_denser_kid(env, kids)
        if denser_kid == (): return None
        tree = [[None for i in range(0,env.m)] for j in range(0,env.n)]
        trace = [[False for i in range(0,env.m)] for j in range(0,env.n)]
        pos = (self.x, self.y)
        queue = [pos]
        tree[pos[0]][pos[1]] = None
        trace[pos[0]][pos[1]] = True
        while len(queue) > 0:
            pos = queue[0]
            queue.remove(queue[0])
            for i in range(0,4):
                d = Directions[i]
                nextpos = (pos[0]+d[0], pos[1]+d[1])
                if is_in_range(nextpos, env.n, env.m):
                    if nextpos == denser_kid:
                        tree[nextpos[0]][nextpos[1]] = pos
                        queue = []
                        break
                    elem = env.environment[nextpos[0]][nextpos[1]]
                    if env.kids[nextpos[0]][nextpos[1]] != 0 and elem != Corral and trace[nextpos[0]][nextpos[1]] == False:
                        queue.append(nextpos)
                        tree[nextpos[0]][nextpos[1]] = pos
                        trace[nextpos[0]][nextpos[1]] = True
                    elif env.kids[nextpos[0]][nextpos[1]] == 0 and elem != Obs and trace[nextpos[0]][nextpos[1]] == False:
                        queue.append(nextpos)
                        tree[nextpos[0]][nextpos[1]] = pos
                        trace[nextpos[0]][nextpos[1]] = True
        return self.build_path(denser_kid, tree)

    def find_denser_kid(self, env, kids):
        result = ()
        maximum = 0
        for kid in kids.values():
            if kid.enable:
                density = kid.calculate_density(env) + 1
                if density > maximum:
                    maximum = density
                    result = (kid.x, kid.y)
        return result     

 