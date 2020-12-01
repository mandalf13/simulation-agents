from environment import *

if __name__=='__main__':
    # Testing                               
    n, m, dirt_p, obs_p, count_kids, t = 10, 10, 10, 10, 5, 10
    e = Environment(n, m, dirt_p, obs_p, count_kids)
    kids = e.generate_kids(count_kids)
    robot_start = e.generate_robot_start()
    robot = RobotA('A', robot_start[0], robot_start[1])
    #robot = RobotB('B', robot_start[0], robot_start[1])
    total = 100 * t
    time_ended = False
    for time in range(1, total):
        if robot_finished(e, kids, robot):
            print("!!Robot Succeded!!")
            break
        if robot_failed(e):
            print("!!Dirt Invasion!!")
            break
        if time % 10 == 0:
            e.generate_obstacles(e.obs_percent)
        robot.action(e, kids)
        for kid in kids.values():
            kid.action(e)              
        #print(e)
        print_env(e,robot)
        if time == total - 1: time_ended = True
    if time_ended:
        if robot_succeded(e,kids,robot):
            print("!!The Robot has kept the house clean!!")
        else:
            print("!!The Robot has failed!!")