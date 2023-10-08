#Authors: Shilpa Mukhopadhyay, Amrita Mohandas
#UIN: 433003777, 534002383
#Instructor: Dr. Jason O'Kane
#TA: Aaron Kingery 
#Course ID: CSCE 752
#Course Title: Robotics and Spatial Intelligence
#Project 2

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, SetPen, Kill
from turtlesim.srv import TeleportAbsolute
import pandas as pd
import numpy as np
import time
from functools import partial
import os
import sys
import math

#global varable to calculate num of line segments allotted to each turtle
num_tasks=0
class MyTurtleNode(Node):
    def __init__(self,  num_of_turtles):
        super().__init__('draw_logo')

        
        self.change_color_os()

        self.declare_parameter('num_turtles', num_of_turtles)

        self.num_turtles = self.get_parameter('num_turtles').value
        

        self.turtle_names = []

        self.linear_x=np.zeros(num_of_turtles)
        self.angular_z=np.zeros(num_of_turtles)
        self.source_x = np.zeros(num_of_turtles)
        self.source_y =np.zeros(num_of_turtles)
        self.goal_x=np.zeros(num_of_turtles)
        self.goal_y = np.zeros(num_of_turtles)
        self.error_curr = np.ones(num_of_turtles) * 500
        self.prev_error=np.zeros(num_of_turtles)

        self.error_threshold_curr =0.19

        self.prev_x=np.zeros(num_of_turtles)
        self.prev_y=np.zeros(num_of_turtles)

        
        self.pen_Client=[]
        self.completed_turtle_count = 0
        
        integer_value = -1   # Replace with the desired integer value

        self.i = np.full(num_of_turtles, integer_value, dtype=int)
        self.prev_i = np.full(num_of_turtles, integer_value, dtype=int)
        self.prev_calc_angle = np.zeros(num_of_turtles)

        self.X1=np.zeros((num_of_turtles,num_tasks))
        self.X2=np.zeros((num_of_turtles,num_tasks))
        self.Y1=np.zeros((num_of_turtles,num_tasks))
        self.Y2=np.zeros((num_of_turtles,num_tasks))
        self.FLAG=np.zeros((num_of_turtles,num_tasks))
        print("shape ofself.X1=", self.X1.shape)

        timer_variable=0.1

        self.turtle_velocity_publisher=[]
        self.turtle_subscriber=[]
        self.timer=[]

        #ambiguity removal

        self.spawn_func()
        time.sleep(4)
        
        #creae publisher subscriber and the callbacks
        for i in range(num_of_turtles):
            param2_cmdvel='/turtle'+str(i+1)+'/cmd_vel'
            self.turtle_velocity_publisher.append( self.create_publisher(Twist, param2_cmdvel, 10) )
            self.timer.append(self.create_timer(timer_variable, partial(self.publisher_callback, i)) )
            param2='/turtle'+str(i+1)+'/pose'
            self.turtle_subscriber.append(self.create_subscription(Pose, param2,partial(self.subscriber_callback,turtleid=i),10))# partial(self.subscriber_callback,X1,Y1,X2,Y2), 10)


        
    #spawn turtles loop and set pen off
    def spawn_func(self):
        self.set_pen_turtle( 1, 255,255,255,0)
        
        for i in range(1, self.num_turtles):
            turtle_name = f'turtle{i + 1}'
            self.spawn_turtles(turtle_name)
            self.turtle_names.append(turtle_name)
            self.set_pen_turtle( 1, 255,255,255,i)
            time.sleep(1)

    #spawn request    
    def spawn_turtles(self, turtle_name):
        spawn_client= self.create_client(Spawn, '/spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "/spawn" not available, waiting...')

        request = Spawn.Request()
        request.name = turtle_name
        request.x = 5.54
        request.y = 5.54
        request.theta = 0.0

        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Spawned turtle: {turtle_name}')
        else:
            self.get_logger().error(f'Failed to spawn turtle: {turtle_name}')

        #self.draw_func()

    #set_pen request
    def set_pen_turtle(self, value, r,g,b,turtleid):

        param2 = '/turtle'+str(turtleid+1)+'/set_pen'
        print("Param2= ", param2)
        pc=self.create_client(SetPen, param2)
        self.pen_Client.append(pc )
        while not self.pen_Client[-1].wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service "/turtlesim/srv/SetPen" not available for turtle{turtleid+1}, waiting...')

        request=SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.off = value 

        future = self.pen_Client[-1].call_async(request)
        print("setpen called")


        if future.result() is not None:
            self.get_logger().info(f'Turtle{turtleid+1} setpen operation done')
        else:
            self.get_logger().error(f'Failed to setpen for turtle{turtleid+1}')

    #change background colors
    def change_color_os(self):
         os.system(f'ros2 param set /turtlesim background_r 80')
         os.system(f'ros2 param set /turtlesim background_g 0')
         os.system(f'ros2 param set /turtlesim background_b 0')

    def change_color(self):
         
         self.clear_client = self.create_client(Empty, '/clear')
         while not self.clear_client.wait_for_service(timeout_sec=1.0):
             self.get_logger().info('Waiting for /clear service')

         self.declare_parameter('background_r',80)
         self.declare_parameter('background_g',0)
         self.declare_parameter('background_b',0)

         r=self.get_parameter('background_r').get_parameter_value().integer_value
         g=self.get_parameter('background_g').get_parameter_value().integer_value
         b=self.get_parameter('background_b').get_parameter_value().integer_value

         r_new = rclpy.Parameter(
              '/background_r',
              rclpy.Parameter.Type.INTEGER,
              80
         )
         
         g_new = rclpy.Parameter(
              '/background_g',
              rclpy.Parameter.Type.INTEGER,
              0
         )
         b_new = rclpy.Parameter(
              '/background_b',
              rclpy.Parameter.Type.INTEGER,
              0
         )
         all_params = [r_new,g_new,b_new]
         self.set_parameters(all_params)
 
    
         request = Empty.Request()
         future = self.clear_client.call_async(request)
         rclpy.spin_until_future_complete(self, future)
         print('COLOR CHANGED')


    #start_drawing function
    def draw_func(self,turtleid,X1,Y1,X2,Y2,FLAG):

        if len(X1)==len(self.X1[turtleid]):

            self.X1[turtleid]=X1
            self.X2[turtleid]=X2
            self.Y1[turtleid]=Y1
            self.Y2[turtleid]=Y2
            self.FLAG[turtleid]=FLAG
        else:
            print("sizeof task=", len(X2))
            self.X1[turtleid][:X1.shape[0]] = X1
            self.Y1[turtleid][:Y1.shape[0]] = Y1  
            self.X2[turtleid][:X2.shape[0]] = X2  
            self.Y2[turtleid][:Y2.shape[0]] = Y2    
            self.FLAG[turtleid][:FLAG.shape[0]] = FLAG           
        
        if self.i[turtleid]==-1:
        
            x1=self.X1[turtleid][0]
            y1=self.Y1[turtleid][0]
            x1 = ( (x1 - 0.0) / (500.0 - 0.0) ) * (11.0 - 0.0) + 0.0
            y1 = ( (y1 - 0.0) / (500.0 - 0.0) ) * (11.0 - 0.0) + 0.0
            if y1 > 5.54:
                diff=y1 - 5.54
                y1 =5.54 - diff
            else:
                diff = 5.54-y1
                y1 = 5.54 + diff
            angle = np.arctan2((y1-5.54),(x1 - 5.54))

            print("x1,y1, angle= ",x1,",",y1,",",angle," for turtle=", turtleid)
            time.sleep(3)

            self.angular_z[turtleid]=angle

            self.linear_x[turtleid] =0.0


            self.publisher_callback(turtleid)
            time.sleep(2)
            self.goal_x[turtleid]=x1
            self.goal_y[turtleid]=y1

            self.error_curr[turtleid] = np.sqrt((self.goal_x[turtleid] -5.54)**2 + (self.goal_y[turtleid] -5.54)**2)

            self.controller_func(turtleid,5.54,5.54)
            self.publisher_callback(turtleid)


            print("first movement done for turtle=", turtleid)
         

            self.prev_calc_angle[turtleid]=angle

    #draw individual line segments  
    def draw_line_segments(self,turtleid, x1,y1,x2,y2 ):

                self.prev_i[turtleid] = self.i[turtleid]
                               
                x1 = ( (x1 - 0.0) / (500.0 - 0.0) ) * (11.0 - 0.0) + 0.0
                y1 = ( (y1 - 0.0) / (500.0 - 0.0) ) * (11.0 - 0.0) + 0.0
                if y1 > 5.54:
                    diff=y1 - 5.54
                    y1 =5.54 - diff
                else:
                    diff = 5.54-y1
                    y1 = 5.54 + diff


                x2 = ( (x2 - 0.0) / (500.0 - 0.0) ) * (11.0 - 0.0) + 0.0
                y2 = ( (y2 - 0.0) / (500.0 - 0.0) ) * (11.0 - 0.0) + 0.0
                if y2 > 5.54:
                    diff=y2 - 5.54
                    y2 =5.54 - diff
                else:
                    diff = 5.54-y2
                    y2 = 5.54 + diff

                print("data read")

                self.goal_x[turtleid]=x2
                self.goal_y[turtleid]=y2
                self.source_x[turtleid]=x1
                self.source_y[turtleid]= y1

                current_calc_angle = np.arctan2((y2-y1),(x2 -x1)) 
               
                if abs(current_calc_angle - self.prev_calc_angle[turtleid])>=0.35:
                     
                     print("New line segment anglular z calculated")
                     
                     self.angular_z[turtleid] = current_calc_angle -self.prev_calc_angle[turtleid]
                     self.linear_x[turtleid] = 0.0
                     self.publisher_callback(turtleid)
 
                     time.sleep(1)
                     self.error_curr[turtleid] = np.sqrt((self.goal_x[turtleid] -self.source_x[turtleid])**2 + (self.goal_y[turtleid] -self.source_y[turtleid])**2)
#                   
                else:
                     self.angular_z[turtleid] = 0.0  
                print("setting calc angular z to previous") 
                self.prev_calc_angle[turtleid] = current_calc_angle    
                

    #publisher callback at every 0.1second -> timer set at constructor
    def publisher_callback(self,turtleid):                           
        print("Publisher called")

        str_set = 'twist_msg_'+str(turtleid+1)+' =Twist()'
        exec(str_set)
        
        str_set_linearx='twist_msg_'+str(turtleid+1)+'.linear.x = self.linear_x[turtleid]' 
        exec(str_set_linearx)                            
        
        str_set_angularz='twist_msg_'+str(turtleid+1)+'.angular.z = self.angular_z[turtleid]'
        exec(str_set_angularz)
        
        str_pub='self.turtle_velocity_publisher[turtleid].publish(twist_msg_'+str(turtleid+1)+')'
        exec(str_pub)
        self.angular_z[turtleid] = 0.0
        print("publisher exiting")
    
    #subscriber call back, check pose, decide if goal reached. 
    # else if turtle is moving towardsgoal
    #or if all job completed their jobs, then kill and reset and then exit
    def subscriber_callback(self, msg:Pose, turtleid):
        
        time.sleep(0.1)
        print("Subscriber called for turtle=", turtleid)

        
        self.prev_error[turtleid] = self.error_curr[turtleid]
        self.error_curr[turtleid]=np.sqrt((self.goal_x[turtleid] - msg.x)**2 + (self.goal_y[turtleid] - msg.y)**2)
        print("Error from subscribed data=",self.error_curr[turtleid]," prev err=",self.prev_error[turtleid]," for turtle=", turtleid) 
        
        if  (self.error_curr[turtleid] <= self.error_threshold_curr) and (self.prev_i[turtleid]==self.i[turtleid]):
                    #if turtle reached end of line segment
                    print("Goal reached for turtle=", turtleid)
                    self.i[turtleid] =self.i[turtleid] + 1
                    print("self.i incremented to = ", self.i[turtleid]," for turtle=", turtleid)
                    self.error_curr[turtleid] = 500.0
        
                    time.sleep(2)

                    #turtle moving towards goal
                    if self.i[turtleid]>=0 and self.i[turtleid] <len(self.X1[turtleid]):
                        if self.FLAG[turtleid][self.i[turtleid]] ==1:
                            self.set_pen_turtle( 0, 255,255,255,turtleid) #on
                            time.sleep(1.5)
                        else:
                            self.set_pen_turtle( 1, 255,255,255,turtleid) #off
                            time.sleep(1.5)
                        print("before drawing line segment")
                        self.error_curr[turtleid]=500.0
                        self.draw_line_segments(turtleid,self.X1[turtleid][self.i[turtleid]],self.Y1[turtleid][self.i[turtleid]],self.X2[turtleid][self.i[turtleid]], self.Y2[turtleid][self.i[turtleid]])
                    
                    #if all turtles completed their jobs
                    if self.i[turtleid] == len(self.X1[turtleid]):
                        self.i[turtleid] = 1000
                        self.completed_turtle_count +=1
                        self.set_pen_turtle( 1, 255,255,255,turtleid) #off
                        time.sleep(1.5)
                    
                    print("No ofturtles completed=", self.completed_turtle_count)
                    
                    #if all turtles completed, kill and reset
                    if self.completed_turtle_count >= self.num_turtles:
                        print("Killing turtle=", turtleid)
                        

                        self.kill_client = self.create_client(Kill, '/kill')
                        while not self.kill_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('Waiting for /Kill service')

                        request = Kill.Request()
                        request.name=f'turtle{turtleid+1}'
                        future = self.kill_client.call_async(request)

                        time.sleep(2)

                        print("Killed ")

                        print("Resetting turtles")
                        

                        self.reset_client = self.create_client(Empty, '/reset')
                        while not self.reset_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('Waiting for /reset service')

                        request = Empty.Request()
                       
                        future = self.reset_client.call_async(request)


                        time.sleep(4)
                        

                        sys.exit()


                         
        #if turtle moving towards goal, call controller to decide next action
        if self.error_curr[turtleid] <= self.prev_error[turtleid]:   
        
            print("New pose received")
            self.source_x[turtleid] = msg.x
            self.source_y[turtleid] = msg.y
            self.controller_func(turtleid, msg.x, msg.y)
            self.prev_x[turtleid] = msg.x
            self.prev_y[turtleid] = msg.y

    
    #PD controller function
    def controller_func(self, turtleid,x1,y1) :
        K_p=0.13 # 0.30 for 2 turtle
        K_d =0.0025 #0,003 for 2 turtles
        delta_t= 1.0

        self.linear_x[turtleid]=K_p * (self.error_curr[turtleid]) + K_d*((self.error_curr[turtleid] - self.prev_error[turtleid])/delta_t)
        #time.sleep(1)

    #draw using all robots simultaneosly
    def draw_parallel(self,start_it,goal_it,X1,Y1,X2,Y2,FLAG):
        for i in range(len(start_it)):
            X1_inst=X1[start_it[i]:goal_it[i]+1]
            Y1_inst=Y1[start_it[i]:goal_it[i]+1]
            X2_inst=X2[start_it[i]:goal_it[i]+1]
            Y2_inst=Y2[start_it[i]:goal_it[i]+1]
            FLAG_inst=FLAG[start_it[i]:goal_it[i]+1]

            self.draw_func(i, X1_inst,Y1_inst,X2_inst,Y2_inst,FLAG_inst)
            

def main(args=None):
    rclpy.init(args=args)
    num_of_turtles = 10

    #dataset creation
    data2=pd.read_csv('~/ros2_ws/src/project2/project2/A_out.csv')
    data3=pd.read_csv('~/ros2_ws/src/project2/project2/A_in.csv')
    data=pd.read_csv('~/ros2_ws/src/project2/project2/T.csv')
    data1=pd.read_csv('~/ros2_ws/src/project2/project2/M.csv')

    in_contour_flag_data=np.ones(len(data))
    in_contour_flag_data1=np.ones(len(data1))
    in_contour_flag_data2=np.ones(len(data2))
    in_contour_flag_data3=np.ones(len(data3))

    new_df_data=pd.DataFrame({'FLAG':in_contour_flag_data})
    new_df_data1=pd.DataFrame({'FLAG':in_contour_flag_data1})
    new_df_data2=pd.DataFrame({'FLAG':in_contour_flag_data2})
    new_df_data3=pd.DataFrame({'FLAG':in_contour_flag_data3})

    new_df_data = pd.concat([data, new_df_data], axis=1)
    new_df_data1 = pd.concat([data1, new_df_data1], axis=1)
    new_df_data2 = pd.concat([data2, new_df_data2], axis=1)
    new_df_data3 = pd.concat([data3, new_df_data3], axis=1)

    a, b = new_df_data.iloc[-1, 2], new_df_data.iloc[-1, 3]
    c, d = new_df_data1.iloc[0, 0], new_df_data1.iloc[0, 1]   
    R = pd.DataFrame([[a, b, c, d, 0]], columns=['X1', 'Y1', 'X2', 'Y2', 'FLAG'])
    result = pd.concat([new_df_data, R, new_df_data1], ignore_index=True, axis=0)

    a, b = result.iloc[-1, 2], result.iloc[-1, 3]
    c, d = new_df_data2.iloc[0, 0], new_df_data2.iloc[0, 1]   
    R = pd.DataFrame([[a, b, c, d, 0]], columns=['X1', 'Y1', 'X2', 'Y2', 'FLAG'])
    result = pd.concat([result, R, new_df_data2], ignore_index=True, axis=0)

    a, b = result.iloc[-1, 2], result.iloc[-1, 3]
    c, d = new_df_data3.iloc[0, 0], new_df_data3.iloc[0, 1]   
    R = pd.DataFrame([[a, b, c, d, 0]], columns=['X1', 'Y1', 'X2', 'Y2', 'FLAG'])
    result = pd.concat([result, R, new_df_data3], ignore_index=True, axis=0)
 

    X1 = result['X1']
    X2 = result['X2']
    Y1 = result['Y1']
    Y2 = result['Y2']
    FLAG = result['FLAG']

    global num_tasks
    num_tasks = math.ceil(len(X1)/num_of_turtles)

    #creating start point and endpoint list
    #  for each line segment assigned to turtle i
    i=0
    start_it=[]
    goal_it=[]
    while i < len(X1):
         start_it.append(i)
         if i+num_tasks-1<=len(X1)-1:
            goal_it.append(i+num_tasks-1)
         else:
             goal_it.append(len(X1)-1)
         i= i+num_tasks
    print("start list=", start_it)
    print("goal_list=", goal_it)


    #object creation
    my_turtle_node = MyTurtleNode(num_of_turtles)
    
    
    if num_of_turtles>1:
        my_turtle_node.draw_parallel(start_it,goal_it,X1,Y1,X2,Y2,FLAG)
    else:    

        my_turtle_node.draw_func(X1,Y1,X2,Y2,FLAG)

    
    rclpy.spin(my_turtle_node)
    my_turtle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()       