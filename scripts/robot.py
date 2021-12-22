#! /usr/bin/env python

import rospy
import sys
import copy
import cv2
import pyttsx3
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import speech_recognition as sr
import numpy as np

from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from cv_bridge import CvBridge,CvBridgeError


class ModelSpawner(object):
    def __init__(self):
        self.RED_BOX_POSE = [0.0,0.05,0.35]
        self.BLUE_BOX_POSE = [-0.15,0.05,0.35]
        self.GREEN_BOX_POSE = [0.15,0.05,0.35]
        self.SPAWN_POSE = Pose()
        self.SPAWN_MODEL_NAME = ""
        self._f = ""
        self.service_topic_1 = "/gazebo/spawn_sdf_model"
        self.service_topic_2 = "/gazebo/delete_model"
        self.spawner = rospy.ServiceProxy(self.service_topic_1, SpawnModel)
        self.destroyer = rospy.ServiceProxy(self.service_topic_2,DeleteModel)
        rospy.wait_for_service(self.service_topic_1)
        rospy.wait_for_service(self.service_topic_2)

    def set_pose(self,model):
        if model == 'red_box':
            self.SPAWN_POSE.position.x = self.RED_BOX_POSE[0]
            self.SPAWN_POSE.position.y = self.RED_BOX_POSE[1]
            self.SPAWN_POSE.position.z = self.RED_BOX_POSE[2]
            self.SPAWN_MODEL_NAME = 'red_package'
            self._f = 'package_red'

        elif model == 'green_box':
            self.SPAWN_POSE.position.x = self.GREEN_BOX_POSE[0]
            self.SPAWN_POSE.position.y = self.GREEN_BOX_POSE[1]
            self.SPAWN_POSE.position.z = self.GREEN_BOX_POSE[2]
            self.SPAWN_MODEL_NAME = 'green_package'
            self._f = 'package_green'

        elif model == 'blue_box':
            self.SPAWN_POSE.position.x = self.BLUE_BOX_POSE[0]
            self.SPAWN_POSE.position.y = self.BLUE_BOX_POSE[1]
            self.SPAWN_POSE.position.z = self.BLUE_BOX_POSE[2]
            self.SPAWN_MODEL_NAME = 'blue_package'
            self._f = 'package_blue'

        else:
            print("Invalid Model")

    def spawn_model(self,color):
        self.set_pose(color)
        self.spawner(model_name = self.SPAWN_MODEL_NAME,
                     model_xml = open('/home/varun/catkin_ws/src/sociopulator/models/'+self._f +'/model.sdf','r').read(),
                     robot_namespace = '/',
                     initial_pose = self.SPAWN_POSE,
                     reference_frame = 'world')

    def delete_model(self,model_name):
        if model_name == 'red_box':
            model_name = 'red_package'

        elif model_name == 'green_box':
            model_name = 'green_package'

        elif model_name == 'blue_box':
            model_name = 'blue_package'

        self.destroyer(model_name)


class Scarapulator(ModelSpawner):

    def __init__(self):

        '''
         ______________________________________________________________
        |
        |Description
        |------------
        | - Class Constructor method.
        |
        |..............................................................
        |
        | Parameters
        | ----------
        |   None
        |
        |..............................................................
        |
        | Constants/variables
        | -------------------
        | * self._planning_group - Name of planning group
        | * self._robot_ns - Name of robot
        | * self._commander - moveit commander
        | * self._scene - name of planning scene.
        | * self._display_trajectory_publisher - publishes data about
        |                                        the robot's trajectory
        | * self._exectute_trajectory_client - action client to execute
        |                                      trajectory.
        |
        | Objects
        | -------
        | self._robot
        |          - robot in planning scene.
        |
        | self._group
        |          - move_group commander to handle trajectories.
        |______________________________________________________________
        '''

        super(Scarapulator,self).__init__()

        #Constants
        self.planning_group_1 = "scara_planner_1"
        self.planning_group_2 = "scara_planner_2"

        #Initialize Node and Moveit Commander.
        moveit_commander.roscpp_initialize(sys.argv)

        #Instantiates
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group_1 = moveit_commander.MoveGroupCommander(self.planning_group_1)
        self._group_2 = moveit_commander.MoveGroupCommander(self.planning_group_2)
        self.trajectory_client = actionlib.SimpleActionClient('execute_trajectory',moveit_msgs.msg.ExecuteTrajectoryAction)
        self.trajectory_client.wait_for_server()

        #Basic Info
        self.planning_frame = self._group_1.get_planning_frame()
        self.eef_link = self._group_1.get_end_effector_link()
        self.group_names = self._robot.get_group_names()

        #Initial Setup
        self._group_1.set_goal_tolerance(0.01)
        self._group_1.set_num_planning_attempts(3)
        self._group_2.set_goal_tolerance(0.001)
        self._group_2.set_num_planning_attempts(3)

        #Joint Angles for boxes
        self.BLUE_BOX = [2.4,-1.2]
        self.RED_BOX = [2.2,-1.23]
        self.GREEN_BOX = [1.95,-1.2]

        #Box Positions
        self.GREEN_BOX_GAZEBO_POSE = [0,0.05,0.35,0.0,0.0,0.0]
        self.BLUE_BOX_GAZEBO_POSE = [-0.15,0.05,0.35,0.0,0.0,0.0]
        self.RED_BOX_GAZEBO_POSE = [0.0,0.05,0.35,0.0,0.0,0.0]

        #Bin Positions in Gazebo
        self.GREEN_BIN_GAZEBO_AREA = [-0.75,-0.55]
        self.RED_BIN_GAZEBO_AREA = [0.75,-0.55]
        self.BLUE_BIN_GAZEBO_AREA = [0.0,-1.45]

        #Problems Faced
        self.SET_POS_FAILED = []
        self.SET_JOINT_FAILED = ['did not go to the position','did not pick up the box','did not pick up the package',
                                 'you didnt go the required position','you did not pick up the package','not pick up',
                                 'you didnt pick up the box','you failed in going to the position','not pick up',
                                 'did not go','set joint angles failed','joint angles failed','you did not go to pick the box',
                                 'you did not go to pick up the box','picking up failed','failed to pick up', 
                                 'you could not got to pick up the package']

        self.PACKAGE_FAILURE = ['Package Crashed','box crashed','box fell down','box fell','package fell','package fell down',
                                'you dropped the package','while moving you dropped the package','package dropped','dropped the box',
                                'dropped the package','dropped','fell','fell down','Dropped']

        self.MISBEHAVIOUR = ['weird thing happened','something happened','skipped the task','skipped','skip','went mad',
                             'mad','misbehaved','reacted weirdly','weird','some weird thing happened']
        self.OTHER = []

        self.FAILURE_TYPES = ['set pose failed',
                              'set joints failed',
                              'crash',
                              'misbehaviour',
                              'other']

        #Solutions
        self.SOLUTION_SET_1 = ['solve the joint failure','Joint Failure','solve the inverse kinematics failure','joint fail',
                               'joint failure','set the joints correctly','reset joints','reset your joints','Reset Joints',
                               'Reset Joint','inverse kinematics','Inverse Kinematics','Inverse','Kinematics','inverse',
                               'kinematics','go to the position again','try going to the same position','reset the joint angles',
                               'reset your joint angles','retry the same plan again']

        self.SOLUTION_SET_2 = ['velocity','retry with less velocity','go slow next time','go slow','slow','slower',
                               'try again with less velocity','please reduce your velocity next time','Slow','Velocity',
                               'Slower','Go Slow','Less Velocity','less velocity','reduce velocity','reduce your velocity',
                               'Reduce Your Velocity']

        self.SOLUTION_SET_3 = ['misbehave']
        self.SOLUTIONS = ['Solve IK Failure',
                          'Retry solution']

        #Variables
        self.pose = ""
        self.position_tried = ""
        self.arm_velocity = 2.0

        #Files
        self.JOINT_LIMITS_1 = '/robot_description_planning/joint_limits/'
        self.JOINT_LIMITS_2 = '/true_planning/joint_limits/'

    def go_to_defined_pose(self,pose_name,group_name):

        '''
         ______________________________________________________________
        |
        |Description
        |------------
        | - Makes the arm go the predefined positions according to the
        |   Robot's SRDF.For this robot theese positions are home,
        |   red_bin_pose,blue_bin_pose, green_bin_pose and pick.
        |
        |..............................................................
        | Parameters
        | ----------
        |   pose_name:str
        |        - The position name like (home,pick...) are given as an
        |          argument.
        |
        |..............................................................
        | Returns
        | -------
        | None
        |______________________________________________________________
        '''

        if group_name == self.planning_group_1:
            rospy.loginfo('\033[94m' + "Going to Pose: {}".format(pose_name) + '\033[0m')
            self._group_1.set_named_target(pose_name) 
            plan = self._group_1.plan()
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            self.trajectory_client.send_goal(goal)
            self.trajectory_client.wait_for_result()
            rospy.loginfo('\033[94m' + "Now at Pose: {}".format(pose_name) + '\033[0m')

        elif group_name == self.planning_group_2:
            rospy.loginfo('\033[94m' + "Going to Pose: {}".format(pose_name) + '\033[0m')
            self._group_2.set_named_target(pose_name) 
            plan = self._group_2.plan()
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            self.trajectory_client.send_goal(goal)
            self.trajectory_client.wait_for_result()
            rospy.loginfo('\033[94m' + "Now at Pose: {}".format(pose_name) + '\033[0m')

    def go_to_pose(self,pose):
        '''
         ______________________________________________________________
        |
        |Description
        |------------
        | - Makes the arm go the custom positions in the simulation
        |   world. These position can be given through geometry_msgs.msg
        |   message format.
        |
        |..............................................................
        | Parameters
        | ----------
        |   cordinates:list >> [x,y,z]
        |        - A list of cartesian cordinates is given as an argument.
        | 
        |
        |..............................................................
        | Returns
        | -------
        | None
        |______________________________________________________________
        '''

        pose_values = self._group_1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        
        self._group_1.set_pose_target(pose) 
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group_1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group_1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def pick_and_place_box(self,box_name,gripper,voice):

        if box_name == "green_box":
            self.position_tried = "green_box"
            self.pose = "green_bin_pose"
            j1 = self.GREEN_BOX[0]
            j2 = self.GREEN_BOX[1]
            self.set_joint_angles([j1,j2])


        elif box_name == "red_box":
            self.position_tried = "red_box"
            self.pose = "red_bin_pose"
            j1 = self.RED_BOX[0]
            j2 = self.RED_BOX[1]

            self.set_joint_angles([j1,j2])
            rospy.sleep(1)

        elif box_name == "blue_box":
            self.position_tried = "blue_box"
            self.pose = "blue_bin_pose"
            j1 = self.BLUE_BOX[0]
            j2 = self.BLUE_BOX[1]

            self.set_joint_angles([j1,j2])
            rospy.sleep(1)

        rospy.sleep(1)

        self.go_to_defined_pose('hold','scara_planner_2')
        rospy.sleep(1)

        gripper.activate()
        rospy.sleep(2)

        self.go_to_defined_pose('default','scara_planner_2')
        rospy.sleep(1)

        self.go_to_defined_pose(self.pose,"scara_planner_1")
        rospy.sleep(1)

        gripper.deactivate()
        rospy.sleep(1)

        self.go_to_defined_pose('pick',"scara_planner_1")
        

    def set_joint_angles(self, arg_list_joint_angles):
        '''This function will be used to set joint angles of UR5 Arm'''

        list_joint_values = self._group_1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group_1.set_joint_value_target(arg_list_joint_angles)
        self._group_1.plan()
        flag_plan = self._group_1.go(wait=True)

        list_joint_values = self._group_1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group_1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def operate(self,gripper,voice):
        pass

    def ask_after_task(self,gripper,voice):
        voice.ask("was the task done correctly?")
        print("Answer the Robot")
        answer = voice.convert_voice_to_text()

        if answer == "yes" or answer == "yeah" or answer == "yup" or answer == "yes it is" or "yes" in answer or "yeah it was" in answer:
            voice.engine.say("Ok. Any other task you want me to perform?")
            voice.engine.runAndWait()
            answer = voice.convert_voice_to_text()
            
            if answer == "yes" or answer == "yeah" or answer == "yup" or answer == "yes it is" or 'yes' in answer:
                voice.engine.say("Ok. Tell me")
                voice.engine.runAndWait()
                answer = voice.convert_voice_to_text()

                return answer

            elif answer == "no" or answer == "nah" or answer == "please no" or answer == "dont" or 'no' in answer:
                voice.engine.say("Ok. let me know if you want me to perform other tasks. I am always at your service")
                voice.engine.runAndWait()
                return answer
                #self.turn_off()

            else:
                return None


        elif answer == "no" or answer == "nah" or answer == "no it was no" or 'no' in answer:
            voice.engine.say("Ok. what is the issue?")
            voice.engine.runAndWait()
            problem = voice.convert_voice_to_text()
            voice.engine.say("ok let me take a look at the issue")
            voice.engine.runAndWait()
            issue = self.look_into_issue(problem)
            self.address_failure(issue,gripper,voice)

            voice.engine.say("Ok. Any other task you want me to perform?")
            voice.engine.runAndWait()
            answer = voice.convert_voice_to_text()
            
            if answer == "yes" or answer == "yeah" or answer == "yup" or answer == "yes it is" or 'yes' in answer:
                voice.engine.say("Ok. Tell me")
                voice.engine.runAndWait()
                answer = voice.convert_voice_to_text()

                return answer

            elif answer == "no" or answer == "nah" or answer == "please no" or answer == "dont" or 'no' in answer:
                voice.engine.say("Ok. let me know if you want me to perform other tasks. I am always at your service")
                voice.engine.runAndWait()
                return answer

        else:
            return None

    def address_failure(self,failure_type,gripper,voice):
        if failure_type == self.FAILURE_TYPES[1]:
            voice.engine.say("I have found the problem. I am not able to set my joint angles. can you help me?")
            voice.engine.runAndWait()
            rospy.sleep(2)

        elif failure_type == self.FAILURE_TYPES[2]:
            voice.engine.say("The package slipped as the force was too much. Can you help me in solving this?")
            voice.engine.runAndWait()
            print(voice.box_name)
            self.delete_model(voice.box_name)
            rospy.sleep(1)
            self.spawn_model(voice.box_name)
        
        answer = voice.convert_voice_to_text()
        voice.engine.say("Ok please tell me how to solve this problem")
        voice.engine.runAndWait()
        solution = voice.convert_voice_to_text()
        self.human_robot_interaction(solution,failure_type,gripper,voice)


    def human_robot_interaction(self,solution,failure_type,gripper,voice):
        voice.engine.say("Okay")
        voice.engine.runAndWait()
        solution_type = self.get_solution(solution,voice)

        if solution_type == self.SOLUTIONS[0]:
            while True:
                self.pick_and_place_box(self.position_tried,gripper,voice)
                voice.ask("Did I Solve the issue correctly?")
                answer = voice.convert_voice_to_text()
                if answer == "yes" or answer =="yeah" or answer == "yup" or "yes" in answer:
                    break

                else:
                    voice.engine.say("Okay. Will Try Again")
                    voice.engine.runAndWait()
                    continue 

            return True          

            
        elif solution_type == self.SOLUTIONS[1]:
            self.arm_velocity -= 0.01
            rospy.set_param(self.JOINT_LIMITS_1 + '/revolute_joint_1/max_velocity',self.arm_velocity)
            rospy.set_param(self.JOINT_LIMITS_1 + '/revolute_joint_2/max_velocity',self.arm_velocity)
            rospy.set_param(self.JOINT_LIMITS_2 + '/revolute_joint_1/max_velocity',self.arm_velocity)
            rospy.set_param(self.JOINT_LIMITS_2 + '/revolute_joint_2/max_velocity',self.arm_velocity)
            while True:
                self.pick_and_place_box(self.position_tried,gripper,voice)
                voice.ask("Did I Solve the issue correctly?")
                answer = voice.convert_voice_to_text()
                if answer == "yes" or answer =="yeah" or answer == "yup" or "yes" in answer:
                    break

                else:
                    voice.engine.say("Okay. Will Try Again")
                    voice.engine.runAndWait()
                    self.arm_velocity = self.arm_velocity - 0.03
                    rospy.set_param(self.JOINT_LIMITS_1 + '/revolute_joint_1/max_velocity',self.arm_velocity)
                    rospy.set_param(self.JOINT_LIMITS_1 + '/revolute_joint_2/max_velocity',self.arm_velocity)
                    rospy.set_param(self.JOINT_LIMITS_2 + '/revolute_joint_1/max_velocity',self.arm_velocity)
                    rospy.set_param(self.JOINT_LIMITS_2 + '/revolute_joint_2/max_velocity',self.arm_velocity)
                    continue 

            return True          

        else:
            voice.engine.say("I don't Understand your solution.Please explain again")
            voice.engine.runAndWait()
            print("Respond to robot by saying yes or ok")
            

    def get_solution(self,solution,voice):
        s1 = 0
        s2 = 0

        for i in range(len(self.SOLUTION_SET_1)):
            solved = self.SOLUTION_SET_1[i]
            if solution == solved or 'forgot' in solution or 'could not pick up' in solution:
                s1 += 1
          
            
        for j in range(len(self.SOLUTION_SET_2)):
            solved = self.SOLUTION_SET_2[j]
            if solution == solved or 'go slow' in solution or 'reduce' in solution or 'slow' in solution or 'velocity' in solution:
                s2 += 1

        max_val = np.argmax([s1,s2])

        if max_val == 0 and s1 != 0:
            solution = self.SOLUTIONS[0]
        
        elif max_val == 1 and s2 != 0:
            solution = self.SOLUTIONS[1]

        else:
            solution = "Not able to understand your solution"
            voice.respond(solution)

        return solution
    

    def look_into_issue(self,problem):
        p1=0
        p2=0
        p3=0
        p4=0

        for i in range(len(self.SET_JOINT_FAILED)):
            issue = self.SET_JOINT_FAILED[i]
            if problem == issue or 'not pick' in problem or 'could not pick up' in problem:
                p1 += 1
          
            
        for j in range(len(self.PACKAGE_FAILURE)):
            issue = self.PACKAGE_FAILURE[j]
            if problem == issue or 'fell' in problem or 'crashed' in problem or 'drop' in problem or 'dropped' in problem:
                p2 += 1

        max_val = np.argmax([p1,p2])

        if max_val == 0 and p1 != 0:
            problem = self.FAILURE_TYPES[1]
        
        elif max_val == 1 and p2 != 0:
            problem = self.FAILURE_TYPES[2]

        else:
            problem = "Not Understood"
            #voice.respond("Not Understood")

        return problem

        

    def __del__(self):
        rospy.loginfo("Scarapulator died")



class ScaraGripper(object):

    def __init__(self):
        self.service_topic_on = "/vacuum_gripper/on"
        self.service_topic_off = "/vacuum_gripper/off"

        self._gripper_on = rospy.ServiceProxy(self.service_topic_on,Empty)
        self._gripper_off = rospy.ServiceProxy(self.service_topic_off,Empty)
        rospy.wait_for_service(self.service_topic_on)
        rospy.wait_for_service(self.service_topic_off)

    def activate(self):
        self._gripper_on()
        print("Gripper Service Activated")

    def deactivate(self):
        self._gripper_off()
        print("Gripper Service Deactivated")

    def __del__(self):
        print("ScaraGripper Deleted")


class VerbalCommunicator(object):
    def __init__(self):

        #object definition
        self._r = sr.Recognizer()
        self.engine = pyttsx3.init('espeak')
        
        #Get Robot Voice Params
        self.voices = self.engine.getProperty('voices')

        #Set Robot's Voice Params
        self.engine.setProperty('rate',125)
        self.engine.setProperty('volume',1.0)

        #Variables
        self.text = None
        self.command_type = None
        self.understood = None
        self.box_name = None

        #Commands list
        self.COMMANDS_1 = ['go to green box', 'pick up green box','green box','can you go to green box',
                           'pick and place green box','green','place the green box in the bin','place the green box',
                           'green package','place the green package','place green package in the bin',
                           'place green package in bin','place the green package in the bin','pick up greenbox',
                           'place green package in the green bin','please the green package','pick up Greenbox','Green Box']
        
        self.COMMANDS_2 = ['go to red box', 'pick up red box','red box','can you go to red box',
                           'pick and place red box','red','place the red box in the bin','place the red box',
                           'red package','place the red package','place red package in the bin',
                           'place red package in bin','place the red package in the bin','Red Box',
                           'place red package in the red bin','please the red package','pick up Redbox',
                           'please the RedBox','please the Redbox','Buddha Redbox','Redbox','pick up Redbox']
        
        self.COMMANDS_3 = ['go to blue box', 'pick up blue box','blue box','can you go to blue box',
                           'pick and place blue box','blue','place the blue box in the bin','place the blue box',
                           'blue package','place the blue package','place blue package in the bin',
                           'place blue package in bin','place the blue package in the bin','Blue Box'
                           'place blue package in the blue bin','please the blue package']

        self.INTERACTIVE_COMMANDS = ['no','stop','wait','you can','yes','yes you can','no you cannot','yeah','nah',
                                     'can you replan','can you retry','try the path again','please try again',
                                     'can you try to pick up the box again','retry','replan','check the joint states',
                                     'check the joint values','check the position', 'go little to your left','go left',
                                     'go little to your right','go right','go little to your front','go front','go back',
                                     'go little to your back','left','right','front','back','try again','find the failure',
                                     'try to pick up the box again','try to pick up the package again','pick it up again']

    
    def recognize_voice(self,command):

        self.command_type = self.check_command(command)
        if self.command_type == "Green Package Task":
            self.command = "place green package in the bin"

        elif self.command_type == "Red Package Task":
            self.command = "place red package in the bin"

        elif self.command_type == "Blue Package Task":
            self.command = "place blue package in the bin"

        elif self.command_type == "Interactive Task":
            self.command = "interaction mode"
            #self.set_interaction_mode(True)

        else:
            self.command  = 'None'

        return self.command


    def check_command(self,command):

        c1=0
        c2=0
        c3=0
        c4=0

        for i in range(len(self.COMMANDS_1)):
            green_box_task = self.COMMANDS_1[i]
            if command == green_box_task or 'green box' in command or 'green package' in command:
                c1 += 1
          
            
        for j in range(len(self.COMMANDS_2)):
            red_box_task = self.COMMANDS_2[j]
            if command == red_box_task or 'red box' in command or 'red package' in command or 'redbox' in command:
                c2 += 1

        for k in range(len(self.COMMANDS_3)):
            blue_box_task = self.COMMANDS_3[k]
            if command == blue_box_task or 'blue box' in command or 'blue package' in command or 'bluebox' in command:
                c3 += 1

        for l in range(len(self.INTERACTIVE_COMMANDS)):
            interactive_task = self.INTERACTIVE_COMMANDS[l]
            if command == interactive_task or 'retry' in command or 'wait' in command:
                c4 += 1

        max_val = np.argmax([c1,c2,c3,c4])
        print(max_val)
        print(c1,c2,c3,c4)

        if max_val == 0 and c1 != 0:
            command = "Green Package Task"
            self.understood = True
        
        elif max_val == 1 and c2 != 0:
            command = "Red Package Task"
            self.understood = True

        elif max_val == 2 and c3 != 0:
            command = "Blue Package Task"
            self.understood = True

        elif max_val == 3 and c4 != 0:
            command = "Interactive Task"
            self.understood = True

        else:
            self.understood = False
            command = "Not Understood"
            self.respond("Not Understood")

        return command


    def report_error(self,error):
        self.engine.say(error)
        self.engine.runAndWait()

    def respond(self,voice_to_text):

        if voice_to_text == "place green package in the bin":
            self.engine.say("okay. Picking up green_box")
            self.engine.runAndWait()
            self.box_name = "green_box"

        elif voice_to_text == "place red package in the bin":
            self.engine.say("okay. Picking up red_box")
            self.engine.runAndWait()
            self.box_name = "red_box" 

        elif voice_to_text == "place blue package in the bin":
            self.engine.say("okay. Picking up blue_box")
            self.engine.runAndWait()
            self.box_name = "blue_box"

        elif voice_to_text == "place all the packages in the bin":
            pass

        else:
            self.engine.say("Sorry I couldn't understand what you said.\
                             But I am constantly improving")
            self.engine.runAndWait()
            self.box_name = None
        
        return self.box_name

    def ask(self,question):
        self.engine.say(question)
        self.engine.runAndWait()
        

    def convert_voice_to_text(self):
        with sr.Microphone() as voice:
            print("Give a Voice Command in 5 secs")
            audio_data = self._r.record(voice,duration = 5)
            #audio_data = self._r.listen(voice)
            print("audio_data",audio_data)
            command = self._r.recognize_google(audio_data)
            print("Task Given:",command)
            return command

    def get_text(self):
        self.command_type = "text"
        command = raw_input("Give a task to the robot: ")
        print("Task Given:",command)
        return command
       


class Camera(object):

    def __init__(self):

        self.image = None
        self.px = None
        self.py = None

        self.x = 100
        self.y = 390
        self.w = 50
        self.h = 600

        #Camera Caliberation Params
        self.ROBOT_X = 0.0
        self.ROBOT_Y = -0.6
        self.ROBOT_Z = 0.0

        self.TABLE_X = 0.0
        self.TABLE_Y = 0.0
        self.TABLE_Z = 0.35

        self.ROBOT_TABLE_X = abs(self.TABLE_X - self.ROBOT_X)
        self.ROBOT_TABLE_Y = abs(self.TABLE_Y - self.ROBOT_Y)
        self.ROBOT_TABLE_Z = abs(self.TABLE_Z - self.ROBOT_Z)

        self.R = [[1,0,0],
                  [0,1,0],
                  [0,0,1]]
        self.D = [[self.ROBOT_TABLE_X,self.ROBOT_TABLE_Y,]]

        self.CM_TO_PIXEL = 11.3/640.0

        self.bridge = CvBridge()

        
    def get_image(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
        return self.image


    def show_image(self,image):
        cv2.imshow('Image',image)

    def get_pixel_cordinates(self,thresh_image,h,w):

        row_sum = np.matrix(np.sum(thresh_image,0))
        row_num = np.matrix(np.arange(w))
        row_mul = np.multiply(row_sum,row_num)
        row_t = np.sum(row_mul)
        row_t_f = np.sum(np.sum(thresh_image))
        self.px = row_t/row_t_f

        column_sum = np.matrix(np.sum(thresh_image,0))
        column_num = np.matrix(np.arange(w))
        column_mul = np.multiply(column_sum,column_num)
        column_t = np.sum(column_mul)
        column_t_f = np.sum(np.sum(thresh_image))
        self.py = column_t/column_t_f

        return self.px, self.py 


    def detect_red(self,img):
        pass
        
    def crop_image(self,image):
        image = image[self.x:self.y, self.w:self.h]
        return image

    def convert_camera_to_robot_frame(self,px,py,R):
        pass




def callback(msg):
    image = cam.get_image(msg)

    crop_image = cam.crop_image(image)
    gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray_image,50,255,cv2.THRESH_BINARY)

    #cv2.imshow('image',image)
    cv2.imshow('crop_image',crop_image)
    cv2.imshow("Threshold",thresh)

    cv2.waitKey(3)



def main():
    '''
    #Initialize the Node
    rospy.init_node('scara_robot',anonymous=True)
    
    #Declaration of Class Objects
    global robot,cam
    robot = Scarapulator()
    cam = Camera()

    #Create A subscriber
    sub = rospy.Subscriber('/camera/color/image_raw',Image,callback)

    rospy.spin()
    '''
    
    #Create the Robot Object
    rospy.init_node('sociopulator',anonymous=True)

    robot = Scarapulator()
    gripper = ScaraGripper()
    voice = VerbalCommunicator()

    
    ip = raw_input("Type Start to start the interaction with the robot: ")
    command = voice.convert_voice_to_text()

    while True:
        command = voice.recognize_voice(command)
        box_name = voice.respond(command)
        robot.pick_and_place_box(box_name,gripper,voice)
        command = robot.ask_after_task(gripper,voice)

        if command == None or command == "no":
            break

        else:
            print("Say you next task to the robot")
            continue
    


        
    
if __name__ == '__main__':
    main()
