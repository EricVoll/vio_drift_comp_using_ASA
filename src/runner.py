#!/usr/bin/env python

import rospy
import rostopic
import time
import sys
import roslaunch
import rospkg
import yaml
import rosbag
import subprocess
import numpy as np
import os
from analyzer import Analyzer
from datetime import datetime

class Runner:
    def __init__(self):
        print("Starting Runner")

        # import yaml
        with open(r'../cfg/runs.yaml') as file:
            self.config = yaml.load(file, Loader=yaml.FullLoader)

        self.runs = self.importConfig()
    
    # Executes all configured test runs
    def Run(self):
        if self.CheckRosMaster() == False:
            print("Ros master is not running. Please start manually.")
            return

        # Print a report
        self.printRunReport(self.runs)

        time.sleep(1)
        
        for run in self.runs:
            
            if run['execute_test']:
                self.ExecuteRun(run)
            
            if self.config['evaluate_error'] != 0:
                self.StartTrajectoryAnalyzer(run)
    
    # start the specified run with all its option
    def ExecuteRun(self, run):
        cli_arguments = run['commands']
        rosbag_file = run['rosbag_file']
        play_duration = run['duration']
        rosbag_speed = run['rate']
        description = run['description']

        print("Running " + description)
        rospy.set_param("use_sim_time", True)
        
        # Prepare roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = self.GetRoslaunchParentFromArguments(cli_arguments, uuid)
        
        # Start Ros packages
        parent.start()
        # Launch rosbag
        subprocess.Popen(['rosbag', 'play', '--clock','--quiet', '--duration', str(run['duration']/rosbag_speed), '--rate',  str(rosbag_speed), rosbag_file])

        # Wait until rosbag has finished playing
        sleep_time = play_duration/rosbag_speed+1

        step_size = 1
        secs = 0
        while secs <= sleep_time and not rospy.is_shutdown():
            self.progress(1.0 * secs / sleep_time*100, run['run_name'] + "(%03ds/%03ds)" % (secs, sleep_time))
            time.sleep(step_size)
            secs += step_size
        self.endProgress()
        
        parent.shutdown()
        rospy.loginfo("Done with run: " + str(description))

    # Returns a RosLaunchParent object with the specified launch files and arguments
    def GetRoslaunchParentFromArguments(self, args, uuid):
        launch_files = []
        for arg in args:
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(arg)[0]
            roslaunch_args = arg[2:]
            launch_files.append((roslaunch_file, roslaunch_args))
        
        parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        return parent

    # Alligns the trajectories and calculates metrics
    def StartTrajectoryAnalyzer(self, run):
        print("Starting the trajectory analyzer on run: " + run['run_name'])
        
        #place the config file in the results dir
        with open(run['results_directory'] + 'eval_cfg.yaml', "w") as text_file:
            text_file.write("align_type: %s\n" % self.config['trajectory_alignment_method'])
            text_file.write("align_num_frames: %s" % self.config['trajectory_alignment_num'])
        
        # Configure the analyzer to cut away the some seconds, since there are
        # unpredicatable IMU spikes when putting the device on the floor (which was done in all cases)
        with open(run['results_directory'] + 'start_end_time.yaml', "w") as text_file:
            text_file.write("start_time_sec: 0\n")
            text_file.write("end_time_sec: 190") # %s" % (run['duration'] - self.config['shorten_trajectory_by_seconds']))

        # start the rpg trajectory aligner and analyzer from 
        # https://github.com/uzh-rpg/rpg_trajectory_evaluation
        subprocess.call(['rosrun rpg_trajectory_evaluation analyze_trajectory_single.py --recalculate_errors ' + run['results_directory']], shell=True)

    def AnalyzeResults(self):
        if(self.config['analyze_results'] != 1):
            rospy.loginfo("Skipping analysis")
            return
        
        result_folder = self.config['results_base_folder'] + datetime.now().strftime("results_%d_%m_%Y_%H_%M_%S")
        analyzer = Analyzer(self.runs, result_folder)
        analyzer.analyze_by_rosbag_file()
        # analyzer.overall_analysis()
        analyzer.analyze_by_method()

    # Checks if the ros master is running
    def CheckRosMaster(self):
        # thanks to https://github.com/ros-visualization/rqt_robot_plugins/blob/eb5a4f702b5b5c92b85aaf9055bf6319f42f4249/rqt_moveit/src/rqt_moveit/moveit_widget.py#L251
        try:
            # Checkif rosmaster is running or not.
            rostopic.get_topic_class('/rosout')
            return True
        except rostopic.ROSTopicIOException as e:
            return False

    # Imports the yaml file run configuration
    def importConfig(self):
        runs = []
        no_runs = [] #runs not executed
        total_duration = 0
        rosbag_durations = {}

        respect_skip_count = True if 'indices' in self.config and len(self.config['indices']) > 0  else False 

        # Count runs
        run_count = 0
        for index, config in enumerate(self.config['configs']):
            if respect_skip_count:
                if index not in self.config['indices']:
                    continue
            else:
                if index < self.config['skip']:
                    continue
                if index >= self.config['skip'] + self.config['count'] and self.config['count'] > 0:
                    break

            if len(self.config['rosbags']) > 0:
                run_count += len(self.config['rosbags'])
            else:
                run_count = run_count + len(config['rosbags'])
        
        # Init progress bar
        test_index = 0.0

        

        for index, config in enumerate(self.config['configs']):
            if respect_skip_count:
                if index not in self.config['indices']:
                    continue
            else:
                if index < self.config['skip']:
                    continue
                if index >= self.config['skip'] + self.config['count'] and self.config['count'] > 0:
                    break

            index_array_index= self.config['indices'].index(index)
                            
            for rosbag_index, rosbag_name in (enumerate(config['rosbags']) if len(self.config['rosbags']) == 0 else enumerate(self.config['rosbags'])):
                # Update progress bar
                test_index = test_index + 1
                self.progress(test_index/run_count*100, "Reading config files")
                
                cli_args = []
                rosbag_file = self.config['rosbag_folder'] + rosbag_name

                if os.path.isfile(rosbag_file) == False:
                    no_runs.append("Could not find file " + rosbag_file + ". Skipping this rosbag.")
                    continue

                rosbag_speed = 1 if 'rosbag_speed' not in config else config['rosbag_speed'] 
                
                # Overwrite rosbag duration for debugging?
                bag_duration = 0
                if self.config['run_tests'] == 0:
                    bag_duration = 0.1

                if bag_duration == 0: # if not overwritten, get it from the rosbag
                    if rosbag_file not in rosbag_durations:
                        info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbag_file], stdout=subprocess.PIPE).communicate()[0], Loader=yaml.FullLoader)
                        bag_duration = info_dict['duration']
                        rosbag_durations[rosbag_file] = bag_duration
                    else:
                        bag_duration = rosbag_durations[rosbag_file]
                
                if self.config['max_run_dur'] > 0 and  bag_duration > self.config['max_run_dur']:
                    bag_duration = self.config['max_run_dur']
                
                total_duration = total_duration + bag_duration
                rosbag_file_name = rosbag_name.split(".")[0]

                # parse launch files
                run_name = str(config['description']) + "_" + rosbag_file_name
                for line in config['launchfiles']:
                    args = line.split()

                    if any(item.startswith('run_name') for item in args) == False:
                        args.append('run_duration:='+str(bag_duration))
                        args.append('run_name:=' + run_name)
                        args.append('output:=' + self.config['output'])
                    cli_args.append(args)

                
                runs.append(
                    {
                        'index': index_array_index,
                        'description': config['description'],
                        'run_name': run_name,
                        'rosbag_file': rosbag_file,
                        'rosbag_file_name': rosbag_file_name,
                        'duration': bag_duration,
                        'rate': rosbag_speed,
                        'commands': cli_args,
                        'results_directory': self.config['results_base_folder']+ run_name +'/',
                        'execute_test': self.config['run_tests'] != 0 and (('only_analysis' in config and config['only_analysis'] == 0) or ('only_analysis' not in config))
                    }
                )

        self.endProgress()

        runs.sort(key=lambda x:x['index'])

        if len(no_runs) > 0:
            print("The following bag files could not be found: ")
            for norun in no_runs:
                print(norun)
        return runs

    # Prints an overview about the test runs which are about to be executed
    def printRunReport(self, runs):
        total_duration = 0
        for index, run in enumerate(runs):
            print("Run " + str(index) + ": " + run['run_name'] + " for " + str(run['duration']) + " secs")
            total_duration += run['duration']
        
        print("Tests running for " + str(total_duration) + " seconds")

    def progress(self, x, title = ""):
        global progress_x
        x = int(x * 40 // 100)
        text = title + ": [" + "#" * x + "-" * (40-x) + ']'
        sys.stdout.write(text + len(text) * chr(8))
        sys.stdout.flush()
        progress_x = x
    
    def endProgress(self):
        sys.stdout.write("#" * (40 - progress_x) + "]\n")
        sys.stdout.flush()

if __name__ == '__main__':


   # data = [('a9f24545-2c9e-4279-b905-89390b9938cb_static', 0.1, 10.0, 2.9372718334198), ('caf44bd7-68e2-45fb-8491-1ea6e0ac6d8b_static', 0.1, 10.0, 2.538942575454712), ('a4eb5097-7e4d-4821-a43f-a7ec54102bea_static', 0.1, 10.0, 2.538942575454712)]
   # dataa = [data[0]]
   # print(dataa)
   # for tup in dataa:
   #     anchor_id = tup[0]
   #     print(anchor_id)




    commander = Runner()
    commander.Run()
    commander.AnalyzeResults()