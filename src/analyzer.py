#!/usr/bin/env python

import rospy
import time
import sys
import roslaunch
import rospkg
import yaml
import rosbag
import subprocess
import numpy as np
import os
from matplotlib import pyplot as plt
import glob
from scipy.signal import savgol_filter
from itertools import groupby
from mpl_toolkits.mplot3d import axes3d
from matplotlib import style
from colour import Color
from pprint import pprint
from pylab import plot, show, savefig, xlim, figure, \
                hold, ylim, legend, boxplot, setp, axes

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.size": 20,
    "font.sans-serif": ["Helvetica"]})

class Analyzer:

    def __init__(self, runs, result_folder):
        self.runs = runs
        self.result_folder = result_folder

        # create the results folder where all the images will go in
        # os.makedirs(result_folder)

        if runs != None:
            for run in self.runs:
                run['stats'] = self.get_run_stats(run)

        self.colors = ['r', 'b', 'y', 'c', 'm', 'k']

    def analyze_by_rosbag_file(self):
        # group runs by rosbag files
        r = sorted(self.runs, key=lambda x:x['rosbag_file'])
        # grouped = [[y['run_name'] for y in self.runs if y['rosbag_file']==x['rosbag_file']] for x in self.runs]
        grouped = groupby(r,key=lambda x:x['rosbag_file'])
        # iterate through the rosbags and create meaningfull analytics
        
        for bagfile, rosbag_group in grouped:
            
            rosbag_group = list(rosbag_group)
            
            fig = plt.figure(figsize=(6, 5.5))
            ax = fig.add_subplot(111,xlabel='$x_{trajectory} [m]$', ylabel='$error~magnitude [m]$')
        
            # find best in group
            bestTrajectory = min(rosbag_group, key=lambda x:x['stats']['absolute_error']['trans']['mean'] + (10 if x['description'] == 'vicon_{fuse}' else 0))

            for index, run in enumerate(rosbag_group):
                X = run['stats']['accumulated_distances_array']
                Y = run['stats']['absolute_error_magn_array']
                yhat = Y #savgol_filter(Y, 51, 3) # window size 51, polynomial order 3
                if run['run_name'] == bestTrajectory['run_name']:
                    ax.fill_between(X, yhat, facecolor='green', alpha=0.1)
                    ax.plot(X, yhat, 'g-', label=run['description'])
                else:
                    ax.plot(X, yhat, self.colors[index%len(self.colors)]+"-", label=run['description'])

                # draw anchor timestamps
                # anchors = run['stats']['anchor_timestamps']
                # if len(anchors) > 0:
                #     ax.scatter(X[anchors], Y[anchors], s=30, marker='x', color='k')
            
            plt.title("$" + rosbag_group[0]['rosbag_file_name'] + "$")
            ax.legend(loc='best')
            plt.show()

    def overall_analysis(self):

        red = Color("green")
        colors = list(red.range_to(Color("red"),100))

        # create dict of rosbag files
        bags = set()
        methods = set()
        for run in self.runs:
            bags.add(run['rosbag_file'])
            methods.add(run['description'])

        bags = sorted(bags)
        methods = sorted(methods)

        # Create the plot data
        bag_to_x = {}
        method_to_y = {}


        x3 = []
        y3 = []
        z3 = []
        dz = []
        # style.use('ggplot')
        for run in self.runs:
            
            if run['description'] not in method_to_y:
                method_to_y[run['description']] = len(method_to_y)
            if run['rosbag_file_name'] not in bag_to_x:
                bag_to_x[run['rosbag_file_name']] = len(bag_to_x)

        for run in self.runs:
            if(run['rosbag_file_name'].endswith("bag4.bag")):
                continue

            x3.append(bag_to_x[run['rosbag_file_name']])
            y3.append(method_to_y[run['description']])
            z3.append(0)
            z = run['stats']['avg_error_magn']
            
            if run['rosbag_file_name'] == "bag7":
                z = 0

            if z > 2:
                z = 0
            dz.append(z)

        # create color array
        cs = []
        max_z = max(dz)
        for i in range(0, len(dz)):
            
            per = int((dz[i]/max_z) * 100)
            if per == 0:
                per += 98
            if per >= 100:
                per = 98
            cs.append(colors[per].hex)


        fig = plt.figure()
        ax1 = fig.add_subplot(111, projection='3d')

        dx = 0.15
        dy = 0.4
        
        ax1.bar3d(x3, y3, z3, dx, dy, dz, color=cs)

        # morph bagnames into more beauty
        bag_names = [x[0:3] + "_" + x[3] for x in bag_to_x.keys()]

        ax1.set_zlabel('$Mean~error~[m]$', labelpad=20)
        ax1.set_xticklabels(["$\mathrm{" + x + "}$" for x in bag_names], fontsize = 18)
        ax1.set_xticks(bag_to_x.values())
        ax1.set_yticklabels(["$\mathrm{" + x + "}$" for x in method_to_y.keys()], fontsize = 22, rotation=45, ha="right") # rotation=45
        ax1.set_yticks(method_to_y.values())
        plt.show()
                

    def analyze_by_method(self):
        
        grouped = groupby(self.runs,key=lambda x:x['description'])

        data_full = []
        data_2nd_half = []
        data_4th_quarter = []
        for description, method_runs in grouped:

            method_name = description
            print(method_name)
            subarr = []
            subarr_2nd_half = []
            subarr_4th_quarter = []

            for run in method_runs:
                if max(run['stats']['absolute_error_magn_array']) < 50:
                    nr = len(run['stats']['absolute_error_magn_array'])
                    subarr.extend(run['stats']['absolute_error_magn_array'])
                    subarr_2nd_half.extend(run['stats']['absolute_error_magn_array'][nr/2:nr-1])
                    subarr_4th_quarter.extend(run['stats']['absolute_error_magn_array'][nr/4*3:nr-1])

            data_full.append((method_name, subarr))
            data_2nd_half.append((method_name, subarr_2nd_half))
            data_4th_quarter.append((method_name, subarr_4th_quarter))

        labels = [x[0] for x in data_full]
        data_full = [x[1] for x in data_full]
        data_2nd_half = [x[1] for x in data_2nd_half]
        data_4th_quarter = [x[1] for x in data_4th_quarter]

        plot_data = []
        for i in range(len(data_full)):
            plot_data.append([data_full[i], data_2nd_half[i], data_4th_quarter[i]])
        
        self.createGroupedPlot(plot_data, 3, labels, ['full', '2n half', '4th quarter'])
        

    def setBoxColors(self,  bp, nr_plots, c):
        for i in range(nr_plots):
            co = c[i]
            setp(bp['boxes'][i],        color=co)
            setp(bp['caps'][2*i],       color=co)
            setp(bp['caps'][2*i+1],     color=co)
            setp(bp['whiskers'][2*i],   color=co)
            setp(bp['whiskers'][2*i+1], color=co)
            #setp(bp['fliers'][i],     color=co)
            #setp(bp['fliers'][2*i+1],   color=co)
            setp(bp['medians'][i],      color=co)

    
    def createGroupedPlot(self, data, nr_plots_per_data, labels, type_lables):
        # Some fake data to plot
        c = ['black', 'black', 'black']
        co = ['k', 'k', 'k']

        fig = figure()
        ax = axes()
        hold(True)

        counter = 1
        for d in data:
            p = []
            for i in range(counter, counter + nr_plots_per_data):
                p.append(i)
                
            bp = boxplot(d, positions = p, widths = 0.6, showfliers=False)
            self.setBoxColors(bp, nr_plots_per_data, c)
            counter += nr_plots_per_data + 1

        
        counter = 1
        for d in data:
            p = []
            for i in range(counter, counter + nr_plots_per_data):
                avg = np.mean(d[i-counter])
                plot([i-.3, i+0.3], [avg, avg], 'b--')
            counter += nr_plots_per_data + 1

        # set axes limits and labels
        for i in range(len(labels)):
            labels[i] = "$" + labels[i] + "$"

        #ax.set_xticklabels(labels, rotation=45, ha='right')
        ax.set_xticklabels(labels)

        ticks = []
        for i in range(len(data)):
            ticks.append( i * nr_plots_per_data + 1 + nr_plots_per_data/2 + i)
        ax.set_xticks(ticks)
        xlim(0,ticks[-1]+nr_plots_per_data/2.0)

        # draw temporary red and blue lines and use them to create a legend

        hs = []
        for i in range(nr_plots_per_data):
            h, = plot([1,1], co[i]+'-')
            hs.append(h)
        if nr_plots_per_data > 1:
            legend(hs, type_lables)

        for h in hs:
            h.set_visible(False)


        ax.set_ylabel('$Error~magnitude~[m]$')


        plt.grid(axis='y', which='both')
        show()


    # Imports / calculates all statistics of the given report
    def get_run_stats(self, run):
        stats = { }
        
        matrix_folder = run['results_directory']

        base_dir = run['results_directory']+'saved_results/traj_est/'

        # Import the relative error data
        relative_error_files = sorted(glob.glob(base_dir + 'relative_error*.yaml'))
        stats['relative_error'] = {}
        for i in range(len(relative_error_files)):
            
            with open(relative_error_files[i]) as file:
                stats['relative_error'][str(i)] = yaml.load(file, Loader=yaml.FullLoader)

        # absolute error data
        abs_error_file = glob.glob(base_dir + 'absolute_err_statistics*.yaml')[0] # there should only be one
        with open(abs_error_file) as file:
            stats['absolute_error'] = yaml.load(file, Loader=yaml.FullLoader)

        # print(glob.glob(base_dir + 'absolute_err_magn*.txt'))
        print base_dir
        abs_error_magnitude = glob.glob(base_dir + 'absolute_err_magn*.txt')[0] # there should only be one
        stats['absolute_error_magn_array'] = np.genfromtxt(abs_error_magnitude, delimiter=' ')
        stats['avg_error_magn'] = np.mean(stats['absolute_error_magn_array'])
        
        accumulated_distances = glob.glob(base_dir + 'accumulated_distances*.txt')[0] # there should only be one
        stats['accumulated_distances_array'] = np.genfromtxt(accumulated_distances, delimiter=' ')

        anchor_file_name = run['results_directory'] + 'anchors.txt'
        print(anchor_file_name)
        if os.path.isfile(anchor_file_name): 
            anchors = np.genfromtxt(anchor_file_name, delimiter=' ')

            if(len(anchors) > 0):
                groundtruth = np.genfromtxt(run['results_directory'] + 'stamped_groundtruth.txt', delimiter=' ')
                first_time_stamp = groundtruth[0][0]
                last_time_stamp = groundtruth[-1][0]
                duration = last_time_stamp - first_time_stamp
                
                # Convert timestamps to run percentages
                for i in range(len(anchors)):
                    anchors[i] = (anchors[i] - first_time_stamp)/duration

                # Convert percentages to indexes of accumulated_distances_array
                nr_distances = len(stats['accumulated_distances_array'])
                for i in range(len(anchors)):
                    anchors[i] = int(nr_distances * anchors[i])

                # they are int already but we have to say that to python
                anchors = anchors.astype(int)

            stats['anchor_timestamps'] = anchors

        else:
            stats['anchor_timestamps'] = np.array([])

        


        return stats


   
    def createInternetPlot(self):
        query_durations = [2.32881047537, 2.20884776433, 2.812710766, 2.85055974272, 4.79453839482, 8.45755432901, 22.5594959557, 35.9912379583]
        max_cov = [0.29867984, 0.38547028, 0.35764667, 0.37062374, 0.29964114, 0.16086658, 0.15392038, 0.05801051]
        labels = ["", r"$100~\frac{Mbit}{s}$", r'$50~\frac{Mbit}{s}$', r"$10~\frac{Mbit}{s}$", r"$5~\frac{Mbit}{s}$", r"$1~\frac{Mbit}{s}$", r"$500~\frac{kbit}{s}$", r"$250~\frac{kbit}{s}$", r"$100~\frac{kbit}{s}$"]
        x = [1,2,3,4,5,6,7,8]

        fig, ax1 = plt.subplots()

        color = 'tab:red'
        ax1.set_xlabel('Internet bandwidth limit')
        ax1.set_ylabel('avg query duration [s]', color=color)
        ax1.plot(x, query_durations, '--', color=color, label='avg query duration')
        ax1.plot([],[], '-', label='Max cov entry')
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.set_xticklabels(labels)

        ax2 = ax1.twinx()

        color = 'tab:blue'
        ax2.set_ylabel('max standard deviation entry [m]', color=color) 
        ax2.plot(x, max_cov, color=color, label='max std entry')
        ax2.tick_params(axis='y', labelcolor=color)


        ax1.legend(loc='center left')

        fig.tight_layout()
        plt.show()
if __name__ == '__main__':
    an = Analyzer(None, None)
    an.createInternetPlot()