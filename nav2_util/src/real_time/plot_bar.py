# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import matplotlib.pyplot as plt
import numpy as np
import re
import argparse


def create_parser():
    parser = argparse.ArgumentParser(description='Navigation2 log file parser configuration')
    parser.add_argument('-f', '--filename', type=str,
                        dest='filename', default='input.log', help='Log file name')
    return parser


def read(logfile):
    title = ""
    desired = 0
    jitter = 0
    X = []
    Y = []
    with open(logfile) as f:

        # Get the graph title
        line = f.readline()
        if line.startswith('Name:'):
            m = re.search('Name: (.*)$', line)
            title = m.group(1)

        # Get the desired rate
        line = f.readline()
        if line.startswith('Desired rate:'):
            m = re.search('Desired rate: (\d+)', line)
            desired = int(m.group(1)) / 1000000

        # Get the jitter margin
        line = f.readline()
        if line.startswith('Jitter margin:'):
            m = re.search('Jitter margin: (\d+)', line)
            jitter = int(m.group(1)) * 0.01

        # Get the data
        while line:
            # Process a line
            if line.startswith('Iteration:'):
                m = re.search('Iteration: (\d+)', line, re.IGNORECASE)
                X.append(int(m.group(1)))
                n = re.search('(\d+) nsecs', line, re.IGNORECASE)
                Y.append(float(n.group(1)))
            line = f.readline()

    # Convert to milliseconds
    Y2 = [y / 1000000 for y in Y]
    return title, desired, jitter, X, Y2


def plot_bar(title, desired, jitter, X, Y):
    fig_size = plt.rcParams["figure.figsize"]
    fig_size[0] = 9
    fig_size[1] = 6
    plt.rcParams["figure.figsize"] = fig_size
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

    ax = plt.subplot()
    ax.margins(x=0)

    # Don't allow the axis to be on top of the data
    ax.set_axisbelow(True)

    # Customize the minor grid
    ax.set_xticks(X)
    ax.set_xticklabels(X)

    # Don't show every label on the x axis
    every_nth = 10
    for n, label in enumerate(ax.xaxis.get_ticklabels()):
        if n % every_nth != 0:
            label.set_visible(False)

    # To specify the number of ticks on both or any single axes
    ax.locator_params(axis='y', nbins=20)

    plt.grid(zorder=0)

    bar_width = 0.8
    plt.bar(X, Y, width=bar_width, zorder=3, color='b')

    max_value = int(desired + desired * jitter)
    min_value = int(desired - desired * jitter)

    t1 = [i for i in X if Y[i] > max_value]
    t2 = [i for i in Y if i > max_value]
    plt.bar(t1, t2, color='r', width=bar_width, zorder=4)

    t10 = [i for i in X if Y[i] < min_value]
    t20 = [i for i in Y if i < min_value]
    plt.bar(t10, t20, color='b', width=bar_width, zorder=4)

    title_obj = plt.title(title, fontsize=18, fontweight='normal', pad=20)
    d_label = '  ' + str(1000 / desired) + "Hz"

    plt.setp(title_obj, color='g')
    plt.xlabel('Iteration', fontsize=15)
    plt.ylabel('Time (ms)', fontsize=15)
    plt.hlines(desired, 0, len(X), colors="g", linestyle="dashed", linewidth=1.0)
    plt.text(len(X), desired, d_label, ha='left', va='center', color="black")
    plt.hlines(max_value, 0, len(X), colors="r", linestyle="solid", linewidth=1.0)
    plt.hlines(min_value, 0, len(X), colors="r", linestyle="solid", linewidth=1.0)
    plt.axhspan(min_value, max_value, color='green', alpha=0.2)

    plt.show()
    return


def main():
    args = create_parser().parse_args()
    title, desired, jitter, X, Y = read(args.filename)
    plot_bar(title, desired, jitter, X, Y)

if __name__ == "__main__":
    main()
