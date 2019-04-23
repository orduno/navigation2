import matplotlib.pyplot as plt
import numpy as np
import re
import argparse

def create_parser():
    parser = argparse.ArgumentParser(description='Realtime Logparser Configuration')
    parser.add_argument('-f', '--filename', type=str, dest='filename', default='logfile.txt', help='Log file name')
    parser.add_argument('-j', '--jitter', type=int, dest='jitter', default=10, help='Acceptable Jitter')

    return parser

title = ""
desired = 0

def read(logfile):
    X = []
    Y = []
    with open(logfile) as f:

        # Get the graph title
        line = f.readline()
        global title
        if line.startswith('Name:'):
          m = re.search('Name: (.*)$', line)
          title = m.group(1)

        # Get the desired looptime
        line = f.readline()
        global desired
        if line.startswith('Desired looptime:'):
          m = re.search('Desired looptime: (\d+)', line)
          desired = int(m.group(1))/1000000

        # Get the data
        while line:
            # process line
            if line.startswith('Iteration:'):
                m = re.search('Iteration: (\d+)', line, re.IGNORECASE)
                #X.append(float(m.group(1)))
                X.append(int(m.group(1)))
                n = re.search('(\d+) nsecs', line, re.IGNORECASE)
                Y.append(float(n.group(1)))
            line = f.readline()

    # Convert to ms
    Y2 = [y / 1000000 for y in Y]
    return X,Y2

def plot_bar(args, X, Y):
    fig_size = plt.rcParams["figure.figsize"]
    fig_size[0] = 9 
    fig_size[1] = 6
    plt.rcParams["figure.figsize"] = fig_size
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

    ax = plt.subplot()
    ax.margins(x=0)

    # Don't allow the axis to be on top of your data
    ax.set_axisbelow(True)

    # Turn on the minor ticks, which are required for the minor grid
    #ax.minorticks_on()

    # Customize the major grid
    #ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')

    # Customize the minor grid
    ax.set_xticks(X)
    ax.set_xticklabels(X)

    every_nth = 10 
    for n, label in enumerate(ax.xaxis.get_ticklabels()):
        if n % every_nth != 0:
            label.set_visible(False)

    # To specify the number of ticks on both or any single axes
    ax.locator_params(axis='y', nbins=20)

    plt.grid(zorder=0)

    bar_width = 0.8

    #plt.bar(X, Y, align='edge', width=0.3, zorder=3)
    plt.bar(X, Y, width=bar_width, zorder=3, color='b')

    max_value = int(desired + desired * (args.jitter * 0.01));
    min_value = int(desired - desired * (args.jitter * 0.01));

    t1 = [i for i in X if Y[i] > max_value]
    t2 = [i for i in Y if i > max_value]
    plt.bar(t1, t2, color='r', width=bar_width, zorder=4)

    t10 = [i for i in X if Y[i] < min_value]
    t20 = [i for i in Y if i < min_value]
    plt.bar(t10, t20, color='b', width=bar_width, zorder=4)

    title_obj = plt.title(title, fontsize=18, fontweight='normal', pad=20)
    plt.setp(title_obj, color='g')
    plt.xlabel('Iteration', fontsize=15)
    plt.ylabel('Loop Time (ms)', fontsize=15)
    plt.hlines(desired, 0, len(X), colors="g", linestyle="dashed", linewidth=1.0)

    d_label = '  ' + str(1000/desired) + "Hz"
    #plt.text(len(X), desired, ' Desired', ha='left', va='center', color="black")
    plt.text(len(X), desired, d_label, ha='left', va='center', color="black")
    plt.hlines(max_value, 0, len(X), colors="r", linestyle="solid", linewidth=1.0)
    plt.hlines(min_value, 0, len(X), colors="r", linestyle="solid", linewidth=1.0)
    plt.axhspan(min_value, max_value, color='green', alpha=0.2)

    plt.show()
    return

def main():
    args = create_parser().parse_args()
    X,Y = read(args.filename)
    plot_bar(args, X, Y)

if __name__ == "__main__":
    main()
