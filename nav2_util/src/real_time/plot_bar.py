import matplotlib.pyplot as plt
import numpy as np
import re
import argparse

def create_parser():
    parser = argparse.ArgumentParser(description='Realtime Logparser Configuration')
    parser.add_argument('-f', '--filename', type=str, dest='filename', default='logfile.txt', help='Log file name')
    parser.add_argument('-d', '--desired', type=int, dest='desired', default=100000000, help='Desired Looptime')
    parser.add_argument('-j', '--jitter', type=int, dest='jitter', default=10, help='Acceptable Jitter')

    return parser

def read(logfile):
    X = []
    Y = []
    with open(logfile) as f:
        line = f.readline()
        while line:
            # process line
            if line.startswith('Iteration:'):
                m = re.search('Iteration: (\d+)', line, re.IGNORECASE)
                #X.append(float(m.group(1)))
                X.append(int(m.group(1)))
                n = re.search('(\d+) nsecs', line, re.IGNORECASE)
                print(n.group(1))
                Y.append(float(n.group(1)))
            line = f.readline()
    return X,Y

def plot_bar(args, X, Y):
    fig_size = plt.rcParams["figure.figsize"]
    fig_size[0] = 9
    fig_size[1] = 6
    plt.rcParams["figure.figsize"] = fig_size

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

    every_nth = 5
    for n, label in enumerate(ax.xaxis.get_ticklabels()):
        if n % every_nth != 0:
            label.set_visible(False)

    # To specify the number of ticks on both or any single axes
    ax.locator_params(axis='y', nbins=20)

    plt.grid(zorder=0)
    bar_width = 0.7

    #plt.bar(X, Y, align='edge', width=0.3, zorder=3)
    plt.bar(X, Y, width=bar_width, zorder=3)

    max_value = int(args.desired + args.desired * (args.jitter * 0.01));
    min_value = int(args.desired - args.desired * (args.jitter * 0.01));

    t1 = [i for i in X if Y[i] > max_value]
    t2 = [i for i in Y if i > max_value]
    plt.bar(t1, t2, color='r', width=bar_width, zorder=4)

    t10 = [i for i in X if Y[i] < min_value]
    t20 = [i for i in Y if i < min_value]
    plt.bar(t10, t20, color='r', width=bar_width, zorder=4)

    #plt.title('Real-Time Computing', fontsize=20, fontweight='bold')
    plt.title(args.filename, fontsize=18, fontweight='normal', pad=20)

    plt.xlabel('Iteration', fontsize=15)
    plt.ylabel('Loop Time (nsec)', fontsize=15)
    plt.hlines(args.desired, 0, len(X), colors="g", linestyle="dashed", linewidth=1.0)
    plt.text(len(X), args.desired, ' Desired', ha='left', va='center', color="black")
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
