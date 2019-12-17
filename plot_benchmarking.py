"""
Script to create figures for benchmarking PMCCRRT algorithm tests
Alex Hirst
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# path to pandas dataframe pickle:
df1 = pd.read_pickle('./benchmarking_results/benchmark_1.pkl')
df2 = pd.read_pickle('./benchmarking_results/benchmark_2.pkl')
df3 = pd.read_pickle('./benchmarking_results/benchmark_3.pkl')
df4 = pd.read_pickle('./benchmarking_results/benchmark_4.pkl')
figure_directory = './figures/'

settings = ['n = 50, TP = 0.90',
            'n = 50, TP = 0.95',
            'n = 100, TP = 0.90',
            'n = 100, TP = 0.95']

################################################################################
# Number of Tree Nodes
################################################################################
fig = plt.figure(figsize=(8, 6), dpi=80, facecolor='w', edgecolor='k')
plt.title("Number of Tree Nodes", fontsize=14)
plt.xlabel("Settings")
plt.ylabel("Mean Number of Nodes", fontsize=14)
plt.xlim(0.5,4.5)
ax = fig.add_subplot(111)
ax.yaxis.grid(True, color='lightgray')
ax.set_axisbelow(True)

ax.bar([1, 2, 3, 4],
        [np.mean(df1.numbernodes),
         np.mean(df2.numbernodes),
         np.mean(df3.numbernodes),
         np.mean(df4.numbernodes)],
        color='silver',
        width=0.5)
# Plot the min/max bars
# ax.plot([1, 1], [np.min(df1.numbernodes), np.max(df1.numbernodes)],
# color='black', linewidth=4)
#
# ax.plot([2, 2], [np.min(df2.numbernodes), np.max(df2.numbernodes)],
#          color='black', linewidth=4)
#
# ax.plot([3, 3], [np.min(df3.numbernodes), np.max(df3.numbernodes)],
# color='black', linewidth=4)
#
# ax.plot([4, 4], [np.min(df4.numbernodes), np.max(df4.numbernodes)],
#    color='black', linewidth=4)


ax.set_xticks([1, 2, 3, 4])
ax.set_xticklabels(settings, fontsize=10)
plt.savefig(figure_directory + 'numbernodes.png', dpi=600)

################################################################################
# Path Length
################################################################################
fig = plt.figure(figsize=(8, 6), dpi=80, facecolor='w', edgecolor='k')
plt.title("Path Length", fontsize=14)
plt.xlabel("Settings")
plt.ylabel("Mean Path Length (m)", fontsize=14)
plt.xlim(0.5,4.5)
ax = fig.add_subplot(111)
ax.yaxis.grid(True, color='lightgray')
ax.set_axisbelow(True)

ax.bar([1, 2, 3, 4],
        [np.mean(df1.pathlength),
         np.mean(df2.pathlength),
         np.mean(df3.pathlength),
         np.mean(df4.pathlength)],
         yerr=[2*np.std(df1.pathlength),
               2*np.std(df2.pathlength),
               2*np.std(df3.pathlength),
               2*np.std(df4.pathlength)],
        error_kw=dict(lw=4),
        color='silver',
        width=0.5)


ax.set_xticks([1, 2, 3, 4])
ax.set_xticklabels(settings, fontsize=10)
plt.savefig(figure_directory + 'pathlength.png', dpi=600)


################################################################################
# Computation Time
################################################################################
fig = plt.figure(figsize=(8, 6), dpi=80, facecolor='w', edgecolor='k')
plt.title("Total Computation Time", fontsize=14)
plt.xlabel("Settings")
plt.ylabel("Mean Time (s)", fontsize=14)
plt.xlim(0.5,4.5)
ax = fig.add_subplot(111)
ax.yaxis.grid(True, color='lightgray')
ax.set_axisbelow(True)

ax.bar([1, 2, 3, 4],
        [np.mean(df1.runtime),
         np.mean(df2.runtime),
         np.mean(df3.runtime),
         np.mean(df4.runtime)],
        color='silver',
        width=0.5)

# Plot the min/max bars
# ax.plot([1, 1], [np.min(df1.runtime), np.max(df1.runtime)],
#          color='black', linewidth=4)
#
# ax.plot([2, 2], [np.min(df2.runtime), np.max(df2.runtime)],
#          color='black', linewidth=4)
#
# ax.plot([3, 3], [np.min(df3.runtime), np.max(df3.runtime)],
# color='black', linewidth=4)
#
# ax.plot([4, 4], [np.min(df4.runtime), np.max(df4.runtime)],
#    color='black', linewidth=4)

ax.set_xticks([1, 2, 3, 4])
ax.set_xticklabels(settings, fontsize=10)
plt.savefig(figure_directory + 'runtimes.png', dpi=600)
