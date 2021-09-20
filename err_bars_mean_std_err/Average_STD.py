import sys
import numpy as np
import statistics
import math
costs = []
times = []
expansions = []
for i in range(1):
    costs.append([])
    times.append([])
    expansions.append([])
    with open(sys.argv[i+1], 'r') as f:
        for line in f:
            if line.startswith('total expands this call ='):
                times[i] += [float(line.split(' ')[9])]
                expansions[i] += [float(line.split(' ')[5][0:-1])]
                costs[i] += [float((line.split(' ')[12]).split('=')[1])]

error_times_mean = []
error_times_std = []
error_costs_mean = []
error_costs_std = []
error_expansions_mean = []
error_expansions_std = []

for i in range(1):
    error_times_mean.append(round(statistics.mean(times[i]), 2))
    error_times_std.append(round(np.std(times[i])/math.sqrt(28),2))
    error_costs_mean.append(round(statistics.mean(costs[i]),2))
    error_costs_std.append(round(np.std(costs[i])/math.sqrt(28),2))
    error_expansions_mean.append(round(statistics.mean(expansions[i]),2))
    error_expansions_std.append(round(np.std(expansions[i])/math.sqrt(28),2))
    print(str(error_times_mean[i]) + " +/- " + str(error_times_std[i]))
    print(str(error_costs_mean[i]) + " +/- " + str(error_costs_std[i]))
    print(str(error_expansions_mean[i]) + " +/- " + str(error_expansions_std[i]) + "\n")