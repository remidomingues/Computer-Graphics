import matplotlib.pyplot as plt

steps = range(1, 8);
values1 = [12289, 7766, 5644, 5482, 4524, 4466, 0, 0];
values2 = [0, 0, 0, 0, 0, 0, 0, 0];
values3 = [0, 0, 0, 0, 0, 0, 0, 0];

fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.axis([min(steps), max(steps), 0, max([max(values1), max(values2)])*1.05 ])
plt.plot(steps, values1, label="No AA")
plt.plot(steps, values2, label="Sequential AA")
plt.plot(steps, values3, label="Multithreaded AA")
handles, labels = ax.get_legend_handles_labels()
lgd = ax.legend(handles, labels, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

# plt.title("Render time")
plt.xlabel('Number of threads')
plt.ylabel('Render time (ms)')
plt.show()

fig.savefig('benchmarks', bbox_extra_artists=(lgd,), bbox_inches='tight')