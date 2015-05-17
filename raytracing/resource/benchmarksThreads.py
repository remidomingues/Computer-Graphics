import matplotlib.pyplot as plt

steps = range(1, 8);
values1 = [6548, 4790, 3607, 3351, 3042, 2594, 2159, 2000];
values2 = [17793, 15916, 14780, 14762, 14335, 14130, 13844, 13244];
values3 = [17793, 13454, 9228, 8965, 6908, 6667, 5923, 5589];

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

fig.savefig('benchmarksThreads', bbox_extra_artists=(lgd,), bbox_inches='tight')

steps = range(1, 8);
values1 = [6548, 4790, 3607, 3351, 3042, 2594, 2159, 2000];
values2 = [17793, 15916, 14780, 14762, 14335, 14130, 13844, 13244];
values3 = [17793, 13454, 9228, 8965, 6908, 6667, 5923, 5589];

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

fig.savefig('benchmarksThreads', bbox_extra_artists=(lgd,), bbox_inches='tight')