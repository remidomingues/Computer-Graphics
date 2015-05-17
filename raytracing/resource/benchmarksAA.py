import matplotlib.pyplot as plt

steps = [2, 4, 8, 16];
disabled = [430, 430, 430, 430];
uniform = [1200, 1200, 1200, 1200];
stochastic = [1100, 1100, 1250, 1450];

fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.axis([min(steps), max(steps), 0, max(stochastic) * 1.05])
plt.plot(steps, disabled, label="Anti-aliasing disabled")
plt.plot(steps, disabled, 'bo')
plt.plot(steps, uniform, label="Uniform 8x")
plt.plot(steps, uniform, 'go')
plt.plot(steps, stochastic, label="Stochastic sampling")
plt.plot(steps, stochastic, 'ro')
handles, labels = ax.get_legend_handles_labels()
lgd = ax.legend(handles, labels, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

#plt.title("Render time")
plt.xlabel('Additional rays per pixel')
plt.ylabel('Render time (ms)')
plt.show()

fig.savefig('benchmarksAA', bbox_extra_artists=(lgd,), bbox_inches='tight')