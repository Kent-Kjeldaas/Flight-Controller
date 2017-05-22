from pylab import *

def MakeGraph(file, x, y):
	filename = file 
	infile = open(filename, 'r')
	val = loadtxt(infile, unpack=True)
	plot(val[x],val[y])
	#plot.xlabel('Smarts')
	#plot.ylabel('Probability')
	#plot.title('Histogram of IQ')
	#plot.text(60, .025, r'$\mu=100,\ \sigma=15$')
	#plot.axis([40, 160, 0, 0.03])
	#plot.grid(True)
	show()
	return;

file = input('Filename:')
x = input('x:')
y = input('y:')
MakeGraph(file, x, y)