from pylab import *

filename = input('filename: ')
infile = open(filename, 'r')
x,y = loadtxt(infile, unpack=True)
plot(x,y)
#plot.xlabel('Smarts')
#plot.ylabel('Probability')
#plot.title('Histogram of IQ')
#plot.text(60, .025, r'$\mu=100,\ \sigma=15$')
#plot.axis([40, 160, 0, 0.03])
#plot.grid(True)
show()