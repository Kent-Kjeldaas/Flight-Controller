from matplotlib import pyplot as plt
from matplotlib import style
import numpy as np

style.use('ggplot')
infile = open('test.txt', 'r')
a,b,c,d,e,f,g,h,i = np.loadtxt(infile, unpack=True)

plt.plot(a,d)
plt.title('Noise')
plt.ylabel('Angle Pitch')
plt.xlabel('Samples')
plt.show()