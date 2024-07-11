import numpy as np

y = 0.5+0.5*np.sin( np.linspace(0,2*np.pi,120,endpoint=False) )
y = np.rint(255*y).astype(int)

with open('sin_data.txt', 'w') as f:
    for i, y_ in enumerate(y):
        f.write(f'{y_:<3d},')
        if (i % 10) == 9:
            f.write('\n')