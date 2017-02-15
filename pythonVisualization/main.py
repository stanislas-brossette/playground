import yaml
from mayavi import mlab

def plotBox(c, s):
    xmin = c[0] - s[0]/2
    xmax = c[0] + s[0]/2
    ymin = c[1] - s[1]/2
    ymax = c[1] + s[1]/2
    zmin = c[2] - s[2]/2
    zmax = c[2] + s[2]/2

    x = [xmin, xmax, xmax, xmin, xmin, xmax, xmax, xmin] 
    y = [ymin, ymin, ymax, ymax, ymin, ymin, ymax, ymax]
    z = [zmin, zmin, zmin, zmin, zmax, zmax, zmax, zmax] 
    triangles = [
            (0, 1, 3), (1,2,3),
            (0, 1, 4), (1,4,5),
            (1, 2, 5), (2,5,6),
            (2, 3, 6), (3,6,7),
            (0, 3, 4), (3,4,7),
            (4, 5, 6), (4,6,7)
            ]

    return mlab.triangular_mesh(x, y, z, triangles)
    

with open("entry.yml", 'r') as stream:
    try:
        content = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

bSize = content['BoxSize']
boxPositions = content['BoxPositions']

# View it.
for pos in boxPositions:
    plotBox(pos, bSize)

mlab.show()
