
import numpy as np
        
ar = np.genfromtxt("/home/eric/statistics_asa/query_times.txt", delimiter=' ')

print("Query Times:")
print(np.mean(ar, axis=0))
# print(np.cov(ar, rowvar = False))
print(np.std(ar, axis=0))

# create the plots

drone_anchors = np.genfromtxt("/home/eric/statistics_asa/anchors_drone.txt", delimiter=' ')
holol_anchors = np.genfromtxt("/home/eric/statistics_asa/anchors.txt", delimiter=' ')

drone_mean = np.mean(drone_anchors, axis=0)
holol_mean = np.mean(holol_anchors, axis=0)

for i in range(len(drone_anchors)):
    drone_anchors[i,:] = drone_anchors[i,:] - drone_mean

for i in range(len(holol_anchors)):
    holol_anchors[i,:] = holol_anchors[i,:] - holol_mean
    if(max(abs(holol_anchors[i,0:3])) > 0.8):
        holol_anchors[i,0:3] = np.array([0,0,0])

print("Std drone anchors")
print(np.std(drone_anchors, axis=0))
print("Std hololens anchors")
print(np.std(holol_anchors, axis=0))



from matplotlib import pyplot as plt
from matplotlib import style
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

def confidence_ellipse(x, y, ax, n_std=3.0, facecolor='none', **kwargs):
    """
    Create a plot of the covariance confidence ellipse of *x* and *y*.

    Parameters
    ----------
    x, y : array-like, shape (n, )
        Input data.

    ax : matplotlib.axes.Axes
        The axes object to draw the ellipse into.

    n_std : float
        The number of standard deviations to determine the ellipse's radiuses.

    **kwargs
        Forwarded to `~matplotlib.patches.Ellipse`

    Returns
    -------
    matplotlib.patches.Ellipse
    """
    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    cov = np.cov(x, y)
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensionl dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor=facecolor, **kwargs)

    # Calculating the stdandard deviation of x from
    # the squareroot of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0, 0]) * n_std
    mean_x = np.mean(x)

    # calculating the stdandard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std
    mean_y = np.mean(y)

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.size": 30,
    "font.sans-serif": ["Helvetica"]})


plt.title("$Anchor~X,~Y~locations$")
da = plt.scatter(drone_anchors[:,0], drone_anchors[:,1], marker="x", c="red")
ha = plt.scatter(holol_anchors[:,0], holol_anchors[:,1], marker="v", c='green')
plt.xlabel('$X~[m]$', labelpad=20)
plt.ylabel('$Y~[m]$', labelpad=20)
plt.grid()
ed = confidence_ellipse(drone_anchors[:,0], drone_anchors[:,1], plt.gca(), n_std=2, edgecolor='red')
eh = confidence_ellipse(holol_anchors[:,0], holol_anchors[:,1], plt.gca(), n_std=2, edgecolor='green')


plt.legend((da,ha,ed,eh),
           ('Drone anchor', 'HoloLens 2 anchor', r'Drone $2\sigma$', r'HL2 $2\sigma$'),
           scatterpoints=1,
           loc='best',
           ncol=2,
           fontsize=20)


plt.show()