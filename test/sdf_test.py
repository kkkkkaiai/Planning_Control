import numpy as np
import matplotlib.pyplot as plt


def gen_sdf(points, x_lim=[-1, 4], y_lim=[-1, 4]):
    # 3 generate the signed distance field
    # 3.1 generate the grid
    x = np.linspace(x_lim[0], x_lim[1], 100)
    y = np.linspace(y_lim[0], y_lim[1], 100)
    X, Y = np.meshgrid(x, y)
    # 3.2 compute the signed distance field
    # 3.2.1 compute the distance from the point to the grid and record the combined gradient of the distance
    dist = np.zeros((len(points), len(x), len(y)))
    gradient = np.zeros((len(x), len(y), 2))
    for i in range(len(points)):
        dist[i] = np.sqrt((X - points[i, 0])**2 + (Y - points[i, 1])**2)

    # 3.2.2 find the least distance index and compute the gradient
    dist_min_index = np.argmin(dist, axis=0)
    gradient[:, :, 0] = X - points[dist_min_index, 0] + 0.0001
    gradient[:, :, 1] = Y - points[dist_min_index, 1] + 0.0001
    gradient = gradient / np.linalg.norm(gradient, axis=2, keepdims=True)
    dist = np.min(dist, axis=0)

    return X, Y, dist, gradient

def main():
    # test generate signed distance field
    # 1 give a figure with size of 5 * 5
    fig_x_lim = [-1, 4]
    fig_y_lim = [-1, 4]
    fig = plt.figure(figsize=(fig_x_lim[1] - fig_x_lim[0], fig_y_lim[1] - fig_y_lim[0]))
    ax = plt.gca()
    
    first_plot = True

    for i in range(5):
        # 2 generate 10 points in the given size of the figure
        points = np.random.rand(10, 2) * 2

        X, Y, dist, gradient = gen_sdf(points, fig_x_lim, fig_y_lim)

        if first_plot:
            first_plot = False
            # 3.3 plot the signed distance field
            # dist_contour = ax.contour(X, Y, dist, 20)
            # 3.4 plot the gradient
            # the quiver is too small
            grad_quiver = ax.quiver(X, Y, gradient[:, :, 0], gradient[:, :, 1], color='black')
            # 3.5 plot the points
            points_draw = ax.scatter(points[:, 0], points[:, 1], color='black', marker='o')
        else:
            # contour = dist_contour.collections[:]
            # print(contour)
            # for coll in contour:
            #     ax.collections.remove(coll)
            #     contour.remove(coll)
            # print(dist)
            # dist_contour.set_array(dist)
            
            grad_quiver.set_UVC(gradient[:, :, 0], gradient[:, :, 1])
            points_draw.set_offsets(points[:, :2]) 
            
            # reset gradient

        # 4 show the result
        plt.xlim(fig_x_lim[0], fig_x_lim[1])
        plt.ylim(fig_y_lim[0], fig_y_lim[1])
        plt.axis('equal')
        plt.pause(1)

if __name__ == '__main__':
    main()


