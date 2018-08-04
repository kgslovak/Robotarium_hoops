class ArenaPlot():
    def __init__(self, ax, x_len, y_len, z_len):
        # Draw "arena limits"
        ax.plot([ x_len,  x_len], [-y_len,  y_len], [   0.0,    0.0], color='k', dashes=[ 1, 1])
        ax.plot([-x_len, -x_len], [-y_len,  y_len], [   0.0,    0.0], color='k', dashes=[ 1, 1])
        ax.plot([ x_len, -x_len], [ y_len,  y_len], [   0.0,    0.0], color='k', dashes=[ 1, 1])
        ax.plot([ x_len, -x_len], [-y_len, -y_len], [   0.0,    0.0], color='k', dashes=[ 1, 1])
        ax.plot([ x_len,  x_len], [ y_len,  y_len], [   0.0, -z_len], color='k', dashes=[ 1, 1])
        ax.plot([ x_len,  x_len], [-y_len, -y_len], [   0.0, -z_len], color='k', dashes=[ 1, 1])
        ax.plot([-x_len, -x_len], [ y_len,  y_len], [   0.0, -z_len], color='k', dashes=[ 1, 1])
        ax.plot([-x_len, -x_len], [-y_len, -y_len], [   0.0, -z_len], color='k', dashes=[ 1, 1])
        ax.plot([ x_len,  x_len], [-y_len,  y_len], [-z_len, -z_len], color='k', dashes=[ 1, 1])
        ax.plot([-x_len, -x_len], [-y_len,  y_len], [-z_len, -z_len], color='k', dashes=[ 1, 1])
        ax.plot([ x_len, -x_len], [ y_len,  y_len], [-z_len, -z_len], color='k', dashes=[ 1, 1])
        ax.plot([ x_len, -x_len], [-y_len, -y_len], [-z_len, -z_len], color='k', dashes=[ 1, 1])

        # Set axes properties
        xlim = max(x_len, y_len, z_len)
        ylim = max(x_len, y_len, z_len)
        zlim = max(x_len, y_len, z_len)

        ax.set_xlim( xlim, -xlim)
        ax.set_ylim(-ylim,  ylim)
        ax.set_zlim( zlim - zlim/2, -zlim - zlim/2)
        
        ax.set_xticks([-1.5,  0.0, 1.5])
        ax.set_yticks([-1.0,  0.0, 1.0])
        ax.set_zticks([ 0.0, -2.0])

        ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.xaxis._axinfo["grid"]['color'] = (1,1,1,0)
        ax.yaxis._axinfo["grid"]['color'] = (1,1,1,0)
        ax.zaxis._axinfo["grid"]['color'] = (1,1,1,0)
