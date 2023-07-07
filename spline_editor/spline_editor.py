import numpy as np
from matplotlib.backend_bases import MouseButton


class SplineEditor:
    """
    A simple interactive editor for BSpline.

    Press 't' to toggle vertex markers on and off.  When vertex markers are on,
    they can be dragged with the mouse.
    """
    showverts = True
    epsilon = 1.0  # max distance to count as a vertex hit

    def __init__(self, spline_path, spline_graph, plot_params):

        self.graph = spline_graph
        self.ax = spline_graph.axes
        canvas = self.ax.figure.canvas
        self.plot_params = plot_params

        self.path = spline_path
        x, y = self.path.points[:,0], self.path.points[:,1]
        self.line, = self.ax.plot(x, y, marker='o', markerfacecolor='r', animated=True)

        self._ind = None  # the active vertex

        canvas.mpl_connect('draw_event', self.on_draw)
        canvas.mpl_connect('button_press_event', self.on_button_press)
        canvas.mpl_connect('key_press_event', self.on_key_press)
        canvas.mpl_connect('button_release_event', self.on_button_release)
        canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas = canvas

    def get_ind_under_point(self, event):
        """
        Return the index of the point closest to the event position or *None*
        if no point is within ``self.epsilon`` to the event position.
        """
        xt, yt = self.path.points[:,0], self.path.points[:,1]
        d = np.sqrt((xt - event.xdata)**2 + (yt - event.ydata)**2)
        ind = d.argmin()
        return ind if d[ind] < self.epsilon else None

    def on_draw(self, event):
        """Callback for draws."""
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        self.path.draw_path( self.graph, self.plot_params )
        self.ax.draw_artist(self.graph)
        self.ax.draw_artist(self.line)
        self.canvas.blit(self.ax.bbox)

    def on_button_press(self, event):
        """Callback for mouse button presses."""
        if (event.inaxes is None
                or event.button != MouseButton.LEFT
                or not self.showverts):
            return
        self._ind = self.get_ind_under_point(event)

    def on_button_release(self, event):
        """Callback for mouse button releases."""
        if (event.button != MouseButton.LEFT
                or not self.showverts):
            return
        self._ind = None

    def on_key_press(self, event):
        """Callback for key presses."""
        if not event.inaxes:
            return
        if event.key == 't':
            self.showverts = not self.showverts
            self.line.set_visible(self.showverts)
            if not self.showverts:
                self._ind = None            
        self.canvas.draw()

    def on_mouse_move(self, event):
        """Callback for mouse movements."""

        if (self._ind is None
                or event.inaxes is None
                or event.button != MouseButton.LEFT
                or not self.showverts):
            return

        pts = self.path.points
        pts[self._ind,0], pts[self._ind,1] = event.xdata, event.ydata

        self.path.set_points(pts)
        x, y = pts[:,0], pts[:,1]
        self.line.set_data(x, y)

        self.canvas.restore_region(self.background)
        self.path.draw_path( self.graph, self.plot_params )
        self.ax.draw_artist(self.graph)
        self.ax.draw_artist(self.line)
        self.canvas.blit(self.ax.bbox)
        self.canvas.draw()
