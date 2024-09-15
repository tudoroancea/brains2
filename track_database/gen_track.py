from argparse import ArgumentParser
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, spatial, interpolate
from shapely.geometry.polygon import Point, LineString, Polygon
from enum import Enum


def save_cones(
    filename: str,
    blue_cones: np.ndarray,
    yellow_cones: np.ndarray,
    orange_cones: np.ndarray,
) -> None:
    """
    Save the cones in CSV file specified by filename. This file will have the following
    format:
        color,X,Y
    """
    with open(filename, "w") as f:
        f.write("color,X,Y\n")
        for cone in blue_cones:
            f.write("blue,{},{}\n".format(cone[0], cone[1]))
        for cone in yellow_cones:
            f.write("yellow,{},{}\n".format(cone[0], cone[1]))
        for cone in orange_cones:
            f.write("orange,{},{}\n".format(cone[0], cone[1]))


# TODO(mattbrth): judge if this is still needed and if we need to keep only on track width
def save_center_line(filename: str, center_line: np.ndarray, track_widths: np.ndarray):
    """
    Save the center line in CSV file specified by filename. This file will have the
    following format:
        X,Y,right_width,left_width
    """
    np.savetxt(
        filename,
        np.hstack((center_line, track_widths)),
        delimiter=",",
        header="x,y,right_width,left_width",
    )


def plot_cones(
    blue_cones: np.ndarray,
    yellow_cones: np.ndarray,
    orange_cones: np.ndarray,
    origin: np.ndarray = np.zeros(2),
    show: bool = True,
):
    """Plot a series of cones with matplotlib."""
    plt.scatter(blue_cones[:, 0], blue_cones[:, 1], s=14, c="b", marker="^")
    plt.scatter(yellow_cones[:, 0], yellow_cones[:, 1], s=14, c="y", marker="^")
    plt.scatter(orange_cones[:, 0], orange_cones[:, 1], s=28, c="orange", marker="^")
    plt.scatter(origin[0], origin[1], c="g", marker="x")
    plt.axis("equal")
    plt.tight_layout()
    if show:
        plt.show()


class Mode(Enum):
    """Possible modes for how Voronoi regions are selected."""

    # Find closest nodes around starting node, results in roundish track shapes.
    EXPAND = 1
    # Find nodes closest to line extending from starting node, results in elongated track shapes.
    EXTEND = 2
    # Select all regions randomly, results in large track shapes.
    RANDOM = 3


def closest_node(node: np.ndarray, nodes: np.ndarray, k: int) -> int:
    """
    Returns the index of the k-th closest node.

    Args:
        node (numpy.ndarray): Node to find k-th closest node to.
        nodes (numpy.ndarray): Available nodes.
        k (int): Number which determines which closest node to return.

    Returns:
        int: Index of k-th closest node.
    """
    deltas = nodes - node
    distance = np.einsum("ij,ij->i", deltas, deltas)
    return np.argpartition(distance, k)[k]


def clockwise_sort(p: np.ndarray) -> np.ndarray:
    """
    Sorts nodes in clockwise order.

    Args:
        p (numpy.ndarray): Points to sort.

    Returns:
        numpy.ndarray: Clockwise sorted points.
    """
    d = p - np.mean(p, axis=0)
    s = np.arctan2(d[:, 0], d[:, 1])
    return p[np.argsort(s), :]


def curvature(
    dx_dt: np.ndarray, d2x_dt2: np.ndarray, dy_dt: np.ndarray, d2y_dt2: np.ndarray
) -> np.ndarray:
    """
    Calculates the curvature along a line.

    Args:
        dx_dt (numpy.ndarray): First derivative of x.
        d2x_dt2 (numpy.ndarray): Second derivative of x.
        dy_dt (numpy.ndarray): First derivative of y.
        d2y_dt2 (numpy.ndarray): Second derivative of y.

    Returns:
        np.ndarray: Curvature along line.
    """
    return (dx_dt**2 + dy_dt**2) ** -1.5 * (dx_dt * d2y_dt2 - dy_dt * d2x_dt2)


def arc_length(x: np.ndarray, y: np.ndarray, R: np.ndarray) -> float:
    """
    Calculates the arc length between to points based on the radius of curvature of the path segment.

    Args:
        x (numpy.ndarray): X-coordinates.
        y (numpy.ndarray): Y-coordinates.
        R (numpy.ndarray): Radius of curvature of track segment in meters.
    Returns:
        (float): Arc length in meters.
    """
    x0, x1 = x[:-1], x[1:]
    y0, y1 = y[:-1], y[1:]
    R = R[:-1]

    distance = np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    theta = 2 * np.arcsin(0.5 * distance / R)
    arc_length = R * theta
    return arc_length


def transformation_matrix(
    displacement: tuple[float, float], angle: float
) -> np.ndarray:
    """
    Translate, then rotate around origin.

    Args:
        displacement (tuple): Distance to translate along both axes.
        angle (float): Angle in radians to rotate.

    Returns:
        numpy.ndarray: 3x3 transformation matrix.
    """
    h, k = displacement
    c, s = np.cos(angle), np.sin(angle)

    M = np.array([[c, -s, h * c - k * s], [s, c, h * s + k * c], [0, 0, 1]])
    return M


class TrackGenerator:
    """
    Generates a random track based on a bounded Voronoi diagram.
    Ensures that the tracks curvature is within limits and that the car starts at a straight section.
    """

    ORANGE_CONES = np.array([[-2.2, 4.7], [2.2, 4.7], [-2.2, 7.3], [2.2, 7.3]])

    def __init__(
        self,
        n_points: int = 60,
        n_regions: int = 20,
        min_bound: float = 0.0,
        max_bound: float = 150.0,
        track_width: float = 3.0,
        cone_spacing: float = 5.0,
        lengh_start_area: float = 6.0,
        curvature_threshold: float = 1 / 3.75,
        straight_threshold: float = 1 / 100.0,
        mode: Mode = Mode.EXTEND,
        z_offset: float = 0.0,
        lat_offset: float = 0.0,
        lon_offset: float = 0.0,
    ):
        # Input parameters
        self._n_points = n_points  # [-]
        self._n_regions = n_regions  # [-]
        self._min_bound = min_bound  # [m]
        self._max_bound = max_bound  # [m]
        self._bounding_box = np.array(
            [self._min_bound, self._max_bound] * 2
        )  # [x_min, x_max, y_min, y_max]
        self._mode = mode

        # Track parameters
        self._track_width = 3.0  # [m]
        self._cone_spacing = 5.0  # [m]
        self._length_start_area = 6.0  # [m]
        self._curvature_threshold = 1.0 / 3.75  # [m^-1]
        self._straight_threshold = 1.0 / 100.0  # [m^-1]

        # Output options
        self._z_offset = z_offset
        self._lat_offset = lat_offset
        self._lon_offset = lon_offset

    def bounded_voronoi(
        self, input_points: np.ndarray, bounding_box: np.ndarray
    ) -> spatial.qhull.Voronoi:
        """
        Creates a Voronoi diagram bounded by the bounding box.
        Mirror input points at edges of the bounding box.
        Then create Voronoi diagram using all five sets of points.
        This prevents having a Voronoi diagram with edges going off to infinity.

        Args:
            input_points (numpy.ndarray): Coordinates of input points for Voronoi diagram.
            bounding_box (numpy.ndarray): Specifies the boundaries of the Voronoi diagram, [x_min, x_max, y_min, y_max].

        Returns:
            scipy.spatial.qhull.Voronoi: Voronoi diagram object.
        """

        def _mirror(boundary, axis):
            mirrored = np.copy(points_center)
            mirrored[:, axis] = 2 * boundary - mirrored[:, axis]
            return mirrored

        x_min, x_max, y_min, y_max = bounding_box

        # Mirror points around each boundary
        points_center = input_points
        points_left = _mirror(x_min, axis=0)
        points_right = _mirror(x_max, axis=0)
        points_down = _mirror(y_min, axis=1)
        points_up = _mirror(y_max, axis=1)
        points = np.concatenate(
            [points_center, points_left, points_right, points_down, points_up]
        )

        # Compute Voronoi
        vor = spatial.Voronoi(points)

        # We only need the section of the Voronoi diagram that is inside the bounding box
        vor.filtered_points = points_center
        vor.filtered_regions = np.array(vor.regions, dtype=object)[
            vor.point_region[: vor.npoints // 5]
        ]
        return vor

    def create_track(self, plot_track: bool = True, plot_voronoi: bool = True):
        """
        Creates a track from the vertices of a Voronoi diagram.
        1.  Create bounded Voronoi diagram.
        2.  Select regions of Voronoi diagram based on selection mode.
        3.  Get the vertices belonging to the regions and sort them clockwise.
        4.  Interpolate between vertices.
        5.  Calculate curvature of track to check wether the curvature threshold is exceeded.
        6.  If curvature threshold is exceeded, remove vertice where the curvature is the highest from its set.
            Repeat steps 4-6 until curvature is within limimts.
        7.  Check if track does not cross itself. If so, go to step 2 and reiterate.
        8.  Find long enough straight section to place start line and start position.
        9.  Translate and rotate track to origin.
        10. Create track yaml file.
        """
        # Create bounded Voronoi diagram
        input_points = np.random.uniform(
            self._min_bound, self._max_bound, (self._n_points, 2)
        )
        vor = self.bounded_voronoi(input_points, self._bounding_box)

        while True:
            if self._mode.value == 1:
                # Pick a random point and find its n closest neighbours
                random_index = np.random.randint(0, self._n_points)
                random_point_indices = [random_index]
                random_point = input_points[random_index]

                for i in range(self._n_regions - 1):
                    closest_point_index = closest_node(
                        random_point, input_points, k=i + 1
                    )
                    random_point_indices.append(closest_point_index)

            elif self._mode.value == 2:
                # Pick a random point, create a line extending from this point and find other points close to this line
                random_index = np.random.randint(0, self._n_points)
                random_heading = np.random.uniform(0, np.pi / 2)
                random_point = input_points[random_index]

                start = (
                    random_point[0]
                    - 1.0 / 2.0 * self._max_bound * np.cos(random_heading),
                    random_point[1]
                    - 1.0 / 2.0 * self._max_bound * np.sin(random_heading),
                )
                end = (
                    random_point[0]
                    + 1.0 / 2.0 * self._max_bound * np.cos(random_heading),
                    random_point[1]
                    + 1.0 / 2.0 * self._max_bound * np.sin(random_heading),
                )
                line = LineString([start, end])
                distances = [Point(p).distance(line) for p in input_points]
                random_point_indices = np.argpartition(distances, self._n_regions)[
                    : self._n_regions
                ]

            elif self._mode.value == 3:
                # Select regions randomly
                random_point_indices = np.random.randint(
                    0, self._n_points, self._n_regions
                )

            # From the Voronoi regions, get the regions belonging to the randomly selected points
            regions = np.array(
                [np.array(region) for region in vor.regions], dtype=object
            )
            random_region_indices = vor.point_region[random_point_indices]
            random_regions = np.concatenate(regions[random_region_indices])

            # Get the vertices belonging to the random regions
            random_vertices = np.unique(vor.vertices[random_regions], axis=0)

            # Sort vertices
            sorted_vertices = clockwise_sort(random_vertices)
            sorted_vertices = np.vstack([sorted_vertices, sorted_vertices[0]])

            while True:
                # Interpolate
                tck, _ = interpolate.splprep(
                    [sorted_vertices[:, 0], sorted_vertices[:, 1]], s=0, per=True
                )
                t = np.linspace(0, 1, 200, endpoint=False)
                x, y = interpolate.splev(t, tck, der=0)
                dx_dt, dy_dt = interpolate.splev(t, tck, der=1)
                d2x_dt2, d2y_dt2 = interpolate.splev(t, tck, der=2)

                # Calculate curvature
                k = curvature(dx_dt, d2x_dt2, dy_dt, d2y_dt2)
                abs_curvature = np.abs(k)

                # Check if curvature exceeds threshold
                peaks, _ = signal.find_peaks(abs_curvature)
                exceeded_peaks = abs_curvature[peaks] > self._curvature_threshold
                max_peak_index = abs_curvature[peaks].argmax()
                is_curvature_exceeded = exceeded_peaks[max_peak_index]

                if is_curvature_exceeded:
                    # Find vertice where curvature is exceeded and delete vertice from sorted vertices. Reiterate
                    max_peak = peaks[max_peak_index]
                    peak_coordinate = (x[max_peak], y[max_peak])
                    vertice = closest_node(peak_coordinate, sorted_vertices, k=0)
                    sorted_vertices = np.delete(sorted_vertices, vertice, axis=0)

                    # Make sure that first and last coordinate are the same for periodic interpolation
                    if not np.array_equal(sorted_vertices[0], sorted_vertices[-1]):
                        sorted_vertices = np.vstack(
                            [sorted_vertices, sorted_vertices[0]]
                        )
                else:
                    break

            # Create track boundaries
            track = Polygon(zip(x, y))
            track_left = track.buffer(self._track_width / 2)
            track_right = track.buffer(-self._track_width / 2)

            # Check if track does not cross itself
            if track.is_valid and track_left.is_valid and track_right.is_valid:
                if (
                    track.geom_type
                    == track_left.geom_type
                    == track_right.geom_type
                    == "Polygon"
                ):
                    break

        # Calculate cone spacing
        cone_spacing_left = np.linspace(
            0,
            track_left.length,
            np.ceil(track_left.length / self._track_width).astype(int) + 1,
        )[:-1]
        cone_spacing_right = np.linspace(
            0,
            track_right.length,
            np.ceil(track_right.length / self._track_width).astype(int) + 1,
        )[:-1]

        # Determine coordinates of cones
        cones_left = np.asarray(
            [
                np.asarray(track_left.exterior.interpolate(sp).xy).flatten()
                for sp in cone_spacing_left
            ]
        )
        cones_right = np.asarray(
            [
                np.asarray(track_right.exterior.interpolate(sp).xy).flatten()
                for sp in cone_spacing_right
            ]
        )

        # Find straight section in track that is at least the length of the start area
        # If such a section cannot be found, adjust the straight_threshold and length_start_area variables
        # There is only a chance of this happening if n_regions == 1
        straight_threshold = (
            self._straight_threshold
            if abs_curvature.min() < self._straight_threshold
            else abs_curvature.min() + 0.1
        )
        straight_sections = abs_curvature[:-1] <= straight_threshold
        distances = arc_length(x, y, 1 / abs_curvature)
        length_straights = distances * straight_sections

        # Find cumulative length of straight sections
        for i in range(1, len(length_straights)):
            if length_straights[i]:
                length_straights[i] += length_straights[i - 1]

        # Find start line
        length_start_area = (
            self._length_start_area
            if length_straights.max() > self._length_start_area
            else length_straights.max()
        )
        try:
            start_line_index = np.where(length_straights > length_start_area)[0][0]
        except IndexError:
            raise Exception(
                "Unable to find suitable starting position. Try to decrease the length of the starting area or different input parameters."
            )
        # find origin
        rel_distances = np.cumsum(distances)
        rel_distances -= rel_distances[start_line_index]
        origin_index = np.min(np.argwhere(rel_distances >= -6.0))
        start_position = np.array([x[origin_index], y[origin_index]])

        # Translate and rotate track to origin
        x = np.concatenate((x[origin_index:], x[:origin_index]))
        y = np.concatenate((y[origin_index:], y[:origin_index]))
        self.blue_cones = cones_left - start_position
        self.yellow_cones = cones_right - start_position
        self.center_line = np.column_stack((x, y)) - start_position
        start_heading = np.arctan2(y[1] - y[0], x[1] - x[0])
        angle_offset = np.pi / 2 - start_heading
        rot = np.array(
            [
                [np.cos(angle_offset), -np.sin(angle_offset)],
                [np.sin(angle_offset), np.cos(angle_offset)],
            ]
        )
        self.blue_cones = self.blue_cones @ rot.T
        self.yellow_cones = self.yellow_cones @ rot.T
        self.center_line = self.center_line @ rot.T

        # Create track file
        if plot_voronoi:
            self._plot_voronoi(
                vor, sorted_vertices, random_point_indices, input_points, x, y
            )
        if plot_track:
            self._plot_track()

    def _plot_voronoi(
        self,
        vor: spatial.qhull.Voronoi,
        sorted_vertices: np.ndarray,
        random_point_indices: np.ndarray,
        input_points: np.ndarray,
        x: np.ndarray,
        y: np.ndarray,
    ) -> None:
        """
        Visualises the voronoi diagram and the resulting track.

        Args:
            vor (scipy.spatial.qhull.Voronoi): Voronoi diagram object.
            sorted_vertices (numpy.ndarray): Selected vertices sorted clockwise.
            random_point_indices (numpy.ndarray): Selected points.
            input_points (numpy.ndarray): All Voronoi points.
        """
        # Plot initial points
        plt.figure()
        plt.plot(vor.filtered_points[:, 0], vor.filtered_points[:, 1], "b.")

        # Plot vertices points
        for region in vor.filtered_regions:
            vertices = vor.vertices[region, :]
            plt.plot(vertices[:, 0], vertices[:, 1], "go")

        # Plot edges
        for region in vor.filtered_regions:
            vertices = vor.vertices[region + [region[0]], :]
            plt.plot(vertices[:, 0], vertices[:, 1], "k-")

        # Plot selected vertices
        plt.scatter(
            sorted_vertices[:, 0],
            sorted_vertices[:, 1],
            color="y",
            s=200,
            label="Selected vertices",
        )

        # Plot selected points
        plt.scatter(
            *input_points[random_point_indices].T,
            s=100,
            marker="x",
            color="b",
            label="Selected points",
        )

        # Plot track
        plt.scatter(x, y)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.axis("equal")
        plt.legend()

    def _plot_track(self):
        """
        Plots the resulting track. The car will start at the origin.
        """
        plt.figure()
        plot_cones(
            self.blue_cones,
            self.yellow_cones,
            self.ORANGE_CONES,
            show=False,
        )
        plt.plot(*self.center_line.T, "k-o", markersize=2)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.axis("equal")
        plt.show()

    def save_track(
        self,
        output_dir: str,
        track_name: str,
    ):
        """Writes the track data to a csv files."""
        track_dir = os.path.abspath(os.path.join(output_dir, track_name))
        os.makedirs(track_dir, exist_ok=True)
        save_cones(
            os.path.join(track_dir, track_name + "_cones.csv"),
            self.blue_cones,
            self.yellow_cones,
            TrackGenerator.ORANGE_CONES,
            [],
        )
        save_center_line(
            os.path.join(track_dir, track_name + "_center_line.csv"),
            self.center_line,
            np.full_like(self.center_line, self._track_width / 2),
        )


if __name__ == "__main__":
    parser = ArgumentParser("random_track_generator")
    parser.add_argument(
        "--output_dir", default=os.path.join(os.path.dirname(__file__), "data")
    )
    parser.add_argument("--track_name", required=True)
    parser.add_argument("--seed", required=False)
    args = parser.parse_args()
    if args.seed is not None:
        np.random.seed(int(args.seed))
    generator = TrackGenerator(
        # lat_offset=51.197682,
        # lon_offset=5.323411,
    )
    generator.create_track(plot_track=True, plot_voronoi=True)
    generator.save_track(args.output_dir, args.track_name)
