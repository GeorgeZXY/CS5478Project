
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import pybullet as p


def plot_rectangle(x_min, y_min, x_max, y_max):
    """
    Plots a rectangle on the current matplotlib axis.

    Args:
        x_min (float): Minimum x-coordinate of the rectangle.
        y_min (float): Minimum y-coordinate of the rectangle.
        x_max (float): Maximum x-coordinate of the rectangle.
        y_max (float): Maximum y-coordinate of the rectangle.
    """
    width = x_max - x_min
    height = y_max - y_min
    rect = Rectangle(
        (x_min, y_min), width, height, edgecolor="black", facecolor="black"
    )
    plt.gca().add_patch(rect)



def get_bbox(id):
        link_ids = [
            i
            for i in range(
                -1, p.getNumJoints(id)
            )
        ]
        aabb_bounds = [
            p.getAABB(id, link_id)
            for link_id in link_ids
        ]
        return aabb_bounds

def get_navigation_map(robot, plot=True):
    ob_bboxs = []
    ob_ids = list(range(p.getNumBodies()))
    # print(nav_obstacle_ids)
    ob_ids.remove(0)  # remove plane
    ob_ids.remove(robot.robotId)

    for obj_id in ob_ids:
        obj_bounds = get_bbox(obj_id)
        for link_bounds in obj_bounds:
            ob_bboxs.append(
                [
                    link_bounds[0][0],
                    link_bounds[0][1],
                    link_bounds[1][0],
                    link_bounds[1][1],
                ]
            )
            
    if plot:
        plt.clf()
        for x_min, y_min, x_max, y_max in ob_bboxs:
            plot_rectangle(x_min, y_min, x_max, y_max)
        
        # Simplified axis bounds calculation
        if ob_bboxs:
            x_min = min([bounds[0] for bounds in ob_bboxs]) - 2
            y_min = min([bounds[1] for bounds in ob_bboxs]) - 2
            x_max = max([bounds[2] for bounds in ob_bboxs]) + 2
            y_max = max([bounds[3] for bounds in ob_bboxs]) + 2
        else:
            x_min, y_min, x_max, y_max = 0, 0, 10, 10
        
        plt.axis([x_min, x_max, y_min, y_max])
        plt.axis("equal")
        plt.title("Navigation Map")
        plt.pause(0.01)
        plt.show()

    return ob_bboxs


