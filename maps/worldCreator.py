import cv2
import numpy as np

def create_world_from_image(image_path, world_path, resolution, origin):
    # Load the image (black and white)
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Threshold the image to make it binary
    _, img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY_INV)

    # Perform connected component analysis
    num_labels, labels = cv2.connectedComponents(img)

    # Create the world file
    with open(world_path, 'w') as f:
        # Write the header
        f.write('<?xml version="1.0"?>\n')
        f.write('<sdf version="1.5">\n')
        f.write('  <world name="default">\n')
        f.write('    <include>\n')
        f.write('      <uri>model://ground_plane</uri>\n')
        f.write('    </include>\n')
        f.write('    <include>\n')
        f.write('      <uri>model://sun</uri>\n')
        f.write('    </include>\n')

        # Write the walls
        for label in range(1, num_labels):
            # Get the coordinates of the pixels in this wall
            y, x = np.where(labels == label)

            # Calculate the position and size of the wall
            x_start, y_start = np.min(x), np.min(y)
            x_end, y_end = np.max(x), np.max(y)
            x = (x_start + x_end) / 2 * resolution + origin[0]
            y = (img.shape[0] - y_start - 1) * resolution + origin[1]
            size_x = (x_end - x_start + 1) * resolution
            size_y = (y_end - y_start + 1) * resolution

            # Write the model for the wall to the Gazebo world file
            f.write('    <model name="wall_{0}">\n'.format(label))
            f.write('      <pose>{0} {1} 0.5 0 0 0</pose>\n'.format(x, y))
            f.write('      <static>true</static>\n')
            f.write('      <link name="link">\n')
            f.write('        <collision name="collision">\n')
            f.write('          <geometry>\n')
            f.write('            <box><size>{0} {1} 1</size></box>\n'.format(size_x, size_y))
            f.write('          </geometry>\n')
            f.write('        </collision>\n')
            f.write('        <visual name="visual">\n')
            f.write('          <geometry>\n')
            f.write('            <box><size>{0} {1} 1</size></box>\n'.format(size_x, size_y))
            f.write('          </geometry>\n')
            f.write('        </visual>\n')
            f.write('      </link>\n')
            f.write('    </model>\n')

        # Write the footer
        f.write('  </world>\n')
        f.write('</sdf>\n')

        
# Usage example
create_world_from_image('/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/levine.pgm', '/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/levine.world', 0.05, [-12.824999, -12.824999])
