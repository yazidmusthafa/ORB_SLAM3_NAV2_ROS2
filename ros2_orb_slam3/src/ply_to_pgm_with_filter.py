import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

# Function to read and extract x and y coordinates from a PLY file
def extract_xy_from_ply(file_path):
    # Read the point cloud file
    pcd = o3d.io.read_point_cloud(file_path)

    # Convert the point cloud to a numpy array
    points = pcd.points

    # Extract x and y coordinates
    x = [point[0] for point in points]  # X is at index 0
    y = [point[2] for point in points]  # Y is at index 2

    return x, y

# Function to remove outliers using RadiusOutlierRemoval
def remove_outliers(pcd, nb_points=2, radius=0.05):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    inlier_cloud = pcd.select_by_index(ind)
    return inlier_cloud

# Function to plot x and y coordinates
def plot_xy(x, y):
    plt.figure(figsize=(10, 8))
    plt.scatter(x, y, s=1, c='blue', alpha=0.5)
    plt.title("X-Y Points from PLY File")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Function to convert the point cloud to a PGM file for NAV2
def save_as_pgm(x, y, resolution=0.03, output_file="map.pgm"):
    # Normalize coordinates to positive values
    x = np.array(x)
    y = np.array(y)
    x_min, y_min = x.min(), y.min()
    x -= x_min
    y -= y_min

    # Determine the size of the grid
    width = int(np.ceil(x.max() / resolution))
    height = int(np.ceil(y.max() / resolution))

    print("Height : ", height,", Width : ", width)

    # Create an empty grid initialized to 255 (free space)
    grid = np.ones((height, width), dtype=np.uint8) * 255

    # Fill grid cells corresponding to point locations with 0 (occupied space)
    for i in range(len(x)):
        grid_x = int(np.floor(x[i] / resolution))
        grid_y = int(np.floor(y[i] / resolution))
        grid_y = height - grid_y - 1  # Invert y-coordinate for top-down view
        grid[grid_y, grid_x] = 0

    # Save the grid as a PGM image
    img = Image.fromarray(grid)
    img.save(output_file)
    print(f"Map saved as {output_file}")

# Main function
if __name__ == "__main__":
    # Specify the path to your PLY file
    ply_file_path = "./saved_map/test_map.ply"  # Replace with the actual file path

    # Read the point cloud file
    pcd = o3d.io.read_point_cloud(ply_file_path)

    # Remove outliers
    filtered_pcd = remove_outliers(pcd)

    # Extract x and y coordinates from the filtered point cloud
    points = filtered_pcd.points
    x_coords = [point[0] for point in points]
    y_coords = [point[2] for point in points]

    # Plot the x and y points
    # plot_xy(x_coords, y_coords)

    # Save the map as a PGM file
    save_as_pgm(x_coords, y_coords, resolution=0.03, output_file="map.pgm")
