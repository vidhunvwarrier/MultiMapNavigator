import numpy as np
from PIL import Image
import yaml
import os
import matplotlib.pyplot as plt
import sqlite3

# Load map data from YAML file
def load_map_data(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    image_path = os.path.join(os.path.dirname(yaml_path), data['image'])
    image = Image.open(image_path).convert('L')
    image_data = np.array(image)
    resolution = data['resolution']
    origin = np.array(data['origin'][:2])
    return image_data, resolution, origin

# extract coordinates of overlapping points
def extract_coordinates(mask, resolution, min_corner, map_height):
    indices = np.argwhere(mask)
    coords = []
    for y, x in indices:
        world_x = min_corner[0] + x * resolution
        world_y = min_corner[1] + (map_height - y - 1) * resolution
        coords.append((world_x, world_y))
    return coords

# Load map data
map1_data, res, map1_orgin = load_map_data("/home/vidhun/map/room1.yaml")
map2_data, res, map2_orgin = load_map_data("/home/vidhun/map/room2.yaml")

# Convert map data to boolean arrays
map1_real_data = np.array(map1_data.shape[::-1]) * res
map2_real_data = np.array(map2_data.shape[::-1]) * res

# Calculate the complete canvas size
min_corner = np.minimum(map1_orgin, map2_orgin)
max_corner = np.maximum(map1_orgin + map1_real_data, map2_real_data + map2_orgin)
canvas_size_m = max_corner - min_corner
canvas_size_pix = (canvas_size_m / res).astype(int)

# Create empty canvas
canvas2 = np.zeros((canvas_size_pix[1], canvas_size_pix[0]), dtype=bool)
canvas1 = np.zeros((canvas_size_pix[1], canvas_size_pix[0]), dtype=bool)

# Fill the canvas with map data
offset1 = ((map1_orgin - min_corner) / res).astype(int)
offset1_img = [canvas_size_pix[1] - offset1[1] - map1_data.shape[0], offset1[0]]
canvas1[offset1_img[0]:offset1_img[0]+map1_data.shape[0],
        offset1_img[1]:offset1_img[1]+map1_data.shape[1]] = map1_data != 205

offset2 = ((map2_orgin - min_corner) / res).astype(int)
offset2_img = [canvas_size_pix[1] - offset2[1] - map2_data.shape[0], offset2[0]]
canvas2[offset2_img[0]:offset2_img[0]+map2_data.shape[0],
        offset2_img[1]:offset2_img[1]+map2_data.shape[1]] = map2_data != 205

# Find overlapping areas
overlapping = canvas1 & canvas2
#extract coordinates of overlapping points
overlapping_points = extract_coordinates(overlapping, res, min_corner, canvas1.shape[0])

# Save overlapping points to a sqlite database
conn = sqlite3.connect("/home/vidhun/map/map.db")
cursor = conn.cursor()
cursor.execute("""
    CREATE TABLE IF NOT EXISTS overlaps (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    x REAL,
    y REAL,
    region TEXT
)
               """)

data = [(x , y , "room1&room2")for x, y in overlapping_points]

cursor.executemany("INSERT INTO overlaps (x, y, region) VALUES (?, ?, ?)", data)
conn.commit()
conn.close()
