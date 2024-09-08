import cv2
import numpy as np
from pathlib import Path

from hashlib import md5


def slice_image(image_path, tile_size):
    image = cv2.imread(image_path)
    img_height, img_width = image.shape[:2]

    md5_hash = md5(image_path.encode()).hexdigest()
    # Calculate number of tiles
    for y in range(0, img_height, tile_size):
        for x in range(0, img_width, tile_size):
            tile = image[y:y + tile_size, x:x + tile_size]
            cv2.imwrite(f"/Volumes/data/robot/images/target_cam/split_{md5_hash}_{x}_{y}.jpg", tile)

if __name__ == "__main__":
    for image_path in Path("/Volumes/data/robot/images/target_cam").glob("*.jpg"):
        if "._" in str(image_path) or "slice_" in str(image_path):
            continue  # Skip hidden files
        print(f"Processing {image_path}")
        tile_size = 265
        try:
            slice_image(str(image_path), tile_size)
            Path(image_path).unlink()
        except Exception as ex:
            print(f"Error processing {image_path}: {ex}")
            continue
