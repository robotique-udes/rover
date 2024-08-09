import cv2
import os
from concurrent.futures import ThreadPoolExecutor
from ament_index_python.packages import get_package_share_directory

def get_resources_directory(package_name):
        package_share_directory = get_package_share_directory(package_name)
        resource_directory = package_share_directory + "/resource/"
        return resource_directory

def resize_image(img_path, scale_percent):
    img = cv2.imread(img_path)
    if img is None:
        print(f"Failed to load image: {img_path}")
        return None
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

def stitch_frames(frames, max_attempts=3):
    for attempt in range(max_attempts):
        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        stitcher.setRegistrationResol(0.6 - 0.1 * attempt)  
        status, panorama = stitcher.stitch(frames)
        if status == cv2.Stitcher_OK:
            return panorama
    return None

if __name__ == '__main__':
    scale_percent = 50
    frames_dir = get_resources_directory('rover_navigation')
    
    sessions = [os.path.join(frames_dir, d) for d in os.listdir(frames_dir) if os.path.isdir(os.path.join(frames_dir, d))]

    if sessions == []:
        print("No sessions found")
        exit()

    print(f"Found {len(sessions)} sessions")
    
    latest_session = max(sessions, key=os.path.getmtime)
    
    image_files = [os.path.join(latest_session, f) for f in os.listdir(latest_session) if f.endswith('.jpg')]
    image_files.sort()  

    if len(image_files) == 0:
        print("No images found")
        exit()

    with ThreadPoolExecutor() as executor:
        frames = list(executor.map(lambda img_path: resize_image(img_path, scale_percent), image_files))

    print(f"Resized {len(frames)} images. Stitching...")
    panorama = stitch_frames(frames)

    if panorama is not None:
        output_path = os.path.join(latest_session, "panorama.jpg")
        cv2.imwrite(output_path, panorama)
        print(f"Panorama saved as {output_path}")
    else:
        print("Failed to create panorama")