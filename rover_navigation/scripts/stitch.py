import cv2
import os
from concurrent.futures import ThreadPoolExecutor

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
    frames_dir = os.path.expanduser("~/panorama_frames")
    
    # Get the latest session directory
    sessions = [os.path.join(frames_dir, d) for d in os.listdir(frames_dir) if os.path.isdir(os.path.join(frames_dir, d))]
    latest_session = max(sessions, key=os.path.getmtime)
    
    # Get all image files from the latest session
    image_files = [os.path.join(latest_session, f) for f in os.listdir(latest_session) if f.endswith('.jpg')]
    image_files.sort()  # Ensure files are in order

    print(f"Found {len(image_files)} images in {latest_session}")

    # Resize images
    with ThreadPoolExecutor() as executor:
        frames = list(executor.map(lambda img_path: resize_image(img_path, scale_percent), image_files))
    
    # Remove any None values (failed loads)
    #frames = [frame for frame in frames if frame is not None]

    print(f"Resized {len(frames)} images. Stitching...")
    panorama = stitch_frames(frames)

    if panorama is not None:
        output_path = os.path.join(latest_session, "panorama.jpg")
        cv2.imwrite(output_path, panorama)
        print(f"Panorama saved as {output_path}")
    else:
        print("Failed to create panorama")