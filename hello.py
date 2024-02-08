import cv2 as cv
import mediapipe as mp
import time
import logging

# Initialize logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Custom result processing function
def print_result(result, output_image, timestamp_ms):
    landmarks = result.pose_landmarks.landmark
    logging.info(f'Frame at {timestamp_ms} ms: Detected {len(landmarks)} landmarks')
    for idx, landmark in enumerate(landmarks[:5]):  # Example: Print only the first 5 landmarks
        logging.info(f'Landmark {idx}: x={landmark.x}, y={landmark.y}, z={landmark.z}, visibility={landmark.visibility}')

# Assuming all the configuration options are defined as in your original code snippet.

options = mp.tasks.vision.PoseLandmarkerOptions(
    base_options=mp.tasks.BaseOptions(model_asset_path='pose_landmarker_heavy.task'),
    running_mode=mp.tasks.vision.RunningMode.LIVE_STREAM,
    result_callback=print_result)

with mp.tasks.vision.PoseLandmarker.create_from_options(options) as landmarker:
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.warning("Can't receive frame (stream end?). Exiting ...")
            break

        # Display the resulting frame
        cv.imshow('Webcam', frame)
        if cv.waitKey(1) == ord('q'):
            break

        # Convert BGR to RGB
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        # Create a MediaPipe Image object
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)

        # Declare frame timestamp for landmarker
        frame_timestamp_ms = int(time.time() * 1000)

        # Send live image data to perform pose landmarking, but limit the logging frequency
        if frame_count % 30 == 0:  # Adjust the modulus value to change the frequency
            landmarker.detect_async(mp_image, frame_timestamp_ms)
        frame_count += 1

cv.destroyAllWindows()
