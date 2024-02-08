import cv2 as cv # opencv
import mediapipe as mp  # mediapipe
import time
# from mediapipe.tasks import python
# from mediapipe.tasks.python import vision

# Configuration Options for mediapipe task
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

# Create a pose landmarker instance with the live stream mode:
def print_result(result: PoseLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    print('pose landmarker result: {}'.format(result))

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='./pose_landmarker_heavy.task'),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result)

# With statement releases resources used by PoseLandmarker after usage
with PoseLandmarker.create_from_options(options) as landmarker:
    
    # Use OpenCV’s VideoCapture to start capturing from the webcam.
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    while True: #loop until break
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Convert BGR to RGB
        # frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        # Display the resulting frame
        cv.imshow('Webcam', frame)
        if cv.waitKey(1) == ord('q'):
            break

        # Convert the frame received from OpenCV to a MediaPipe’s Image object.
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB,
                            data=frame)

        # Declare frame timestamp for landmarker
        frame_timestamp_ms = int(time.time() * 1000)

        # Send live image data to perform pose landmarking.
        landmarker.detect_async(mp_image, frame_timestamp_ms)

    # When everything done, release the capture
cap.release()
cv.destroyAllWindows()