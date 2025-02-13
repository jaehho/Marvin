import { PoseLandmarker, FilesetResolver, DrawingUtils } from "https://cdn.skypack.dev/@mediapipe/tasks-vision@0.10.0";

export async function usePoseDetection(runningMode = "VIDEO", numPoses = 1) {
  const vision = await FilesetResolver.forVisionTasks(
    "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.0/wasm"
  );
  const poseLandmarker = await PoseLandmarker.createFromOptions(vision, {
    baseOptions: {
      modelAssetPath:
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task",
      delegate: "GPU"
    },
    runningMode,
    numPoses
  });

  // detectPose accepts a video element, a canvas element, and an optional timestamp.
  // It draws the overlay (using MediaPipe's drawing utilities) on the provided canvas.
  async function detectPose(videoElement, canvasElement, timestamp) {
    const canvasCtx = canvasElement.getContext("2d");
    const drawingUtils = new DrawingUtils(canvasCtx);

    // Match canvas size to video dimensions.
    canvasElement.width = videoElement.videoWidth;
    canvasElement.height = videoElement.videoHeight;
    canvasCtx.clearRect(0, 0, canvasElement.width, canvasElement.height);

    const ts = timestamp || performance.now();
    const result = await poseLandmarker.detectForVideo(videoElement, ts);

    // Draw landmarks and connectors on the overlay canvas.
    for (const landmark of result.landmarks) {
      drawingUtils.drawLandmarks(landmark, {
        radius: (data) => DrawingUtils.lerp(data.from?.z ?? 0, -0.15, 0.1, 5, 1)
      });
      drawingUtils.drawConnectors(landmark, PoseLandmarker.POSE_CONNECTIONS);
    }
    return result;
  }

  return { poseLandmarker, detectPose };
}