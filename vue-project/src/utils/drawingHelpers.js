import { PoseLandmarker } from "https://cdn.skypack.dev/@mediapipe/tasks-vision@0.10.0";

export function drawNormalizedLandmarks(landmarksArray, canvas) {
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);

  if (!landmarksArray || landmarksArray.length === 0) return;

  // Ensure landmarksArray is an array of arrays.
  if (!Array.isArray(landmarksArray[0])) {
    landmarksArray = [landmarksArray];
  }

  // Get connection pairs from PoseLandmarker.
  const connections = PoseLandmarker.POSE_CONNECTIONS;

  landmarksArray.forEach(landmarks => {
    // Draw each landmark as a red circle.
    ctx.fillStyle = "red";
    landmarks.forEach(point => {
      const x = point.x * width;
      const y = point.y * height;
      ctx.beginPath();
      ctx.arc(x, y, 5, 0, 2 * Math.PI);
      ctx.fill();
    });
    // Draw connections between landmarks.
    if (connections) {
      ctx.strokeStyle = "blue";
      ctx.lineWidth = 2;
      connections.forEach(connection => {
        const startIndex = connection.start;
        const endIndex = connection.end;
        if (landmarks[startIndex] && landmarks[endIndex]) {
          const startX = landmarks[startIndex].x * width;
          const startY = landmarks[startIndex].y * height;
          const endX = landmarks[endIndex].x * width;
          const endY = landmarks[endIndex].y * height;
          ctx.beginPath();
          ctx.moveTo(startX, startY);
          ctx.lineTo(endX, endY);
          ctx.stroke();
        }
      });
    }
  });
}
