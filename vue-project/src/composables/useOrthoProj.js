import { PoseLandmarker } from "https://cdn.skypack.dev/@mediapipe/tasks-vision@0.10.0";

export function drawOrthographicProjections(landmarksArray, canvas) {
  const ctx = canvas.getContext("2d");
  const canvasWidth = canvas.width;
  const canvasHeight = canvas.height;
  ctx.clearRect(0, 0, canvasWidth, canvasHeight);
  if (!landmarksArray || landmarksArray.length === 0) return;

  // Ensure landmarksArray is an array of arrays.
  if (!Array.isArray(landmarksArray[0])) {
    landmarksArray = [landmarksArray];
  }

  landmarksArray.forEach((landmarks) => {
    // Calculate the bounding box of the landmarks.
    let minX = Infinity,
      maxX = -Infinity,
      minY = Infinity,
      maxY = -Infinity;
    landmarks.forEach((point) => {
      if (point.x < minX) minX = point.x;
      if (point.x > maxX) maxX = point.x;
      if (point.y < minY) minY = point.y;
      if (point.y > maxY) maxY = point.y;
    });

    // Add a margin (e.g., 5% of the bounding box dimensions).
    const marginX = 0.05 * (maxX - minX);
    const marginY = 0.05 * (maxY - minY);
    minX = Math.max(0, minX - marginX);
    maxX = Math.min(1, maxX + marginX);
    minY = Math.max(0, minY - marginY);
    maxY = Math.min(1, maxY + marginY);

    // Remap each landmark so that the bounding box spans the entire canvas.
    ctx.fillStyle = "red";
    landmarks.forEach((point) => {
      const normX = ((point.x - minX) / (maxX - minX)) * canvasWidth;
      const normY = ((point.y - minY) / (maxY - minY)) * canvasHeight;
      ctx.beginPath();
      ctx.arc(normX, normY, 5, 0, 2 * Math.PI);
      ctx.fill();
    });

    // Draw connections between landmarks.
    const connections = PoseLandmarker.POSE_CONNECTIONS;
    if (connections) {
      ctx.strokeStyle = "blue";
      ctx.lineWidth = 2;
      connections.forEach((connection) => {
        const startIndex = connection.start;
        const endIndex = connection.end;
        if (landmarks[startIndex] && landmarks[endIndex]) {
          const startX =
            ((landmarks[startIndex].x - minX) / (maxX - minX)) * canvasWidth;
          const startY =
            ((landmarks[startIndex].y - minY) / (maxY - minY)) * canvasHeight;
          const endX =
            ((landmarks[endIndex].x - minX) / (maxX - minX)) * canvasWidth;
          const endY =
            ((landmarks[endIndex].y - minY) / (maxY - minY)) * canvasHeight;
          ctx.beginPath();
          ctx.moveTo(startX, startY);
          ctx.lineTo(endX, endY);
          ctx.stroke();
        }
      });
    }
  });
}
