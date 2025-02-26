<template>
  <div class="video-chat-container">
    <h2>Your Peer ID: {{ localPeerId }}</h2>
    <!-- Flex container for remote video and 2x2 grid -->
    <div class="video-layout">
      <!-- Remote Video (Left Column) -->
      <div class="remote-video">
        <h3>Remote Video</h3>
        <video ref="remoteVideo" autoplay playsinline></video>
      </div>
      <!-- 2x2 Grid Container (Right Column) -->
      <div class="grid-container">
        <!-- Top Left: Local Video with Overlay -->
        <div class="local-video">
          <h3>Local Video</h3>
          <div class="video-wrapper">
            <video ref="localVideo" autoplay playsinline></video>
            <canvas ref="localOverlayCanvas" class="overlay-canvas"></canvas>
          </div>
        </div>
        <!-- Top Right: World Landmark 1 (XZ) -->
        <div class="world-landmark">
          <h3>World Landmark 1 (XZ)</h3>
          <canvas ref="worldCanvasXZ" class="world-canvas"></canvas>
        </div>
        <!-- Bottom Left: World Landmark 2 (YZ) -->
        <div class="world-landmark">
          <h3>World Landmark 2 (YZ)</h3>
          <canvas ref="worldCanvasYZ" class="world-canvas"></canvas>
        </div>
        <!-- Bottom Right: World Landmark 3 (XY) -->
        <div class="world-landmark">
          <h3>World Landmark 3 (XY)</h3>
          <canvas ref="worldCanvasXY" class="world-canvas"></canvas>
        </div>
      </div>
    </div>
    <div class="controls">
      <input v-model="targetPeerId" placeholder="Enter Peer ID to Call" />
      <button @click="initiateCall">Call</button>
      <button @click="togglePoseDetection">
        {{ isPoseDetectionEnabled ? "Disable" : "Enable" }} Pose Detection
      </button>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted } from "vue";
import Peer from "peerjs";
import { PoseLandmarker, FilesetResolver, DrawingUtils } from "@mediapipe/tasks-vision";

interface Landmark {
  x: number;
  y: number;
  z: number;
}

// -------------
// Reactive State & Refs
// -------------
const peerInstance = ref<Peer | null>(null);
const localPeerId = ref("");
const targetPeerId = ref("");
const localMediaStream = ref<MediaStream | null>(null);
const activeCall = ref<any>(null);
const turnServers = ref<any[]>([]);
const poseDetector = ref<any>(null);
const isPoseDetectionEnabled = ref(true);
const isDetectionLoopRunning = ref(false);

// New reactive ref for data connection
const dataConnection = ref<any>(null);

// Template element refs
const localVideo = ref<HTMLVideoElement | null>(null);
const remoteVideo = ref<HTMLVideoElement | null>(null);
const localOverlayCanvas = ref<HTMLCanvasElement | null>(null);
const worldCanvasXZ = ref<HTMLCanvasElement | null>(null);
const worldCanvasYZ = ref<HTMLCanvasElement | null>(null);
const worldCanvasXY = ref<HTMLCanvasElement | null>(null);

// -------------
// Helper Functions
// -------------
const clearCanvas = (canvas: HTMLCanvasElement | null) => {
  canvas?.getContext("2d")?.clearRect(0, 0, canvas.width, canvas.height);
};

const setCanvasSize = (canvas: HTMLCanvasElement, video: HTMLVideoElement) => {
  canvas.width = video.videoWidth;
  canvas.height = video.videoHeight;
};

const clearCanvases = (...canvases: (HTMLCanvasElement | null)[]) => {
  canvases.forEach(clearCanvas);
};

/**
 * Returns the projected coordinates for a given landmark on the specified plane.
 */
const orthogonalProjection = (
  landmark: Landmark,
  plane: "xy" | "xz" | "yz",
  width: number,
  height: number
) => {
  const centerX = width / 2;
  const centerY = height / 2;
  let posX = 0,
      posY = 0;
  switch (plane) {
    case "xy":
      posX = landmark.x;
      posY = landmark.y;
      break;
    case "xz":
      posX = landmark.x;
      posY = landmark.z;
      break;
    case "yz":
      posX = landmark.z;
      posY = landmark.y;
      break;
  }
  return { x: posX * width + centerX, y: posY * height + centerY };
};

const keyLandmarkIndices = new Set([11, 12, 13, 14, 15, 16, 23, 24]);

/**
 * Draws world landmarks on a canvas using the specified coordinate plane.
 * @param worldLandmarks Array of landmark arrays.
 * @param canvas The target canvas element.
 * @param plane The coordinate plane to use: "xy", "xz", or "yz".
 */
function drawOrthogonalLandmarks(
  worldLandmarks: Landmark[][],
  canvas: HTMLCanvasElement,
  plane: "xy" | "xz" | "yz"
) {
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    console.error("Failed to get canvas context");
    return;
  }
  const { width, height } = canvas;
  ctx.clearRect(0, 0, width, height);
  if (!worldLandmarks.length) return;

  worldLandmarks.forEach((landmarks) => {
    // Draw individual landmarks
    ctx.fillStyle = "red";
    keyLandmarkIndices.forEach((index) => {
      if (index < landmarks.length) {
        const { x, y } = orthogonalProjection(landmarks[index], plane, width, height);
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, 2 * Math.PI);
        ctx.fill();
      }
    });
    // Draw landmark connections
    ctx.strokeStyle = "blue";
    ctx.lineWidth = 2;
    PoseLandmarker.POSE_CONNECTIONS.forEach(({ start, end }) => {
      if (
        keyLandmarkIndices.has(start) &&
        keyLandmarkIndices.has(end) &&
        landmarks[start] &&
        landmarks[end]
      ) {
        const startPt = orthogonalProjection(landmarks[start], plane, width, height);
        const endPt = orthogonalProjection(landmarks[end], plane, width, height);
        ctx.beginPath();
        ctx.moveTo(startPt.x, startPt.y);
        ctx.lineTo(endPt.x, endPt.y);
        ctx.stroke();
      }
    });
  });
}

// -------------
// Pose Detection Setup
// -------------
async function initializePoseDetection(
  runningMode: "IMAGE" | "VIDEO" = "VIDEO",
  numPoses: number = 1
) {
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

  async function detectPoseOnVideo(
    video: HTMLVideoElement,
    canvas: HTMLCanvasElement,
    timestamp?: number
  ) {
    const ctx = canvas.getContext("2d")!;
    setCanvasSize(canvas, video);
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const ts = timestamp || performance.now();
    const result = await poseLandmarker.detectForVideo(video, ts);
    const drawingUtils = new DrawingUtils(ctx);

    result.landmarks.forEach((landmarks: any) => {
      drawingUtils.drawLandmarks(landmarks, {
        radius: (data: { from?: { z?: number } }) =>
          DrawingUtils.lerp(data.from?.z ?? 0, -0.15, 0.1, 5, 1)
      });
      drawingUtils.drawConnectors(landmarks, PoseLandmarker.POSE_CONNECTIONS);
    });
    return result;
  }

  return { poseLandmarker, detectPoseOnVideo };
}

// -------------
// TURN Server & Media Setup
// -------------
async function fetchTurnServerCredentials() {
  try {
    const res = await fetch(
      "https://marvin.metered.live/api/v1/turn/credentials?apiKey=bca769dfeca57f21e5ceb9eada5afbbf71cd"
    );
    const turnData = await res.json();
    turnServers.value = [
      { urls: "stun:stun.l.google.com:19302" },
      { urls: "stun:stun1.l.google.com:19302" },
      { urls: "stun:stun2.l.google.com:19302" },
      ...turnData
    ];
  } catch (error) {
    console.error("TURN server fetch error:", error);
  }
}

async function initializeLocalStream() {
  try {
    localMediaStream.value = await navigator.mediaDevices.getUserMedia({
      video: true,
      audio: true
    });
    if (localVideo.value) {
      localVideo.value.srcObject = localMediaStream.value;
    }
    if (isPoseDetectionEnabled.value) startPoseDetectionLoop();
  } catch (error) {
    console.error("Media device error:", error);
  }
}

// -------------
// Peer Connection
// -------------
function initializePeerConnection() {
  peerInstance.value = new Peer({ config: { iceServers: turnServers.value } });
  peerInstance.value.on("open", (id) => (localPeerId.value = id));

  // Handle incoming data connection: simply log received key landmarks
  peerInstance.value.on("connection", (conn) => {
    dataConnection.value = conn;
    conn.on("data", (data: any) => {
      console.log("Received key landmarks:", data);
    });
  });

  // Handle incoming call
  peerInstance.value.on("call", async (incomingCall) => {
    await initializeLocalStream();
    if (localMediaStream.value) {
      incomingCall.answer(localMediaStream.value);
    }
    incomingCall.on("stream", (remoteStreamData: MediaStream) => {
      if (remoteVideo.value) remoteVideo.value.srcObject = remoteStreamData;
    });
    activeCall.value = incomingCall;
  });

  initializeLocalStream();
}

function initiateCall() {
  if (!targetPeerId.value) {
    alert("Enter a Peer ID to call!");
    return;
  }
  if (peerInstance.value && localMediaStream.value) {
    // Open a data connection to the target peer
    dataConnection.value = peerInstance.value.connect(targetPeerId.value);
    
    activeCall.value = peerInstance.value.call(targetPeerId.value, localMediaStream.value);
    activeCall.value.on("stream", (remoteStreamData: MediaStream) => {
      if (remoteVideo.value) remoteVideo.value.srcObject = remoteStreamData;
    });
  } else {
    console.error("Peer or local stream missing.");
  }
}

// -------------
// Pose Detection Loop
// -------------
function togglePoseDetection() {
  isPoseDetectionEnabled.value = !isPoseDetectionEnabled.value;
  if (isPoseDetectionEnabled.value && localMediaStream.value) {
    startPoseDetectionLoop();
  } else {
    clearCanvases(
      localOverlayCanvas.value,
      worldCanvasXZ.value,
      worldCanvasYZ.value,
      worldCanvasXY.value
    );
  }
}

async function startPoseDetectionLoop() {
  if (!isPoseDetectionEnabled.value || isDetectionLoopRunning.value) return;
  isDetectionLoopRunning.value = true;
  if (
    !localVideo.value ||
    !localOverlayCanvas.value ||
    !worldCanvasXZ.value ||
    !worldCanvasYZ.value ||
    !worldCanvasXY.value
  )
    return;

  const video = localVideo.value;
  const overlay = localOverlayCanvas.value;
  const orthoCanvases = [
    worldCanvasXZ.value,
    worldCanvasYZ.value,
    worldCanvasXY.value
  ];

  async function loop() {
    if (!isPoseDetectionEnabled.value) {
      clearCanvases(overlay, ...orthoCanvases);
      isDetectionLoopRunning.value = false;
      return;
    }

    setCanvasSize(overlay, video);
    orthoCanvases.forEach((canvas) => setCanvasSize(canvas, video));

    if (video.readyState >= video.HAVE_ENOUGH_DATA) {
      try {
        const result = await poseDetector.value.detectPoseOnVideo(
          video,
          overlay,
          performance.now()
        );

        // Draw landmarks locally
        drawOrthogonalLandmarks(result.worldLandmarks, orthoCanvases[0], "xz");
        drawOrthogonalLandmarks(result.worldLandmarks, orthoCanvases[1], "yz");
        drawOrthogonalLandmarks(result.worldLandmarks, orthoCanvases[2], "xy");

        // Extract key landmarks from each pose
        const keyWorldLandmarks = result.worldLandmarks.map((landmarks: Landmark[]) =>
          landmarks.filter((_, idx) => keyLandmarkIndices.has(idx))
        );

        // Send key landmarks via data connection if available
        if (dataConnection.value && dataConnection.value.open) {
          dataConnection.value.send(keyWorldLandmarks);
        }
      } catch (error) {
        console.error("Pose detection error:", error);
      }
    }
    requestAnimationFrame(loop);
  }
  loop();
}

// -------------
// Lifecycle Hook
// -------------
onMounted(async () => {
  await fetchTurnServerCredentials();
  poseDetector.value = await initializePoseDetection("VIDEO", 1);
  initializePeerConnection();
});
</script>

<style scoped>
.video-chat-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 20px;
}

/* Flex container to hold remote video and grid */
.video-layout {
  display: flex;
  gap: 20px;
  align-items: flex-start;
}

/* Styles for the remote video */
.remote-video {
  text-align: center;
}

.remote-video video {
  width: 800px;
  height: 600px;
  background: black;
  border: 1px solid #ccc;
}

/* 2x2 Grid for the local video & world landmarks */
.grid-container {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  grid-gap: 20px;
  justify-items: center;
  align-items: center;
}

.local-video,
.world-landmark {
  text-align: center;
}

.video-wrapper {
  position: relative;
  width: 400px;
  height: 300px;
}

video {
  width: 400px;
  height: 300px;
  background: black;
  border: 1px solid #ccc;
  transform: scaleX(-1);
}

.overlay-canvas {
  position: absolute;
  top: 0;
  left: 0;
  width: 400px;
  height: 300px;
  pointer-events: none;
  transform: scaleX(-1);
}

.world-canvas {
  border: 1px solid #aaa;
  width: 400px;
  height: 300px;
  transform: scaleX(-1);
}

.controls {
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
  justify-content: center;
}

input {
  padding: 5px;
  width: 250px;
}

button {
  padding: 8px 12px;
  cursor: pointer;
}
</style>
