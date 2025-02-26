<template>
  <div class="video-chat-container">
    <h2>Your Peer ID: {{ peerId }}</h2>
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
            <canvas ref="localOverlay" class="overlay-canvas"></canvas>
          </div>
        </div>
        <!-- Top Right: World Landmark 1 (XZ) -->
        <div class="world-landmark">
          <h3>World Landmark 1 (XZ)</h3>
          <canvas ref="worldCoordinates1" class="world-canvas"></canvas>
        </div>
        <!-- Bottom Left: World Landmark 2 (YZ) -->
        <div class="world-landmark">
          <h3>World Landmark 2 (YZ)</h3>
          <canvas ref="worldCoordinates2" class="world-canvas"></canvas>
        </div>
        <!-- Bottom Right: World Landmark 3 (XY) -->
        <div class="world-landmark">
          <h3>World Landmark 3 (XY)</h3>
          <canvas ref="worldCoordinates3" class="world-canvas"></canvas>
        </div>
      </div>
    </div>
    <div class="controls">
      <input v-model="remotePeerId" placeholder="Enter Peer ID to Call" />
      <button @click="callPeer">Call</button>
      <button @click="togglePoseDetection">
        {{ poseEnabled ? "Disable" : "Enable" }} Pose Detection
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
const peer = ref<Peer | null>(null);
const peerId = ref("");
const remotePeerId = ref("");
const localStream = ref<MediaStream | null>(null);
const call = ref<any>(null);
const iceServers = ref<any[]>([]);
const poseDetection = ref<any>(null);
const poseEnabled = ref(true);
const detectionLoopRunning = ref(false);

// Template element refs
const localVideo = ref<HTMLVideoElement | null>(null);
const remoteVideo = ref<HTMLVideoElement | null>(null);
const localOverlay = ref<HTMLCanvasElement | null>(null);
const worldCoordinates1 = ref<HTMLCanvasElement | null>(null);
const worldCoordinates2 = ref<HTMLCanvasElement | null>(null);
const worldCoordinates3 = ref<HTMLCanvasElement | null>(null);

// -------------
// Utility Functions
// -------------
const clearCanvas = (canvas: HTMLCanvasElement | null) => {
  canvas?.getContext("2d")?.clearRect(0, 0, canvas.width, canvas.height);
};

/**
 * Draws world landmarks on a canvas using the specified coordinate plane.
 * @param worldLandmarks Array of landmark arrays.
 * @param canvas The target canvas element.
 * @param plane The coordinate plane to use: "xy", "xz", or "yz".
 */
function orthoLandmarks(
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

  if (!worldLandmarks.length) {
    return;
  }

  // Translate origin to canvas center
  const centerX = width / 2;
  const centerY = height / 2;

  // Only draw selected landmarks
  const validLandmarks = new Set([11, 12, 13, 14, 15, 16, 23, 24]);

  worldLandmarks.forEach((landmarks) => {
    ctx.fillStyle = "red";
    validLandmarks.forEach((index) => {
      if (index < landmarks.length) {
        const { x, y, z } = landmarks[index];
        let posX = 0;
        let posY = 0;
        switch (plane) {
          case "xy":
            posX = x;
            posY = y;
            break;
          case "xz":
            posX = x;
            posY = z;
            break;
          case "yz":
            posX = z;
            posY = y;
            break;
        }
        const canvasX = posX * width + centerX;
        const canvasY = posY * height + centerY;
        ctx.beginPath();
        ctx.arc(canvasX, canvasY, 5, 0, 2 * Math.PI);
        ctx.fill();
      }
    });

    // Draw landmark connections
    ctx.strokeStyle = "blue";
    ctx.lineWidth = 2;
    PoseLandmarker.POSE_CONNECTIONS.forEach(({ start, end }) => {
      if (
        validLandmarks.has(start) &&
        validLandmarks.has(end) &&
        landmarks[start] &&
        landmarks[end]
      ) {
        let startX = 0,
          startY = 0,
          endX = 0,
          endY = 0;
        // Get start point based on the selected plane
        switch (plane) {
          case "xy":
            startX = landmarks[start].x;
            startY = landmarks[start].y;
            break;
          case "xz":
            startX = landmarks[start].x;
            startY = landmarks[start].z;
            break;
          case "yz":
            startX = landmarks[start].z;
            startY = landmarks[start].y;
            break;
        }
        // Get end point based on the selected plane
        switch (plane) {
          case "xy":
            endX = landmarks[end].x;
            endY = landmarks[end].y;
            break;
          case "xz":
            endX = landmarks[end].x;
            endY = landmarks[end].z;
            break;
          case "yz":
            endX = landmarks[end].z;
            endY = landmarks[end].y;
            break;
        }
        ctx.beginPath();
        ctx.moveTo(startX * width + centerX, startY * height + centerY);
        ctx.lineTo(endX * width + centerX, endY * height + centerY);
        ctx.stroke();
      }
    });
  });
}

// -------------
// Pose Detection Setup
// -------------
async function setupPoseDetection(
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

  async function detectPose(
    video: HTMLVideoElement,
    canvas: HTMLCanvasElement,
    timestamp?: number
  ) {
    const ctx = canvas.getContext("2d")!;
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
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

  return { poseLandmarker, detectPose };
}

// -------------
// TURN Server & Media Setup
// -------------
async function loadTurnServers() {
  try {
    const res = await fetch(
      "https://marvin.metered.live/api/v1/turn/credentials?apiKey=bca769dfeca57f21e5ceb9eada5afbbf71cd"
    );
    const turnData = await res.json();
    iceServers.value = [
      { urls: "stun:stun.l.google.com:19302" },
      { urls: "stun:stun1.l.google.com:19302" },
      { urls: "stun:stun2.l.google.com:19302" },
      ...turnData
    ];
  } catch (error) {
    console.error("TURN server fetch error:", error);
  }
}

async function setupLocalStream() {
  try {
    localStream.value = await navigator.mediaDevices.getUserMedia({
      video: true,
      audio: true
    });
    if (localVideo.value) {
      localVideo.value.srcObject = localStream.value;
    }
    if (poseEnabled.value) runPoseDetectionLoop();
  } catch (error) {
    console.error("Media device error:", error);
  }
}

// -------------
// Peer Connection
// -------------
function initPeer() {
  peer.value = new Peer({ config: { iceServers: iceServers.value } });
  peer.value.on("open", (id) => (peerId.value = id));

  // Handle incoming call
  peer.value.on("call", async (incomingCall) => {
    await setupLocalStream();
    if (localStream.value) {
      incomingCall.answer(localStream.value);
    }
    incomingCall.on("stream", (remoteStreamData: MediaStream) => {
      if (remoteVideo.value) remoteVideo.value.srcObject = remoteStreamData;
    });
    call.value = incomingCall;
  });

  setupLocalStream();
}

function callPeer() {
  if (!remotePeerId.value) {
    alert("Enter a Peer ID to call!");
    return;
  }
  if (peer.value && localStream.value) {
    call.value = peer.value.call(remotePeerId.value, localStream.value);
    call.value.on("stream", (remoteStreamData: MediaStream) => {
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
  poseEnabled.value = !poseEnabled.value;
  if (poseEnabled.value && localStream.value) {
    runPoseDetectionLoop();
  } else {
    clearCanvas(localOverlay.value);
    clearCanvas(worldCoordinates1.value);
    clearCanvas(worldCoordinates2.value);
    clearCanvas(worldCoordinates3.value);
  }
}

async function runPoseDetectionLoop() {
  if (!poseEnabled.value || detectionLoopRunning.value) return;
  detectionLoopRunning.value = true;
  if (
    !localVideo.value ||
    !localOverlay.value ||
    !worldCoordinates1.value ||
    !worldCoordinates2.value ||
    !worldCoordinates3.value
  )
    return;

  const video = localVideo.value;
  const overlay = localOverlay.value;
  // Array of world landmark canvases
  const orthoCanvases = [
    worldCoordinates1.value,
    worldCoordinates2.value,
    worldCoordinates3.value
  ];

  async function loop() {
    if (!poseEnabled.value) {
      clearCanvas(overlay);
      orthoCanvases.forEach(clearCanvas);
      detectionLoopRunning.value = false;
      return;
    }

    overlay.width = video.videoWidth;
    overlay.height = video.videoHeight;
    orthoCanvases.forEach((canvas) => {
      if (canvas) {
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;
      }
    });

    if (video.readyState >= video.HAVE_ENOUGH_DATA) {
      try {
        const result = await poseDetection.value.detectPose(
          video,
          overlay,
          performance.now()
        );
        if (orthoCanvases[0])
          orthoLandmarks(result.worldLandmarks, orthoCanvases[0], "xz");
        if (orthoCanvases[1])
          orthoLandmarks(result.worldLandmarks, orthoCanvases[1], "yz");
        if (orthoCanvases[2])
          orthoLandmarks(result.worldLandmarks, orthoCanvases[2], "xy");
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
  await loadTurnServers();
  poseDetection.value = await setupPoseDetection("VIDEO", 1);
  initPeer();
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
  width: 800px;  /* Same as the overall grid width */
  height: 600px; /* Same as the overall grid height */
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
  transform: scaleX(-1); /* Mirror */
}

.overlay-canvas {
  position: absolute;
  top: 0;
  left: 0;
  width: 400px;
  height: 300px;
  pointer-events: none;
  transform: scaleX(-1); /* Mirror */
}

.world-canvas {
  border: 1px solid #aaa;
  width: 400px;
  height: 300px;
  transform: scaleX(-1); /* Mirror */
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
