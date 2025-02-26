<template>
  <div class="video-chat-container">
    <h2>Your Peer ID: {{ peerId }}</h2>
    <div class="video-area">
      <!-- Local Video with Overlay -->
      <div class="local-video">
        <h3>Local Video</h3>
        <div class="video-wrapper">
          <video ref="localVideo" autoplay playsinline></video>
          <canvas ref="localOverlay" class="overlay-canvas"></canvas>
        </div>
      </div>
      <!-- Remote Video -->
      <div class="remote-video">
        <h3>Remote Video</h3>
        <video ref="remoteVideo" autoplay playsinline></video>
      </div>
      <!-- Pose Landmarks Canvas -->
      <div class="pose-landmarks">
        <h3>Normalized Pose Landmarks</h3>
        <canvas ref="normalizedCanvas" class="normalized-canvas"></canvas>
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
}

// -------------
// Reactive State & Refs
// -------------
const peer = ref<Peer | null>(null);
const peerId = ref("");
const remotePeerId = ref("");
const localStream = ref<MediaStream | null>(null);
const remoteStream = ref<MediaStream | null>(null);
const call = ref<any>(null);
const iceServers = ref<any[]>([]);
const poseDetection = ref<any>(null);
const poseEnabled = ref(true);
const detectionLoopRunning = ref(false);

// Template element refs
const localVideo = ref<HTMLVideoElement | null>(null);
const remoteVideo = ref<HTMLVideoElement | null>(null);
const localOverlay = ref<HTMLCanvasElement | null>(null);
const normalizedCanvas = ref<HTMLCanvasElement | null>(null);

// -------------
// Utility Functions
// -------------
const clearCanvas = (canvas: HTMLCanvasElement | null) => {
  canvas?.getContext("2d")?.clearRect(0, 0, canvas.width, canvas.height);
};

function orthoLandmarks(worldLandmarks: Landmark[][], canvas: HTMLCanvasElement) {
  console.log("drawLandmarks called", { landmarksData: worldLandmarks, canvas });

  const ctx = canvas.getContext("2d")!;
  const width = canvas.width, height = canvas.height;
  console.log("Canvas dimensions", { width, height });

  ctx.clearRect(0, 0, width, height);
  console.log("landmarks", worldLandmarks);

  if (!worldLandmarks.length) {
    console.log("No landmarks received, exiting function.");
    return;
  }

  // Translate origin to center
  const centerX = width / 2;
  const centerY = height / 2;

  worldLandmarks.forEach((landmarks, groupIndex) => {
    // Draw landmarks
    ctx.fillStyle = "red";
    landmarks.forEach(({ x, y }, index) => {
      // Convert from normalized coordinates to centered canvas coordinates
      const canvasX = x * width + centerX;
      const canvasY = y * height + centerY;

      console.log(`Drawing landmark ${index} at (${canvasX.toFixed(2)}, ${canvasY.toFixed(2)}) on canvas`);

      ctx.beginPath();
      ctx.arc(canvasX, canvasY, 5, 0, 2 * Math.PI);
      ctx.fill();
    });

    // Draw connections using MediaPipe's POSE_CONNECTIONS
    ctx.strokeStyle = "blue";
    ctx.lineWidth = 2;
    PoseLandmarker.POSE_CONNECTIONS.forEach(({ start, end }) => {
      if (landmarks[start] && landmarks[end]) {
        const startX = landmarks[start].x * width + centerX;
        const startY = landmarks[start].y * height + centerY;
        const endX = landmarks[end].x * width + centerX;
        const endY = landmarks[end].y * height + centerY;

        console.log(`Drawing connection from (${startX.toFixed(2)}, ${startY.toFixed(2)}) to (${endX.toFixed(2)}, ${endY.toFixed(2)})`);

        ctx.beginPath();
        ctx.moveTo(startX, startY);
        ctx.lineTo(endX, endY);
        ctx.stroke();
      }
    });
  });
}

// -------------
// Pose Detection Setup
// -------------
async function setupPoseDetection(runningMode: "IMAGE" | "VIDEO" = "VIDEO", numPoses: number = 1) {
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
      remoteStream.value = remoteStreamData;
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
      remoteStream.value = remoteStreamData;
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
    clearCanvas(normalizedCanvas.value);
  }
}

async function runPoseDetectionLoop() {
  if (!poseEnabled.value || detectionLoopRunning.value) return;
  detectionLoopRunning.value = true;
  if (!localVideo.value || !localOverlay.value || !normalizedCanvas.value) return;

  const video = localVideo.value;
  const overlay = localOverlay.value;
  const normCanvas = normalizedCanvas.value;

  async function loop() {
    if (!poseEnabled.value) {
      clearCanvas(overlay);
      clearCanvas(normCanvas);
      detectionLoopRunning.value = false;
      return;
    }

    overlay.width = video.videoWidth;
    overlay.height = video.videoHeight;
    normCanvas.width = video.videoWidth;
    normCanvas.height = video.videoHeight;

    if (video.readyState >= video.HAVE_ENOUGH_DATA) {
      try {
        const result = await poseDetection.value.detectPose(video, overlay, performance.now());
        console.log("Pose detection result:", result);
        orthoLandmarks(result.worldLandmarks, normCanvas);
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

.video-area {
  display: flex;
  gap: 20px;
  flex-wrap: wrap;
  justify-content: center;
}

.local-video,
.remote-video,
.pose-landmarks {
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
}

.overlay-canvas {
  position: absolute;
  top: 0;
  left: 0;
  width: 400px;
  height: 300px;
  pointer-events: none;
}

.normalized-canvas {
  border: 1px solid #aaa;
  width: 400px;
  height: 300px;
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
