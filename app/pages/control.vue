<template>
  <div class="video-chat-container">
    <h2>Your Peer ID: {{ peerId }}</h2>
    <div class="video-area">
      <!-- Local Video with Overlay -->
      <div class="local-video">
        <h3>Local Video</h3>
        <div class="video-wrapper">
          <video ref="localVideo" autoplay playsinline></video>
          <!-- Overlay canvas drawn by MediaPipe's drawing utilities -->
          <canvas ref="localOverlay" class="overlay-canvas"></canvas>
        </div>
      </div>
      <!-- Remote Video -->
      <div class="remote-video">
        <h3>Remote Video</h3>
        <video ref="remoteVideo" autoplay playsinline></video>
      </div>
      <!-- Normalized Pose Landmarks Canvas -->
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
import { ref, onMounted, onBeforeUnmount } from "vue";
import Peer from "peerjs";
import { PoseLandmarker, FilesetResolver, DrawingUtils } from "@mediapipe/tasks-vision";

interface Landmark {
  x: number;
  y: number;
}

/**
 * Draws normalized landmarks and their connections onto a given canvas.
 */
function drawOrthographicProjections(
  landmarksArray: Landmark[][] | Landmark[],
  canvas: HTMLCanvasElement
): void {
  const ctx = canvas.getContext("2d")!;
  const canvasWidth = canvas.width;
  const canvasHeight = canvas.height;
  ctx.clearRect(0, 0, canvasWidth, canvasHeight);
  if (!landmarksArray || landmarksArray.length === 0) return;

  // Ensure we work with an array of landmark arrays.
  const landmarksGroups: Landmark[][] = Array.isArray(landmarksArray[0])
    ? (landmarksArray as Landmark[][])
    : [landmarksArray as Landmark[]];

  landmarksGroups.forEach((landmarks) => {
    let minX = Infinity,
      maxX = -Infinity,
      minY = Infinity,
      maxY = -Infinity;
    landmarks.forEach((point) => {
      minX = Math.min(minX, point.x);
      maxX = Math.max(maxX, point.x);
      minY = Math.min(minY, point.y);
      maxY = Math.max(maxY, point.y);
    });

    // Add a 5% margin to the bounding box.
    const marginX = 0.05 * (maxX - minX);
    const marginY = 0.05 * (maxY - minY);
    minX = Math.max(0, minX - marginX);
    maxX = Math.min(1, maxX + marginX);
    minY = Math.max(0, minY - marginY);
    maxY = Math.min(1, maxY + marginY);

    // Draw each landmark.
    ctx.fillStyle = "red";
    landmarks.forEach((point) => {
      const normX = ((point.x - minX) / (maxX - minX)) * canvasWidth;
      const normY = ((point.y - minY) / (maxY - minY)) * canvasHeight;
      ctx.beginPath();
      ctx.arc(normX, normY, 5, 0, 2 * Math.PI);
      ctx.fill();
    });

    // Draw connectors between landmarks.
    const connections = PoseLandmarker.POSE_CONNECTIONS;
    if (connections) {
      ctx.strokeStyle = "blue";
      ctx.lineWidth = 2;
      connections.forEach(({ start, end }) => {
        if (landmarks[start] && landmarks[end]) {
          const startX = ((landmarks[start].x - minX) / (maxX - minX)) * canvasWidth;
          const startY = ((landmarks[start].y - minY) / (maxY - minY)) * canvasHeight;
          const endX = ((landmarks[end].x - minX) / (maxX - minX)) * canvasWidth;
          const endY = ((landmarks[end].y - minY) / (maxY - minY)) * canvasHeight;
          ctx.beginPath();
          ctx.moveTo(startX, startY);
          ctx.lineTo(endX, endY);
          ctx.stroke();
        }
      });
    }
  });
}

type RunningMode = "IMAGE" | "VIDEO";

/**
 * Sets up the MediaPipe PoseLandmarker.
 */
async function usePoseDetection(
  runningMode: RunningMode = "VIDEO",
  numPoses: number = 1
): Promise<{
  poseLandmarker: any;
  detectPose: (
    videoElement: HTMLVideoElement,
    canvasElement: HTMLCanvasElement,
    timestamp?: number
  ) => Promise<any>;
}> {
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
    videoElement: HTMLVideoElement,
    canvasElement: HTMLCanvasElement,
    timestamp?: number
  ): Promise<any> {
    const canvasCtx = canvasElement.getContext("2d") as CanvasRenderingContext2D;
    const drawingUtils = new DrawingUtils(canvasCtx);

    // Match canvas size to video dimensions.
    canvasElement.width = videoElement.videoWidth;
    canvasElement.height = videoElement.videoHeight;
    canvasCtx.clearRect(0, 0, canvasElement.width, canvasElement.height);

    const ts = timestamp || performance.now();
    const result = await poseLandmarker.detectForVideo(videoElement, ts);

    result.landmarks.forEach((landmark: any) => {
      drawingUtils.drawLandmarks(landmark, {
        radius: (data: { from?: { z?: number } }) =>
          DrawingUtils.lerp(data.from?.z ?? 0, -0.15, 0.1, 5, 1)
      });
      drawingUtils.drawConnectors(landmark, PoseLandmarker.POSE_CONNECTIONS);
    });
    return result;
  }

  return { poseLandmarker, detectPose };
}

// ----------------------
// Reactive & Template Refs
// ----------------------
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

// ----------------------
// Helper Functions
// ----------------------
/** Clears the provided canvas. */
const clearCanvas = (canvas: HTMLCanvasElement | null) => {
  canvas?.getContext("2d")?.clearRect(0, 0, canvas.width, canvas.height);
};

/** Fetch TURN server credentials and configure ICE servers. */
const fetchTurnServers = async () => {
  try {
    const response = await fetch(
      "https://marvin.metered.live/api/v1/turn/credentials?apiKey=bca769dfeca57f21e5ceb9eada5afbbf71cd"
    );
    const turnServers = await response.json();
    iceServers.value = [
      { urls: "stun:stun.l.google.com:19302" },
      { urls: "stun:stun1.l.google.com:19302" },
      { urls: "stun:stun2.l.google.com:19302" },
      ...turnServers
    ];
  } catch (error) {
    console.error("Failed to fetch TURN servers:", error);
  }
};

/** Gets the local media stream and sets it to the local video element. */
const getLocalStream = async () => {
  try {
    localStream.value = await navigator.mediaDevices.getUserMedia({
      video: true,
      audio: true
    });
    if (localVideo.value) {
      localVideo.value.srcObject = localStream.value;
    }
    if (poseEnabled.value) {
      startPoseDetectionLoop();
    }
  } catch (error) {
    console.error("Error accessing media devices:", error);
  }
};

/** Initializes the Peer connection and its event listeners. */
const initializePeer = () => {
  peer.value = new Peer({ config: { iceServers: iceServers.value } });
  peer.value.on("open", (id) => {
    peerId.value = id;
  });
  peer.value.on("call", async (incomingCall) => {
    await getLocalStream();
    if (localStream.value) {
      incomingCall.answer(localStream.value);
    } else {
      console.error("Local stream is not available.");
      return;
    }
    incomingCall.on("stream", (remoteStreamVal: MediaStream) => {
      if (remoteVideo.value) {
        remoteVideo.value.srcObject = remoteStreamVal;
      }
      remoteStream.value = remoteStreamVal;
    });
    call.value = incomingCall;
  });
  getLocalStream();
};

/** Initiates a call to the specified remote peer. */
const callPeer = () => {
  if (!remotePeerId.value) {
    alert("Enter a Peer ID to call!");
    return;
  }
  if (peer.value && localStream.value) {
    call.value = peer.value.call(remotePeerId.value, localStream.value);
    call.value.on("stream", (remoteStreamVal: MediaStream) => {
      if (remoteVideo.value) {
        remoteVideo.value.srcObject = remoteStreamVal;
      }
      remoteStream.value = remoteStreamVal;
    });
  } else {
    console.error("Peer or Local stream not available.");
  }
};

/** Toggles pose detection on/off. */
const togglePoseDetection = () => {
  poseEnabled.value = !poseEnabled.value;
  if (poseEnabled.value && localStream.value) {
    startPoseDetectionLoop();
  } else {
    clearCanvas(localOverlay.value);
    clearCanvas(normalizedCanvas.value);
  }
};

/**
 * Runs the pose detection loop using requestAnimationFrame.
 * It updates the overlay and normalized canvases with the latest detections.
 */
const startPoseDetectionLoop = async () => {
  if (!poseEnabled.value || detectionLoopRunning.value) return;
  detectionLoopRunning.value = true;
  if (!localVideo.value || !localOverlay.value || !normalizedCanvas.value) return;

  const video = localVideo.value;
  const overlayCanvas = localOverlay.value;
  const normCanvas = normalizedCanvas.value;

  const loop = async () => {
    if (!poseEnabled.value) {
      clearCanvas(overlayCanvas);
      clearCanvas(normCanvas);
      detectionLoopRunning.value = false;
      return;
    }

    // Match canvases to video dimensions.
    overlayCanvas.width = video.videoWidth;
    overlayCanvas.height = video.videoHeight;
    normCanvas.width = video.videoWidth;
    normCanvas.height = video.videoHeight;

    if (video.readyState >= video.HAVE_ENOUGH_DATA) {
      try {
        const now = performance.now();
        const result = await poseDetection.value.detectPose(video, overlayCanvas, now);
        drawOrthographicProjections(result.landmarks, normCanvas);
      } catch (error) {
        console.error("Error during pose detection:", error);
      }
    }
    requestAnimationFrame(loop);
  };
  loop();
};

// ----------------------
// Lifecycle Hooks
// ----------------------
onMounted(async () => {
  await fetchTurnServers();
  poseDetection.value = await usePoseDetection("VIDEO", 1);
  initializePeer();
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
