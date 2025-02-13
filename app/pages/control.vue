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
      <button @click="endCall">End Call</button>
    </div>
  </div>
</template>

<script setup lang="ts">
import Peer from 'peerjs';

// Reactive state
const peer = ref<Peer | null>(null);
const peerId = ref('');
const remotePeerId = ref('');
const localStream = ref<MediaStream | null>(null);
const remoteStream = ref<MediaStream | null>(null);
const call = ref<any>(null);
const iceServers = ref<any[]>([]);
const poseDetection = ref<any>(null);
const poseEnabled = ref(true);
const detectionLoopRunning = ref(false);

// Template refs
const localVideo = ref<HTMLVideoElement | null>(null);
const remoteVideo = ref<HTMLVideoElement | null>(null);
const localOverlay = ref<HTMLCanvasElement | null>(null);
const normalizedCanvas = ref<HTMLCanvasElement | null>(null);

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
      ...turnServers,
    ];
  } catch (error) {
    console.error("Failed to fetch TURN servers:", error);
  }
};

const getLocalStream = async () => {
  try {
    localStream.value = await navigator.mediaDevices.getUserMedia({
      video: true,
      audio: true,
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

const endCall = () => {
  if (call.value) {
    call.value.close();
    call.value = null;
  }
  if (localStream.value) {
    localStream.value.getTracks().forEach((track) => track.stop());
    localStream.value = null;
    if (localVideo.value) localVideo.value.srcObject = null;
  }
  if (remoteStream.value) {
    remoteStream.value.getTracks().forEach((track) => track.stop());
    remoteStream.value = null;
    if (remoteVideo.value) remoteVideo.value.srcObject = null;
  }
  if (peer.value) {
    peer.value.destroy();
    peer.value = null;
  }
  detectionLoopRunning.value = false;
};

const togglePoseDetection = () => {
  poseEnabled.value = !poseEnabled.value;
  if (poseEnabled.value && localStream.value) {
    startPoseDetectionLoop();
  } else {
    if (localOverlay.value) {
      localOverlay.value
        .getContext("2d")
        ?.clearRect(0, 0, localOverlay.value.width, localOverlay.value.height);
    }
    if (normalizedCanvas.value) {
      normalizedCanvas.value
        .getContext("2d")
        ?.clearRect(0, 0, normalizedCanvas.value.width, normalizedCanvas.value.height);
    }
  }
};

const startPoseDetectionLoop = async () => {
  if (!poseEnabled.value || detectionLoopRunning.value) return;
  detectionLoopRunning.value = true;
  if (!localVideo.value || !localOverlay.value || !normalizedCanvas.value) return;
  const video = localVideo.value;
  const overlayCanvas = localOverlay.value;
  const normCanvas = normalizedCanvas.value;
  
  const loop = async () => {
    if (!poseEnabled.value) {
      overlayCanvas.getContext("2d")?.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);
      normCanvas.getContext("2d")?.clearRect(0, 0, normCanvas.width, normCanvas.height);
      detectionLoopRunning.value = false;
      return;
    }
    // Update canvas sizes to match the video.
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

// Lifecycle hook
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
