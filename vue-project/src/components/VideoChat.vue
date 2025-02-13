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

<script>
import Peer from "peerjs";
import { usePoseDetection } from "../composables/usePoseDetection.js";
import { drawNormalizedLandmarks } from "../utils/drawingHelpers.js";

export default {
  name: "VideoChat",
  data() {
    return {
      peer: null,
      peerId: "",
      remotePeerId: "",
      localStream: null,
      remoteStream: null,
      call: null,
      iceServers: [],
      poseDetection: null,
      poseEnabled: true,
      detectionLoopRunning: false,
    };
  },
  async mounted() {
    await this.fetchTurnServers();
    this.poseDetection = await usePoseDetection("VIDEO", 1);
    this.initializePeer();
  },
  methods: {
    async fetchTurnServers() {
      try {
        const response = await fetch(
          "https://marvin.metered.live/api/v1/turn/credentials?apiKey=bca769dfeca57f21e5ceb9eada5afbbf71cd"
        );
        const turnServers = await response.json();
        this.iceServers = [
          { urls: "stun:stun.l.google.com:19302" },
          { urls: "stun:stun1.l.google.com:19302" },
          { urls: "stun:stun2.l.google.com:19302" },
          ...turnServers
        ];
      } catch (error) {
        console.error("Failed to fetch TURN servers:", error);
      }
    },
    initializePeer() {
      this.peer = new Peer({ config: { iceServers: this.iceServers } });
      this.peer.on("open", (id) => {
        this.peerId = id;
      });
      this.peer.on("call", async (incomingCall) => {
        await this.getLocalStream();
        incomingCall.answer(this.localStream);
        incomingCall.on("stream", (remoteStream) => {
          this.$refs.remoteVideo.srcObject = remoteStream;
          this.remoteStream = remoteStream;
        });
        this.call = incomingCall;
      });
      this.getLocalStream();
    },
    async getLocalStream() {
      try {
        this.localStream = await navigator.mediaDevices.getUserMedia({
          video: true,
          audio: true,
        });
        this.$refs.localVideo.srcObject = this.localStream;
        if (this.poseEnabled) {
          this.startPoseDetectionLoop();
        }
      } catch (error) {
        console.error("Error accessing media devices:", error);
      }
    },
    callPeer() {
      if (!this.remotePeerId) {
        alert("Enter a Peer ID to call!");
        return;
      }
      this.call = this.peer.call(this.remotePeerId, this.localStream);
      this.call.on("stream", (remoteStream) => {
        this.$refs.remoteVideo.srcObject = remoteStream;
        this.remoteStream = remoteStream;
      });
    },
    endCall() {
      if (this.call) {
        this.call.close();
        this.call = null;
      }
      if (this.localStream) {
        this.localStream.getTracks().forEach((track) => track.stop());
        this.localStream = null;
        this.$refs.localVideo.srcObject = null;
      }
      if (this.remoteStream) {
        this.remoteStream.getTracks().forEach((track) => track.stop());
        this.remoteStream = null;
        this.$refs.remoteVideo.srcObject = null;
      }
      if (this.peer) {
        this.peer.destroy();
        this.peer = null;
      }
      this.detectionLoopRunning = false;
    },
    togglePoseDetection() {
      this.poseEnabled = !this.poseEnabled;
      if (this.poseEnabled && this.localStream) {
        this.startPoseDetectionLoop();
      } else {
        this.$refs.localOverlay.getContext("2d").clearRect(0, 0, this.$refs.localOverlay.width, this.$refs.localOverlay.height);
        this.$refs.normalizedCanvas.getContext("2d").clearRect(0, 0, this.$refs.normalizedCanvas.width, this.$refs.normalizedCanvas.height);
      }
    },
    async startPoseDetectionLoop() {
      if (!this.poseEnabled) return;
      this.detectionLoopRunning = true;
      const video = this.$refs.localVideo;
      const overlayCanvas = this.$refs.localOverlay;
      const normCanvas = this.$refs.normalizedCanvas;

      const loop = async () => {
        if (!this.poseEnabled) {
          overlayCanvas.getContext("2d").clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);
          normCanvas.getContext("2d").clearRect(0, 0, normCanvas.width, normCanvas.height);
          this.detectionLoopRunning = false;
          return;
        }

        // Update canvas sizes to match video dimensions.
        overlayCanvas.width = video.videoWidth;
        overlayCanvas.height = video.videoHeight;
        normCanvas.width = video.videoWidth;
        normCanvas.height = video.videoHeight;

        if (video.readyState >= video.HAVE_ENOUGH_DATA) {
          try {
            const now = performance.now();
            // Run pose detection (which draws the overlay on overlayCanvas).
            const result = await this.poseDetection.detectPose(video, overlayCanvas, now);
            // Use our drawing helper to draw normalized landmarks (with connections) on the separate canvas.
            drawNormalizedLandmarks(result.landmarks, normCanvas);
          } catch (error) {
            console.error("Error during pose detection:", error);
          }
        }
        requestAnimationFrame(loop);
      };
      loop();
    },
  },
};
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
