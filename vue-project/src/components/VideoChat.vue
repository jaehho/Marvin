<!-- src/components/VideoChat.vue -->
<template>
  <div class="video-container">
    <h2>Your Peer ID: <span>{{ peerId }}</span></h2>
    
    <div class="main-content">
      <!-- Video Section -->
      <div class="video-section">
        <!-- Local Video with Overlay -->
        <div class="local-video">
          <h3>Local Video</h3>
          <div style="position: relative;">
            <video ref="localVideo" autoplay playsinline></video>
            <!-- This canvas draws the pose overlay on top of the video -->
            <canvas ref="localOverlayCanvas" class="overlay_canvas"></canvas>
          </div>
        </div>
        <!-- Remote Video -->
        <div class="remote-video">
          <h3>Remote Video</h3>
          <video ref="remoteVideo" autoplay playsinline></video>
        </div>
      </div>
      
      <!-- Landmarks Section: Normalized Landmarks -->
      <div class="landmarks-section">
        <h3>Normalized Pose Landmarks</h3>
        <canvas ref="poseCanvas" class="pose_canvas"></canvas>
      </div>
    </div>
    
    <div class="controls-panel">
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
      poseEnabled: true,         // Toggle flag for pose detection
      detectionLoopRunning: false
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
      this.peer = new Peer({
        config: { iceServers: this.iceServers }
      });
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
          audio: true
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
        this.localStream.getTracks().forEach(track => track.stop());
        this.localStream = null;
        this.$refs.localVideo.srcObject = null;
      }
      if (this.remoteStream) {
        this.remoteStream.getTracks().forEach(track => track.stop());
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
        // Clear both canvases when disabled.
        this.$refs.localOverlayCanvas.getContext("2d").clearRect(0, 0, this.$refs.localOverlayCanvas.width, this.$refs.localOverlayCanvas.height);
        this.$refs.poseCanvas.getContext("2d").clearRect(0, 0, this.$refs.poseCanvas.width, this.$refs.poseCanvas.height);
      }
    },
    async startPoseDetectionLoop() {
      if (!this.poseEnabled) return;
      this.detectionLoopRunning = true;
      const video = this.$refs.localVideo;
      const overlayCanvas = this.$refs.localOverlayCanvas;
      const normCanvas = this.$refs.poseCanvas;
      
      const loop = async () => {
        if (!this.poseEnabled) {
          // If detection is disabled, clear canvases and exit.
          overlayCanvas.getContext("2d").clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);
          normCanvas.getContext("2d").clearRect(0, 0, normCanvas.width, normCanvas.height);
          this.detectionLoopRunning = false;
          return;
        }
        
        // Update canvas dimensions to match the video.
        overlayCanvas.width = video.videoWidth;
        overlayCanvas.height = video.videoHeight;
        normCanvas.width = video.videoWidth;
        normCanvas.height = video.videoHeight;
        
        if (video.readyState >= video.HAVE_ENOUGH_DATA) {
          try {
            const now = performance.now();
            // Run pose detection; this will draw landmarks on overlayCanvas.
            const result = await this.poseDetection.detectPose(video, overlayCanvas, now);
            console.log("Detection result:", result);
            // Draw normalized landmarks (scaled to canvas dimensions) on the separate canvas.
            this.drawNormalizedLandmarks(result.landmarks, normCanvas);
          } catch (error) {
            console.error("Error during pose detection:", error);
          }
        }
        requestAnimationFrame(loop);
      };
      loop();
    },
    drawNormalizedLandmarks(landmarksArray, canvas) {
      const ctx = canvas.getContext("2d");
      const width = canvas.width;
      const height = canvas.height;
      ctx.clearRect(0, 0, width, height);
      
      if (!landmarksArray || landmarksArray.length === 0) return;
      
      // For each set of landmarks (for each detected person)
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
        // Optionally, draw connections (using blue lines).
        if (this.poseDetection && this.poseDetection.poseLandmarker) {
          const connections = this.poseDetection.poseLandmarker.constructor.POSE_CONNECTIONS;
          if (connections) {
            ctx.strokeStyle = "blue";
            ctx.lineWidth = 2;
            connections.forEach(([start, end]) => {
              if (landmarks[start] && landmarks[end]) {
                const startX = landmarks[start].x * width;
                const startY = landmarks[start].y * height;
                const endX = landmarks[end].x * width;
                const endY = landmarks[end].y * height;
                ctx.beginPath();
                ctx.moveTo(startX, startY);
                ctx.lineTo(endX, endY);
                ctx.stroke();
              }
            });
          }
        }
      });
    }
  }
};
</script>

<style scoped>
.video-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 10px;
}
.main-content {
  display: flex;
  gap: 20px;
  width: 100%;
  justify-content: center;
  flex-wrap: wrap;
}
.video-section {
  display: flex;
  flex-direction: column;
  gap: 10px;
}
.local-video,
.remote-video {
  text-align: center;
}
video {
  width: 400px;
  height: 300px;
  background: black;
  border: 1px solid #ccc;
}
.overlay_canvas {
  position: absolute;
  top: 0;
  left: 0;
  width: 400px;
  height: 300px;
  pointer-events: none;
}
.landmarks-section {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 10px;
  border: 1px solid #ccc;
  background: #fff;
}
.pose_canvas {
  border: 1px solid #aaa;
  width: 400px;
  height: 300px;
}
.controls-panel {
  margin-top: 10px;
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
