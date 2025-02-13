<template>
  <div class="video-container">
    <h2>Your Peer ID: <span>{{ peerId }}</span></h2>
    <input v-model="remotePeerId" placeholder="Enter Peer ID to Call" />
    <button @click="callPeer">Call</button>

    <!-- Local video and pose overlay -->
    <div style="position: relative;">
      <video ref="localVideo" autoplay playsinline></video>
      <canvas ref="localCanvas" class="output_canvas"></canvas>
    </div>

    <!-- Remote video and pose overlay -->
    <div style="position: relative;">
      <video ref="remoteVideo" autoplay playsinline></video>
      <canvas ref="remoteCanvas" class="output_canvas"></canvas>
    </div>

    <div class="controls">
      <button @click="endCall">End Call</button>
    </div>
  </div>
</template>

<script>
import Peer from "peerjs";
import { usePoseDetection } from "../composables/usePoseDetection.js";

export default {
  data() {
    return {
      peer: null,
      peerId: "",
      remotePeerId: "",
      localStream: null,
      remoteStream: null,
      call: null,
      iceServers: [],
      poseDetection: null
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
        console.log("ICE Servers:", this.iceServers);
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
        console.log("My Peer ID:", id);
      });
      this.peer.on("call", async (incomingCall) => {
        await this.getLocalStream();
        incomingCall.answer(this.localStream);
        incomingCall.on("stream", (remoteStream) => {
          this.$refs.remoteVideo.srcObject = remoteStream;
          this.remoteStream = remoteStream;
          this.startPoseDetection(remoteStream, this.$refs.remoteCanvas);
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
        this.startPoseDetection(this.localStream, this.$refs.localCanvas);
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
        this.startPoseDetection(remoteStream, this.$refs.remoteCanvas);
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
    },
    startPoseDetection(stream, canvasElement) {
      // Create an off-screen video element to run detection
      const video = document.createElement("video");
      video.srcObject = stream;
      video.play();
      const detect = async () => {
        if (video.readyState >= video.HAVE_ENOUGH_DATA) {
          await this.poseDetection.detectPose(video, canvasElement);
        }
        // Continue detection as long as the stream is active
        if (stream) requestAnimationFrame(detect);
      };
      detect();
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
video {
  width: 400px;
  height: 300px;
  background: black;
  border: 1px solid #ccc;
}
canvas {
  position: absolute;
  top: 0;
  left: 0;
  width: 400px;
  height: 300px;
  pointer-events: none;
}
input {
  margin: 10px;
  padding: 5px;
  width: 250px;
}
button {
  margin: 5px;
  padding: 8px 12px;
  cursor: pointer;
}
</style>
