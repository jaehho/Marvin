<template>
  <div class="video-container">
    <h2>Your Peer ID: <span>{{ peerId }}</span></h2>
    <input v-model="remotePeerId" placeholder="Enter Peer ID to Call" />
    <button @click="callPeer">Call</button>

    <video ref="localVideo" autoplay playsinline></video>
    <video ref="remoteVideo" autoplay playsinline></video>

    <div class="controls">
      <button @click="endCall">End Call</button>
    </div>
  </div>
</template>

<script>
import Peer from "peerjs";

export default {
  data() {
    return {
      peer: null,
      peerId: "", // Store the local peer ID
      remotePeerId: "", // Input for calling another peer
      localStream: null,
      remoteStream: null,
      call: null,
    };
  },
  async mounted() {
    this.peer = new Peer(); // Automatically generate a unique ID

    this.peer.on("open", (id) => {
      this.peerId = id; // Store the peer ID
      console.log("My peer ID:", id);
    });

    this.peer.on("call", (incomingCall) => {
      incomingCall.answer(this.localStream); // Answer the call
      incomingCall.on("stream", (remoteStream) => {
        this.$refs.remoteVideo.srcObject = remoteStream;
      });

      this.call = incomingCall;
    });

    this.localStream = await navigator.mediaDevices.getUserMedia({
      video: true,
      audio: true,
    });

    this.$refs.localVideo.srcObject = this.localStream;
  },
  methods: {
    callPeer() {
      if (!this.remotePeerId) {
        alert("Enter a Peer ID to call!");
        return;
      }

      this.call = this.peer.call(this.remotePeerId, this.localStream);

      this.call.on("stream", (remoteStream) => {
        this.$refs.remoteVideo.srcObject = remoteStream;
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
