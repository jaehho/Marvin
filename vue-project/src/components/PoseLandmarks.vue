<!-- src/components/PoseLandmarks.vue -->
<template>
    <div class="pose-landmarks">
      <h3>Pose Landmarks</h3>
      <ul v-if="landmarks && landmarks.length">
        <li v-for="(landmark, index) in landmarks" :key="index">
          <strong>Landmark {{ index }}:</strong>
          <ul>
            <li v-for="(point, idx) in landmark" :key="idx">
              x: {{ point.x.toFixed(2) }}, y: {{ point.y.toFixed(2) }}, z: {{ point.z.toFixed(2) }}
            </li>
          </ul>
        </li>
      </ul>
      <div v-else>No landmarks detected.</div>
    </div>
  </template>
  
  <script>
  export default {
    name: "PoseLandmarks",
    props: {
      landmarks: {
        type: Array,
        default: () => []
      }
    }
  };
  </script>
  
  <style scoped>
  .pose-landmarks {
    max-height: 400px;
    overflow-y: auto;
    padding: 10px;
    background: #000000;
  }
  </style>
  