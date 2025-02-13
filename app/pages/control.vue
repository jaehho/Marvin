<template>
  <div>
    <VideoChat />
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import VideoChat from "~/components/VideoChat.vue";

export default defineComponent({
  components: {
    VideoChat,
  },
});
</script>
