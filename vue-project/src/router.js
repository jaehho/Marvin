import { createRouter, createWebHistory } from "vue-router";
import VideoChat from "./components/VideoChat.vue";

const routes = [
  { path: "/", component: VideoChat },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
