<template>
    <div>
      <h1>ROS Web Interface</h1>
      <p>Check the console for messages.</p>
    </div>
  </template>
  
  <script setup>
  import { onMounted } from "vue";
  
  // Ensure the script runs when the component is mounted
  onMounted(() => {
    // Load roslibjs dynamically to prevent SSR issues
    const script = document.createElement("script");
    script.src = "https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js";
    script.onload = () => {
      initializeROS();
    };
    document.head.appendChild(script);
  
    function initializeROS() {
      if (typeof ROSLIB === "undefined") {
        console.error("ROSLIB not loaded");
        return;
      }
  
      // Establish a connection to the ROS bridge
      const ros = new ROSLIB.Ros({
        url: "ws://localhost:9090", // Change this to your rosbridge server address
      });
  
      // Connection event handlers
      ros.on("connection", () => {
        console.log("Connected to rosbridge server.");
      });
  
      ros.on("error", (error) => {
        console.error("Error connecting to rosbridge server:", error);
      });
  
      ros.on("close", () => {
        console.log("Connection to rosbridge server closed.");
      });
  
      // Example: Subscribe to a topic
      const listener = new ROSLIB.Topic({
        ros: ros,
        name: "/chatter", // Change to your topic name
        messageType: "std_msgs/String",
      });
  
      listener.subscribe((message) => {
        console.log("Received message on " + listener.name + ": " + message.data);
      });
  
      // Example: Publish to a topic
      const talker = new ROSLIB.Topic({
        ros: ros,
        name: "/chatter",
        messageType: "std_msgs/String",
      });
  
      const message = new ROSLIB.Message({
        data: "Hello from roslibjs!",
      });
  
      talker.publish(message);
    }
  });
  </script>
  