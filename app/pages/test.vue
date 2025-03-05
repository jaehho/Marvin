<template>
    <div>
      <h1>ROS Web Interface</h1>
      <p>Check the console for messages.</p>
      <button @click="publishMessage">Publish Message</button>
    </div>
  </template>
  
  <script setup>
  import { onMounted } from "vue";
  
  // Declare talker in a scope accessible to the publish function.
  let talker = null;
  
  function publishMessage() {
    if (!talker) {
      console.error("Talker is not ready yet.");
      return;
    }
    
    // Create a new message and publish it when the button is clicked.
    const message = new ROSLIB.Message({
      data: "Hello from the button!"
    });
    talker.publish(message);
    console.log("Published message: Hello from the button!");
  }
  
  onMounted(() => {
    // Load roslibjs dynamically to prevent SSR issues.
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
  
      // Establish a connection to the ROS bridge.
      const ros = new ROSLIB.Ros({
        url: "ws://localhost:9090" // Change this to your rosbridge server address.
      });
  
      // Connection event handlers.
      ros.on("connection", () => {
        console.log("Connected to rosbridge server.");
      });
  
      ros.on("error", (error) => {
        console.error("Error connecting to rosbridge server:", error);
      });
  
      ros.on("close", () => {
        console.log("Connection to rosbridge server closed.");
      });
  
      // Example: Subscribe to a topic.
      const listener = new ROSLIB.Topic({
        ros: ros,
        name: "/chatter", // Change to your topic name.
        messageType: "std_msgs/String",
      });
  
      listener.subscribe((message) => {
        console.log("Received message on " + listener.name + ": " + message.data);
      });
  
      // Example: Create a publisher (talker) to a topic.
      talker = new ROSLIB.Topic({
        ros: ros,
        name: "/chatter",
        messageType: "std_msgs/String",
      });
    }
  });
  </script>
  