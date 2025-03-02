<template>
    <div class="container">
      <h1>ROS2 Topic Publisher Test</h1>
      <button @click="publishMessage">Publish Message</button>
      <p>Status: {{ status }}</p>
    </div>
  </template>
  
  <script>
  import ROSLIB from 'roslib'
  
  export default {
    name: 'RosPublisher',
    data() {
      return {
        ros: null,
        status: 'Connecting...'
      }
    },
    mounted() {
      // Establish a connection to the rosbridge server
      this.ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
      });
  
      this.ros.on('connection', () => {
        console.log('Connected to ROS bridge.');
        this.status = 'Connected to ROS.';
      });
  
      this.ros.on('error', (error) => {
        console.error('Error connecting to ROS bridge:', error);
        this.status = 'Error connecting to ROS.';
      });
  
      this.ros.on('close', () => {
        console.log('Connection to ROS bridge closed.');
        this.status = 'Connection closed.';
      });
    },
    methods: {
      publishMessage() {
        // Create a topic instance for publishing
        const topic = new ROSLIB.Topic({
          ros: this.ros,
          name: '/test_topic',
          messageType: 'std_msgs/String'
        });
  
        // Define the message to be published
        const message = new ROSLIB.Message({
          data: 'Hello, ROS2 from Nuxt!'
        });
  
        // Publish the message
        topic.publish(message);
        console.log('Published message:', message);
      }
    }
  }
  </script>
  
  <style scoped>
  .container {
    max-width: 600px;
    margin: 2rem auto;
    padding: 1rem;
    text-align: center;
  }
  h1 {
    font-size: 2rem;
    margin-bottom: 1rem;
  }
  button {
    padding: 0.75rem 1.5rem;
    font-size: 1rem;
    cursor: pointer;
  }
  p {
    margin-top: 1rem;
    font-style: italic;
  }
  </style>
  