<script>
import 'animate.css'
</script>

<style>
@import url("animate.css");
</style>

<template>
  <header>
    <nav>
      <ul>
        <li><NuxtLink to="/">Home</NuxtLink></li>
        <li><NuxtLink to="/about">About</NuxtLink></li>
      </ul>
    </nav>
  </header>
  <div>
    <h1>This is app.vue</h1>
    <AppAlert>
      This is an auto-imported component from app.vue
    </AppAlert>
    <NuxtPage />
    <img src="/marvin.jpg" alt="marvin" style="width: 30%; height: auto;" />
    <h1 class="animate__animated animate__bounce">An animated element</h1>
  </div>
</template>
