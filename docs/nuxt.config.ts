// https://nuxt.com/docs/api/configuration/nuxt-config
export default defineNuxtConfig({
  ssr: false, // Disable server-side rendering if not needed
  compatibilityDate: "2024-11-01",
  devtools: { enabled: true },
  target: 'static', // Enables static site generation
  app: {
    baseURL: '/marvin/', // Replace <REPO_NAME> with your repository name
  },
});
