import { PoseLandmarker, FilesetResolver, DrawingUtils } from "@mediapipe/tasks-vision";

type RunningMode = "IMAGE" | "VIDEO";

export async function usePoseDetection(runningMode: RunningMode = "VIDEO", numPoses: number = 1): Promise<{
	poseLandmarker: any;
	detectPose: (videoElement: HTMLVideoElement, canvasElement: HTMLCanvasElement, timestamp?: number) => Promise<any>;
}> {
	// ...existing code...
	const vision = await FilesetResolver.forVisionTasks(
		"https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.0/wasm"
	);
	const poseLandmarker = await PoseLandmarker.createFromOptions(vision, {
		baseOptions: {
			modelAssetPath:
				"https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task",
			delegate: "GPU"
		},
		runningMode,
		numPoses
	});

	async function detectPose(
		videoElement: HTMLVideoElement,
		canvasElement: HTMLCanvasElement,
		timestamp?: number
	): Promise<any> {
		const canvasCtx = canvasElement.getContext("2d") as CanvasRenderingContext2D;
		const drawingUtils = new DrawingUtils(canvasCtx);

		// Match canvas size to video dimensions.
		canvasElement.width = videoElement.videoWidth;
		canvasElement.height = videoElement.videoHeight;
		canvasCtx.clearRect(0, 0, canvasElement.width, canvasElement.height);

		const ts = timestamp || performance.now();
		const result = await poseLandmarker.detectForVideo(videoElement, ts);

		// Draw landmarks and connectors on the overlay canvas.
		for (const landmark of result.landmarks) {
			drawingUtils.drawLandmarks(landmark, {
				radius: (data: { from?: { z?: number } }) =>
					DrawingUtils.lerp(data.from?.z ?? 0, -0.15, 0.1, 5, 1)
			});
			drawingUtils.drawConnectors(landmark, PoseLandmarker.POSE_CONNECTIONS);
		}
		return result;
	}

	// ...existing code...
	return { poseLandmarker, detectPose };
}
