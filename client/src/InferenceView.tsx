import type { InferencePayload } from "./types"

interface InferenceViewParams {
  payload: InferencePayload | null
}

export default function InferenceView({ payload }: InferenceViewParams) {
  return <> <h1>Inference</h1>

    {payload === null ? <p>Waiting for connection...</p> : (
      <>
        <h3>Detected: {payload.prediction}</h3>
        <h3>Scores:</h3>
        <ul>
          {Object.entries(payload.scores).map(([label, score]) =>
            <li><p>{score}% {label}</p></li>)}
        </ul>
      </>
    )}
  </>
}
