import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts'
import type { InferencePayload } from "./types"

interface InferenceViewParams {
  payload: InferencePayload | null
}

export default function InferenceView({ payload }: InferenceViewParams) {
  const chartData = payload ? Object.values(payload.scores) : []

  return <> <h1>Inference</h1>

    {payload === null ? <p>Waiting for connection...</p> : (
      <>
        <h3>Detected: {payload.prediction}</h3>

        <h3>Confidence Scores:</h3>
        <ResponsiveContainer width="100%" height={400}>
          <BarChart data={chartData} margin={{ top: 20, right: 30, left: 20, bottom: 5 }}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="label" />
            <YAxis domain={[0, 1]} />
            <Tooltip />
            <Legend />
            <Bar dataKey="score" fill="#8884d8" name="Confidence" />
          </BarChart>
        </ResponsiveContainer>

        <h3>Scores:</h3>
        <ul>
          {Object.values(payload.scores).map((score) => (
            <li key={score.label}><p>{score.label}: {score.score.toFixed(4)}</p></li>
          ))}
        </ul>
      </>
    )}
  </>
}
