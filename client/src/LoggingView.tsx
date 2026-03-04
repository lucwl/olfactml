import { useEffect, useState } from "react"
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from "recharts"
import type { LoggingPayload } from "./types"

interface LoggingViewParams {
  payload: LoggingPayload | null
}

interface DataPoint {
  time: number
  t: number
  p: number
  h: number
  gas: number
}

const MAX_POINTS = 100

export default function LoggingView({ payload }: LoggingViewParams) {
  const [history, setHistory] = useState<DataPoint[]>([])

  useEffect(() => {
    if (payload) {
      setHistory(prev => {
        const newPoint: DataPoint = {
          time: Date.now(),
          t: payload.t,
          p: payload.p,
          h: payload.h,
          gas: payload.gas
        }
        const updated = [...prev, newPoint]
        return updated.slice(-MAX_POINTS)
      })
    }
  }, [payload])

  const formatTime = (timestamp: number | string) => {
    const date = new Date(timestamp)
    return date.toLocaleTimeString()
  }

  return <>
    <h1>Logging</h1>

    {payload === null ? <p>Waiting for connection...</p> : (
      <>
        <h3>Recording {payload.lbl}...</h3>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '20px', marginTop: '20px' }}>
          <div>
            <h4>Temperature</h4>
            <ResponsiveContainer width="100%" height={200}>
              <LineChart data={history}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" tickFormatter={formatTime} />
                <YAxis />
                <Tooltip labelFormatter={formatTime} />
                <Line type="monotone" dataKey="t" stroke="#ff6b6b" dot={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
          <div>
            <h4>Pressure</h4>
            <ResponsiveContainer width="100%" height={200}>
              <LineChart data={history}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" tickFormatter={formatTime} />
                <YAxis />
                <Tooltip labelFormatter={formatTime} />
                <Line type="monotone" dataKey="p" stroke="#4ecdc4" dot={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
          <div>
            <h4>Humidity</h4>
            <ResponsiveContainer width="100%" height={200}>
              <LineChart data={history}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" tickFormatter={formatTime} />
                <YAxis />
                <Tooltip labelFormatter={formatTime} />
                <Line type="monotone" dataKey="h" stroke="#45b7d1" dot={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
          <div>
            <h4>Gas Resistance</h4>
            <ResponsiveContainer width="100%" height={200}>
              <LineChart data={history}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" tickFormatter={formatTime} />
                <YAxis />
                <Tooltip labelFormatter={formatTime} />
                <Line type="monotone" dataKey="gas" stroke="#96ceb4" dot={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>
      </>
    )}
  </>
}

