import { useEffect, useState, useRef, useMemo } from "react"
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

const MAX_POINTS = 5000

export default function LoggingView({ payload }: LoggingViewParams) {
  const [historyBySensor, setHistoryBySensor] = useState<Record<number, DataPoint[]>>({})
  const [selectedSensor, setSelectedSensor] = useState<number>(0)
  const batchRef = useRef<Record<number, DataPoint[]>>({})
  const updateTimerRef = useRef<NodeJS.Timeout | null>(null)

  useEffect(() => {
    if (payload) {
      const newPoint: DataPoint = {
        time: Date.now(),
        t: payload.t,
        p: payload.p,
        h: payload.h,
        gas: payload.gas
      }

      if (!batchRef.current[payload.idx]) {
        batchRef.current[payload.idx] = []
      }
      batchRef.current[payload.idx].push(newPoint)

      if (updateTimerRef.current === null) {
        updateTimerRef.current = setTimeout(() => {
          setHistoryBySensor(prev => {
            const updated = { ...prev }
            for (const [idx, points] of Object.entries(batchRef.current)) {
              const sensorIdx = Number(idx)
              const existing = updated[sensorIdx] || []
              updated[sensorIdx] = [...existing, ...points].slice(-MAX_POINTS)
            }
            batchRef.current = {}
            return updated
          })
          updateTimerRef.current = null
        }, 50)
      }

      setSelectedSensor(prev => prev === 0 && Object.keys(historyBySensor).length === 0 ? payload.idx : prev)
    }
  }, [payload])

  const history = historyBySensor[selectedSensor] || []
  const availableSensors = Object.keys(historyBySensor).map(Number).sort((a, b) => a - b)

  const formatTime = (timestamp: number | string) => {
    const date = new Date(timestamp)
    return date.toLocaleTimeString()
  }

  const formatGasResistance = (value: number) => {
    if (value >= 1e6) return `${(value / 1e6).toFixed(1)}M`
    if (value >= 1e3) return `${(value / 1e3).toFixed(1)}k`
    return value.toFixed(0)
  }

  return <div style={{
    background: '#ffffff',
    border: '1px solid #d0d0d0',
    padding: '1.5rem'
  }}>
    <div style={{
      borderBottom: '1px solid #d0d0d0',
      paddingBottom: '1rem',
      marginBottom: '1.5rem',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'baseline'
    }}>
      <h1 style={{
        fontSize: '0.875rem',
        fontWeight: '400',
        color: '#666',
        textTransform: 'uppercase',
        letterSpacing: '0.1em'
      }}>BME680 / Live Data</h1>
      {payload && (
        <span style={{
          fontSize: '0.8rem',
          color: '#999'
        }}>label={payload.lbl}</span>
      )}
    </div>

    {payload === null ? (
      <p style={{
        textAlign: 'center',
        padding: '3rem',
        fontSize: '0.875rem',
        color: '#999'
      }}>// waiting for websocket connection</p>
    ) : (
      <>
        <div style={{
          marginBottom: '1.5rem'
        }}>
          <label htmlFor="sensor-select" style={{
            marginRight: '0.5rem',
            fontSize: '0.875rem',
            color: '#666'
          }}>sensor_idx:</label>
          <select
            id="sensor-select"
            value={selectedSensor}
            onChange={(e) => setSelectedSensor(Number(e.target.value))}
            style={{
              padding: '0.25rem 0.5rem',
              fontSize: '0.875rem',
              border: '1px solid #d0d0d0',
              background: '#ffffff',
              color: '#1a1a1a',
              cursor: 'pointer',
              fontFamily: 'IBM Plex Mono, monospace'
            }}
          >
            {availableSensors.map(idx => (
              <option key={idx} value={idx}>{idx}</option>
            ))}
          </select>
        </div>

        <div style={{
          background: '#fafafa',
          padding: '1rem',
          border: '1px solid #e0e0e0',
          marginBottom: '1rem',
          overflow: 'hidden'
        }}>
          <h4 style={{
            fontSize: '0.75rem',
            fontWeight: '400',
            marginBottom: '0.75rem',
            color: '#666',
            textTransform: 'uppercase',
            letterSpacing: '0.05em'
          }}>gas_resistance_ohm</h4>
          <div style={{ width: '100%', height: '250px' }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={history} margin={{ top: 5, right: 5, left: 5, bottom: 5 }}>
                <CartesianGrid strokeDasharray="1 3" stroke="#d0d0d0" />
                <XAxis
                  dataKey="time"
                  tickFormatter={formatTime}
                  tick={{ fontSize: 10, fill: '#666' }}
                  stroke="#999"
                  scale="point"
                />
                <YAxis tickFormatter={formatGasResistance} tick={{ fontSize: 10, fill: '#666' }} stroke="#999" width={60} />
                <Tooltip labelFormatter={formatTime} contentStyle={{ background: '#ffffff', border: '1px solid #d0d0d0', fontSize: '0.75rem' }} />
                <Line type="linear" dataKey="gas" stroke="#00c070" strokeWidth={1.5} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>

        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '1rem' }}>
          <div style={{
            background: '#fafafa',
            padding: '1rem',
            border: '1px solid #e0e0e0'
          }}>
            <h4 style={{
              fontSize: '0.75rem',
              fontWeight: '400',
              marginBottom: '0.75rem',
              color: '#666',
              textTransform: 'uppercase',
              letterSpacing: '0.05em'
            }}>temp_c</h4>
            <ResponsiveContainer width="100%" height={180}>
              <LineChart data={history}>
                <CartesianGrid strokeDasharray="1 3" stroke="#d0d0d0" />
                <XAxis dataKey="time" tickFormatter={formatTime} tick={{ fontSize: 10, fill: '#666' }} stroke="#999" />
                <YAxis tick={{ fontSize: 10, fill: '#666' }} stroke="#999" />
                <Tooltip labelFormatter={formatTime} contentStyle={{ background: '#ffffff', border: '1px solid #d0d0d0', fontSize: '0.75rem' }} />
                <Line type="linear" dataKey="t" stroke="#ff6b35" strokeWidth={1.5} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
          <div style={{
            background: '#fafafa',
            padding: '1rem',
            border: '1px solid #e0e0e0'
          }}>
            <h4 style={{
              fontSize: '0.75rem',
              fontWeight: '400',
              marginBottom: '0.75rem',
              color: '#666',
              textTransform: 'uppercase',
              letterSpacing: '0.05em'
            }}>pressure_hpa</h4>
            <ResponsiveContainer width="100%" height={180}>
              <LineChart data={history}>
                <CartesianGrid strokeDasharray="1 3" stroke="#d0d0d0" />
                <XAxis dataKey="time" tickFormatter={formatTime} tick={{ fontSize: 10, fill: '#666' }} stroke="#999" />
                <YAxis tick={{ fontSize: 10, fill: '#666' }} stroke="#999" />
                <Tooltip labelFormatter={formatTime} contentStyle={{ background: '#ffffff', border: '1px solid #d0d0d0', fontSize: '0.75rem' }} />
                <Line type="linear" dataKey="p" stroke="#4a9eff" strokeWidth={1.5} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
          <div style={{
            background: '#fafafa',
            padding: '1rem',
            border: '1px solid #e0e0e0'
          }}>
            <h4 style={{
              fontSize: '0.75rem',
              fontWeight: '400',
              marginBottom: '0.75rem',
              color: '#666',
              textTransform: 'uppercase',
              letterSpacing: '0.05em'
            }}>humidity_pct</h4>
            <ResponsiveContainer width="100%" height={180}>
              <LineChart data={history}>
                <CartesianGrid strokeDasharray="1 3" stroke="#d0d0d0" />
                <XAxis dataKey="time" tickFormatter={formatTime} tick={{ fontSize: 10, fill: '#666' }} stroke="#999" />
                <YAxis tick={{ fontSize: 10, fill: '#666' }} stroke="#999" />
                <Tooltip labelFormatter={formatTime} contentStyle={{ background: '#ffffff', border: '1px solid #d0d0d0', fontSize: '0.75rem' }} />
                <Line type="linear" dataKey="h" stroke="#00d9ff" strokeWidth={1.5} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>
      </>
    )}
  </div>
}

