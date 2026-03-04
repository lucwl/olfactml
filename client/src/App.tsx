import { useEffect, useState } from 'react'
import type { LoggingPayload, InferencePayload, Message } from './types'
import { Mode } from "./types"
import InferenceView from "./InferenceView"
import LoggingView from "./LoggingView"

function App() {
  const [mode, setMode] = useState<Mode>(Mode.Inference)
  const [loggingPayload, setLoggingPayload] = useState<LoggingPayload | null>(null)
  const [inferencePayload, setInferencePayload] = useState<InferencePayload | null>(null)

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:3000/ws')

    ws.onopen = () => {
      console.log('Connected to WebSocket')
    }

    ws.onmessage = (event) => {
      const message: Message<any> = JSON.parse(event.data)

      setMode(message.op)

      switch (message.op) {
        case Mode.Logging: {
          const payload: LoggingPayload = message.d as LoggingPayload
          break
        }
        case Mode.Inference: {
          const payload: InferencePayload = message.d as InferencePayload
        }
      }
    }

    ws.onerror = (error) => {
      console.error('WebSocket error:', error)
    }

    ws.onclose = () => {
      console.log('Disconnected from WebSocket')
    }

    return () => {
      ws.close()
    }
  }, [])

  return (
    <div>
      {mode == Mode.Inference ? <InferenceView payload={inferencePayload} /> : <LoggingView payload={loggingPayload} />}
    </div>
  )
}

export default App
