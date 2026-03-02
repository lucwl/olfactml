import { useEffect, useState } from 'react'

function App() {
  const [messages, setMessages] = useState<string[]>([])

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:3000/ws')

    ws.onopen = () => {
      console.log('Connected to WebSocket')
    }

    ws.onmessage = (event) => {
      setMessages((prev) => [...prev, event.data])
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
      <h1>WebSocket Messages</h1>
      <div>
        {messages.map((msg, i) => (
          <div key={i}>Received: {msg}</div>
        ))}
      </div>
    </div>
  )
}

export default App
