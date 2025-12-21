

import React, { useState } from 'react';

export default function ChatWidget() {
  const [message, setMessage] = useState('');
  const [responses, setResponses] = useState([]);

  const sendMessage = async () => {
    if (!message) return;

    // Add user message to chat
    setResponses([...responses, { sender: 'user', text: message }]);

    // Call FastAPI backend
    try {
      const res = await fetch('http://127.0.0.1:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message }),
      });
      const data = await res.json();
      setResponses((prev) => [...prev, { sender: 'bot', text: data.answer }]);
    } catch (err) {
      setResponses((prev) => [...prev, { sender: 'bot', text: 'Error: Backend not reachable' }]);
    }

    setMessage('');
  };

  return (
    <div
      style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        width: '300px',
        height: '300px',
        backgroundColor: '#ffffff',
        border: '1px solid #ddd',
        borderRadius: '8px',
        padding: '12px',
        boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
        zIndex: 9999,
        display: 'flex',
        flexDirection: 'column',
      }}
    >
      <h4 style={{ marginTop: 0 }}>📘 Book Assistant</h4>

      <div style={{ flex: 1, overflowY: 'auto', fontSize: '14px', marginBottom: '8px' }}>
        {responses.map((r, i) => (
          <div key={i} style={{ marginBottom: '4px', color: r.sender === 'bot' ? 'blue' : 'green' }}>
            <b>{r.sender}:</b> {r.text}
          </div>
        ))}
      </div>

      <div style={{ display: 'flex' }}>
        <input
          style={{ flex: 1, padding: '4px' }}
          value={message}
          onChange={(e) => setMessage(e.target.value)}
          placeholder="Type your message..."
        />
        <button onClick={sendMessage}>Send</button>
      </div>
    </div>
  );
}

