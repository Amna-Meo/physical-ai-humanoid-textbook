import React, { useState, useEffect, useRef } from 'react';

const AIChat = ({ sessionId, chapterContext = null }) => {
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionIdState, setSessionIdState] = useState(sessionId);
  const messagesEndRef = useRef(null);

  // Simulate creating a new chat session if none provided
  useEffect(() => {
    if (!sessionIdState) {
      const createSession = async () => {
        try {
          // Simulate API call
          await new Promise(resolve => setTimeout(resolve, 300));
          const newSessionId = Math.floor(Math.random() * 10000);
          setSessionIdState(newSessionId);
        } catch (error) {
          console.error('Error creating chat session:', error);
        }
      };

      createSession();
    }
  }, [sessionIdState]);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Sample initial messages for context
  useEffect(() => {
    if (sessionIdState && messages.length === 0) {
      setMessages([
        {
          id: 1,
          role: 'assistant',
          content: `Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. I can help explain concepts, answer questions, and provide examples based on the textbook content. ${
            chapterContext ? `We're currently discussing "${chapterContext.title}".` : 'Ask me anything about Physical AI and Humanoid Robotics!'
          }`,
          timestamp: new Date().toISOString(),
          sources: chapterContext ? [chapterContext.id] : []
        }
      ]);
    }
  }, [sessionIdState, chapterContext]);

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputMessage.trim() || isLoading) return;

    const userMessage = {
      id: messages.length + 1,
      role: 'user',
      content: inputMessage,
      timestamp: new Date().toISOString(),
      sources: []
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputMessage('');
    setIsLoading(true);

    try {
      // Simulate API call to AI service
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Mock AI response based on user input
      let aiResponse = "I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. I can help explain concepts, answer questions, and provide examples based on the textbook content.";

      if (inputMessage.toLowerCase().includes('physical ai') || inputMessage.toLowerCase().includes('embodiment')) {
        aiResponse = `Physical AI represents a paradigm shift where cognitive systems are embodied in physical form and interact with the real world. Unlike traditional AI, Physical AI systems must navigate physics, dynamics, and real-world uncertainty. The key principles include embodiment (intelligence emerging from agent-environment interaction), real-time processing, uncertainty management, and energy efficiency.`;
      } else if (inputMessage.toLowerCase().includes('humanoid') || inputMessage.toLowerCase().includes('robot')) {
        aiResponse = `Humanoid robotics is a key application area for Physical AI. Humanoid robots must perform complex tasks in human environments, requiring sophisticated locomotion, manipulation, and interaction capabilities. They face challenges like the sim-to-real gap, safety, scalability, and learning efficiency.`;
      } else if (inputMessage.toLowerCase().includes('challenge') || inputMessage.toLowerCase().includes('difficulty')) {
        aiResponse = `The main technical challenges in Physical AI include: 1) The sim-to-real gap - bridging differences between simulation and real-world performance, 2) Safety - ensuring safe operation around humans and environments, 3) Scalability - developing systems that operate reliably across diverse scenarios, and 4) Learning efficiency - acquiring new skills with minimal physical interaction.`;
      } else {
        aiResponse = `Based on the textbook content, Physical AI and Humanoid Robotics involve complex interactions between cognitive systems and physical environments. The field addresses challenges like real-time processing, uncertainty management, and energy efficiency. ${
          chapterContext
            ? `In the context of "${chapterContext.title}", this concept is particularly relevant to the ${chapterContext.metadata?.concepts?.join(', ') || 'core principles'} discussed in this chapter.`
            : 'These principles form the foundation for understanding embodied AI systems.'
        }`;
      }

      const aiMessage = {
        id: messages.length + 2,
        role: 'assistant',
        content: aiResponse,
        timestamp: new Date().toISOString(),
        sources: chapterContext ? [chapterContext.id] : []
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Error getting AI response:', error);

      const errorMessage = {
        id: messages.length + 2,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date().toISOString(),
        sources: [],
        isError: true
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const clearChat = () => {
    setMessages([
      {
        id: 1,
        role: 'assistant',
        content: `Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. I can help explain concepts, answer questions, and provide examples based on the textbook content. ${
          chapterContext ? `We're currently discussing "${chapterContext.title}".` : 'Ask me anything about Physical AI and Humanoid Robotics!'
        }`,
        timestamp: new Date().toISOString(),
        sources: chapterContext ? [chapterContext.id] : []
      }
    ]);
  };

  return (
    <div className="flex flex-col h-full bg-gray-50 rounded-lg shadow-md">
      <div className="bg-white p-4 border-b border-gray-200 rounded-t-lg">
        <div className="flex justify-between items-center">
          <h3 className="text-lg font-semibold text-gray-800">AI Assistant</h3>
          <div className="flex space-x-2">
            <span className="text-xs bg-blue-100 text-blue-800 px-2 py-1 rounded">
              Session: {sessionIdState || 'New'}
            </span>
            <button
              onClick={clearChat}
              className="text-xs bg-gray-200 hover:bg-gray-300 text-gray-700 px-2 py-1 rounded transition-colors"
            >
              Clear
            </button>
          </div>
        </div>
        {chapterContext && (
          <div className="mt-2 text-sm text-gray-600">
            Context: <span className="font-medium">{chapterContext.title}</span>
          </div>
        )}
      </div>

      <div className="flex-1 overflow-y-auto p-4 space-y-4 max-h-96">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`flex ${message.role === 'user' ? 'justify-end' : 'justify-start'}`}
          >
            <div
              className={`max-w-[80%] rounded-lg p-3 ${
                message.role === 'user'
                  ? 'bg-blue-500 text-white'
                  : message.isError
                  ? 'bg-red-100 text-red-800 border border-red-200'
                  : 'bg-gray-200 text-gray-800'
              }`}
            >
              <div className="whitespace-pre-wrap">{message.content}</div>
              <div className={`text-xs mt-1 ${message.role === 'user' ? 'text-blue-100' : 'text-gray-500'}`}>
                {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
              {message.sources && message.sources.length > 0 && (
                <div className="mt-1 text-xs italic opacity-75">
                  Sources: {message.sources.join(', ')}
                </div>
              )}
            </div>
          </div>
        ))}
        {isLoading && (
          <div className="flex justify-start">
            <div className="bg-gray-200 text-gray-800 rounded-lg p-3 max-w-[80%]">
              <div className="flex space-x-2">
                <div className="w-2 h-2 bg-gray-500 rounded-full animate-bounce"></div>
                <div className="w-2 h-2 bg-gray-500 rounded-full animate-bounce delay-75"></div>
                <div className="w-2 h-2 bg-gray-500 rounded-full animate-bounce delay-150"></div>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSendMessage} className="bg-white p-4 border-t border-gray-200 rounded-b-lg">
        <div className="flex space-x-2">
          <input
            type="text"
            value={inputMessage}
            onChange={(e) => setInputMessage(e.target.value)}
            placeholder="Ask about Physical AI, Humanoid Robotics, or textbook content..."
            className="flex-1 px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            disabled={isLoading}
          />
          <button
            type="submit"
            disabled={!inputMessage.trim() || isLoading}
            className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            Send
          </button>
        </div>
        <div className="mt-2 text-xs text-gray-500">
          AI responses are grounded in textbook content. Responses are for educational purposes.
        </div>
      </form>
    </div>
  );
};

export default AIChat;