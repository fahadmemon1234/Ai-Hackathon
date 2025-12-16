import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

function ChatbotWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleTextSelected = (event) => {
      setSelectedText(event.detail);
    };

    document.addEventListener('textSelected', handleTextSelected);

    return () => {
      document.removeEventListener('textSelected', handleTextSelected);
    };
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) { // If opening, clear selected text to avoid stale context
      setSelectedText('');
    }
  };

  const handleInputChange = (event) => {
    setInput(event.target.value);
  };

  const sendMessage = async (message, contextMode, textSelection = null) => {
    if (!message.trim()) return;

    const userMessage = { sender: 'user', text: message };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const payload = {
        message: message,
        context_mode: contextMode,
        ...(textSelection && { selected_text: textSelection }),
      };

      const response = await fetch('http://localhost:8000/api/chat', { // Assuming backend runs on 8000
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage = { sender: 'bot', text: data.response };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error('Error fetching chatbot response:', error);
      setMessages((prevMessages) => [...prevMessages, { sender: 'bot', text: 'Sorry, something went wrong.' }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSubmitFullBook = (event) => {
    event.preventDefault();
    sendMessage(input, 'full_book');
  };

  const handleSubmitSelectedText = (event) => {
    event.preventDefault();
    if (!selectedText) return;
    sendMessage(input, 'selected_text', selectedText);
    setSelectedText(''); // Clear selected text after sending
  };

  return (
    <div className={styles.chatbotContainer}>
      <button className={styles.chatButton} onClick={toggleChat}>
        {isOpen ? 'Close Chat' : 'Open Chat'}
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>Book Chatbot</h3>
            <button className={styles.closeButton} onClick={toggleChat}>&times;</button>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
                {msg.text}
              </div>
            ))}
            {isLoading && <div className={styles.loadingIndicator}>Typing...</div>}
          </div>
          <form onSubmit={handleSubmitFullBook} className={styles.chatInputForm}>
            <input
              type="text"
              value={input}
              onChange={handleInputChange}
              placeholder="Ask a question about the book..."
              className={styles.chatInputField}
              disabled={isLoading}
            />
            {selectedText && (
              <button
                type="button" // Important: type="button" to prevent form submission
                onClick={handleSubmitSelectedText}
                className={styles.sendButton}
                disabled={isLoading}
              >
                Ask about selection
              </button>
            )}
            <button type="submit" className={styles.sendButton} disabled={isLoading}>Send</button>
          </form>
          {selectedText && (
            <div className={styles.selectedTextPreview}>
              Selected: "{selectedText.substring(0, 50)}..."
              <button className={styles.clearSelectionButton} onClick={() => setSelectedText('')}>Clear</button>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

export default ChatbotWidget;