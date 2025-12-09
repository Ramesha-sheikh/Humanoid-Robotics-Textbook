import React from 'react';

const RagChatbot = () => {
  return (
    <div style={{
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      backgroundColor: '#007bff',
      color: 'white',
      borderRadius: '50%',
      width: '60px',
      height: '60px',
      display: 'flex',
      justifyContent: 'center',
      alignItems: 'center',
      cursor: 'pointer',
      boxShadow: '0px 4px 8px rgba(0, 0, 0, 0.2)'
    }}>
      Chat
    </div>
  );
};

export default RagChatbot;
