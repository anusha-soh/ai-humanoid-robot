import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}