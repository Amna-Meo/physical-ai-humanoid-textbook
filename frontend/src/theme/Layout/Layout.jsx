import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '../../components/ChatWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatWidget />
    </>
  );
}

