import React, {type ReactNode} from 'react';
import RagChatbot from '../../../src/components/RagChatbot'; // Relative path to RAGChatBot
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  return (
    <>
      <Layout {...props} />
      <RagChatbot />
    </>
  );
}
