import React, { type ReactNode } from 'react';
import DocSidebar from '@theme-original/DocSidebar';
import type DocSidebarType from '@theme/DocSidebar';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof DocSidebarType>;

/**
 * Sidebar wrapper - personalization removed per user request
 */
export default function DocSidebarWrapper(props: Props): ReactNode {
  // Just return the default sidebar without personalization
  return <DocSidebar {...props} />;
}
