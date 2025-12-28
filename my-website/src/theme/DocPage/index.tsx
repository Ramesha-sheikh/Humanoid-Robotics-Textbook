import React, { type ReactNode } from 'react';
import DocPage from '@theme-original/DocPage';
import type DocPageType from '@theme/DocPage';
import type { WrapperProps } from '@docusaurus/types';
import { ProtectedContent } from '../../components/Auth/ProtectedContent';

type Props = WrapperProps<typeof DocPageType>;

/**
 * DocPage Wrapper - Protects all documentation pages
 * Requires users to sign up/login before accessing book content
 */
export default function DocPageWrapper(props: Props): ReactNode {
  return (
    <ProtectedContent requireAuth={true}>
      <DocPage {...props} />
    </ProtectedContent>
  );
}
