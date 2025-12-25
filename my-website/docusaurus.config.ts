import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Digital Intelligence with the Physical World',
  favicon: 'img/logo.png',

  customFields: {
    geminiApiKey: process.env.GEMINI_API_KEY || '',
  },

  future: {
    v4: true,
  },

  // ✅ Vercel live URL
  url: 'https://humanoid-robotics-textbook-psi.vercel.app',
  // ✅ Base URL for Vercel deploy
  baseUrl: '/',

  organizationName: 'GIAIC-Students',
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  plugins: [],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/GIAIC-Students/Physical-AI-Humanoid-Robotics-Textbook/tree/main/my-website/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/GIAIC-Students/Physical-AI-Humanoid-Robotics-Textbook/tree/main/my-website/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Tutorial',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/Ramesha-sheikh',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {
              label: 'Introduction',
              to: '/docs/introduction',
            },
            {
              label: 'Tutorial',
              to: '/docs/introduction',
            },
            {
              label: 'Blog',
              to: '/blog',
            },
          ],
        },
        {
          title: 'Connect',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Ramesha-sheikh',
            },
            {
              label: 'LinkedIn',
              href: 'https://www.linkedin.com/in/rameesha20/',
            },
            {
              label: 'Facebook',
              href: 'https://www.facebook.com/ramesha.javed.2025',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Physical AI',
              to: '/docs/introduction',
            },
            {
              label: 'Humanoid Robotics',
              to: '/docs/introduction',
            },
            {
              label: 'Tutorials',
              to: '/docs/introduction',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with passion for robotics education.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
