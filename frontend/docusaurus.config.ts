import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Generative AI with Hard Robotics for Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Custom fields accessible via useDocusaurusContext()
  customFields: {
    apiUrl: process.env.BACKEND_URL || 'http://localhost:8000',
    authServerUrl: process.env.AUTH_SERVER_URL || 'http://localhost:3001',
    chatkitDomainKey: process.env.CHATKIT_DOMAIN_KEY || 'localhost',
  },

  // ChatKit CDN script - CRITICAL for ChatKit UI to render
  scripts: [
    {
      src: 'https://cdn.platform.openai.com/deployments/chatkit/chatkit.js',
      async: true,
    },
  ],

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://bilalinit.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need 
  organizationName: 'bilalinit', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoids-book', // Usually your repo name.

  onBrokenLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/bilalinit/Physical-AI-Humanoids-book/tree/001-physical-ai-book',
          // Enable Mermaid diagrams for ROS graphs, TF trees, and state machines
          remarkPlugins: [require('mdx-mermaid')],
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    [
      require.resolve("@docusaurus/plugin-client-redirects"),
      {
        redirects: [
          // Add any redirects here if needed
        ],
      },
    ],
    // Mermaid is now built into Docusaurus 3, just need to enable it in docs options
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI  & Humanoids',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'docsSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/bilalinit',
          label: 'GitHub',
          position: 'right',
        },
        {
          type: 'custom-NavbarUserMenu',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Physical AI Book',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'linked-in',
              href: 'https://www.linkedin.com/in/bilal-saeed-9aa904284?utm_source=share_via&utm_content=profile&utm_medium=member_android',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com',
            },

          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/bilalinit',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoids Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
