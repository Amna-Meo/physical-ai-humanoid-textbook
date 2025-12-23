/** @type {import('@docusaurus/types').DocusaurusConfig} */
module.exports = {
    title: "Physical AI & Humanoid Robotics",
    tagline: "AI-Native Textbook for Panaversity Hackathon",
    url: "https://Amna-Meo.github.io", // Replace with your GitHub username
    baseUrl: "/physical-ai-humanoid-textbook/", // Your repository name
    onBrokenLinks: "warn",
    favicon: "img/favicon.ico",
    organizationName: "Amna-Meo", // Your GitHub username
    projectName: "physical-ai-humanoid-textbook", // Repository name
    trailingSlash: false,
    customFields: {
        markdown: {
            hooks: {
                onBrokenMarkdownLinks: 'warn',
            },
        },
    },
    presets: [
        [
            "@docusaurus/preset-classic",
            {
                docs: {
                    path: "docs",
                    routeBasePath: "/", // <-- NEW: folder containing Markdown files
                    sidebarPath: require.resolve("./sidebars.js"),
                    editUrl:
                        "https://github.com/Amna-Meo/physical-ai-humanoid-textbook/edit/main/frontend/docs/",
                },
                blog: false, // We don't need a blog for the textbook
                theme: {
                    customCss: require.resolve("./src/css/custom.css"),
                },
            },
        ],
    ],
    themeConfig: {
        navbar: {
            title: "Physical AI & Humanoid Robotics",
            logo: {
                alt: "Panaversity Logo",
                src: "img/logo.png", // Optional logo
            },
            items: [
                {
                    type: "doc",
                    docId: "textbook-content/intro",
                    position: "left",
                    label: "Chapters",
                },
                {
                    href: "https://github.com/Amna-Meo/physical-ai-humanoid-textbook",
                    label: "GitHub",
                    position: "right",
                },
            ],
        },
        footer: {
            style: "dark",
            links: [
                {
                    title: "Learn",
                    items: [
                        {
                            label: "Chapters",
                            to: "/docs/intro",
                        },
                    ],
                },
                {
                    title: "Community",
                    items: [
                        {
                            label: "Panaversity",
                            href: "https://panaversity.org",
                        },
                    ],
                },
                {
                    title: "More",
                    items: [
                        {
                            label: "GitHub",
                            href: "https://github.com/Amna-Meo/physical-ai-humanoid-textbook",
                        },
                    ],
                },
            ],
            copyright: `© ${new Date().getFullYear()} Amna Meo. All rights reserved.`,
        },
    },
};
