import type { NextConfig } from "next";

// GitHub Pages serves project sites below a repository-specific path. The
// workflow provides that path so generated asset URLs work in production while
// local development continues to run at the root URL.
const isGitHubPagesBuild = process.env.GITHUB_PAGES_BUILD === "true";
const basePath = process.env.NEXT_PUBLIC_BASE_PATH;

const nextConfig: NextConfig = {
  ...(isGitHubPagesBuild ? {
    output: "export",
    ...(basePath ? { basePath } : {}),
  } : {}),
};

export default nextConfig;
