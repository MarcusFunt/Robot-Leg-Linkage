import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import "./globals.css";
import "./workbench.css";
import RobotWorkbench from "./robot-workbench";

const basePath = process.env.NEXT_PUBLIC_BASE_PATH ?? "";
const geistSans = Geist({ variable: "--font-geist-sans", subsets: ["latin"] });
const geistMono = Geist_Mono({ variable: "--font-geist-mono", subsets: ["latin"] });

export const metadata: Metadata = {
  title: "Robot Leg Linkage Lab",
  description: "Robot-leg focused four-bar linkage kinematics, path, motion, load, and design comparison workbench.",
  other: { "codex-preview": "development" },
  icons: { icon: `${basePath}/favicon.svg`, shortcut: `${basePath}/favicon.svg` },
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="en">
      <body className={`${geistSans.variable} ${geistMono.variable}`}>
        {children}
        <RobotWorkbench />
      </body>
    </html>
  );
}
