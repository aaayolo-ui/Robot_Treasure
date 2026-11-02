import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Robot Treasure · 实车监控",
  description: "地图轨迹、四路激光、灰度和YAW监控，支持电脑串口连接开关。",
  icons: {
    icon: "/favicon.svg",
    shortcut: "/favicon.svg",
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="zh-CN">
      <body className="antialiased">{children}</body>
    </html>
  );
}
