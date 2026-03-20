import "@/styles/globals.css";
import Head from "next/head";
import Simulator from "components/Simulator";

export default function App({ Component, pageProps }) {
  return (
    <div className="min-h-screen bg-gradient-to-br from-indigo-50 via-white to-cyan-50">
      <Head>
        <title>MDP Algorithm Simulator</title>
        <meta name="description" content="Multi-Disciplinary Project Algorithm Simulator" />
        <link rel="icon" href="/favicon.ico" />
      </Head>
      <Simulator />
    </div>
  );
}
