import Link from 'next/link';

export default function Home() {
  return (
    <div>
      <h1>Vargard Dashboard</h1>
      <ul>
        <li><Link href="/telemetry">Telemetry</Link></li>
        <li><Link href="/alerts">Alerts</Link></li>
        <li><Link href="/live">Live View</Link></li>
      </ul>
    </div>
  );
}
