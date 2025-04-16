import useSWR from 'swr'

const fetcher = (url) => fetch(url).then((res) => res.json())

export default function Home() {
  const { data: telemetry, error: telError } = useSWR('/api/telemetry', fetcher, { refreshInterval: 1000 })
  const { data: alerts, error: alertsError } = useSWR('/api/alerts', fetcher, { refreshInterval: 1000 })

  if (telError || alertsError) return <div>Failed to load data</div>
  if (!telemetry || !alerts) return <div>Loading...</div>

  return (
    <div style={{ padding: '2rem', fontFamily: 'sans-serif' }}>
      <h1>Vargard Dashboard</h1>
      <section>
        <h2>Telemetry</h2>
        <pre>{JSON.stringify(telemetry, null, 2)}</pre>
      </section>
      <section>
        <h2>Alerts</h2>
        <pre>{JSON.stringify(alerts, null, 2)}</pre>
      </section>
    </div>
  )
}