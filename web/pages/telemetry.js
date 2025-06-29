import { useEffect, useState } from 'react';

export default function Telemetry() {
  const [data, setData] = useState([]);

  useEffect(() => {
    const evt = new EventSource('http://localhost:5000/telemetry/stream');
    evt.onmessage = (e) => {
      try {
        setData((d) => [...d.slice(-99), JSON.parse(e.data)]);
      } catch {}
    };
    return () => evt.close();
  }, []);

  return (
    <div>
      <h1>Telemetry</h1>
      <pre>{JSON.stringify(data[data.length - 1] || {}, null, 2)}</pre>
    </div>
  );
}
