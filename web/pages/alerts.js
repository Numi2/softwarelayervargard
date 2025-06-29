import { useEffect, useState } from 'react';

export default function Alerts() {
  const [data, setData] = useState([]);

  useEffect(() => {
    const evt = new EventSource('http://localhost:5000/alerts/stream');
    evt.onmessage = (e) => {
      try {
        setData((d) => [...d.slice(-99), JSON.parse(e.data)]);
      } catch {}
    };
    return () => evt.close();
  }, []);

  return (
    <div>
      <h1>Alerts</h1>
      <ul>
        {data.slice().reverse().map((a, idx) => (
          <li key={idx}>{a.description || JSON.stringify(a)}</li>
        ))}
      </ul>
    </div>
  );
}
