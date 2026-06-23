import { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';

// ==========================================
// 1. KOMPONEN KECIL YANG BISA DIPAKAI ULANG (REUSABLE)
// ==========================================

// Komponen Kotak Indikator Sensor
function SensorCard({ title, value, unit, color = '#61dafb' }) {
  return (
    <div style={{ backgroundColor: '#282c34', color: color, padding: '15px', borderRadius: '8px', boxShadow: '0 4px 6px rgba(0,0,0,0.1)' }}>
      <h4 style={{ color: '#aaa', margin: '0 0 5px 0', fontSize: '14px' }}>{title}</h4>
      <h2 style={{ margin: 0, fontSize: '24px' }}>{value} <span style={{ fontSize: '14px' }}>{unit}</span></h2>
    </div>
  );
}

// Komponen Tombol Kontrol
function ControlButton({ label, onClick, color = '#e0e0e0', textColor = '#000', disabled = false }) {
  return (
    <button 
      onClick={onClick} 
      disabled={disabled}
      style={{
        padding: '12px', fontSize: '15px', fontWeight: 'bold', cursor: disabled ? 'not-allowed' : 'pointer',
        border: 'none', borderRadius: '6px', backgroundColor: disabled ? '#cccccc' : color, color: textColor,
        transition: '0.2s', boxShadow: '0 2px 4px rgba(0,0,0,0.05)'
      }}
    >
      {label}
    </button>
  );
}

// ==========================================
// 2. KOMPONEN UTAMA (DASHBOARD)
// ==========================================
function App() {
  const [isConnected, setIsConnected] = useState(false);
  const rosRef = useRef(null);

  // State Sensor (Mudah ditambah tinggal tambah state baru)
  const [telemetry, setTelemetry] = useState({
    sensorDepan: '0',
    sensorKiri: '0',
    baterai: '0',
  });

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
    rosRef.current = ros;

    ros.on('connection', () => setIsConnected(true));
    ros.on('error', (err) => console.error(err));
    ros.on('close', () => setIsConnected(false));

    // DAFTAR SUBSCRIBER (Sangat rapi dan terkumpul di satu tempat)
    const subDepan = new ROSLIB.Topic({ ros, name: '/sensor_depan', messageType: 'std_msgs/String' });
    const subKiri = new ROSLIB.Topic({ ros, name: '/sensor_kiri', messageType: 'std_msgs/String' });
    const subBaterai = new ROSLIB.Topic({ ros, name: '/baterai', messageType: 'std_msgs/String' });

    subDepan.subscribe((msg) => setTelemetry(prev => ({ ...prev, sensorDepan: msg.data })));
    subKiri.subscribe((msg) => setTelemetry(prev => ({ ...prev, sensorKiri: msg.data })));
    subBaterai.subscribe((msg) => setTelemetry(prev => ({ ...prev, baterai: msg.data })));

    return () => {
      subDepan.unsubscribe(); subKiri.unsubscribe(); subBaterai.unsubscribe();
      if (rosRef.current) rosRef.current.close();
    };
  }, []);

  // Fungsi publish global yang sangat clean
  const publishMessage = (topicName, payload) => {
    if (!isConnected || !rosRef.current) return;
    const topic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: 'std_msgs/String'
    });
    topic.publish({ data: payload });
  };

  return (
    <div style={{ padding: '2rem', fontFamily: 'sans-serif', maxWidth: '700px', margin: '0 auto' }}>
      <header style={{ textAlign: 'center', marginBottom: '30px' }}>
        <h2>KRAI Scalable Dashboard</h2>
        <span style={{ color: isConnected ? 'green' : 'red', fontWeight: 'bold' }}>
          {isConnected ? '🟢 Connected to Robot' : '🔴 Disconnected'}
        </span>
      </header>

      {/* Bagian Monitoring: Tinggal panggil SensorCard */}
      <section style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '15px', marginBottom: '30px' }}>
        <SensorCard title="Sensor Depan" value={telemetry.sensorDepan} unit="cm" />
        <SensorCard title="Sensor Kiri" value={telemetry.sensorKiri} unit="cm" />
        <SensorCard title="Daya Baterai" value={telemetry.baterai} unit="%" color="#4CAF50" />
      </section>

      {/* Bagian Kontrol Roda */}
      <h3 style={{ borderBottom: '1px solid #ddd', paddingBottom: '5px' }}>🎮 Pergerakan Roda</h3>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '10px', marginBottom: '25px' }}>
        <div/>
        <ControlButton label="MAJU" color="#4CAF50" textColor="white" onClick={() => publishMessage('/perintah_gerak', 'Maju')} disabled={!isConnected} />
        <div/>
        <ControlButton label="KIRI" color="#2196F3" textColor="white" onClick={() => publishMessage('/perintah_gerak', 'Kiri')} disabled={!isConnected} />
        <ControlButton label="STOP" color="#f44336" textColor="white" onClick={() => publishMessage('/perintah_gerak', 'Stop')} disabled={!isConnected} />
        <ControlButton label="KANAN" color="#2196F3" textColor="white" onClick={() => publishMessage('/perintah_gerak', 'Kanan')} disabled={!isConnected} />
      </div>

      {/* Bagian Mekanisme/Sistem */}
      <h3 style={{ borderBottom: '1px solid #ddd', paddingBottom: '5px' }}>⚙️ Mekanisme Robot</h3>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '10px' }}>
        <ControlButton label="Reset System" onClick={() => publishMessage('/mode_robot', 'Reset')} disabled={!isConnected} />
        <ControlButton label="🔥 Tembak Cincin" color="#FF9800" textColor="white" onClick={() => publishMessage('/kontrol_aktuator', 'Tembak')} disabled={!isConnected} />
      </div>
    </div>
  );
}

export default App;