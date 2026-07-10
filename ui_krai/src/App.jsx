import { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import './App.css';

function SensorCard({ title, value, unit, colorClass = 'text-cyan-400' }) {
  return (
    <div className="bg-[#282c34] p-[15px] rounded-lg shadow-md">
      <h4 className="text-gray-400 m-0 mb-[5px] text-[14px] font-medium">{title}</h4>
      <h2 className={`m-0 text-2xl font-bold ${colorClass}`}>
        {value} <span className="text-[14px] font-normal text-gray-300">{unit}</span>
      </h2>
    </div>
  );
}

// Komponen tombol di-upgrade untuk mendukung event tahan (Pointer Down/Up/Leave)
function ControlButton({ label, onClick, onPointerDown, onPointerUp, onPointerLeave, bgClass = 'bg-gray-200 hover:bg-gray-300 text-black', disabled = false }) {
  return (
    <button 
      onClick={onClick} 
      onPointerDown={onPointerDown}
      onPointerUp={onPointerUp}
      onPointerLeave={onPointerLeave}
      // Tambahan 'touch-none' agar layar tidak ikut tergeser (scroll) saat ditahan di HP
      className={`p-3 text-[15px] font-bold rounded-md transition-all shadow-sm duration-200 touch-none
        ${disabled ? 'bg-gray-300 text-gray-500 cursor-not-allowed' : bgClass}`}
    >
      {label}
    </button>
  );
}

function App() {
  const [isConnected, setIsConnected] = useState(false);
  const [capitState, setCapit] = useState(false);
  const rosRef = useRef(null);
  
  // Referensi untuk menyimpan loop pengiriman data (spam)
  const intervalRef = useRef(null); 

  const [telemetry, setTelemetry] = useState({
    sudutRobot: '0',
    sensorKiri: '0',
    lowLevel: [],
    baterai: '0',
    capitSenjata: '0',
    sensor: []
  });

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
    rosRef.current = ros;

    ros.on('connection', () => setIsConnected(true));
    ros.on('error', (err) => console.error(err));
    ros.on('close', () => setIsConnected(false));

    const subDepan = new ROSLIB.Topic({ ros, name: '/robot_yaw', messageType: 'std_msgs/Float32' });
    const subKiri = new ROSLIB.Topic({ ros, name: '/sensor_kiri', messageType: 'std_msgs/String' });
    const subBaterai = new ROSLIB.Topic({ ros, name: '/baterai', messageType: 'std_msgs/String' });
    const subLowLevel = new ROSLIB.Topic({ ros, name: '/motor_feedback', messageType: 'std_msgs/Float32MultiArray' });
    const subSensor = new ROSLIB.Topic({ ros, name: '/sensor', messageType: 'std_msgs/Float32MultiArray' });
    const subCapit = new ROSLIB.Topic({ ros, name: '/capit_cmd', messageType: 'std_msgs/Float32' });

    subDepan.subscribe((msg) => setTelemetry(prev => ({ ...prev, sudutRobot: msg.data })));
    subKiri.subscribe((msg) => setTelemetry(prev => ({ ...prev, sensorKiri: msg.data })));
    subBaterai.subscribe((msg) => setTelemetry(prev => ({ ...prev, baterai: msg.data })));
    subLowLevel.subscribe((msg) => setTelemetry(prev => ({ ...prev, lowLevel: msg.data })));
    subCapit.subscribe((msg) => setTelemetry(prev => ({ ...prev, capitSenjata: msg.data })));
    subSensor.subscribe((msg) => setTelemetry(prev => ({ ...prev, sensor: msg.data })));

    return () => {
      subDepan.unsubscribe(); subKiri.unsubscribe(); subBaterai.unsubscribe(); subLowLevel.unsubscribe(); subCapit.unsubscribe(); subSensor.unsubscribe();
      if (rosRef.current) rosRef.current.close();
      if (intervalRef.current) clearInterval(intervalRef.current); // Bersihkan interval saat keluar
    };
  }, []);
  
  const publishMessage = (topicName, messageType, payload) => {
    if (!isConnected || !rosRef.current) return;
    const topic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: messageType
    });
    
    const msg = typeof payload === 'object' ? payload : { data: payload };
    topic.publish(msg);
  };

  // Logika baru untuk menahan tombol (Loop 10Hz)
  const startMoving = (linearX, linearY, angularZ) => {
    if (!isConnected) return;
    
    // Hentikan interval lama jika tiba-tiba terpencet tombol lain
    if (intervalRef.current) clearInterval(intervalRef.current);

    const sendMsg = () => {
      const twistMsg = {
        linear: { x: linearX, y: linearY, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angularZ }
      };
      publishMessage('/cmd_vel', 'geometry_msgs/Twist', twistMsg);
    };

    sendMsg(); // Tembak 1 kali agar instan merespon
    intervalRef.current = setInterval(sendMsg, 100); // Lanjutkan setiap 100ms
  };

  // Logika untuk berhenti saat tombol dilepas
  const stopMoving = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    const twistMsg = {
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 }
    };
    publishMessage('/cmd_vel', 'geometry_msgs/Twist', twistMsg);
  };

  const toggleCapit = () => {
    const newCapitState = !capitState;
    setCapit(newCapitState);
    publishMessage('/capit_cmd', 'std_msgs/Float32', newCapitState ? 1 : 0);
  }

  const stopClimb = () => {
    publishMessage('/motor_climb', 'std_msgs/Float32', 0);
    publishMessage('/climb', 'std_msgs/Float32', 0);
  }
  
  return (
    <div className="p-8 font-sans max-w-[700px] mx-auto text-gray-800">
      <header className="text-center mb-[30px]">
        <h2 className="text-4xl font-bold mb-2 text-gray-900">Tobarania Dashboard</h2>
        <span className={`font-bold ${isConnected ? 'text-green-600' : 'text-red-600'}`}>
          {isConnected ? '🟢 Connected to Robot' : '🔴 Disconnected'}
        </span>
      </header>

      {/* Grid Monitoring */}
      <h3 className="border-b border-gray-300 pb-1 mb-4 text-xl font-semibold">Ultrasonik</h3>
      <section className="grid grid-cols-4 gap-[15px]">
        {/* <SensorCard title="Depan Bawah" value={telemetry.sensor[0].toFixed(2)} unit="" colorClass="text-green-500" /> */}
        <SensorCard title="Kanan" value={parseFloat(telemetry.sensor[7]).toFixed(2)} unit="" colorClass="text-green-500" />
        <SensorCard title="Depan Bawah" value={parseFloat(telemetry.sensor[0]).toFixed(2)} unit="" colorClass="text-green-500" />
        <SensorCard title="Depan Atas" value={parseFloat(telemetry.sensor[9]).toFixed(2)} unit="" colorClass="text-green-500" />
        <SensorCard title="Kiri" value={parseFloat(telemetry.sensor[10]).toFixed(2)} unit="" colorClass="text-green-500" />
      </section>
      <h3 className="border-b border-gray-300 pb-1 mb-4 text-xl font-semibold">Ultrasonik</h3>

      <section className="grid grid-cols-4 gap-[15px] mb-[30px]">
        <SensorCard title="Capit" value={parseFloat(telemetry.capitSenjata).toFixed(2)} unit="" />
        <SensorCard title="Proxy Capit" value={parseFloat(telemetry.sensor[1]).toFixed(2)} unit="" colorClass="text-green-500" />
        <SensorCard title="Proxy Turun" value={parseFloat(telemetry.sensor[6]).toFixed(2)} unit="" colorClass="text-green-500" />
        <SensorCard title="Proxy Lifter" value={parseFloat(telemetry.sensor[8]).toFixed(2)} unit="" colorClass="text-green-500" />
      </section>
      
      <section className="grid grid-cols-2 gap-[15px] mb-[30px]">
        <SensorCard title="Sudut Robot" value={parseFloat(telemetry.sensor[3]).toFixed(2)} unit="deg" />
        <SensorCard title="Cahaya" value={parseFloat(telemetry.sensor[2]).toFixed(2)} unit="" colorClass="text-green-500" />
      </section>

      <section className="grid grid-cols-4 gap-[15px] mb-[30px]">
        <SensorCard title="Kiri Belakang" value={parseFloat(telemetry.lowLevel[3]).toFixed(2)} unit="" />
        <SensorCard title="Kiri Depan" value={parseFloat(telemetry.lowLevel[1]).toFixed(2)} unit="" />
        <SensorCard title="Kanan Depan" value={parseFloat(telemetry.lowLevel[0]).toFixed(2)} unit="" />
        <SensorCard title="Kanan Belakang" value={parseFloat(telemetry.lowLevel[2]).toFixed(2)} unit="" />
      </section>

      {/* Capit */}
      <h3 className="border-b border-gray-300 pb-1 mb-4 text-xl font-semibold">Capit</h3>
      <div className="grid grid-cols-3 gap-2.5 mb-6">
        <ControlButton label="Capit / Lepas" bgClass={`${!capitState ? 'bg-green-500' : 'bg-gray-500'} hover:bg-green-600 text-white`} onClick={() => toggleCapit()} disabled={!isConnected} />
      </div>

      {/* Kontrol KRAI cmd_vel */}
      <h3 className="border-b border-gray-300 pb-1 mb-4 text-xl font-semibold">Pergerakan Robot</h3>
      
      {/* Tombol Maju / Serong */}
      <div className="grid grid-cols-3 gap-2.5 mb-3">
        <ControlButton label="↖️ SERONG KIRI" bgClass="bg-cyan-600 hover:bg-cyan-700 text-white" onPointerDown={() => startMoving(0.5, 0.5, 0)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
        <ControlButton label="▲ MAJU" bgClass="bg-cyan-600 hover:bg-cyan-700 text-white" onPointerDown={() => startMoving(0.5, 0, 0)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
        <ControlButton label="SERONG KANAN ↗️" bgClass="bg-cyan-600 hover:bg-cyan-700 text-white" onPointerDown={() => startMoving(0.5, -0.5, 0)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
      </div>
      
      {/* Tombol Geser / Stop */}
      <div className="grid grid-cols-3 gap-2.5 mb-3">
        <ControlButton label="◀ GESER KIRI" bgClass="bg-cyan-600 hover:bg-cyan-700 text-white" onPointerDown={() => startMoving(0, 1.0, 0)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
        <ControlButton label="🛑 STOP (Manual)" bgClass="bg-red-600 hover:bg-red-700 text-white shadow-md border border-red-800" onClick={stopMoving} disabled={!isConnected} />
        <ControlButton label="GESER KANAN ▶" bgClass="bg-cyan-600 hover:bg-cyan-700 text-white" onPointerDown={() => startMoving(0, -1.0, 0)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
      </div>
      
      {/* Tombol Mundur / Serong Mundur */}
      <div className="grid grid-cols-3 gap-2.5 mb-3">
        <ControlButton label="↙️ MUNDUR KIRI" bgClass="bg-cyan-600 hover:bg-cyan-700 text-white" onPointerDown={() => startMoving(-0.5, 0.5, 0)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
        <ControlButton label="▼ MUNDUR" bgClass="bg-cyan-600 hover:bg-cyan-700 text-white" onPointerDown={() => startMoving(-0.5, 0, 0)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
        <ControlButton label="MUNDUR KANAN ↘️" bgClass="bg-cyan-600 hover:bg-cyan-700 text-white" onPointerDown={() => startMoving(-0.5, -0.5, 0)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
      </div>

      {/* Tombol Rotasi */}
      <div className="grid grid-cols-2 gap-2.5 mb-6 mt-4">
        <ControlButton label="PUTAR KIRI (CCW)" bgClass="bg-indigo-500 hover:bg-indigo-600 text-white" onPointerDown={() => startMoving(0, 0, 0.5)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
        <ControlButton label="PUTAR KANAN (CW)" bgClass="bg-indigo-500 hover:bg-indigo-600 text-white" onPointerDown={() => startMoving(0, 0, -0.5)} onPointerUp={stopMoving} onPointerLeave={stopMoving} disabled={!isConnected} />
      </div>

      <h3 className="border-b border-gray-300 pb-1 mb-4 text-xl font-semibold">MEIHUA</h3>
      <div className="grid grid-cols-2 gap-2.5 mb-3">
        {/* Menggunakan onPointerDown dan onPointerUp agar Climb juga bersifat "tahan-untuk-gerak" */}
        <ControlButton label="MAJU" bgClass="bg-green-500 hover:bg-green-600 text-white" onPointerDown={() => publishMessage('/motor_climb', 'std_msgs/Float32', 2)} onPointerUp={stopClimb} onPointerLeave={stopClimb} disabled={!isConnected} />
        <ControlButton label="MUNDUR" bgClass="bg-green-500 hover:bg-green-600 text-white" onPointerDown={() => publishMessage('/motor_climb', 'std_msgs/Float32', 1)} onPointerUp={stopClimb} onPointerLeave={stopClimb} disabled={!isConnected} />
        <ControlButton label="ANGKAT" bgClass="bg-blue-500 hover:bg-blue-600 text-white" onPointerDown={() => publishMessage('/climb', 'std_msgs/Float32', 2)} onPointerUp={stopClimb} onPointerLeave={stopClimb} disabled={!isConnected} />
        <ControlButton label="TURUNKAN" bgClass="bg-blue-500 hover:bg-blue-600 text-white" onPointerDown={() => publishMessage('/climb', 'std_msgs/Float32', 1)} onPointerUp={stopClimb} onPointerLeave={stopClimb} disabled={!isConnected} />
      </div>
    </div>
  );
}

export default App;