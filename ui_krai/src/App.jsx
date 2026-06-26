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

// 2. Komponen Tombol Kontrol (Menggunakan Tailwind)
function ControlButton({ label, onClick, bgClass = 'bg-gray-200 hover:bg-gray-300 text-black', disabled = false }) {
  return (
    <button 
      onClick={onClick} 
      disabled={disabled}
      className={`p-3 text-[15px] font-bold rounded-md transition-all shadow-sm duration-200
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

  const [telemetry, setTelemetry] = useState({
    sudutRobot: '0',
    sensorKiri: '0',
    lowLevel: [],
    baterai: '0',
    capitSenjata: '0',
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
    const subCapit = new ROSLIB.Topic({ ros, name: '/capit_cmd', messageType: 'std_msgs/Float32' });

    subDepan.subscribe((msg) => setTelemetry(prev => ({ ...prev, sudutRobot: msg.data })));
    subKiri.subscribe((msg) => setTelemetry(prev => ({ ...prev, sensorKiri: msg.data })));
    subBaterai.subscribe((msg) => setTelemetry(prev => ({ ...prev, baterai: msg.data })));
    subLowLevel.subscribe((msg) => setTelemetry(prev => ({ ...prev, lowLevel: msg.data })));
    subCapit.subscribe((msg) => setTelemetry(prev => ({ ...prev, capitSenjata: msg.data })));

    return () => {
      subDepan.unsubscribe(); subKiri.unsubscribe(); subBaterai.unsubscribe(); subLowLevel.unsubscribe(); subCapit.unsubscribe();
      if (rosRef.current) rosRef.current.close();
    };
  }, []);
  console.log(telemetry.capitSenjata);
  

  const publishMessage = (topicName, payload) => {
    if (!isConnected || !rosRef.current) return;
    const topic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: 'std_msgs/String'
    });
    topic.publish({ data: payload });
  };

  const toggleCapit = () => {
    const newCapitState = !capitState;
    setCapit(newCapitState);
    publishMessage('/capit_cmd', newCapitState ? 1 : 0);
  }
  
  return (
    <div className="p-8 font-sans max-w-[700px] mx-auto text-gray-800">
      <header className="text-center mb-[30px]">
        <h2 className="text-4xl font-bold mb-2 text-gray-900">KRAI Scalable Dashboard</h2>
        <span className={`font-bold ${isConnected ? 'text-green-600' : 'text-red-600'}`}>
          {isConnected ? '🟢 Connected to Robot' : '🔴 Disconnected'}
        </span>
      </header>

      {/* Grid Monitoring */}
      <section className="grid grid-cols-3 gap-[15px] mb-[30px]">
        <SensorCard title="Sudut Robot" value={telemetry.sudutRobot} unit="deg" />
        <SensorCard title="Capit" value={telemetry.capitSenjata} unit="" />
        <SensorCard title="Daya Baterai" value={telemetry.baterai} unit="%" colorClass="text-green-500" />
      </section>

      <section className="grid grid-cols-4 gap-[15px] mb-[30px]">
        <SensorCard title="Kiri Belakang" value={telemetry.lowLevel[1]} unit="" />
        <SensorCard title="Kiri Depan" value={telemetry.lowLevel[3]} unit="" />
        <SensorCard title="Kanan Depan" value={telemetry.lowLevel[0]} unit="" />
        <SensorCard title="Kanan Belakang" value={telemetry.lowLevel[2]} unit="" />
      </section>

      {/* Kontrol Roda */}
      <h3 className="border-b border-gray-300 pb-1 mb-4 text-xl text-white font-semibold">Capit</h3>
      <div className="grid grid-cols-3 gap-2.5 mb-6">
        <ControlButton label="Capit" bgClass={`${!capitState ? 'bg-green-500' : 'bg-gray-500'} hover:bg-green-600 text-white`} onClick={() => toggleCapit()} disabled={!isConnected} />
      </div>
      {/* Kontrol Roda */}
      <h3 className="border-b border-gray-300 pb-1 mb-4 text-xl text-white font-semibold">🎮 Pergerakan Roda</h3>
      <div className="grid grid-cols-3 gap-2.5 mb-6">
        <div/>
        <ControlButton label="MAJU" bgClass="bg-green-500 hover:bg-green-600 text-white" onClick={() => publishMessage('/perintah_gerak', 'Maju')} disabled={!isConnected} />
        <div/>
        <ControlButton label="KIRI" bgClass="bg-blue-500 hover:bg-blue-600 text-white" onClick={() => publishMessage('/perintah_gerak', 'Kiri')} disabled={!isConnected} />
        <ControlButton label="STOP" bgClass="bg-red-500 hover:bg-red-600 text-white" onClick={() => publishMessage('/perintah_gerak', 'Stop')} disabled={!isConnected} />
        <ControlButton label="KANAN" bgClass="bg-blue-500 hover:bg-blue-600 text-white" onClick={() => publishMessage('/perintah_gerak', 'Kanan')} disabled={!isConnected} />
      </div>

      {/* Mekanisme/Sistem */}
      <h3 className="border-b border-gray-300 pb-1 mb-4 text-xl font-semibold">⚙️ Mekanisme Robot</h3>
      <div className="grid grid-cols-2 gap-2.5">
        <ControlButton label="Reset System" bgClass="bg-gray-600 hover:bg-gray-700 text-white" onClick={() => publishMessage('/mode_robot', 'Reset')} disabled={!isConnected} />
        <ControlButton label="Yoo" bgClass="bg-orange-500 hover:bg-orange-600 text-white" onClick={() => publishMessage('/kontrol_aktuator', 'Tembak')} disabled={!isConnected} />
      </div>
    </div>
  );
}

export default App;