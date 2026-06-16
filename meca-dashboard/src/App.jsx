import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import axios from 'axios';
import './App.css'; 

function App() {
  // --- STATES ---
  const [rosStatus, setRosStatus] = useState("Connecting to ROS 2...");
  const [robotPose, setRobotPose] = useState({ x: 0.0, y: 0.0, theta: 0.0 });
  const [robotVel, setRobotVel] = useState({ linear: 0.0, angular: 0.0 });
  
  const [chatLog, setChatLog] = useState([{ sender: "System", text: "Meca Station is online." }]);
  const [inputText, setInputText] = useState("");
  const chatEndRef = useRef(null);

  const rosRef = useRef(null);
  const cmdVelRef = useRef(null);


  const [isListening, setIsListening] = useState(false);
  // --- HÀM XỬ LÝ NHẬN DIỆN GIỌNG NÓI ---
  const startVoiceRecognition = () => {
  // Kiểm tra trình duyệt có hỗ trợ không
  const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  
  if (!SpeechRecognition) {
    alert("Trình duyệt của bạn không hỗ trợ nhận diện giọng nói. Hãy dùng Chrome!");
    return;
  }

  const recognition = new SpeechRecognition();
  recognition.lang = 'en-US'; // Thiết lập nhận diện Tiếng Anh
  recognition.interimResults = false;
  recognition.maxAlternatives = 1;

  recognition.onstart = () => {
    setIsListening(true);
  };

  recognition.onresult = (event) => {
    const transcript = event.results[0][0].transcript;
    setInputText(transcript); // Đưa câu vừa nói vào ô nhập liệu
    
    // Tự động gửi câu lệnh đi luôn
    sendVoiceCommand(transcript);
  };

  recognition.onerror = (event) => {
    console.error("Lỗi Voice:", event.error);
    setIsListening(false);
  };

  recognition.onend = () => {
    setIsListening(false);
  };

  recognition.start();
  };

// Hàm phụ để gửi lệnh ngay sau khi nhận diện xong
const sendVoiceCommand = async (text) => {
  setChatLog(prev => [...prev, { sender: "User", text: text }]);
  try {
    const res = await axios.post('http://127.0.0.1:8000/chat', { message: text });
    setChatLog(prev => [...prev, { sender: "Meca AI", text: res.data.reply }]);
  } catch (e) {
    setChatLog(prev => [...prev, { sender: "System", text: "AI Server Error!" }]);
  }
};
  // --- ROS 2 CONNECTION & TELEMETRY ---
  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://127.0.0.1:9090' });
    rosRef.current = ros;

    ros.on('connection', () => setRosStatus("✅ Connected to Meca-Core"));
    ros.on('error', () => setRosStatus("❌ ROS 2 Connection Error"));
    ros.on('close', () => setRosStatus("⚠️ Disconnected"));

    cmdVelRef.current = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    const poseListener = new ROSLIB.Topic({
      ros: ros,
      name: '/amcl_pose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    });
    poseListener.subscribe((msg) => {
      const q = msg.pose.pose.orientation;
      const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * (180 / Math.PI);
      setRobotPose({ x: msg.pose.pose.position.x.toFixed(2), y: msg.pose.pose.position.y.toFixed(2), theta: theta.toFixed(1) });
    });

    const velListener = new ROSLIB.Topic({
      ros: ros,
      name: '/odom_encoder', 
      messageType: 'nav_msgs/Odometry'
    });
    velListener.subscribe((msg) => {
      setRobotVel({
        linear: msg.twist.twist.linear.x.toFixed(2),
        angular: msg.twist.twist.angular.z.toFixed(2)
      });
    });

    const progressListener = new ROSLIB.Topic({
      ros: ros,
      name: '/ai_progress', 
      messageType: 'std_msgs/msg/String'
    });
    
    progressListener.subscribe((msg) => {
      // Khi nhận được thông báo, tự động đẩy vào khung Chat với tên "System"
      setChatLog(prev => [...prev, { sender: "System", text: msg.data }]);
    });

    return () => {
      poseListener.unsubscribe();
      velListener.unsubscribe();
      progressListener.unsubscribe(); // Nhớ thêm dòng gỡ kết nối này
      ros.close();
    };
  }, []);

  useEffect(() => { chatEndRef.current?.scrollIntoView({ behavior: "smooth" }); }, [chatLog]);

  // --- HANDLERS ---
  const sendDriveCommand = (lx, ly, az) => {
    axios.post('http://127.0.0.1:8000/teleop', {
      linear_x: lx,
      linear_y: ly,
      angular_z: az
    }).catch(err => console.error("Lỗi Teleop:", err));
  };

  const handleStopRobot = () => sendDriveCommand(0.0, 0.0, 0.0);

  const handleSendChat = async () => {
    if (!inputText.trim()) return;
    setChatLog(prev => [...prev, { sender: "User", text: inputText }]);
    const currentMsg = inputText;
    setInputText(""); 
    try {
      const res = await axios.post('http://127.0.0.1:8000/chat', { message: currentMsg });
      setChatLog(prev => [...prev, { sender: "Meca AI", text: res.data.reply }]);
    } catch (e) {
      setChatLog(prev => [...prev, { sender: "System", text: "AI Server Error!" }]);
    }
  };

  const handleOpenRViz = async () => {
    try {
      await axios.post('http://127.0.0.1:8000/launch-rviz');
    } catch (e) {
      alert("Cannot connect to Python Backend (Port 8000)");
    }
  };


  // --- UI RENDER ---
  return (
    <div className="app-wrapper">
      
      {/* HEADER NẰM TRÊN CÙNG TÁCH BIỆT */}
      <header className="dashboard-header">
        <div className="header-left">
          <img src="/logo-bk.png" alt="Logo Bách Khoa" className="bk-logo" />
          <div className="title-group">
            <h1>MECA STATION</h1>
            <p className="subtitle">Đồ án: Hệ thống Robot Mecanum tích hợp SLAM và LLM</p>
          </div>
        </div>
        
        <div className="header-right">
          <div className="student-info">
            <p><span className="label">SVTH:</span> Lê Văn Nhật</p>
            <p><span className="label">MSSV:</span> 2212393</p>
          </div>
        </div>
      </header>

      {/* KHU VỰC NỘI DUNG CHÍNH NẰM BÊN DƯỚI (CHIA 2 CỘT) */}
      <div className="main-content">
        
        {/* LEFT PANEL: TELEMETRY & TELEOP */}
        <div className="panel left-panel">
          <h2 className="panel-title">TELEMETRY</h2>
          <div className="title-divider"></div>
          
          <div className="status-box"><strong>Status:</strong> <span className="highlight">{rosStatus}</span></div>
          
          <div className="telemetry-grid">
            <div className="data-card">
              <h4>Position X / Y</h4>
              <p className="data-value">{robotPose.x}m / {robotPose.y}m</p>
            </div>
            <div className="data-card">
              <h4>Yaw Angle</h4>
              <p className="data-value">{robotPose.theta}°</p>
            </div>
            <div className="data-card">
              <h4>Linear Vel</h4>
              <p className="data-value">{robotVel.linear} m/s</p>
            </div>
            <div className="data-card">
              <h4>Angular Vel</h4>
              <p className="data-value">{robotVel.angular} rad/s</p>
            </div>
          </div>

          <h2 className="panel-title" style={{marginTop: '25px'}}> MECANUM CONTROLS MANUAL</h2>
          <div className="title-divider"></div>
          
          <div className="teleop-pad">
            <div className="row">
              <button onMouseDown={()=>sendDriveCommand(0.2, 0.2, 0)} onMouseUp={handleStopRobot}>↖</button>
              <button onMouseDown={()=>sendDriveCommand(0.2, 0.0, 0)} onMouseUp={handleStopRobot}>⬆ Forward</button>
              <button onMouseDown={()=>sendDriveCommand(0.2, -0.2, 0)} onMouseUp={handleStopRobot}>↗</button>
            </div>
            <div className="row">
              <button onMouseDown={()=>sendDriveCommand(0.0, 0.2, 0)} onMouseUp={handleStopRobot}>⬅ Strafe L</button>
              <button className="stop-btn" onClick={handleStopRobot}>🛑 STOP</button>
              <button onMouseDown={()=>sendDriveCommand(0.0, -0.2, 0)} onMouseUp={handleStopRobot}>Strafe R ➡</button>
            </div>
            <div className="row">
              <button onMouseDown={()=>sendDriveCommand(0.0, 0.0, 0.5)} onMouseUp={handleStopRobot}>↺ Rotate</button>
              <button onMouseDown={()=>sendDriveCommand(-0.2, 0.0, 0)} onMouseUp={handleStopRobot}>⬇ Backward</button>
              <button onMouseDown={()=>sendDriveCommand(0.0, 0.0, -0.5)} onMouseUp={handleStopRobot}>Rotate ↻</button>
            </div>
          </div>
        </div>

        {/* RIGHT PANEL: AI COMMANDER & RVIZ BUTTON */}
        <div className="panel right-panel">
          <div className="tools-header">
            <h2 className="panel-title">AI COMMANDER</h2>
            <button className="rviz-btn" onClick={handleOpenRViz}>
                LAUNCH RVIZ2
            </button>
          </div>
          
          <div className="chat-window">
            {chatLog.map((msg, idx) => (
              <div key={idx} className={`chat-msg ${msg.sender === 'User' ? 'my-msg' : (msg.sender === 'System' ? 'sys-msg' : 'ai-msg')}`}>
                <strong>{msg.sender}:</strong> <span>{msg.text}</span>
              </div>
            ))}
            <div ref={chatEndRef} />
          </div>
          <div className="chat-input-area">
            <input 
              type="text" value={inputText} 
              onChange={e => setInputText(e.target.value)} 
              onKeyDown={e => e.key === 'Enter' && handleSendChat()}
              placeholder="Command the robot..." 
            />
              {/* NÚT MICRO MỚI */}
              <button 
              className={`mic-btn ${isListening ? 'listening' : ''}`} 
              onClick={startVoiceRecognition}
              title="Voice Command"
                >
                {isListening ? '🛑' : '🎤'}
            </button>
            <button onClick={handleSendChat}>SEND</button>
          </div>
        </div>
        
      </div> {/* Kết thúc main-content */}
    </div> /* Kết thúc app-wrapper */
  );
}

export default App;