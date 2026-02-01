import { WebAPIClient } from "./webAPIClient.js";
import { WebSocketClientMapping, WebSocketClientControl } from "./webSocketClient.js"
import { SpinnerControl, NavMenuControl } from "./UIControl.js";
import { VideoControls } from "./videoControls.js";

const webClient = new WebAPIClient();

function* iterXY(flat) {
    for (let i = 0; i < flat.length; i += 2) {
        yield [flat[i], flat[i + 1]];
    }
}

class ConfigControls {
  constructor() {
    this.fieldset = document.getElementById("rover_container_fieldset");

    this.slider = document.getElementById("speed");
    this.valueLabel = document.getElementById("value");
    this.log = document.getElementById("log");
    this.body = document.getElementsByTagName("body")[0];

    this.btn_reset  = document.getElementById("btn_reset");
    this.btn_save  = document.getElementById("btn_save");
    this.save_spinner = new SpinnerControl(this.btn_save);
    this.map_name = document.getElementById("map_name");

    this.btn_up    = document.getElementById("btn_up");
    this.btn_left  = document.getElementById("btn_left");
    this.btn_right = document.getElementById("btn_right");
    this.btn_down  = document.getElementById("btn_down");

    this.zoomMultiplier = 2.0;
    this.zoomIncrement = 0.2;
    this.zoom_in   = document.getElementById("zoom_in");
    this.zoom_out  = document.getElementById("zoom_out");
    this.canvasHolder = document.getElementById("canvas_holder");  

    this.slamIMG = null;
    this.slamIMGctx = null;  
    this.pressedKeys = {};

    this.saving_map = false
    this.currentInterval = undefined;

    this.currentSpeed = parseFloat(this.slider.value);

    this.slider.oninput = this.onchangeSpeed.bind(this);
    this.btn_save.onclick = this.onclickSaveMap.bind(this);

    this.canvasAsset = `<canvas id="map_canvas" class="map-canvas"></canvas>`;
    this.canvasExists = false;
  }

  initWebSockets() {
    this.webSocketClientControl = new WebSocketClientControl();
    this.webSocketClientControl.onMapSaved = this.onMapSaved.bind(this);
    this.webSocketClientControl.onReceiveMessage = this.onReceiveMessage.bind(this);

    this.webSocketClientMapping = new WebSocketClientMapping();
    this.webSocketClientMapping.updateSlamIMG = this.updateSlamIMG.bind(this);
  }

  init()
  {
    webClient.onInit((speed, urlRtc, useSlam)=>
      { 
        console.log(`slam: ${useSlam}`);       

        if (useSlam)
        {
          this.initWebSockets();

          this.setInitialSpeed(speed);
          this.fieldset.disabled = false;      

          const videoCtrl = new VideoControls(urlRtc, document.getElementById('start_stream'),
                                                      document.getElementById('stop_stream'),
                                                      document.getElementById('stream_status'),
                                                      document.getElementById("video_holder"));
        }
        else
        {
          this.fieldset.disabled = true;
          this.canvasHolder.innerHTML = "<h3>No SLAM Mode</h3>";       
        }

     });

    this.btn_reset.onclick = () => { this.webSocketClientControl.requestReset(); };

    this.map_name.addEventListener('input', () => { 
      this.btn_save.disabled = this.map_name.value.trim() === ''; 
    });

    this.registerButton(this.btn_up,    "btn_up");
    this.registerButton(this.btn_left,  "btn_left");
    this.registerButton(this.btn_right, "btn_right");
    this.registerButton(this.btn_down,  "btn_down");

    this.zoom_in.onclick = () => { this.zoomMultiplier += this.zoomIncrement; };

    this.zoom_out.onclick = () => {
      this.zoomMultiplier -= this.zoomIncrement;
      if (this.zoomMultiplier <= this.zoomIncrement)
      {
        this.zoomMultiplier = this.zoomIncrement;
      }
    }
  }

  onReceiveMessage(message) 
  {
    this.log.textContent += JSON.stringify(message) + "\n";
  }

  onMapSaved()
  {
    this.saving_map = true;

    this.save_spinner.stopSpin();
    this.btn_reset.disabled = false;
    this.btn_save.disabled = false;
    this.map_name.disabled = false;
    this.btn_save.classList.remove('waitButton');

  }

  initCanvasIfYes() {
    if (this.canvasExists)
    {
      return;
    }

    this.canvasExists = true;
    this.canvasHolder.innerHTML = this.canvasAsset;

    this.slamIMG = document.getElementById("map_canvas");
    this.slamIMGctx = this.slamIMG.getContext("2d");  
    this.slamIMGctx.imageSmoothingEnabled = false;    
  }

  async updateSlamIMG(imgBuffer, pose, points) {
    this.initCanvasIfYes();

    const bitmap = await createImageBitmap(imgBuffer);

    this.slamIMG.width = bitmap.width;
    this.slamIMG.height = bitmap.height;
    this.slamIMG.style.width = bitmap.width * this.zoomMultiplier + "px";
    this.slamIMG.style.height = bitmap.height * this.zoomMultiplier + "px";

    this.slamIMGctx.clearRect(0, 0, this.slamIMG.width, this.slamIMG.height);
    this.slamIMGctx.drawImage(bitmap, 0, 0);

    const [x, y, theta] = pose;

    this.slamIMGctx.save();
    this.drawRobot(x, y, theta);
    this.slamIMGctx.restore();

    // console.log(points);
    this.slamIMGctx.save();
    this.drawPCL(points);  
    this.slamIMGctx.restore();  

  }

  drawRobot(x, y, theta) {
    this.slamIMGctx.strokeStyle = "blue";
    this.slamIMGctx.fillStyle = "blue";
    this.slamIMGctx.lineWidth = 1;

    // body
    this.slamIMGctx.beginPath();
    this.slamIMGctx.arc(x, y, 3, 0, Math.PI * 2);
    this.slamIMGctx.fill();

    // heading
    const hx = x + Math.cos(theta) * 6;
    const hy = y + Math.sin(theta) * -6;

    this.slamIMGctx.beginPath();
    this.slamIMGctx.moveTo(x, y);
    this.slamIMGctx.lineTo(hx, hy);
    this.slamIMGctx.stroke();
  }

  drawPCL(points) {
    
    this.slamIMGctx.fillStyle = "red";

    for (const [x, y] of iterXY(points)) {
        this.slamIMGctx.fillRect(x - 1, y - 1, 1, 1);       
    }
  }

  async onchangeSpeed()
  {
    this.currentSpeed = parseFloat(this.slider.value);
    this.valueLabel.textContent = "...";
    
    await webClient.setSpeed(this.currentSpeed);

    this.valueLabel.textContent = this.currentSpeed.toFixed(2);
  }

  setInitialSpeed(speed)
  {
      this.currentSpeed = speed;

      this.slider.value = this.currentSpeed;
      this.valueLabel.textContent = this.currentSpeed.toFixed(2);
  } 

  onclickSaveMap()
  {
    const name = this.map_name.value;
    this.saving_map = true;

    this.save_spinner.setSpin("Saving...");
    this.btn_reset.disabled = true;
    this.btn_save.disabled = true;
    this.map_name.disabled = true;
    this.btn_save.classList.add('waitButton');

    this.webSocketClientControl.saveMap(name);
  }

  registerButton (button, btn_name)
  {
    button.addEventListener("pointerdown", (e) => {
      e.preventDefault(); // stop text selection
      this.pressInterval(btn_name);
    });

    button.addEventListener("pointerup", () => {
      this.stopMovement();
    });
    button.addEventListener("pointerleave", () => {
      this.stopMovement();
    });
    this.body.onmouseup = () => {
      this.stopMovement();
    };
  }

  stopMovement() {
    clearInterval(this.currentInterval);
  }

  pressInterval(btn_name) {
    this.currentInterval = setInterval(() => {
      this.webSocketClientControl.sendDpadInput(btn_name, this.currentSpeed);      
    }, 10);
  }
}

const confControl = new ConfigControls();
confControl.init();

const nvctrl = new NavMenuControl();



