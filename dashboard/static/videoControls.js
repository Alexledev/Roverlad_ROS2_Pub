import { SpinnerControl } from "./UIControl.js";
import { CameraStream } from "./cameraStream.js"

export class VideoControls {
  constructor (webRTCURL, btnStart, btnStop, status, videoHolder) {   

    if (!btnStart || !btnStop || !status || !videoHolder)
        throw new Error("Missing a needed element for video init");


    this.btnStart = btnStart;
    this.startSpinner = new SpinnerControl(this.btnStart);

    this.btnStop = btnStop;
    this.status = status;    
    this.videoHolder = videoHolder;

    this.videoAsset = `<video id="video_stream" width="320" height="240" autoplay playsinline controls></video>`;
    this.video_stream = null;

    this.btnStart.onclick = this.onStart.bind(this);
    this.btnStop.onclick = this.onStop.bind(this);
    this.camStream = new CameraStream(webRTCURL);    

    this.camStream.onVideoStarting = this.onVideoStarting.bind(this);
    this.camStream.onVideoStarted = this.onVideoStarted.bind(this);
    this.camStream.onConnectionStateChange = this.onConnectionStateChange.bind(this);
  }

  updateStatus(msg) {
    this.status.textContent = 'Video: ' + msg;
  }

  onVideoStarting () {
    this.btnStart.disabled = true;
    this.updateStatus('Connecting...');
    this.startSpinner.setSpin();
  }

  onVideoStarted () {
    this.btnStop.disabled = false;  
    this.startSpinner.stopSpin(true);  
  }

  onConnectionStateChange (connectionState) {
    this.updateStatus(connectionState);
    if (connectionState === 'failed' ||  connectionState === 'disconnected' ||  connectionState === 'closed') {
        this.btnStart.disabled = false;
        this.btnStop.disabled = true;
    }
  }

  initVideoIfYes() {
    if (!this.video_stream)
      this.videoHolder.innerHTML = this.videoAsset;

    this.video_stream = document.getElementById("video_stream");   
    this.video_stream.style.display = "block";

    this.video_stream.onenterpictureinpicture = this.onEnterPIP.bind(this);
    this.video_stream.onleavepictureinpicture = this.onLeavePIP.bind(this);
  }

  onEnterPIP() {

    if (!this.video_stream.srcObject) return;
    this.video_stream.style.display = "none";
  }

  onLeavePIP() {
    if (!this.video_stream.srcObject) return;
    this.video_stream.style.display = "block";
  }

  onStart () {
    this.initVideoIfYes();
    this.camStream.bindVideo(this.video_stream);

    this.camStream.startRecording();
  }

  onStop () {
    this.camStream.stopRecording();

    this.video_stream.srcObject = null;
    this.video_stream.style.visibility = "hidden";

    if (document.pictureInPictureElement) {
        document.exitPictureInPicture();
    }

    this.btnStart.disabled = false;
    this.btnStop.disabled = true;

    this.updateStatus('Disconnected');
  }
}