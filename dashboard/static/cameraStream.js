
export class CameraStream {
  constructor (webRTCURL) {

    if (!webRTCURL)
        throw new Error("Need WebRTC URL");

    this.rtcURL = webRTCURL;

    this.peerConnection = null;
    this.videoObj = null;
  }
        
  bindVideo (videoObj) {
    this.videoObj = videoObj;
  }

  onVideoStarting = () => {}
  onVideoConnected = () => {}
  onVideoStarted = () => {}
  onConnectionStateChange = (state) => {}

  async startRecording() {    
    this.onVideoStarting();

    this.peerConnection = new RTCPeerConnection();

    this.peerConnection.addTransceiver("video", { direction: "recvonly" });
    this.peerConnection.addTransceiver("audio", { direction: "recvonly" });

    this.peerConnection.addEventListener('track', (evt) => {
      if (!this.videoObj)
        throw new Error("No video element provided");

      let s = evt.streams[0];
      if (evt.track.kind === 'video') {        
          this.videoObj.srcObject = s;
          this.onVideoConnected();
      }
    });

    this.peerConnection.addEventListener('connectionstatechange', () => {
        this.onConnectionStateChange(this.peerConnection.connectionState);
    });

    // Create offer
    const offer = await this.peerConnection.createOffer();
    await this.peerConnection.setLocalDescription(offer);

    // Send offer to server
    // 
    const response = await fetch(this.rtcURL, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            sdp: this.peerConnection.localDescription.sdp,
            type: this.peerConnection.localDescription.type
        })
    });

    const answer = await response.json();
    await this.peerConnection.setRemoteDescription(answer);    

    this.onVideoStarted();
  }

  async stopRecording() {
    if (this.peerConnection) {
        this.peerConnection.close();
        this.peerConnection = null;
    }
  }
}

   