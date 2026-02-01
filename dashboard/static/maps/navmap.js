import { WebAPIClient } from "../webAPIClient.js";
import { WebSocketClientNavMap } from "../webSocketClient.js";
import { SpinnerControl, NavMenuControl } from "../UIControl.js";
import { VideoControls } from "../videoControls.js";

const webClient = new WebAPIClient();

class Navmap {
    ws;
    img;
    cont;
    flag;
    clone;

    constructor() {
        this.img = document.getElementById("map_stream");
        this.cont = document.getElementsByClassName("img-container")[0];
        this.flag = document.getElementsByClassName("flag-pole")[0];

        this.onMouseMove = (e) => this.moveFlag(e);
        this.onMouseUp   = (e) => this.drop(e);        
    } 
    
    init() {  
        this.flag.addEventListener("mousedown", (e) => {
            this.flag.classList.add("drag-clone");

            this.moveFlag(e);

            this.cont.addEventListener("mousemove", this.onMouseMove);
            this.cont.addEventListener("mouseup", this.onMouseUp);
        });
    }

    moveFlag(e) {

        // this.setFlagPos(e.clientX, e.clientY);
        self.rect = this.img.getBoundingClientRect();
        const x = e.clientX - self.rect.left;
        const y = e.clientY - self.rect.top;
        this.setFlagPos(x, y);
    }

    setFlagPos(x, y) {
        this.flag.style.setProperty("--x", `${x}px`);
        this.flag.style.setProperty("--y", `${y}px`);
    }

    setFlagPosNavCoord(posX, posY) {
        const {x, y} = this.navCoordToMap(posX, posY)
        this.setFlagPos(x, y)
    }

    setGoalOnMap = (data) => {}

    drop(e) {
        this.cont.removeEventListener("mousemove", this.onMouseMove);
        this.cont.removeEventListener("mouseup", this.onMouseUp);

        // DROP EVENT
        const x = e.clientX - self.rect.left;
        const y = e.clientY - self.rect.top;

        const scaleX = this.img.naturalWidth /  self.rect.width;
        const scaleY = this.img.naturalHeight /  self.rect.height;

        const imgX = Math.round(x * scaleX);
        const imgY = Math.round(y * scaleY);

        // webSocket.setGoalOnMap([imgX, imgY, this.img.naturalWidth, this.img.naturalHeight]);
        this.setGoalOnMap([imgX, imgY, this.img.naturalWidth, this.img.naturalHeight]);

        this.flag.classList.remove("drag-clone");
    }

    navCoordToMap(imgX, imgY) {
        self.rect = this.img.getBoundingClientRect();
        const x = imgX * self.rect.width  / this.img.naturalWidth;
        const y = imgY * self.rect.height / this.img.naturalHeight;
        return {x, y}
    }


}


function* iterXY(flat) {
    for (let i = 0; i < flat.length; i += 2) {
        yield [flat[i], flat[i + 1]];
    }
}

class MapImageStream {
    constructor() {
        this.fieldset = document.getElementById("rover_container_fieldset");

        this.imageContainer = document.getElementById("img_container");

        this.img = document.getElementById("map_stream");
        this.btn = document.getElementById("btn_start");
        this.startSpinner = new SpinnerControl(this.btn);

        this.robot = document.getElementsByClassName("circle-wrap")[0];
        this.canvas = document.getElementById("pointcloud");       
        this.log = document.getElementById("log");
        this.webSocket = null;

        this.navmap = null;
    } 

    initWebSocket() {  
        this.webSocket = new WebSocketClientNavMap();
        this.webSocket.dataPosAndPointcloud = this.dataPosAndPointcloud.bind(this);
        this.webSocket.dataStartup = this.dataStartup.bind(this);
        this.webSocket.checkGoal = this.checkGoal.bind(this);
        this.webSocket.reloadMap = this.reloadMap.bind(this);
        this.webSocket.startFinish = this.startFinish.bind(this);
        this.webSocket.setGoalMarker = this.setGoalMarker.bind(this);
    }

    init() {
        this.btn.onclick = () => {
            this.startSpinner.setSpin();
            this.webSocket.requestStart();
        }

        webClient.onInit((speed, urlRtc, useSlam)=>
        { 
            console.log(`slam: ${useSlam}`);     
            if (useSlam)
            {
                this.fieldset.disabled = true;
                this.imageContainer.innerHTML = "<h3>No Navigation Mode</h3>"
            }
            else
            {
                const videoCtrl = new VideoControls(urlRtc, document.getElementById('start_stream'),
                                                        document.getElementById('stop_stream'),
                                                        document.getElementById('stream_status'),
                                                        document.getElementById("video_holder"));
                this.initWebSocket();                
                this.navmap = new Navmap();
                this.navmap.init();
                this.navmap.setGoalOnMap = (data) => {
                    this.webSocket.setGoalOnMap(data);
                }

                this.fieldset.disabled = false;
            }
            
        });
    }

    reloadCallback = () => {}

    setGoalMarker (x, y)
    {
        this.navmap.setFlagPosNavCoord(x, y);
    }

    dataPosAndPointcloud(pose, points)
    {
        this.bindDataToPos(pose);
        this.drawPointCloud(points);
    }

    dataStartup(data)
    {
        let start = data[0];
        let imgName = data[1];

        this.bindDataToImg(imgName);

        this.btn.disabled = start == true;
    }

    startFinish()
    {
        console.log("finish");
        this.startSpinner.stopSpin(true);
    }

    checkGoal(data)
    {
        let dist = data[0];
        let finished = data[1];

        if (dist === 0 && finished === true)
        {
            this.log.textContent += "Invalid Goal" + "\n";
        }
        else if (dist > 0 && finished === true)
        {
            this.log.textContent += "Reached Goal" + "\n";
        }
    }

    reloadMap()
    {
        console.log("Reloaded at MapImageStream");
        this.bindDataToImg("current_map.png");
        this.btn.disabled = false;
        this.reloadCallback();
    }

    bindDataToImg(imgName) {
        this.img.src = `/static/maps/working_maps/${imgName}?timestamp=${Date.now()}`;

        this.img.onload = () => {
            this.canvas.width = this.img.width;
            this.canvas.height = this.img.height;

            this.ctx = this.canvas.getContext("2d");
            this.ctx.fillStyle = "#FF0000";
        };
    }

    drawPointCloud(points) {
        if (!this.ctx)
        {
            return;
        }

        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        for (const [x, y] of iterXY(points)) {
            this.mapToScreen(x, y, this.img, ({x, y}) => {
                this.ctx.beginPath();
                this.ctx.arc(x, y, 2, 0, Math.PI * 2); // radius = 2px
                this.ctx.fill();
            });            
        }
    }

    bindDataToPos(data) {
        this.mapToScreen(data[0], data[1], this.img, ({x, y}) => {
            let r = -1 * data[2];
            this.robot.style.transform = `translate(${x}px, ${y}px) rotate(${r}rad)`;

        });
    }

    mapToScreen(px, py, imgEl, cb) {
        if (imgEl.naturalWidth === 0 || imgEl.naturalHeight === 0) {
            setTimeout(() => this.mapToScreen(px, py, imgEl, cb), 100);
            return;
        }

        const rect = imgEl.getBoundingClientRect();

        cb({
            x: Math.round(px * rect.width  / imgEl.naturalWidth),
            y: Math.round(py * rect.height / imgEl.naturalHeight)
        });
    }

}

let mapStream = new MapImageStream();
mapStream.init();

class MapsList {
    constructor(mapImgStream) {
        if (!(mapImgStream instanceof MapImageStream)) {
            throw new TypeError("Expected MapImageStream");
        }
        this.mapImgStream = mapImgStream;
        this.mapImgStream.reloadCallback = this.reloadMap.bind(this);
        
        this.btn_getList = document.getElementById("btn_mapls");
        this.mapsSpinner = new SpinnerControl(this.btn_getList);

        this.btn_select = document.getElementById('btn_mapslt');
        this.selectionSpinner = new SpinnerControl(this.btn_select);

        this.btn_delete = document.getElementById('btn_delete');
        this.deletionSpinner = new SpinnerControl(this.btn_delete);

        this.maplist = document.getElementById('mapList');
        this.title = document.getElementById('Title');
        this.previewImg = document.getElementById('previewImg');  
        this.btn_download = document.getElementById('btn_download');
        this.fileInput = document.getElementById('fileInput');
        this.btn_upload = document.getElementById('btn_upload');

        this.selected_map = "";
        this.using_map = "hall_lite"

        this.btn_getList.onclick  = this.requestMapsList.bind(this);
        this.btn_select.onclick   = this.onSelectMap.bind(this);
        this.btn_delete.onclick   = this.onDeleteMap.bind(this);
        this.btn_download.onclick = this.onDownloadMap.bind(this);
        this.btn_upload.onclick   = this.onUploadMap.bind(this);

        this.fileInput.addEventListener("change", () => {
            this.btn_upload.disabled = this.fileInput.files.length === 0;
        });
    }

    reloadMap () {
        console.log("Reloaded at MapsList");
        this.selectionSpinner.stopSpin();
    }


    onUploadMap() {
        this.btn_upload.disabled = true;
        this.upload();
    }

    async onDownloadMap() {
        this.btn_download.disabled = true;

        const url = await webClient.downloadMap(this.selected_map);
        const a = document.createElement("a");
        a.href = url;
        a.download = `${this.selected_map}.zip`;
        a.click();
        window.URL.revokeObjectURL(url);
    }

    onDeleteMap(e) {
        if (!confirm("Are you sure?")) {
            e.preventDefault();
            return;
        }

        this.btn_delete.disabled = true;
        this.deleteSelectedMap();
    }

    onSelectMap() {
        this.btn_select.disabled = true;
        this.using_map = this.selected_map;
        this.title.textContent = this.selected_map;

        this.selectionSpinner.setSpin();
        this.mapImgStream.webSocket.setMap(this.selected_map);
    }

    async requestMapsList() {
        this.selected_map = "";
        this.mapsSpinner.setSpin();

        let names = await webClient.getMapsList();
        this.updateMapList(names);

        this.mapsSpinner.stopSpin();
    }

    async upload() {
        let file = this.fileInput.files[0];

        const form = new FormData();
        if (this.selected_map && this.selected_map !== "")
        {
            form.append("file", file, this.selected_map);            
        }
        else
        {
            form.append("file", file);
        }

        if (await webClient.uploadMap(form))
        {
            this.requestMapsList();
        }
        this.btn_upload.disabled = false;
    }

    updateMapList(names){
        this.maplist.innerHTML = '';

        for (const name of names)
        {
            const li = document.createElement('li');
            li.classList.add("list-group-item");
            li.classList.add("list-group-item-action")
            li.textContent = name;
            li.onclick = (e) => {
                this.onMapSelect(li);
                this.selected_map = name;
                this.btn_download.disabled = false;
            }            
            this.maplist.appendChild(li);                        
        }
    }

    async onMapSelect(li){
        let name = li.textContent;

        for (const m of this.maplist.children)
        {
            m.classList.remove("active");
        }

        li.classList.add("active");  

        this.bindPreviewImg(name);

        if (name != this.using_map)
            this.btn_delete.disabled = false;
        else
            this.btn_delete.disabled = true;
    }

    async bindPreviewImg(name) {
       
        this.previewImg.src = await webClient.getMapPreview(name);
        this.btn_select.disabled = false;
    }

    async deleteSelectedMap() {
        this.deletionSpinner.setSpin();

        if (await webClient.deleteMap(this.selected_map))
        {
            this.requestMapsList();
            this.previewImg.src = "/static/maps/placeholder.jpg";

            this.deletionSpinner.stopSpin(true);
        } 
    }
}

let mapsList = new MapsList(mapStream);
mapsList.requestMapsList();

const nvctrl = new NavMenuControl();

