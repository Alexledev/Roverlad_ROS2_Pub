
export class WebAPIClient {
    constructor() {
    
        this.speed = 0;
        this.rtcURL = "";
    }

    isOK(res, msg, displayPopup=false) {
        if (!res.ok) {
            if (displayPopup === true){
                alert(msg);
            }
            
            throw new Error(msg);        }
    }

    async onInit(callback) {
        const res = await fetch(`/api/init?timestamp=${Date.now()}`);
        this.isOK(res, "Init Failed");

        const data = await res.json();

        this.speed  = data["speed"];
        this.rtcURL = data["url_webRTC"];
        this.useSlam = data["use_slam"];

        callback(this.speed, this.rtcURL, this.useSlam);
    }

    async setSpeed(speed)
    {
        const res = await fetch(`/api/speed/setValue/${speed}`);
        this.isOK(res, "Set speed failed");
    }

    async saveSpeed(speed)
    {
        const res = await fetch(`/api/speed/save/${speed}`, {method: "PUT"});
        this.isOK(res, "Save speed failed");
    }

    async uploadMap(form) {
        const res = await fetch("/api/mapUpload", {
            method: "POST",
            body: form
        });
        this.isOK(res, "Upload failed");

        const data = await res.json();

        return data["extracted"];
    }

    async getMapsList() {
        const res = await fetch("/api/maps");

        this.isOK(res, "Couldn't get map list");

        const data = await res.json();
        return data["value"];
    }

    async deleteMap(name_map) {
        const res = await fetch(`/api/maps/${encodeURIComponent(name_map)}`, {method: "DELETE"});
        
        this.isOK(res, "Deletion failed");

        const got = await res.json();
        return got["value"] === true;
    }

    async getMapPreview(name_map) {
        const res = await fetch(`/api/mapPreview/${name_map}`);
        const blob = await res.blob();
        
        return URL.createObjectURL(blob);
    }

    async downloadMap(name_map) {
        const res = await fetch(`/api/mapDownload/${name_map}`);
        const blob = await res.blob();

        return URL.createObjectURL(blob);
    }

    async loadSettings() {
        const res = await fetch("/api/setting/load");

        this.isOK(res, "Couldn't load settings");

        const data = await res.json();
        return data;
    }

    async saveSettings(dataSettings) {
        const res = await fetch("/api/setting/save", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(dataSettings) 
        });
        this.isOK(res, "Save failed");

        const data = await res.json();
    }

    async requestReset(value) {

        const jsonObj = {password: value};
        const res = await fetch("/api/setting/requestReset", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(jsonObj) 
        });

        this.isOK(res, "Couldn't reset. Something went wrong.", true);
    }
}