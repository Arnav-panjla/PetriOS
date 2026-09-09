#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

// =====================================================
// WIFI
// =====================================================

const char* WIFI_SSID = "Manashvi";
const char* WIFI_PASSWORD = "12345678";

WebServer server(80);


// =====================================================
// AI THINKER ESP32-CAM
// =====================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0

#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// =====================================================
// WEB PAGE
// =====================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<title>ESP32-CAM QR Scanner</title>

<style>

body {
    background: black;
    color: white;
    font-family: Arial;
    text-align: center;
    margin: 0;
    padding: 20px;
}

#cameraBox {
    position: relative;
    display: inline-block;
}

#video {
    width: 640px;
    max-width: 90vw;
    display: block;
}

#overlay {
    position: absolute;
    left: 0;
    top: 0;
    width: 100%;
    height: 100%;
    pointer-events: none;
}

#result {
    margin: 20px auto;
    padding: 15px;
    background: #222;
    width: 640px;
    max-width: 85vw;
    box-sizing: border-box;
    border-radius: 10px;
    word-break: break-all;
}

#qrText {
    margin-top: 10px;
    color: lime;
    font-size: 20px;
}

#status {
    margin: 10px;
    font-size: 18px;
}

</style>

</head>

<body>

<h1>ESP32-CAM QR Scanner</h1>

<div id="status">
Starting camera...
</div>

<div id="cameraBox">

<img
    id="video"
    src="/stream"
>

<canvas
    id="overlay">
</canvas>

</div>

<div id="result">

<b>Detected QR:</b>

<div id="qrText">
None
</div>

</div>


<script src="https://cdn.jsdelivr.net/npm/jsqr@1.4.0/dist/jsQR.js"></script>


<script>

var video = document.getElementById("video");

var overlay = document.getElementById("overlay");

var ctx = overlay.getContext("2d");

var statusText = document.getElementById("status");

var qrText = document.getElementById("qrText");

var scanCanvas = document.createElement("canvas");

var scanCtx = scanCanvas.getContext("2d");

var lastQR = "";

var lastSendTime = 0;


// =====================================================
// START
// =====================================================

setTimeout(function() {

    if (typeof jsQR === "undefined") {

        statusText.innerHTML =
            "QR decoder failed to load";

        return;
    }

    statusText.innerHTML =
        "Scanning for QR code...";

    scanQR();

}, 2000);


// =====================================================
// SCAN
// =====================================================

function scanQR() {

    if (video.naturalWidth == 0 ||
        video.naturalHeight == 0) {

        statusText.innerHTML =
            "Waiting for camera...";

        setTimeout(scanQR, 500);

        return;
    }


    var width = video.naturalWidth;

    var height = video.naturalHeight;


    // Set canvas size

    if (scanCanvas.width != width ||
        scanCanvas.height != height) {

        scanCanvas.width = width;
        scanCanvas.height = height;

        overlay.width = width;
        overlay.height = height;
    }


    // Copy camera image

    scanCtx.drawImage(
        video,
        0,
        0,
        width,
        height
    );


    // Get pixels

    var imageData;

    try {

        imageData =
            scanCtx.getImageData(
                0,
                0,
                width,
                height
            );

    } catch(error) {

        console.log(error);

        setTimeout(scanQR, 200);

        return;
    }


    // =================================================
    // DECODE QR
    // =================================================

    var code = jsQR(
        imageData.data,
        width,
        height,
        {
            inversionAttempts: "attemptBoth"
        }
    );


    // Clear previous box

    ctx.clearRect(
        0,
        0,
        width,
        height
    );


    // =================================================
    // QR FOUND
    // =================================================

    if (code) {

        statusText.innerHTML =
            "QR CODE DETECTED";


        // Draw green box

        ctx.beginPath();

        ctx.moveTo(
            code.location.topLeftCorner.x,
            code.location.topLeftCorner.y
        );

        ctx.lineTo(
            code.location.topRightCorner.x,
            code.location.topRightCorner.y
        );

        ctx.lineTo(
            code.location.bottomRightCorner.x,
            code.location.bottomRightCorner.y
        );

        ctx.lineTo(
            code.location.bottomLeftCorner.x,
            code.location.bottomLeftCorner.y
        );

        ctx.closePath();

        ctx.lineWidth = 6;

        ctx.strokeStyle = "lime";

        ctx.stroke();


        // QR label

        ctx.fillStyle = "lime";

        ctx.font = "bold 18px Arial";

        ctx.fillText(
            "QR DETECTED",
            code.location.topLeftCorner.x,
            Math.max(
                22,
                code.location.topLeftCorner.y - 8
            )
        );


        // Display QR content

        qrText.innerHTML =
            code.data;


        // =================================================
        // SEND TO ESP32
        // =================================================

        var now = Date.now();


        if (
            code.data != lastQR ||
            now - lastSendTime > 3000
        ) {

            lastQR =
                code.data;

            lastSendTime =
                now;


            fetch(
                "/qr?data=" +
                encodeURIComponent(code.data)
            )

            .then(function(response) {

                console.log(
                    "QR sent to ESP32:",
                    code.data
                );

            })

            .catch(function(error) {

                console.log(
                    "QR send error:",
                    error
                );

            });
        }

    }

    else {

        statusText.innerHTML =
            "Scanning for QR code...";

    }


    // Scan again

    setTimeout(
        scanQR,
        100
    );
}

</script>

</body>

</html>

)rawliteral";


// =====================================================
// ROOT
// =====================================================

void handleRoot()
{
    server.send_P(
        200,
        "text/html",
        INDEX_HTML
    );
}


// =====================================================
// CAMERA STREAM
// =====================================================

void handleStream()
{
    WiFiClient client =
        server.client();

    client.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n"
    );


    while (client.connected())
    {

        camera_fb_t *fb =
            esp_camera_fb_get();


        if (!fb)
        {
            Serial.println(
                "Camera capture failed"
            );

            break;
        }


        client.printf(
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n"
            "\r\n",
            fb->len
        );


        client.write(
            fb->buf,
            fb->len
        );


        client.print(
            "\r\n"
        );


        esp_camera_fb_return(
            fb
        );


        delay(80);
    }
}


// =====================================================
// QR RECEIVED
// =====================================================

void handleQR()
{

    if (!server.hasArg("data"))
    {
        server.send(
            400,
            "text/plain",
            "No QR data"
        );

        return;
    }


    String data =
        server.arg("data");


    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "QR CODE DETECTED"
    );

    Serial.println(
        "================================"
    );

    Serial.print(
        "Payload: "
    );

    Serial.println(
        data
    );

    Serial.println(
        "================================"
    );


    server.send(
        200,
        "text/plain",
        "OK"
    );
}


// =====================================================
// CAMERA INITIALIZATION
// =====================================================

bool initCamera()
{

    camera_config_t config = {};


    config.ledc_channel =
        LEDC_CHANNEL_0;

    config.ledc_timer =
        LEDC_TIMER_0;


    config.pin_d0 =
        Y2_GPIO_NUM;

    config.pin_d1 =
        Y3_GPIO_NUM;

    config.pin_d2 =
        Y4_GPIO_NUM;

    config.pin_d3 =
        Y5_GPIO_NUM;

    config.pin_d4 =
        Y6_GPIO_NUM;

    config.pin_d5 =
        Y7_GPIO_NUM;

    config.pin_d6 =
        Y8_GPIO_NUM;

    config.pin_d7 =
        Y9_GPIO_NUM;


    config.pin_xclk =
        XCLK_GPIO_NUM;

    config.pin_pclk =
        PCLK_GPIO_NUM;

    config.pin_vsync =
        VSYNC_GPIO_NUM;

    config.pin_href =
        HREF_GPIO_NUM;


    config.pin_sccb_sda =
        SIOD_GPIO_NUM;

    config.pin_sccb_scl =
        SIOC_GPIO_NUM;


    config.pin_pwdn =
        PWDN_GPIO_NUM;

    config.pin_reset =
        RESET_GPIO_NUM;


    config.xclk_freq_hz =
        20000000;


    config.pixel_format =
        PIXFORMAT_JPEG;


    config.frame_size =
        FRAMESIZE_QVGA;


    config.jpeg_quality =
        12;


    config.fb_count =
        1;


    config.fb_location =
        CAMERA_FB_IN_DRAM;


    config.grab_mode =
        CAMERA_GRAB_WHEN_EMPTY;


    Serial.println(
        "Initializing camera..."
    );


    esp_err_t err =
        esp_camera_init(
            &config
        );


    if (err != ESP_OK)
    {

        Serial.print(
            "Camera initialization failed: 0x"
        );

        Serial.println(
            err,
            HEX
        );

        return false;
    }


    Serial.println(
        "Camera initialized successfully!"
    );


    return true;
}


// =====================================================
// SETUP
// =====================================================

void setup()
{

    Serial.begin(
        115200
    );


    delay(1000);


    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "ESP32-CAM QR SCANNER"
    );

    Serial.println(
        "================================"
    );


    // Camera

    if (!initCamera())
    {

        Serial.println(
            "CAMERA FAILED"
        );


        while (true)
        {
            delay(1000);
        }
    }


    // WiFi

    Serial.println(
        "Connecting to WiFi..."
    );


    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );


    while (
        WiFi.status() != WL_CONNECTED
    )
    {

        delay(500);

        Serial.print(".");
    }


    Serial.println();

    Serial.println(
        "WiFi connected!"
    );


    Serial.print(
        "ESP32-CAM IP address: "
    );

    Serial.println(
        WiFi.localIP()
    );


    // Routes

    server.on(
        "/",
        HTTP_GET,
        handleRoot
    );


    server.on(
        "/stream",
        HTTP_GET,
        handleStream
    );


    server.on(
        "/qr",
        HTTP_GET,
        handleQR
    );


    server.begin();


    Serial.println(
        "Web server started!"
    );


    Serial.print(
        "Open: http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        "/"
    );
}


// =====================================================
// LOOP
// =====================================================

void loop()
{

    server.handleClient();

    delay(2);
}