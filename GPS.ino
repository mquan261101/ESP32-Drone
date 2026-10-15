#include <TinyGPS++.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <ESPmDNS.h>

// ==========================================
// CẤU HÌNH PHẦN CỨNG
// ==========================================
#define GPS_RX_PIN 32
#define GPS_TX_PIN 33

TinyGPSPlus gps;
HardwareSerial GPSserial(1);

// ==========================================
// CẤU HÌNH MẠNG & WEB SERVER
// ==========================================
const char* ssid = "ID";
const char* password = "PASSWORD";

WiFiServer server(80);

// ==========================================
// BIẾN TOÀN CỤC: GPS & BỘ NHỚ QUỸ ĐẠO
// ==========================================
bool hasFix = false;
float lastLat = 0.0, lastLon = 0.0;
float lastSpeed = 0.0, lastCourse = 0.0, lastAlt = 0.0; 
int lastSats = 0;
unsigned long lastFixTime = 0;

#define MAX_POINTS 200
float latHistory[MAX_POINTS];
float lonHistory[MAX_POINTS];
int pointCount = 0;

// ==========================================
// HÀM XỬ LÝ GIAO DIỆN WEB
// ==========================================
void handleRoot(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE html>");
  client.println("<html><head> <meta charset='utf-8'>");
  client.println("<title>ESP32 GPS Tracker</title>");
  client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  client.println("<link rel='stylesheet' href='https://unpkg.com/leaflet@1.7.1/dist/leaflet.css'/>");
  client.println("<script src='https://unpkg.com/leaflet@1.7.1/dist/leaflet.js'></script>");
  client.println("<style>");
  client.println("html, body{margin:0;padding:0;height:100%;}");
  client.println("#map{height:90%;}");
  client.println("#status{height:10%; padding: 8px;font-family:sans-serif;font-size:14px; box-sizing:border-box;}");
  client.println("</style>");
  client.println("</head><body>");
  client.println("<div id='status'>Đang tải dữ liệu GPS...</div>");
  client.println("<div id='map'></div>");
  client.println("<script>");
  client.println("var map=L.map('map').setView([10.762622,106.660172],15);");
  client.println("L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);");
  client.println("var marker=L.marker([10.762622,106.660172]).addTo(map);");
  client.println("var polyline=L.polyline([], {color:'red', weight: 4}).addTo(map);"); 
  
  client.println("function updateFromServer() {");
  client.println(" fetch('/data').then(r=>r.json()).then(data=>{");
  client.println("  if(data.hasFix) {");
  client.println("   var latlng=[data.lat, data.lon];");
  client.println("   marker.setLatLng(latlng);");
  
  client.println("   if(data.path && data.path.length>0){");
  client.println("    polyline.setLatLngs(data.path);");
  client.println("    map.fitBounds(polyline.getBounds(), {maxZoom:18});");
  client.println("   } else {");
  client.println("    map.setView(latlng, 18);");
  client.println("   }");
  
  client.println("   var popupHtml='<b>* Vị trí hiện tại</b><br>' +");
  client.println("   '* Tốc độ: ' + data.speed.toFixed(1) + ' km/h<br>' +");
  client.println("   '* Hướng: ' + data.course.toFixed(1) + '&deg;<br>' +");
  client.println("   '* Độ cao: ' + data.alt.toFixed(1) + ' m<br>' +");
  client.println("   '* Vệ tinh: ' + data.sats;");
  
  client.println("   marker.bindPopup(popupHtml).openPopup();");
  client.println("   document.getElementById('status').innerText='GPS OK - Sats: ' + data.sats + ' | Đang ghi quỹ đạo';");
  client.println("  } else {");
  client.println("   document.getElementById('status').innerText='Đang dò vệ tinh hoặc mất sóng GPS.';");
  client.println("  }");
  client.println(" }).catch(e=>{console.log(e);});");
  client.println("}");
  client.println("setInterval(updateFromServer, 1000);");
  client.println("updateFromServer();");
  client.println("</script>");
  client.println("</body></html>");
}

// ==========================================
// HÀM GỬI DỮ LIỆU ĐẾN BẢN ĐỒ
// ==========================================
void handleData(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  client.print("{");
  client.print("\"hasFix\":"); client.print(hasFix ? "true" : "false");
  client.print(",\"lat\":"); client.print(lastLat, 6);
  client.print(",\"lon\":"); client.print(lastLon, 6);
  client.print(",\"speed\":"); client.print(lastSpeed, 1);
  client.print(",\"course\":"); client.print(lastCourse, 1);
  client.print(",\"alt\":"); client.print(lastAlt, 1);
  client.print(",\"sats\":"); client.print(lastSats);
  
  client.print(",\"path\":[");
  for (int i = 0; i < pointCount; i++) {
    client.print("[");
    client.print(latHistory[i], 6);
    client.print(",");
    client.print(lonHistory[i], 6);
    client.print("]");
    if (i < pointCount - 1) client.print(",");
  }
  client.print("]");
  client.print("}");
}

// ==========================================
// SETUP HỆ THỐNG
// ==========================================
void setup() {
  Serial.begin(115200);
  GPSserial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Serial.println("Dang ket noi WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nKet noi WiFi thanh cong!");
  Serial.print("IP hien tai: ");
  Serial.println(WiFi.localIP());

  // [THÊM MỚI] Khởi tạo mDNS
  if (!MDNS.begin(hostname)) {
    Serial.println("Loi khoi tao mDNS!");
  } else {
    Serial.print("mDNS khoi tao thanh cong! Truy cap: http://");
    Serial.println("drone.local");
    MDNS.addService("http", "tcp", 80); 
  }

  server.begin();
}

// ==========================================
// LOOP CHÍNH
// ==========================================
void loop() {
  while (GPSserial.available()) {
    gps.encode(GPSserial.read());
  }

  if (gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    
    lastSpeed = gps.speed.kmph();
    lastCourse = gps.course.deg();
    lastAlt = gps.altitude.meters();
    lastSats = gps.satellites.value();
    
    hasFix = gps.location.isValid(); 
    lastFixTime = millis();
    
    if (hasFix) {
      if (pointCount < MAX_POINTS) {
        latHistory[pointCount] = lastLat;
        lonHistory[pointCount] = lastLon;
        pointCount++;
      } else {
        for (int i = 1; i < MAX_POINTS; i++) {
          latHistory[i - 1] = latHistory[i];
          lonHistory[i - 1] = lonHistory[i];
        }
        latHistory[MAX_POINTS - 1] = lastLat;
        lonHistory[MAX_POINTS - 1] = lastLon;
      }
    }
  }

  if (hasFix && (millis() - lastFixTime > 5000)) {
    hasFix = false; 
  }

  WiFiClient client = server.available();
  if (client) {
    unsigned long start = millis();
    while (!client.available() && (millis() - start < 2000)) { delay(1); }
    
    if (client.available()) {
      String req = client.readStringUntil('\r');
      client.readStringUntil('\n');
      while (client.available()) client.read();
      
      if (req.indexOf("GET /data") >= 0) {
        handleData(client);
      } else {
        handleRoot(client);
      }
    }
    delay(1);
    client.stop();
  }
}