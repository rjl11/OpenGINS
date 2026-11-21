#include "preview_http.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

#include <opencv2/imgcodecs.hpp>

#include "preview_metrics.hpp"

namespace flow {

namespace {

// The page. Plain JS, no libraries: the unit has no internet.
const char k_page[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Front camera</title>
<style>
:root{--bg:#111;--fg:#ddd;--dim:#888;--ok:#4c4;--bad:#e44;--bar:#3a7bd5}
body{margin:0;padding:12px 16px;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,sans-serif}
h1{font-size:18px;margin:0 0 8px}
canvas{display:block;width:100%;max-width:960px;image-rendering:pixelated;background:#000}
.row{display:flex;flex-wrap:wrap;gap:8px 20px;margin:10px 0;max-width:960px}
.row div{min-width:130px}.k{color:var(--dim);font-size:13px}.v{font-size:18px;font-variant-numeric:tabular-nums}
#bar{height:14px;background:#333;max-width:960px;margin:4px 0}#barf{height:100%;width:0;background:var(--bar)}
button,label{font:inherit;color:var(--fg)}button{background:#333;border:1px solid #555;padding:6px 12px;border-radius:4px}
#msg{color:var(--bad)}.ok{color:var(--ok)}.bad{color:var(--bad)}
</style></head><body>
<h1>Front camera preview</h1>
<canvas id="c" width="640" height="400"></canvas>
<div class="row">
<label><input type="checkbox" id="ov" checked> features and box</label>
<label><input type="checkbox" id="zm"> zoom on the focus box</label>
<span id="msg"></span>
</div>
<div class="k">Focus (inside the box; relative, depends on the scene; turn the lens until it peaks)</div>
<div id="bar"><div id="barf"></div></div>
<div class="row">
<div><div class="k">focus</div><div class="v" id="f">-</div></div>
<div><div class="k">peak</div><div class="v" id="fp">-</div></div>
<div><button id="rs">reset peak</button></div>
</div>
<div class="row">
<div><div class="k">mean grey</div><div class="v" id="mean">-</div></div>
<div><div class="k">black / white</div><div class="v" id="clip">-</div></div>
<div><div class="k">exposure / gain</div><div class="v" id="exp">-</div></div>
</div>
<div class="row">
<div><div class="k">features</div><div class="v" id="nt">-</div></div>
<div><div class="k">frames / s</div><div class="v" id="rate">-</div></div>
<div><div class="k">valid</div><div class="v" id="val">-</div></div>
<div><div class="k">flow per frame</div><div class="v" id="fm">-</div></div>
<div><div class="k">dropped</div><div class="v" id="dr">-</div></div>
<div><div class="k">image</div><div class="v" id="sz">-</div></div>
<div id="lb" hidden><div class="k">lidar range / amp</div><div class="v" id="lr">-</div></div>
<div id="lb2" hidden><div class="k">lidar rate / ok</div><div class="v" id="lq">-</div></div>
</div>
<script>
const $=id=>document.getElementById(id),c=$('c'),g=c.getContext('2d');
let peak=0;
$('rs').onclick=()=>{peak=0};
function fmt(x,d){return x==null?'-':x.toFixed(d)}
function draw(bmp,s){
  const r=s.roi,zoom=$('zm').checked;
  const sx=zoom?r[0]:0,sy=zoom?r[1]:0,sw=zoom?r[2]:bmp.width,sh=zoom?r[3]:bmp.height;
  if(c.width!=sw||c.height!=sh){c.width=sw;c.height=sh}
  g.drawImage(bmp,sx,sy,sw,sh,0,0,sw,sh);
  if($('ov').checked){
    const k=bmp.width/s.proc_w;
    g.strokeStyle='#ff0';g.lineWidth=1;
    if(!zoom){g.strokeRect(r[0]+.5,r[1]+.5,r[2]-1,r[3]-1);
      g.beginPath();g.moveTo(sw/2-12,sh/2);g.lineTo(sw/2+12,sh/2);g.moveTo(sw/2,sh/2-12);g.lineTo(sw/2,sh/2+12);g.stroke()}
    g.fillStyle=s.valid?'#0f0':'#f40';
    for(const p of s.pts){const x=p[0]*k-sx,y=p[1]*k-sy;if(x>=0&&y>=0&&x<sw&&y<sh)g.fillRect(x-1.5,y-1.5,3,3)}
  }
  peak=Math.max(peak,s.focus);
  $('f').textContent=fmt(s.focus,0);$('fp').textContent=fmt(peak,0);
  $('barf').style.width=(peak>0?100*s.focus/peak:0)+'%';
  $('mean').textContent=fmt(s.mean,0);
  $('clip').textContent=fmt(s.pct_black,1)+' / '+fmt(s.pct_white,1)+' %';
  $('clip').className='v '+(s.pct_white>1||s.pct_black>5?'bad':'');
  $('exp').textContent=(s.exposure_us/1000).toFixed(2)+' ms / '+fmt(s.gain,1);
  $('nt').textContent=s.n_tracked+(s.phase?' · phase':'');$('nt').className='v '+(s.valid?'ok':'bad');
  $('rate').textContent=fmt(s.msg_per_s,1);$('val').textContent=fmt(s.pct_valid,0)+' %';
  $('fm').textContent=fmt(s.flow_ms,2)+' ms';$('dr').textContent=s.dropped;
  $('sz').textContent=s.w+'x'+s.h;
  const L=s.lidar;$('lb').hidden=$('lb2').hidden=!L;
  if(L){$('lr').textContent=L.valid?fmt(L.range_m,3)+' m'+(L.held?' held':'')+' / '+L.amp:'-';
    $('lr').className='v '+(L.valid&&!L.held?'ok':'bad');
    $('lq').textContent=fmt(L.hz,0)+' Hz / '+fmt(L.pct_ok,0)+' %'}
}
async function tick(){
  try{
    const r=await fetch('frame.jpg',{cache:'no-store'});
    if(!r.ok)throw new Error(await r.text());
    const bmp=await createImageBitmap(await r.blob());
    const s=await (await fetch('stats.json',{cache:'no-store'})).json();
    draw(bmp,s);$('msg').textContent='';
  }catch(e){$('msg').textContent='no frame: '+e.message}
  setTimeout(tick,250);
}
tick();
</script></body></html>
)HTML";

bool send_all(int c, const void* p, size_t n)
{
    const char* b = static_cast<const char*>(p);
    while (n > 0) {
        const ssize_t k = ::send(c, b, n, MSG_NOSIGNAL);
        if (k < 0 && errno == EINTR) continue;
        if (k <= 0) return false;
        b += k;
        n -= static_cast<size_t>(k);
    }
    return true;
}

void reply(int c, const char* status, const char* type, const void* body, size_t n)
{
    char h[256];
    const int k = std::snprintf(h, sizeof h,
                                "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
                                "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                                status, type, n);
    if (send_all(c, h, static_cast<size_t>(k))) send_all(c, body, n);
}

void reply_text(int c, const char* status, const std::string& s)
{
    reply(c, status, "text/plain; charset=utf-8", s.data(), s.size());
}

}  // namespace

bool PreviewServer::start(uint16_t port, std::string& err)
{
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        err = std::strerror(errno);
        return false;
    }
    const int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(port);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0 || ::listen(fd, 4) < 0) {
        err = "port " + std::to_string(port) + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }
    fd_ = fd;
    stop_ = false;
    thread_ = std::thread([this] { loop(); });
    return true;
}

void PreviewServer::stop()
{
    stop_ = true;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

void PreviewServer::offer(const cv::Mat& img, const std::vector<cv::Point2f>& pts, const PreviewInfo& info)
{
    {
        std::lock_guard<std::mutex> l(mu_);
        img.copyTo(latest_.img);
        latest_.pts = pts;
        latest_.info = info;
        latest_.n++;
    }
    want_.store(false, std::memory_order_relaxed);
    cv_.notify_all();
}

bool PreviewServer::fresh(Snap& out, int timeout_ms)
{
    std::unique_lock<std::mutex> l(mu_);
    const uint64_t n0 = latest_.n;
    want_.store(true, std::memory_order_relaxed);
    const bool got = cv_.wait_for(l, std::chrono::milliseconds(timeout_ms),
                                  [&] { return latest_.n != n0 || stop_; }) &&
                     latest_.n != n0;
    if (!got) {
        want_.store(false, std::memory_order_relaxed);
        return false;
    }
    latest_.img.copyTo(out.img);  // the flow loop may overwrite latest_ next
    out.pts = latest_.pts;
    out.info = latest_.info;
    out.n = latest_.n;
    return true;
}

std::vector<std::string> PreviewServer::urls(uint16_t port)
{
    std::vector<std::string> v;
    ifaddrs*                 ifs = nullptr;
    if (::getifaddrs(&ifs) != 0) return v;
    for (ifaddrs* i = ifs; i; i = i->ifa_next) {
        if (!i->ifa_addr || i->ifa_addr->sa_family != AF_INET) continue;
        const auto* s = reinterpret_cast<const sockaddr_in*>(i->ifa_addr);
        if ((ntohl(s->sin_addr.s_addr) >> 24) == 127) continue;
        char ip[INET_ADDRSTRLEN];
        ::inet_ntop(AF_INET, &s->sin_addr, ip, sizeof ip);
        v.push_back("http://" + std::string(ip) + ":" + std::to_string(port) + "/");
    }
    ::freeifaddrs(ifs);
    return v;
}

void PreviewServer::loop()
{
    // Below the flow loop: the page may lag, the flow must not.
    ::setpriority(PRIO_PROCESS, static_cast<id_t>(::syscall(SYS_gettid)), 10);
    while (!stop_) {
        pollfd p{fd_, POLLIN, 0};
        if (::poll(&p, 1, 200) <= 0) continue;
        const int c = ::accept4(fd_, nullptr, nullptr, SOCK_CLOEXEC);
        if (c < 0) continue;
        timeval tv{2, 0};  // a stalled browser can't hold the thread
        ::setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        ::setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
        serve(c);
        ::close(c);
    }
}

std::string PreviewServer::stats_json(const Snap& s) const
{
    const cv::Rect      roi = focus_roi(s.img.size());
    const double        focus = focus_laplacian_var(s.img, roi);
    const ExposureStats e = exposure_stats(s.img);
    const PreviewInfo&  i = s.info;
    std::string         j;
    char                b[512];
    std::snprintf(b, sizeof b,
                  "{\"seq\":%u,\"w\":%d,\"h\":%d,\"proc_w\":%d,\"proc_h\":%d,\"roi\":[%d,%d,%d,%d],"
                  "\"focus\":%.1f,\"mean\":%.1f,\"pct_black\":%.2f,\"pct_white\":%.2f,"
                  "\"exposure_us\":%d,\"gain\":%.2f,\"n_tracked\":%d,\"valid\":%s,\"phase\":%s,\"dropped\":%llu,"
                  "\"msg_per_s\":%.1f,\"pct_valid\":%.1f,\"flow_ms\":%.2f",
                  i.seq, s.img.cols, s.img.rows, i.proc_w, i.proc_h, roi.x, roi.y, roi.width, roi.height, focus,
                  e.mean, e.pct_black, e.pct_white, i.exposure_us, static_cast<double>(i.gain), i.n_tracked,
                  i.valid ? "true" : "false", i.phase ? "true" : "false", static_cast<unsigned long long>(i.dropped), i.msg_per_s, i.pct_valid,
                  i.flow_ms);
    j = b;
    if (i.lidar) {
        std::snprintf(b, sizeof b,
                      ",\"lidar\":{\"valid\":%s,\"held\":%s,\"range_m\":%.3f,\"amp\":%u,\"hz\":%.1f,\"pct_ok\":%.1f}",
                      i.range_valid ? "true" : "false", i.range_held ? "true" : "false",
                      static_cast<double>(i.range_m), i.range_amp, i.lidar_hz, i.lidar_pct_ok);
        j += b;
    }
    j += ",\"pts\":[";
    for (size_t k = 0; k < s.pts.size(); k++) {
        std::snprintf(b, sizeof b, "%s[%.1f,%.1f]", k ? "," : "", static_cast<double>(s.pts[k].x),
                      static_cast<double>(s.pts[k].y));
        j += b;
    }
    j += "]}";
    return j;
}

void PreviewServer::serve(int c)
{
    char   req[2048];
    size_t n = 0;
    while (n < sizeof req - 1) {
        const ssize_t k = ::recv(c, req + n, sizeof req - 1 - n, 0);
        if (k < 0 && errno == EINTR) continue;
        if (k <= 0) break;
        n += static_cast<size_t>(k);
        req[n] = 0;
        if (std::strstr(req, "\r\n\r\n") || std::strstr(req, "\n\n")) break;
    }
    req[n] = 0;
    char method[8] = {0}, path[256] = {0};
    if (std::sscanf(req, "%7s %255s", method, path) != 2) return;
    if (std::strcmp(method, "GET") != 0) {
        reply_text(c, "405 Method Not Allowed", "GET only\n");
        return;
    }
    if (char* q = std::strchr(path, '?')) *q = 0;

    if (std::strcmp(path, "/") == 0 || std::strcmp(path, "/index.html") == 0) {
        reply(c, "200 OK", "text/html; charset=utf-8", k_page, sizeof k_page - 1);
    } else if (std::strcmp(path, "/frame.jpg") == 0) {
        if (!fresh(served_, 1000)) {
            reply_text(c, "503 Service Unavailable", "no frame from the camera within 1 s\n");
            return;
        }
        std::vector<uchar> jpg;
        cv::imencode(".jpg", served_.img, jpg, {cv::IMWRITE_JPEG_QUALITY, 90});
        reply(c, "200 OK", "image/jpeg", jpg.data(), jpg.size());
    } else if (std::strcmp(path, "/stats.json") == 0) {
        // Figures for the frame the page shows; a fresh one if none yet.
        if (served_.img.empty() && !fresh(served_, 1000)) {
            reply_text(c, "503 Service Unavailable", "no frame from the camera within 1 s\n");
            return;
        }
        const std::string j = stats_json(served_);
        reply(c, "200 OK", "application/json", j.data(), j.size());
    } else {
        reply_text(c, "404 Not Found", "not found\n");
    }
}

}  // namespace flow
