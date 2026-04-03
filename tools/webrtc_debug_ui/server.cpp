#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct Options {
    std::string host = "0.0.0.0";
    int port = 8090;
    std::string zlm_host = "127.0.0.1";
    int zlm_http_port = 8080;
    std::string telemetry_path;
    std::string default_app = "live";
    std::string default_stream = "camera";
};

struct HttpRequest {
    std::string method;
    std::string target;
    std::string path;
    std::string query;
    std::unordered_map<std::string, std::string> headers;
    std::string body;
};

std::atomic<bool> g_running {true};

std::string toLower(std::string text) {
    for (char& ch : text) {
        if (ch >= 'A' && ch <= 'Z') {
            ch = static_cast<char>(ch - 'A' + 'a');
        }
    }
    return text;
}

std::string trim(const std::string& text) {
    std::size_t begin = 0;
    while (begin < text.size() && (text[begin] == ' ' || text[begin] == '\t' || text[begin] == '\r' || text[begin] == '\n')) {
        ++begin;
    }
    std::size_t end = text.size();
    while (end > begin && (text[end - 1] == ' ' || text[end - 1] == '\t' || text[end - 1] == '\r' || text[end - 1] == '\n')) {
        --end;
    }
    return text.substr(begin, end - begin);
}

std::string jsonEscape(const std::string& text) {
    std::string out;
    out.reserve(text.size() + 8);
    for (char ch : text) {
        switch (ch) {
            case '\\':
                out += "\\\\";
                break;
            case '"':
                out += "\\\"";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += ch;
                break;
        }
    }
    return out;
}

std::string readWholeFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.good()) {
        return {};
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

bool readLastNonEmptyLine(const std::string& path, std::string* line) {
    if (line == nullptr || path.empty()) {
        return false;
    }

    std::ifstream in(path, std::ios::binary);
    if (!in.good()) {
        return false;
    }

    in.seekg(0, std::ios::end);
    std::streamoff pos = in.tellg();
    if (pos <= 0) {
        return false;
    }

    std::string reversed;
    reversed.reserve(1024);

    while (pos > 0) {
        --pos;
        in.seekg(pos);
        char ch = 0;
        in.get(ch);
        if (!in.good()) {
            break;
        }
        if (ch == '\n') {
            if (!reversed.empty()) {
                break;
            }
            continue;
        }
        reversed.push_back(ch);
    }

    if (reversed.empty()) {
        return false;
    }

    line->assign(reversed.rbegin(), reversed.rend());
    return true;
}

std::string statusText(int code) {
    switch (code) {
        case 200:
            return "OK";
        case 204:
            return "No Content";
        case 400:
            return "Bad Request";
        case 404:
            return "Not Found";
        case 405:
            return "Method Not Allowed";
        case 502:
            return "Bad Gateway";
        default:
            return "OK";
    }
}

bool sendAll(int fd, const char* data, std::size_t len) {
    std::size_t sent = 0;
    while (sent < len) {
        const ssize_t n = send(fd, data + sent, len - sent, 0);
        if (n <= 0) {
            return false;
        }
        sent += static_cast<std::size_t>(n);
    }
    return true;
}

bool sendResponse(int fd, int code, const std::string& content_type, const std::string& body, bool extra_cors = true) {
    std::ostringstream out;
    out << "HTTP/1.1 " << code << ' ' << statusText(code) << "\r\n";
    out << "Content-Type: " << content_type << "\r\n";
    out << "Content-Length: " << body.size() << "\r\n";
    out << "Cache-Control: no-store\r\n";
    if (extra_cors) {
        out << "Access-Control-Allow-Origin: *\r\n";
        out << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
        out << "Access-Control-Allow-Headers: Content-Type\r\n";
    }
    out << "Connection: close\r\n\r\n";
    const std::string header = out.str();
    return sendAll(fd, header.data(), header.size()) && sendAll(fd, body.data(), body.size());
}

bool parseRequest(int fd, HttpRequest* req) {
    if (req == nullptr) {
        return false;
    }

    std::string raw;
    raw.reserve(8192);
    char buffer[4096];

    while (raw.find("\r\n\r\n") == std::string::npos) {
        const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0) {
            return false;
        }
        raw.append(buffer, static_cast<std::size_t>(n));
        if (raw.size() > (1U << 20)) {
            return false;
        }
    }

    const std::size_t header_end = raw.find("\r\n\r\n");
    const std::string header_text = raw.substr(0, header_end);
    std::istringstream hs(header_text);

    std::string request_line;
    if (!std::getline(hs, request_line)) {
        return false;
    }
    request_line = trim(request_line);
    std::istringstream rl(request_line);
    std::string version;
    if (!(rl >> req->method >> req->target >> version)) {
        return false;
    }

    const std::size_t qpos = req->target.find('?');
    req->path = qpos == std::string::npos ? req->target : req->target.substr(0, qpos);
    req->query = qpos == std::string::npos ? "" : req->target.substr(qpos + 1);

    std::string line;
    while (std::getline(hs, line)) {
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const std::size_t sep = line.find(':');
        if (sep == std::string::npos) {
            continue;
        }
        std::string key = toLower(trim(line.substr(0, sep)));
        std::string value = trim(line.substr(sep + 1));
        req->headers[key] = value;
    }

    std::size_t content_length = 0;
    const auto it = req->headers.find("content-length");
    if (it != req->headers.end()) {
        content_length = static_cast<std::size_t>(std::strtoul(it->second.c_str(), nullptr, 10));
    }

    req->body = raw.substr(header_end + 4);
    while (req->body.size() < content_length) {
        const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0) {
            return false;
        }
        req->body.append(buffer, static_cast<std::size_t>(n));
    }
    if (req->body.size() > content_length) {
        req->body.resize(content_length);
    }

    return true;
}

int connectToHost(const std::string& host, int port) {
    addrinfo hints {};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* result = nullptr;
    const std::string port_text = std::to_string(port);
    if (getaddrinfo(host.c_str(), port_text.c_str(), &hints, &result) != 0) {
        return -1;
    }

    int fd = -1;
    for (addrinfo* p = result; p != nullptr; p = p->ai_next) {
        fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (connect(fd, p->ai_addr, p->ai_addrlen) == 0) {
            break;
        }
        close(fd);
        fd = -1;
    }

    freeaddrinfo(result);
    return fd;
}

bool proxyWebrtc(const Options& opt, const HttpRequest& req, int client_fd) {
    const int upstream_fd = connectToHost(opt.zlm_host, opt.zlm_http_port);
    if (upstream_fd < 0) {
        const std::string body = "{\"code\":-1,\"msg\":\"proxy connect failed\"}";
        return sendResponse(client_fd, 502, "application/json; charset=utf-8", body);
    }

    const auto host_it = req.headers.find("host");
    const std::string host_header = host_it == req.headers.end() ? opt.zlm_host : host_it->second;
    std::ostringstream ureq;
    ureq << "POST /index/api/webrtc";
    if (!req.query.empty()) {
        ureq << '?' << req.query;
    }
    ureq << " HTTP/1.1\r\n";
    ureq << "Host: " << host_header << "\r\n";
    ureq << "Content-Type: text/plain;charset=utf-8\r\n";
    ureq << "Content-Length: " << req.body.size() << "\r\n";
    ureq << "Connection: close\r\n\r\n";
    const std::string header = ureq.str();

    bool ok = sendAll(upstream_fd, header.data(), header.size());
    if (ok && !req.body.empty()) {
        ok = sendAll(upstream_fd, req.body.data(), req.body.size());
    }

    std::string upstream_response;
    char buf[4096];
    while (ok) {
        const ssize_t n = recv(upstream_fd, buf, sizeof(buf), 0);
        if (n < 0) {
            ok = false;
            break;
        }
        if (n == 0) {
            break;
        }
        upstream_response.append(buf, static_cast<std::size_t>(n));
    }

    close(upstream_fd);

    if (!ok || upstream_response.empty()) {
        const std::string body = "{\"code\":-1,\"msg\":\"proxy read failed\"}";
        return sendResponse(client_fd, 502, "application/json; charset=utf-8", body);
    }

    const std::size_t header_end = upstream_response.find("\r\n\r\n");
    if (header_end == std::string::npos) {
        const std::string body = "{\"code\":-1,\"msg\":\"invalid upstream response\"}";
        return sendResponse(client_fd, 502, "application/json; charset=utf-8", body);
    }

    std::string status_line;
    {
        std::istringstream is(upstream_response.substr(0, header_end));
        std::getline(is, status_line);
    }
    int status_code = 200;
    {
        std::istringstream ss(status_line);
        std::string http_ver;
        ss >> http_ver >> status_code;
    }

    std::string content_type = "application/json; charset=utf-8";
    {
        std::istringstream is(upstream_response.substr(0, header_end));
        std::string line;
        while (std::getline(is, line)) {
            line = trim(line);
            if (line.empty()) {
                continue;
            }
            const std::size_t sep = line.find(':');
            if (sep == std::string::npos) {
                continue;
            }
            const std::string key = toLower(trim(line.substr(0, sep)));
            if (key == "content-type") {
                content_type = trim(line.substr(sep + 1));
                break;
            }
        }
    }

    const std::string body = upstream_response.substr(header_end + 4);
    return sendResponse(client_fd, status_code, content_type, body);
}

bool handleClient(const Options& opt, int client_fd, const std::string& web_root) {
    HttpRequest req;
    if (!parseRequest(client_fd, &req)) {
        const std::string body = "{\"code\":-1,\"msg\":\"bad request\"}";
        return sendResponse(client_fd, 400, "application/json; charset=utf-8", body);
    }

    if (req.method == "OPTIONS") {
        return sendResponse(client_fd, 204, "text/plain; charset=utf-8", "");
    }

    if (req.method == "GET" && req.path == "/api/config") {
        const char* rtc_port_env = std::getenv("RK3588_WEBRTC_RTC_PORT");
        const std::string rtc_port = rtc_port_env != nullptr ? rtc_port_env : "8000";
        std::ostringstream body;
        body << '{'
             << "\"zlm_host\":\"" << jsonEscape(opt.zlm_host) << "\","
             << "\"zlm_http_port\":" << opt.zlm_http_port << ','
             << "\"default_app\":\"" << jsonEscape(opt.default_app) << "\","
             << "\"default_stream\":\"" << jsonEscape(opt.default_stream) << "\","
             << "\"telemetry_enabled\":" << (opt.telemetry_path.empty() ? "false" : "true") << ','
             << "\"default_play_url\":\"rtc://" << jsonEscape(opt.zlm_host) << ':' << jsonEscape(rtc_port)
             << '/' << jsonEscape(opt.default_app) << '/' << jsonEscape(opt.default_stream) << "\""
             << '}';
        return sendResponse(client_fd, 200, "application/json; charset=utf-8", body.str());
    }

    if (req.method == "GET" && req.path == "/api/telemetry/latest") {
        std::string line;
        if (!readLastNonEmptyLine(opt.telemetry_path, &line)) {
            return sendResponse(client_fd, 200, "application/json; charset=utf-8", "{\"ok\":false,\"snapshot\":null}");
        }
        const std::string trimmed = trim(line);
        const bool looks_json = !trimmed.empty() && trimmed.front() == '{' && trimmed.back() == '}';
        if (!looks_json) {
            return sendResponse(client_fd, 200, "application/json; charset=utf-8", "{\"ok\":false,\"snapshot\":null}");
        }
        const std::string body = std::string("{\"ok\":true,\"snapshot\":") + trimmed + "}";
        return sendResponse(client_fd, 200, "application/json; charset=utf-8", body);
    }

    if (req.method == "POST" && req.path == "/api/webrtc") {
        return proxyWebrtc(opt, req, client_fd);
    }

    if (req.method != "GET") {
        const std::string body = "{\"code\":-1,\"msg\":\"method not allowed\"}";
        return sendResponse(client_fd, 405, "application/json; charset=utf-8", body);
    }

    std::string file_path;
    std::string content_type;
    if (req.path == "/" || req.path == "/index.html") {
        file_path = web_root + "/index.html";
        content_type = "text/html; charset=utf-8";
    } else {
        const std::string body = "{\"code\":-1,\"msg\":\"not found\"}";
        return sendResponse(client_fd, 404, "application/json; charset=utf-8", body);
    }

    const std::string body = readWholeFile(file_path);
    if (body.empty()) {
        const std::string payload = "{\"code\":-1,\"msg\":\"index file not found\"}";
        return sendResponse(client_fd, 404, "application/json; charset=utf-8", payload);
    }
    return sendResponse(client_fd, 200, content_type, body);
}

void signalHandler(int) {
    g_running.store(false);
}

bool parseInt(const std::string& text, int* out) {
    if (out == nullptr || text.empty()) {
        return false;
    }
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == nullptr || *end != '\0') {
        return false;
    }
    *out = static_cast<int>(value);
    return true;
}

Options parseArgs(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if ((key == "--host" || key == "--port" || key == "--zlm-host" || key == "--zlm-http-port" ||
             key == "--telemetry-path" || key == "--default-app" || key == "--default-stream") && i + 1 >= argc) {
            std::cerr << "missing value for " << key << '\n';
            std::exit(2);
        }

        if (key == "--host") {
            opt.host = argv[++i];
        } else if (key == "--port") {
            if (!parseInt(argv[++i], &opt.port)) {
                std::cerr << "invalid --port\n";
                std::exit(2);
            }
        } else if (key == "--zlm-host") {
            opt.zlm_host = argv[++i];
        } else if (key == "--zlm-http-port") {
            if (!parseInt(argv[++i], &opt.zlm_http_port)) {
                std::cerr << "invalid --zlm-http-port\n";
                std::exit(2);
            }
        } else if (key == "--telemetry-path") {
            opt.telemetry_path = argv[++i];
        } else if (key == "--default-app") {
            opt.default_app = argv[++i];
        } else if (key == "--default-stream") {
            opt.default_stream = argv[++i];
        } else if (key == "-h" || key == "--help") {
            std::cout
                << "Usage: webrtc_debug_ui_server [options]\n"
                << "  --host <host>\n"
                << "  --port <port>\n"
                << "  --zlm-host <host>\n"
                << "  --zlm-http-port <port>\n"
                << "  --telemetry-path <path>\n"
                << "  --default-app <name>\n"
                << "  --default-stream <name>\n";
            std::exit(0);
        } else {
            std::cerr << "unknown option: " << key << '\n';
            std::exit(2);
        }
    }

    return opt;
}

}  // namespace

int main(int argc, char** argv) {
    const Options opt = parseArgs(argc, argv);

    std::string web_root;
    {
        std::string exe_path;
        std::vector<char> buf(4096, '\0');
        const ssize_t n = readlink("/proc/self/exe", buf.data(), buf.size() - 1);
        if (n > 0) {
            exe_path.assign(buf.data(), static_cast<std::size_t>(n));
            const std::size_t slash = exe_path.find_last_of('/');
            if (slash != std::string::npos) {
                const std::string parent = exe_path.substr(0, slash);
                web_root = parent + "/../tools/webrtc_debug_ui";
            }
        }
    }
    if (web_root.empty()) {
        web_root = "tools/webrtc_debug_ui";
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "socket failed: " << std::strerror(errno) << '\n';
        return 1;
    }

    int yes = 1;
    (void)setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(opt.port));
    if (inet_pton(AF_INET, opt.host.c_str(), &addr.sin_addr) <= 0) {
        if (opt.host == "0.0.0.0") {
            addr.sin_addr.s_addr = INADDR_ANY;
        } else {
            std::cerr << "invalid host: " << opt.host << '\n';
            close(server_fd);
            return 1;
        }
    }

    if (bind(server_fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind failed: " << std::strerror(errno) << '\n';
        close(server_fd);
        return 1;
    }

    if (listen(server_fd, 32) < 0) {
        std::cerr << "listen failed: " << std::strerror(errno) << '\n';
        close(server_fd);
        return 1;
    }

    std::cout << "RK3588 WebRTC debug UI (C++) listening on http://" << opt.host << ':' << opt.port
              << " (ZLM http " << opt.zlm_host << ':' << opt.zlm_http_port
              << ", telemetry=" << (opt.telemetry_path.empty() ? "disabled" : opt.telemetry_path) << ")\n";

    while (g_running.load()) {
        sockaddr_in client_addr {};
        socklen_t client_len = sizeof(client_addr);
        const int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }

        std::thread([opt, client_fd, web_root] {
            (void)handleClient(opt, client_fd, web_root);
            close(client_fd);
        }).detach();
    }

    close(server_fd);
    return 0;
}
