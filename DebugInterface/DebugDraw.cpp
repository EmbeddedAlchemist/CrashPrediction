#include "DebugDraw.hpp"
#include <box2d/b2_world.h>
#include <ExLog/ExLog.hpp>

using namespace ArduinoJson;


DebugDraw::DebugDraw(const std::string host, std::uint16_t port) {
    jdoc.to<JsonObject>();
    jarr = jdoc["draw_actions"].to<JsonArray>();
    svrThread = std::make_unique<std::thread>([=]() {this->serverThreadTask(host, port); });
}

DebugDraw::~DebugDraw()
{

}

void DebugDraw::serverThreadTask(const std::string host, std::uint16_t port) {
    svr.Options(R"(/.*)", [](const httplib::Request &req, httplib::Response &res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.set_header("Access-Control-Max-Age", "86400");
        res.status = 200;
        });

    svr.Get("/get_draw_actions", [&](const httplib::Request &, httplib::Response &res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.set_header("Access-Control-Max-Age", "86400");
        if (world == nullptr) {
            res.status = 500;
            return;
        }
        mutex.lock();
        jarr.clear();
        this->SetFlags(e_shapeBit | e_jointBit | e_aabbBit | e_pairBit | e_centerOfMassBit);
        world->DebugDraw();
        std::string response;
        ArduinoJson::serializeJson(jdoc, response);
        mutex.unlock();
        res.set_content(response, "application/json");
        });

    svr.Get("/run", [&](const httplib::Request &, httplib::Response &res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.set_header("Access-Control-Max-Age", "86400");
        if (world == nullptr) {
            res.status = 500;
            return;
        }
        world->Step(0.1, 10, 8);
        LOG_I << "Debug Run Successful";
        res.set_content(R"({"err_status":"OK"})", "application/json");
        });
    svr.listen(host, port);
}

void DebugDraw::DrawPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color) {
    auto obj = jarr.add<JsonObject>();
    obj["type"] = "polygon";
    auto jcolor = obj["color"].to<JsonObject>();
    jcolor["r"] = color.r;
    jcolor["g"] = color.g;
    jcolor["b"] = color.b;
    jcolor["a"] = color.a;
    auto jvertices = obj["vertices"].to<JsonArray>();
    for (int i = 0; i < vertexCount; i++) {
        auto jvertex = jvertices.add<JsonObject>();
        jvertex["x"] = vertices[i].x;
        jvertex["y"] = vertices[i].y;
    }
}

void DebugDraw::DrawSolidPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color) {
    auto obj = jarr.add<JsonObject>();
    obj["type"] = "polygon_solid";
    auto jcolor = obj["color"].to<JsonObject>();
    jcolor["r"] = color.r;
    jcolor["g"] = color.g;
    jcolor["b"] = color.b;
    jcolor["a"] = color.a;
    auto jvertices = obj["vertices"].to<JsonArray>();
    for (int i = 0; i < vertexCount; i++) {
        auto jvertex = jvertices.add<JsonObject>();
        jvertex["x"] = vertices[i].x;
        jvertex["y"] = vertices[i].y;
    }
}

void DebugDraw::DrawCircle(const b2Vec2 &center, float radius, const b2Color &color) {
    auto obj = jarr.add<JsonObject>();
    obj["type"] = "circle";
    auto jcolor = obj["color"].to<JsonObject>();
    jcolor["r"] = color.r;
    jcolor["g"] = color.g;
    jcolor["b"] = color.b;
    jcolor["a"] = color.a;
    auto jcenter = obj["center"].to<JsonObject>();
    jcenter["x"] = center.x;
    jcenter["y"] = center.y;
    obj["radius"] = radius;
}

void DebugDraw::DrawSolidCircle(const b2Vec2 &center, float radius, const b2Vec2 &axis, const b2Color &color) {
    auto obj = jarr.add<JsonObject>();
    obj["type"] = "circle_solid";
    auto jcolor = obj["color"].to<JsonObject>();
    jcolor["r"] = color.r;
    jcolor["g"] = color.g;
    jcolor["b"] = color.b;
    jcolor["a"] = color.a;
    auto jcenter = obj["center"].to<JsonObject>();
    jcenter["x"] = center.x;
    jcenter["y"] = center.y;
    obj["radius"] = radius;
}

void DebugDraw::DrawSegment(const b2Vec2 &p1, const b2Vec2 &p2, const b2Color &color) {
    auto obj = jarr.add<JsonObject>();
    obj["type"] = "segment";
    auto jcolor = obj["color"].to<JsonObject>();
    jcolor["r"] = color.r;
    jcolor["g"] = color.g;
    jcolor["b"] = color.b;
    jcolor["a"] = color.a;
    auto jp1 = obj["p1"].to<JsonObject>();
    jp1["x"] = p1.x;
    jp1["y"] = p1.y;
    auto jp2 = obj["p2"].to<JsonObject>();
    jp2["x"] = p2.x;
    jp2["y"] = p2.y;

}

void DebugDraw::DrawTransform(const b2Transform &xf)
{

}

void DebugDraw::DrawPoint(const b2Vec2 &p, float size, const b2Color &color) {
    auto obj = jarr.add<JsonObject>();
    obj["type"] = "point";
    auto jcolor = obj["color"].to<JsonObject>();
    jcolor["r"] = color.r;
    jcolor["g"] = color.g;
    jcolor["b"] = color.b;
    jcolor["a"] = color.a;
    auto jp = obj["p"].to<JsonObject>();
    jp["x"] = p.x;
    jp["y"] = p.y;
    obj["size"] = size;
}

void DebugDraw::bindToWorld(b2World *w){
    world = w;
    world->SetDebugDraw(this);
}

